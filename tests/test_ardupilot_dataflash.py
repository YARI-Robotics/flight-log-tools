from __future__ import annotations

import shutil
import struct
import sys
import unittest
from pathlib import Path

# Tests intentionally build tiny synthetic logs instead of storing sample flight
# logs in the repository. That keeps the test fixtures small and avoids carrying
# any real location data in an open-source repo.
REPO_ROOT = Path(__file__).resolve().parents[1]
GPS_RELOCATION_ROOT = REPO_ROOT / "gps-relocation"
if str(GPS_RELOCATION_ROOT) not in sys.path:
    sys.path.insert(0, str(GPS_RELOCATION_ROOT))
TEST_TEMP_ROOT = REPO_ROOT / "pytest-cache-files-ardupilot-relocation"

from relocate_ardupilot_bin import (
    DATAFLASH_MAGIC,
    ORIGIN,
    default_relocated_path,
    parse_dataflash_format,
    relocate_ardupilot_bin,
)


def _message(type_id: int, payload: bytes) -> bytes:
    """Build one DataFlash message: magic bytes, type ID, then payload."""

    return DATAFLASH_MAGIC + bytes([type_id]) + payload


def _fmt_record(type_id: int, length: int, name: str, format_text: str, columns: str) -> bytes:
    """Build an FMT record that teaches the parser a message layout."""

    payload = struct.pack(
        "<BB4s16s64s",
        type_id,
        length,
        name.encode("ascii"),
        format_text.encode("ascii"),
        columns.encode("ascii"),
    )
    return _message(0x80, payload)


def _gps_record(latitude_deg: float, longitude_deg: float, altitude_m: float) -> bytes:
    """Create one GPS record using ArduPilot's scaled integer coordinate fields."""

    payload = struct.pack(
        "<Qiii",
        1,
        round(latitude_deg * 10_000_000),
        round(longitude_deg * 10_000_000),
        round(altitude_m * 100),
    )
    return _message(0x81, payload)


def _minimal_dataflash(records: list[bytes]) -> bytes:
    """Build the smallest DataFlash log needed to exercise GPS relocation."""

    gps_length = 3 + struct.calcsize("<Qiii")
    return b"".join(
        [
            _fmt_record(0x81, gps_length, "GPS", "QLLe", "TimeUS,Lat,Lng,Alt"),
            *records,
        ]
    )


def _read_gps_record(path: Path) -> tuple[float, float, float]:
    """Read the first GPS record from a relocated synthetic DataFlash log."""

    data = path.read_bytes()
    offset = 89
    self_type = data[offset + 2]
    assert self_type == 0x81
    time_us, raw_lat, raw_lng, raw_alt = struct.unpack_from("<Qiii", data, offset + 3)
    assert time_us == 1
    return raw_lat / 10_000_000.0, raw_lng / 10_000_000.0, raw_alt / 100.0


class ArduPilotLogRelocationTests(unittest.TestCase):
    """Behavior checks for ArduPilot DataFlash parsing and relocation."""

    def setUp(self) -> None:
        """Create a scratch directory for generated logs."""

        TEST_TEMP_ROOT.mkdir(exist_ok=True)

    def tearDown(self) -> None:
        """Remove generated logs so the repository stays clean after tests."""

        shutil.rmtree(TEST_TEMP_ROOT, ignore_errors=True)

    def test_parse_dataflash_format_offsets_scaled_fields(self) -> None:
        """FMT parsing should compute offsets and unit multipliers correctly."""

        fmt = parse_dataflash_format(0x81, 23, "GPS", "QLLe", "TimeUS,Lat,Lng,Alt")

        self.assertEqual(fmt.fields["TimeUS"].offset, 0)
        self.assertEqual(fmt.fields["Lat"].offset, 8)
        self.assertEqual(fmt.fields["Lng"].offset, 12)
        self.assertEqual(fmt.fields["Alt"].offset, 16)
        self.assertEqual(fmt.fields["Lat"].multiplier, 1.0e-7)
        self.assertEqual(fmt.fields["Alt"].multiplier, 0.01)

    def test_relocates_ardupilot_bin_to_default_origin(self) -> None:
        """A one-point GPS log should move exactly to ORIGIN and create only the log file."""

        original = _minimal_dataflash([_gps_record(12.0, 77.0, 100.0)])
        input_path = TEST_TEMP_ROOT / "input.BIN"
        output_path = TEST_TEMP_ROOT / "relocated.BIN"
        input_path.write_bytes(original)

        report = relocate_ardupilot_bin(input_path, output_path)
        lat, lon, alt = _read_gps_record(output_path)

        self.assertAlmostEqual(lat, ORIGIN["latitude_deg"], places=7)
        self.assertAlmostEqual(lon, ORIGIN["longitude_deg"], places=7)
        self.assertAlmostEqual(alt, ORIGIN["altitude_m"], places=2)
        self.assertEqual(report["source_anchor"]["topic"], "GPS")
        self.assertEqual(report["changed_fields"]["GPS"]["Lat"], 1)
        self.assertEqual(report["changed_fields"]["GPS"]["Lng"], 1)
        self.assertNotIn("report_path", report)
        self.assertFalse(output_path.with_suffix(output_path.suffix + ".relocation-report.json").exists())

    def test_default_output_path_appends_relocated_to_filename(self) -> None:
        """The CLI default output path should append '-relocated' before the suffix."""

        self.assertEqual(
            default_relocated_path(Path("flight.BIN")),
            Path("flight-relocated.BIN"),
        )

    def test_relocates_large_offset_without_safe_bounds_policy(self) -> None:
        """The script should not enforce geography policy beyond the configured ORIGIN."""

        original = _minimal_dataflash(
            [
                _gps_record(12.0, 77.0, 100.0),
                _gps_record(13.0, 78.0, 120.0),
            ]
        )
        input_path = TEST_TEMP_ROOT / "input.BIN"
        output_path = TEST_TEMP_ROOT / "relocated.BIN"
        input_path.write_bytes(original)

        report = relocate_ardupilot_bin(input_path, output_path)

        self.assertTrue(output_path.exists())
        self.assertEqual(report["changed_fields"]["GPS"]["Lat"], 2)
        self.assertIn("relocated_bounds", report)


if __name__ == "__main__":
    unittest.main()
