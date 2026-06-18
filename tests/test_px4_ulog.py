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
TEST_TEMP_ROOT = REPO_ROOT / "pytest-cache-files-px4-relocation"

from relocate_px4_ulog import (
    ORIGIN,
    ULOG_MAGIC,
    default_relocated_path,
    parse_format_definition,
    relocate_px4_ulog,
)


def _ulog_message(type_code: str, payload: bytes) -> bytes:
    """Build one raw ULog message: two-byte size, one-byte type, then payload."""

    return struct.pack("<HB", len(payload), ord(type_code)) + payload


def _minimal_ulog(data_messages: list[bytes]) -> bytes:
    """Build the smallest ULog needed to exercise GPS relocation.

    It contains a valid header, one format definition for
    vehicle_gps_position, one subscription mapping, and caller-supplied data
    records.
    """

    header = ULOG_MAGIC + b"\x01" + struct.pack("<Q", 0)
    fmt = _ulog_message(
        "F",
        b"vehicle_gps_position:uint64_t timestamp;double latitude_deg;"
        b"double longitude_deg;float altitude_msl_m;uint8_t[3] _padding0;",
    )
    add = _ulog_message("A", struct.pack("<BH", 0, 42) + b"vehicle_gps_position")
    return header + fmt + add + b"".join(data_messages)


def _gps_data(latitude_deg: float, longitude_deg: float, altitude_m: float) -> bytes:
    """Create one vehicle_gps_position data record."""

    payload = struct.pack("<H", 42)
    payload += struct.pack("<Qddf3B", 1, latitude_deg, longitude_deg, altitude_m, 0, 0, 0)
    return _ulog_message("D", payload)


def _read_gps_record(path: Path) -> tuple[float, float, float]:
    """Read the first GPS record from a relocated synthetic ULog."""

    data = path.read_bytes()
    offset = 16
    while offset < len(data):
        size, type_byte = struct.unpack_from("<HB", data, offset)
        payload_offset = offset + 3
        payload = data[payload_offset : payload_offset + size]
        if chr(type_byte) == "D":
            msg_id = struct.unpack_from("<H", payload, 0)[0]
            if msg_id == 42:
                timestamp, lat, lon, alt = struct.unpack_from("<Qddf", payload, 2)
                assert timestamp == 1
                return lat, lon, alt
        offset = payload_offset + size
    raise AssertionError("GPS data record not found")


class Px4LogRelocationTests(unittest.TestCase):
    """Behavior checks for PX4 ULog parsing and relocation."""

    def setUp(self) -> None:
        """Create a scratch directory for generated logs."""

        TEST_TEMP_ROOT.mkdir(exist_ok=True)

    def tearDown(self) -> None:
        """Remove generated logs so the repository stays clean after tests."""

        shutil.rmtree(TEST_TEMP_ROOT, ignore_errors=True)

    def test_parse_format_definition_handles_type_arrays(self) -> None:
        """PX4 format parsing should account for array fields when computing offsets."""

        layout = parse_format_definition("sensor_gps:uint64_t timestamp;int32_t lat;int32_t lon;uint8_t[4] _padding0;")

        self.assertIsNotNone(layout)
        assert layout is not None
        self.assertEqual(layout.fields["timestamp"].offset, 0)
        self.assertEqual(layout.fields["lat"].offset, 8)
        self.assertEqual(layout.fields["lon"].offset, 12)
        self.assertEqual(layout.fields["_padding0"].size, 4)
        self.assertEqual(layout.size, 20)

    def test_relocates_px4_ulog_to_default_origin(self) -> None:
        """A one-point GPS log should move exactly to ORIGIN and create only the log file."""

        source_lat = 12.0000000
        source_lon = 77.0000000
        source_alt = 100.0
        original = _minimal_ulog([_gps_data(source_lat, source_lon, source_alt)])

        input_path = TEST_TEMP_ROOT / "input.ulg"
        output_path = TEST_TEMP_ROOT / "relocated.ulg"
        input_path.write_bytes(original)

        report = relocate_px4_ulog(input_path, output_path)
        lat, lon, alt = _read_gps_record(output_path)

        self.assertAlmostEqual(lat, ORIGIN["latitude_deg"], places=7)
        self.assertAlmostEqual(lon, ORIGIN["longitude_deg"], places=7)
        self.assertAlmostEqual(alt, ORIGIN["altitude_m"], places=3)
        self.assertEqual(report["source_anchor"]["topic"], "vehicle_gps_position")
        self.assertEqual(report["changed_fields"]["vehicle_gps_position"]["latitude_deg"], 1)
        self.assertNotIn("report_path", report)
        self.assertFalse(output_path.with_suffix(output_path.suffix + ".relocation-report.json").exists())

    def test_default_output_path_appends_relocated_to_filename(self) -> None:
        """The CLI default output path should append '-relocated' before the suffix."""

        self.assertEqual(
            default_relocated_path(Path("flight.ulg")),
            Path("flight-relocated.ulg"),
        )

    def test_relocates_large_offset_without_safe_bounds_policy(self) -> None:
        """The script should not enforce geography policy beyond the configured ORIGIN."""

        original = _minimal_ulog(
            [
                _gps_data(12.0, 77.0, 100.0),
                _gps_data(13.0, 78.0, 120.0),
            ]
        )

        input_path = TEST_TEMP_ROOT / "input.ulg"
        output_path = TEST_TEMP_ROOT / "relocated.ulg"
        input_path.write_bytes(original)

        report = relocate_px4_ulog(input_path, output_path)

        self.assertTrue(output_path.exists())
        self.assertEqual(report["changed_fields"]["vehicle_gps_position"]["latitude_deg"], 2)
        self.assertIn("relocated_bounds", report)


if __name__ == "__main__":
    unittest.main()
