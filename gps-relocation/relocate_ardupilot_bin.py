#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Copyright (c) 2026 YARI Robotics
#
# Relocates absolute GPS/global-position data in ArduPilot DataFlash logs to a
# synthetic origin before sharing or uploading logs. See README.md for usage and privacy notes.

from __future__ import annotations

import argparse
import json
import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any

# ArduPilot DataFlash binary messages start with this two-byte marker. The
# parser uses it to find message boundaries and to reject non-DataFlash files.
DATAFLASH_MAGIC = b"\xa3\x95"

# Spherical Earth radius used for the local north/east offset approximation.
# This is accurate enough for normal drone-scale flight paths.
EARTH_RADIUS_M = 6_378_137.0

# Default destination used by the script. Users who want a different synthetic
# location can edit these values before running the script.
ORIGIN = {
    "name": "Default inland synthetic origin",
    "latitude_deg": 24.59949377106599,
    "longitude_deg": 72.70822904293095,
    "altitude_m": 1182.0,
    "elevation_source": "OpenTopoData srtm30m",
}

# ArduPilot FMT records describe message fields using compact one-letter type
# codes. This table maps each DataFlash type code to:
#   1. the Python struct format needed to read/write the bytes
#   2. an optional multiplier that converts raw stored values to human units
# For example, "L" is a latitude/longitude int scaled by 1e-7 degrees.
FORMAT_TO_STRUCT = {
    "a": ("64s", None),
    "b": ("b", None),
    "B": ("B", None),
    "g": ("e", None),
    "h": ("h", None),
    "H": ("H", None),
    "i": ("i", None),
    "I": ("I", None),
    "f": ("f", None),
    "n": ("4s", None),
    "N": ("16s", None),
    "Z": ("64s", None),
    "c": ("h", 0.01),
    "C": ("H", 0.01),
    "e": ("i", 0.01),
    "E": ("I", 0.01),
    "L": ("i", 1.0e-7),
    "d": ("d", None),
    "M": ("b", None),
    "q": ("q", None),
    "Q": ("Q", None),
}

# ArduPilot message names where absolute GPS/global coordinates can appear.
# These include GPS, position estimates, home/origin records, missions, terrain,
# fences/rally points, camera triggers, ADS-B/AIS, and simulation messages.
COORDINATE_TOPICS = {
    "ADSB",
    "AHR2",
    "AIS1",
    "AIS4",
    "CAM",
    "CMD",
    "DSTL",
    "EAHR",
    "FNCE",
    "GPS",
    "GPS2",
    "MISE",
    "OAVG",
    "ORGN",
    "POS",
    "RALY",
    "RBCH",
    "RGPJ",
    "RSLL",
    "RSO2",
    "RSO3",
    "SIM",
    "TERR",
    "TRIG",
}

# Field-name pairs that are treated as latitude/longitude coordinates.
COORDINATE_PAIRS = [
    ("Lat", "Lng"),
    ("Lat", "Lon"),
    ("lat", "lon"),
    ("OLat", "OLng"),
    ("GLat", "GLng"),
]

# Altitude fields that represent absolute altitude and should move with the
# synthetic origin while preserving the original vertical profile.
ALTITUDE_FIELDS = {
    "Alt",
    "GAlt",
    "OAlt",
    "RelAlt",
}


@dataclass(frozen=True)
class FieldLayout:
    """A field's name, DataFlash type, byte offset, size, and unit multiplier."""

    name: str
    format_char: str
    struct_format: str
    offset: int
    size: int
    multiplier: float | None = None


@dataclass(frozen=True)
class DataFlashFormat:
    """Decoded layout for one ArduPilot message type from an FMT record."""

    type_id: int
    name: str
    length: int
    format: str
    columns: tuple[str, ...]
    fields: dict[str, FieldLayout]
    unsupported: tuple[str, ...] = ()


@dataclass(frozen=True)
class DataFlashMessage:
    """One DataFlash message plus its file offset and decoded format."""

    offset: int
    type_id: int
    format: DataFlashFormat
    payload_offset: int


@dataclass(frozen=True)
class CoordinatePair:
    """The latitude and longitude fields that must be rewritten together."""

    lat_field: FieldLayout
    lon_field: FieldLayout


@dataclass(frozen=True)
class Anchor:
    """First valid coordinate in the log; all later points are moved relative to it."""

    latitude_deg: float
    longitude_deg: float
    altitude_m: float | None
    topic: str
    type_id: int
    message_index: int


def _decode_c_string(value: bytes) -> str:
    """Decode null-terminated ASCII strings stored inside FMT records."""

    return value.split(b"\0", 1)[0].decode("ascii", errors="replace").strip()


def parse_dataflash_format(
    type_id: int,
    length: int,
    name: str,
    format_text: str,
    columns_text: str,
) -> DataFlashFormat:
    """Parse an ArduPilot FMT definition into field layouts and byte offsets."""

    columns = tuple(part.strip() for part in columns_text.split(",") if part.strip())
    fields: dict[str, FieldLayout] = {}
    unsupported: list[str] = []
    offset = 0
    for index, format_char in enumerate(format_text):
        mapping = FORMAT_TO_STRUCT.get(format_char)
        field_name = columns[index] if index < len(columns) else f"_field_{index}"
        if mapping is None:
            unsupported.append(field_name)
            continue
        struct_format, multiplier = mapping
        size = struct.calcsize("<" + struct_format)
        fields[field_name] = FieldLayout(
            name=field_name,
            format_char=format_char,
            struct_format=struct_format,
            offset=offset,
            size=size,
            multiplier=multiplier,
        )
        offset += size
    return DataFlashFormat(
        type_id=type_id,
        name=name,
        length=length,
        format=format_text,
        columns=columns,
        fields=fields,
        unsupported=tuple(unsupported),
    )


def _default_fmt_format() -> DataFlashFormat:
    """Return the built-in layout for FMT messages themselves."""

    return parse_dataflash_format(
        0x80,
        89,
        "FMT",
        "BBnNZ",
        "Type,Length,Name,Format,Columns",
    )


def _read_field(data: bytes | bytearray, payload_offset: int, field: FieldLayout) -> Any:
    """Read one field from a DataFlash message payload."""

    return struct.unpack_from("<" + field.struct_format, data, payload_offset + field.offset)[0]


def _write_field(data: bytearray, payload_offset: int, field: FieldLayout, decoded_value: float) -> None:
    """Write a decoded human-unit value back using the field's storage format."""

    raw_value = decoded_value
    if field.multiplier is not None:
        raw_value = decoded_value / field.multiplier
    if field.struct_format[-1] not in {"f", "d", "e"}:
        raw_value = round(raw_value)
    struct.pack_into("<" + field.struct_format, data, payload_offset + field.offset, raw_value)


def _decoded_number(data: bytes | bytearray, payload_offset: int, field: FieldLayout) -> float | None:
    """Read a numeric field and apply its multiplier, if any."""

    value = _read_field(data, payload_offset, field)
    if isinstance(value, (bytes, bytearray)):
        return None
    number = float(value)
    if field.multiplier is not None:
        number *= field.multiplier
    return number


def _valid_lat_lon(latitude: float | None, longitude: float | None) -> bool:
    """Reject missing, invalid, zero, or out-of-range coordinates."""

    if latitude is None or longitude is None:
        return False
    return (
        math.isfinite(latitude)
        and math.isfinite(longitude)
        and -90.0 <= latitude <= 90.0
        and -180.0 <= longitude <= 180.0
        and not (abs(latitude) < 1e-12 and abs(longitude) < 1e-12)
    )


def _coordinate_pairs(fmt: DataFlashFormat) -> list[CoordinatePair]:
    """Return coordinate field pairs for message types that may expose location."""

    if fmt.name not in COORDINATE_TOPICS:
        return []
    pairs: list[CoordinatePair] = []
    for lat_name, lon_name in COORDINATE_PAIRS:
        lat_field = fmt.fields.get(lat_name)
        lon_field = fmt.fields.get(lon_name)
        if lat_field and lon_field:
            pairs.append(CoordinatePair(lat_field=lat_field, lon_field=lon_field))
    return pairs


def _altitude_fields(fmt: DataFlashFormat) -> list[FieldLayout]:
    """Return altitude fields for coordinate-bearing DataFlash message types."""

    if fmt.name not in COORDINATE_TOPICS:
        return []
    return [field for name, field in fmt.fields.items() if name in ALTITUDE_FIELDS]


def iter_dataflash_messages(data: bytes) -> tuple[list[DataFlashMessage], dict[int, DataFlashFormat]]:
    """Parse messages in file order and learn formats from FMT records."""

    if len(data) < 3 or data[:2] != DATAFLASH_MAGIC:
        raise ValueError("Input is not an ArduPilot DataFlash binary log.")

    formats: dict[int, DataFlashFormat] = {0x80: _default_fmt_format()}
    messages: list[DataFlashMessage] = []
    offset = 0
    while offset + 3 <= len(data):
        if data[offset : offset + 2] != DATAFLASH_MAGIC:
            next_offset = data.find(DATAFLASH_MAGIC, offset + 1)
            if next_offset < 0:
                break
            offset = next_offset
            continue

        type_id = data[offset + 2]
        fmt = formats.get(type_id)
        if fmt is None:
            offset += 3
            continue
        end = offset + fmt.length
        if end > len(data):
            break
        payload_offset = offset + 3
        message = DataFlashMessage(
            offset=offset,
            type_id=type_id,
            format=fmt,
            payload_offset=payload_offset,
        )
        messages.append(message)

        if fmt.name == "FMT":
            fmt_type = int(_read_field(data, payload_offset, fmt.fields["Type"]))
            fmt_length = int(_read_field(data, payload_offset, fmt.fields["Length"]))
            fmt_name = _decode_c_string(_read_field(data, payload_offset, fmt.fields["Name"]))
            fmt_format = _decode_c_string(_read_field(data, payload_offset, fmt.fields["Format"]))
            fmt_columns = _decode_c_string(_read_field(data, payload_offset, fmt.fields["Columns"]))
            if fmt_name and fmt_format:
                formats[fmt_type] = parse_dataflash_format(
                    fmt_type,
                    fmt_length,
                    fmt_name,
                    fmt_format,
                    fmt_columns,
                )
        offset = end
    return messages, formats


def _first_anchor(messages: list[DataFlashMessage], data: bytes) -> Anchor | None:
    """Find the first valid global coordinate to use as the relocation anchor."""

    for index, message in enumerate(messages):
        fmt = message.format
        for pair in _coordinate_pairs(fmt):
            lat = _decoded_number(data, message.payload_offset, pair.lat_field)
            lon = _decoded_number(data, message.payload_offset, pair.lon_field)
            if not _valid_lat_lon(lat, lon):
                continue
            altitude = None
            for field in _altitude_fields(fmt):
                altitude = _decoded_number(data, message.payload_offset, field)
                if altitude is not None and math.isfinite(altitude):
                    break
            return Anchor(
                latitude_deg=float(lat),
                longitude_deg=float(lon),
                altitude_m=altitude,
                topic=fmt.name,
                type_id=message.type_id,
                message_index=index,
            )
    return None


def _relocate_lat_lon(latitude_deg: float, longitude_deg: float, anchor: Anchor) -> tuple[float, float]:
    """Move a coordinate by preserving its north/east offset from the anchor."""

    anchor_lat_rad = math.radians(anchor.latitude_deg)
    d_north = math.radians(latitude_deg - anchor.latitude_deg) * EARTH_RADIUS_M
    d_east = math.radians(longitude_deg - anchor.longitude_deg) * EARTH_RADIUS_M * math.cos(anchor_lat_rad)
    dest_lat = ORIGIN["latitude_deg"] + math.degrees(d_north / EARTH_RADIUS_M)
    dest_lat_rad = math.radians(dest_lat)
    dest_lon = ORIGIN["longitude_deg"] + math.degrees(d_east / (EARTH_RADIUS_M * max(math.cos(dest_lat_rad), 1e-9)))
    return dest_lat, dest_lon


def default_relocated_path(input_path: Path) -> Path:
    """Return the public CLI default: original filename plus '-relocated' before the suffix."""
    return input_path.with_name(f"{input_path.stem}-relocated{input_path.suffix}")


def relocate_ardupilot_bin(
    input_path: Path,
    output_path: Path,
    report_path: Path | None = None,
) -> dict[str, Any]:
    """Relocate all known absolute GPS/global coordinate fields in a DataFlash log."""

    source = input_path.read_bytes()
    messages, formats = iter_dataflash_messages(source)

    # Use the first real GPS/global coordinate as the source anchor. Every later
    # coordinate is moved by its north/east offset from this anchor.
    anchor = _first_anchor(messages, source)
    if anchor is None:
        raise ValueError("No valid ArduPilot latitude/longitude coordinate was found in the DataFlash log.")

    relocated = bytearray(source)
    # Shift absolute altitudes so the first anchor sits on the destination ground
    # elevation, while preserving the original climb/descent shape.
    alt_delta_m = (
        float(ORIGIN["altitude_m"]) - anchor.altitude_m
        if anchor.altitude_m is not None and math.isfinite(anchor.altitude_m)
        else 0.0
    )
    changed_fields: dict[str, dict[str, int]] = {}
    skipped_topics: dict[str, str] = {}
    relocated_bounds = {
        "min_latitude_deg": math.inf,
        "max_latitude_deg": -math.inf,
        "min_longitude_deg": math.inf,
        "max_longitude_deg": -math.inf,
    }

    def increment(topic: str, field: str) -> None:
        changed_fields.setdefault(topic, {})[field] = changed_fields.setdefault(topic, {}).get(field, 0) + 1

    # DataFlash logs define their own FMT records. We only touch messages whose
    # FMT name and field names are known to carry absolute global coordinates.
    for message in messages:
        fmt = message.format
        pairs = _coordinate_pairs(fmt)
        if not pairs:
            continue
        if fmt.unsupported:
            skipped_topics[fmt.name] = "format_contains_unsupported_fields"
            continue
        pair_changed = False
        for pair in pairs:
            lat = _decoded_number(relocated, message.payload_offset, pair.lat_field)
            lon = _decoded_number(relocated, message.payload_offset, pair.lon_field)
            if not _valid_lat_lon(lat, lon):
                continue
            new_lat, new_lon = _relocate_lat_lon(float(lat), float(lon), anchor)
            relocated_bounds["min_latitude_deg"] = min(relocated_bounds["min_latitude_deg"], new_lat)
            relocated_bounds["max_latitude_deg"] = max(relocated_bounds["max_latitude_deg"], new_lat)
            relocated_bounds["min_longitude_deg"] = min(relocated_bounds["min_longitude_deg"], new_lon)
            relocated_bounds["max_longitude_deg"] = max(relocated_bounds["max_longitude_deg"], new_lon)
            _write_field(relocated, message.payload_offset, pair.lat_field, new_lat)
            _write_field(relocated, message.payload_offset, pair.lon_field, new_lon)
            increment(fmt.name, pair.lat_field.name)
            increment(fmt.name, pair.lon_field.name)
            pair_changed = True

        if not pair_changed:
            continue
        for field in _altitude_fields(fmt):
            value = _decoded_number(relocated, message.payload_offset, field)
            if value is None or not math.isfinite(value):
                continue
            _write_field(relocated, message.payload_offset, field, value + alt_delta_m)
            increment(fmt.name, field.name)

    if not changed_fields:
        raise ValueError("No ArduPilot coordinate fields were relocated.")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(relocated)
    bounds = None
    if math.isfinite(relocated_bounds["min_latitude_deg"]):
        bounds = {key: round(value, 7) for key, value in relocated_bounds.items()}

    report = {
        "format": "ardupilot_dataflash",
        "mode": "synthetic_relocation",
        "input_path": str(input_path),
        "output_path": str(output_path),
        "destination_origin": ORIGIN,
        "source_anchor": {
            "latitude_deg": round(anchor.latitude_deg, 7),
            "longitude_deg": round(anchor.longitude_deg, 7),
            "altitude_m": round(anchor.altitude_m, 3) if anchor.altitude_m is not None else None,
            "topic": anchor.topic,
            "type_id": anchor.type_id,
            "message_index": anchor.message_index,
        },
        "altitude_delta_m": round(alt_delta_m, 3),
        "relocated_bounds": bounds,
        "changed_fields": changed_fields,
        "skipped_topics": skipped_topics,
        "defined_coordinate_topics": sorted(fmt.name for fmt in formats.values() if _coordinate_pairs(fmt)),
        "topic_count": len(changed_fields),
    }

    if report_path is not None:
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
        report["report_path"] = str(report_path)
    return report


def build_arg_parser() -> argparse.ArgumentParser:
    """Build the command-line interface for the standalone script."""

    parser = argparse.ArgumentParser(
        description=(
            "Relocate absolute GPS/global coordinates in an ArduPilot DataFlash log to the default synthetic origin."
        )
    )
    parser.add_argument("input", type=Path, help="Input ArduPilot .bin/.BIN DataFlash file.")
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=None,
        help="Optional output path. Defaults to <input-name>-relocated.<suffix>.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run relocation from the command line and print a compact summary."""

    args = build_arg_parser().parse_args(argv)
    output_path = args.output or default_relocated_path(args.input)
    report = relocate_ardupilot_bin(args.input, output_path)
    print(f"Relocated ArduPilot DataFlash log written to: {report['output_path']}")
    print(f"Changed topics: {', '.join(sorted(report['changed_fields']))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
