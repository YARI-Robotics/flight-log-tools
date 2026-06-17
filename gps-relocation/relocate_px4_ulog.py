#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Copyright (c) 2026 YARI Robotics
#
# Relocates absolute GPS/global-position data in PX4 ULog files to a synthetic
# origin before sharing or uploading logs. See README.md for usage and privacy notes.

from __future__ import annotations

import argparse
import json
import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any


# PX4 ULog files start with this fixed byte signature. Checking it lets us fail
# early if a user accidentally passes a non-ULog file.
ULOG_MAGIC = b"ULog\x01\x12\x35"

# The ULog header is 16 bytes; after that, the file is a sequence of typed
# messages. Each message tells us its payload size and message kind.
ULOG_HEADER_SIZE = 16

# Default destination used by the script. Users who want a different synthetic
# location can edit these values before running the script.
ORIGIN = {
    "name": "Default inland synthetic origin",
    "latitude_deg": 24.59949377106599,
    "longitude_deg": 72.70822904293095,
    "altitude_m": 1182.0,
    "elevation_source": "OpenTopoData srtm30m",
}

# Spherical Earth radius used for the local north/east offset approximation.
# This is accurate enough for normal drone-scale flight paths.
EARTH_RADIUS_M = 6_378_137.0

# ULog format definitions use PX4 type names. This table maps each type to the
# Python struct format character and byte size needed to read/write it.
PRIMITIVE_TYPES = {
    "int8_t": ("b", 1),
    "uint8_t": ("B", 1),
    "int16_t": ("h", 2),
    "uint16_t": ("H", 2),
    "int32_t": ("i", 4),
    "uint32_t": ("I", 4),
    "int64_t": ("q", 8),
    "uint64_t": ("Q", 8),
    "float": ("f", 4),
    "double": ("d", 8),
    "bool": ("?", 1),
    "char": ("c", 1),
}

# Field-name pairs that are treated as latitude/longitude coordinates.
GPS_PAIR_FIELDS = [
    ("latitude_deg", "longitude_deg"),
    ("lat", "lon"),
    ("latitude", "longitude"),
    ("ref_lat", "ref_lon"),
]

# Altitude fields that represent absolute altitude and should move with the
# synthetic origin while preserving the original vertical profile.
ALTITUDE_FIELDS = {
    "alt",
    "alt_ellipsoid",
    "altitude",
    "altitude_msl_m",
    "altitude_ellipsoid_m",
    "ref_alt",
}

# PX4 topics where absolute GPS/global coordinates commonly appear. Local
# position topics are included because they can carry absolute reference fields.
PX4_COORDINATE_TOPICS = {
    "vehicle_gps_position",
    "sensor_gps",
    "vehicle_global_position",
    "vehicle_global_position_groundtruth",
    "vehicle_local_position",
    "vehicle_local_position_groundtruth",
    "estimator_global_position",
    "estimator_local_position",
    "home_position",
    "navigator_mission_item",
    "transponder_report",
}


@dataclass(frozen=True)
class ULogMessage:
    """One raw ULog message plus its location inside the file."""

    offset: int
    size: int
    type_code: str
    payload_offset: int
    payload: bytes


@dataclass(frozen=True)
class FieldLayout:
    """A field's name, type, byte offset, and size inside a topic payload."""

    name: str
    type_name: str
    offset: int
    size: int
    array_length: int = 1


@dataclass(frozen=True)
class FormatLayout:
    """Decoded PX4 topic layout from a ULog format-definition message."""

    name: str
    fields: dict[str, FieldLayout]
    size: int
    unsupported_fields: tuple[str, ...] = ()


@dataclass(frozen=True)
class DataSubscription:
    """Maps a numeric ULog data message ID to a PX4 topic name."""

    msg_id: int
    multi_id: int
    topic: str


@dataclass(frozen=True)
class CoordinatePair:
    """The latitude and longitude fields that must be rewritten together."""

    topic: str
    lat_field: FieldLayout
    lon_field: FieldLayout


@dataclass(frozen=True)
class Anchor:
    """First valid coordinate in the log; all later points are moved relative to it."""

    latitude_deg: float
    longitude_deg: float
    altitude_m: float | None
    topic: str
    msg_id: int
    message_index: int


def _read_message(data: bytes, offset: int) -> ULogMessage | None:
    """Read one ULog message header and payload from a byte offset."""

    if offset + 3 > len(data):
        return None
    size, type_byte = struct.unpack_from("<HB", data, offset)
    payload_offset = offset + 3
    end = payload_offset + size
    if end > len(data):
        return None
    return ULogMessage(
        offset=offset,
        size=size,
        type_code=chr(type_byte),
        payload_offset=payload_offset,
        payload=data[payload_offset:end],
    )


def iter_ulog_messages(data: bytes) -> list[ULogMessage]:
    """Validate the ULog header and return every message in file order."""

    if len(data) < ULOG_HEADER_SIZE or data[: len(ULOG_MAGIC)] != ULOG_MAGIC:
        raise ValueError("Input is not a PX4 ULog file.")
    messages: list[ULogMessage] = []
    offset = ULOG_HEADER_SIZE
    while offset < len(data):
        message = _read_message(data, offset)
        if message is None:
            raise ValueError(f"Malformed ULog message at byte offset {offset}.")
        messages.append(message)
        offset = message.payload_offset + message.size
    return messages


def _parse_array_type(type_name: str, field_name: str) -> tuple[str, str, int]:
    """Normalize PX4 array syntax such as uint8_t[4] or field_name[4]."""

    array_length = 1
    if "[" in type_name and type_name.endswith("]"):
        base_type, count = type_name[:-1].split("[", 1)
        type_name = base_type
        array_length = int(count)
    if "[" in field_name and field_name.endswith("]"):
        base_name, count = field_name[:-1].split("[", 1)
        field_name = base_name
        array_length = int(count)
    return type_name, field_name, array_length


def parse_format_definition(definition: str) -> FormatLayout | None:
    """Parse a PX4 ULog F message into a topic layout with byte offsets."""

    if ":" not in definition:
        return None
    name, fields_text = definition.split(":", 1)
    name = name.strip()
    if not name:
        return None

    offset = 0
    fields: dict[str, FieldLayout] = {}
    unsupported: list[str] = []
    for raw_field in fields_text.split(";"):
        raw_field = raw_field.strip()
        if not raw_field:
            continue
        parts = raw_field.split()
        if len(parts) != 2:
            unsupported.append(raw_field)
            continue
        type_name, field_name, array_length = _parse_array_type(parts[0], parts[1])
        primitive = PRIMITIVE_TYPES.get(type_name)
        if primitive is None:
            unsupported.append(raw_field)
            continue
        _, primitive_size = primitive
        size = primitive_size * array_length
        fields[field_name] = FieldLayout(
            name=field_name,
            type_name=type_name,
            offset=offset,
            size=size,
            array_length=array_length,
        )
        offset += size

    return FormatLayout(name=name, fields=fields, size=offset, unsupported_fields=tuple(unsupported))


def parse_ulog_metadata(data: bytes) -> tuple[list[ULogMessage], dict[str, FormatLayout], dict[int, DataSubscription]]:
    """Collect topic layouts and subscription IDs needed to decode data records."""

    messages = iter_ulog_messages(data)
    formats: dict[str, FormatLayout] = {}
    subscriptions: dict[int, DataSubscription] = {}

    for message in messages:
        if message.type_code == "F":
            text = message.payload.decode("utf-8", errors="replace").rstrip("\x00")
            layout = parse_format_definition(text)
            if layout:
                formats[layout.name] = layout
        elif message.type_code == "A" and len(message.payload) >= 3:
            multi_id = message.payload[0]
            msg_id = struct.unpack_from("<H", message.payload, 1)[0]
            topic = message.payload[3:].decode("utf-8", errors="replace").rstrip("\x00")
            subscriptions[msg_id] = DataSubscription(msg_id=msg_id, multi_id=multi_id, topic=topic)

    return messages, formats, subscriptions


def _read_scalar(data: bytes | bytearray, offset: int, field: FieldLayout) -> float | int | bool | None:
    """Read one non-array primitive field from a payload."""

    primitive = PRIMITIVE_TYPES.get(field.type_name)
    if primitive is None or field.array_length != 1:
        return None
    fmt, _ = primitive
    return struct.unpack_from("<" + fmt, data, offset + field.offset)[0]


def _write_scalar(data: bytearray, offset: int, field: FieldLayout, value: float) -> None:
    """Write one non-array primitive field back into a payload."""

    primitive = PRIMITIVE_TYPES.get(field.type_name)
    if primitive is None or field.array_length != 1:
        return
    fmt, _ = primitive
    if field.type_name.startswith(("int", "uint")):
        value = round(value)
    struct.pack_into("<" + fmt, data, offset + field.offset, value)


def _valid_lat_lon(latitude: float | int | None, longitude: float | int | None) -> bool:
    """Reject missing, invalid, zero, or out-of-range coordinates."""

    if latitude is None or longitude is None:
        return False
    lat = float(latitude)
    lon = float(longitude)
    return (
        math.isfinite(lat)
        and math.isfinite(lon)
        and -90.0 <= lat <= 90.0
        and -180.0 <= lon <= 180.0
        and not (abs(lat) < 1e-12 and abs(lon) < 1e-12)
    )


def _scaled_coordinate(value: float | int | None, field: FieldLayout) -> float | None:
    """Convert PX4 coordinates to degrees.

    Some PX4 logs store degrees directly as floats/doubles; others store
    integer degrees scaled by 1e7.
    """

    if value is None:
        return None
    number = float(value)
    if field.type_name in {"int32_t", "uint32_t", "int64_t", "uint64_t"}:
        number /= 10_000_000.0
    return number


def _unscale_coordinate(value: float, field: FieldLayout) -> float:
    """Convert a degree value back to the field's on-disk representation."""

    if field.type_name in {"int32_t", "uint32_t", "int64_t", "uint64_t"}:
        return value * 10_000_000.0
    return value


def _scaled_altitude(value: float | int | None, field: FieldLayout) -> float | None:
    """Convert PX4 altitude fields to meters."""

    if value is None:
        return None
    number = float(value)
    if field.type_name in {"int32_t", "uint32_t", "int64_t", "uint64_t"}:
        number /= 1000.0
    return number


def _unscale_altitude(value: float, field: FieldLayout) -> float:
    """Convert meters back to the altitude field's on-disk representation."""

    if field.type_name in {"int32_t", "uint32_t", "int64_t", "uint64_t"}:
        return value * 1000.0
    return value


def _coordinate_pairs(topic: str, layout: FormatLayout) -> list[CoordinatePair]:
    """Return coordinate field pairs for topics that may expose global position."""

    if topic not in PX4_COORDINATE_TOPICS:
        return []
    pairs: list[CoordinatePair] = []
    for lat_name, lon_name in GPS_PAIR_FIELDS:
        lat_field = layout.fields.get(lat_name)
        lon_field = layout.fields.get(lon_name)
        if lat_field and lon_field:
            pairs.append(CoordinatePair(topic=topic, lat_field=lat_field, lon_field=lon_field))
    return pairs


def _topic_altitude_fields(topic: str, layout: FormatLayout) -> list[FieldLayout]:
    """Return altitude fields for coordinate-bearing topics."""

    if topic not in PX4_COORDINATE_TOPICS:
        return []
    return [field for name, field in layout.fields.items() if name in ALTITUDE_FIELDS]


def _record_payload_offset(message: ULogMessage) -> int | None:
    """Return the payload offset after the data record's two-byte message ID."""

    if message.type_code != "D" or message.size < 2:
        return None
    return message.payload_offset + 2


def _record_msg_id(message: ULogMessage) -> int | None:
    """Return the subscription ID for a ULog data record."""

    if message.type_code != "D" or message.size < 2:
        return None
    return struct.unpack_from("<H", message.payload, 0)[0]


def _first_anchor(
    messages: list[ULogMessage],
    formats: dict[str, FormatLayout],
    subscriptions: dict[int, DataSubscription],
    data: bytes,
) -> Anchor | None:
    """Find the first valid global coordinate to use as the relocation anchor."""

    for index, message in enumerate(messages):
        msg_id = _record_msg_id(message)
        payload_offset = _record_payload_offset(message)
        if msg_id is None or payload_offset is None:
            continue
        subscription = subscriptions.get(msg_id)
        if subscription is None:
            continue
        layout = formats.get(subscription.topic)
        if layout is None:
            continue
        for pair in _coordinate_pairs(subscription.topic, layout):
            lat_value = _scaled_coordinate(_read_scalar(data, payload_offset, pair.lat_field), pair.lat_field)
            lon_value = _scaled_coordinate(_read_scalar(data, payload_offset, pair.lon_field), pair.lon_field)
            if not _valid_lat_lon(lat_value, lon_value):
                continue
            altitude = None
            for alt_field in _topic_altitude_fields(subscription.topic, layout):
                altitude = _scaled_altitude(_read_scalar(data, payload_offset, alt_field), alt_field)
                if altitude is not None and math.isfinite(altitude):
                    break
            return Anchor(
                latitude_deg=float(lat_value),
                longitude_deg=float(lon_value),
                altitude_m=altitude,
                topic=subscription.topic,
                msg_id=msg_id,
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
    dest_lon = ORIGIN["longitude_deg"] + math.degrees(
        d_east / (EARTH_RADIUS_M * max(math.cos(dest_lat_rad), 1e-9))
    )
    return dest_lat, dest_lon


def default_relocated_path(input_path: Path) -> Path:
    """Return the public CLI default: original filename plus '-relocated' before the suffix."""
    return input_path.with_name(f"{input_path.stem}-relocated{input_path.suffix}")


def relocate_px4_ulog(input_path: Path, output_path: Path, report_path: Path | None = None) -> dict[str, Any]:
    """Relocate all known absolute GPS/global coordinate fields in a PX4 ULog."""

    source = input_path.read_bytes()
    messages, formats, subscriptions = parse_ulog_metadata(source)

    # Use the first real GPS/global coordinate as the source anchor. Every later
    # coordinate is moved by its north/east offset from this anchor.
    anchor = _first_anchor(messages, formats, subscriptions, source)
    if anchor is None:
        raise ValueError("No valid PX4 latitude/longitude coordinate was found in the ULog.")

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

    # ULog data records point to a subscribed topic ID. We only touch topics and
    # fields that are known to carry absolute global coordinates.
    for message in messages:
        msg_id = _record_msg_id(message)
        payload_offset = _record_payload_offset(message)
        if msg_id is None or payload_offset is None:
            continue
        subscription = subscriptions.get(msg_id)
        if subscription is None:
            continue
        layout = formats.get(subscription.topic)
        if layout is None:
            continue
        pairs = _coordinate_pairs(subscription.topic, layout)
        if not pairs:
            continue
        if layout.unsupported_fields:
            skipped_topics[subscription.topic] = "format_contains_unsupported_fields"
            continue

        pair_changed = False
        for pair in pairs:
            lat_value = _scaled_coordinate(
                _read_scalar(relocated, payload_offset, pair.lat_field), pair.lat_field
            )
            lon_value = _scaled_coordinate(
                _read_scalar(relocated, payload_offset, pair.lon_field), pair.lon_field
            )
            if not _valid_lat_lon(lat_value, lon_value):
                continue
            new_lat, new_lon = _relocate_lat_lon(float(lat_value), float(lon_value), anchor)
            relocated_bounds["min_latitude_deg"] = min(relocated_bounds["min_latitude_deg"], new_lat)
            relocated_bounds["max_latitude_deg"] = max(relocated_bounds["max_latitude_deg"], new_lat)
            relocated_bounds["min_longitude_deg"] = min(relocated_bounds["min_longitude_deg"], new_lon)
            relocated_bounds["max_longitude_deg"] = max(relocated_bounds["max_longitude_deg"], new_lon)
            _write_scalar(relocated, payload_offset, pair.lat_field, _unscale_coordinate(new_lat, pair.lat_field))
            _write_scalar(relocated, payload_offset, pair.lon_field, _unscale_coordinate(new_lon, pair.lon_field))
            increment(subscription.topic, pair.lat_field.name)
            increment(subscription.topic, pair.lon_field.name)
            pair_changed = True

        if not pair_changed:
            continue
        for alt_field in _topic_altitude_fields(subscription.topic, layout):
            value = _scaled_altitude(_read_scalar(relocated, payload_offset, alt_field), alt_field)
            if value is None or not math.isfinite(value):
                continue
            _write_scalar(relocated, payload_offset, alt_field, _unscale_altitude(value + alt_delta_m, alt_field))
            increment(subscription.topic, alt_field.name)

    if not changed_fields:
        raise ValueError("No PX4 coordinate fields were relocated.")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(relocated)

    bounds = None
    if math.isfinite(relocated_bounds["min_latitude_deg"]):
        bounds = {key: round(value, 7) for key, value in relocated_bounds.items()}

    report = {
        "format": "px4_ulog",
        "mode": "synthetic_relocation",
        "input_path": str(input_path),
        "output_path": str(output_path),
        "destination_origin": ORIGIN,
        "source_anchor": {
            "latitude_deg": round(anchor.latitude_deg, 7),
            "longitude_deg": round(anchor.longitude_deg, 7),
            "altitude_m": round(anchor.altitude_m, 3) if anchor.altitude_m is not None else None,
            "topic": anchor.topic,
            "msg_id": anchor.msg_id,
            "message_index": anchor.message_index,
        },
        "altitude_delta_m": round(alt_delta_m, 3),
        "relocated_bounds": bounds,
        "changed_fields": changed_fields,
        "skipped_topics": skipped_topics,
        "topic_count": len(changed_fields),
    }

    if report_path is not None:
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
        report["report_path"] = str(report_path)
    return report


def build_arg_parser() -> argparse.ArgumentParser:
    """Build the command-line interface for the standalone script."""

    parser = argparse.ArgumentParser(
        description="Relocate absolute GPS/global coordinates in a PX4 ULog to the default synthetic origin."
    )
    parser.add_argument("input", type=Path, help="Input PX4 .ulg file.")
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
    report = relocate_px4_ulog(args.input, output_path)
    print(f"Relocated PX4 ULog written to: {report['output_path']}")
    print(f"Changed topics: {', '.join(sorted(report['changed_fields']))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
