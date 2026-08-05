#!/usr/bin/env python3
"""SR-75 HIL-F23-D1: mission-derived release heading helper.

Computes the initial great-circle bearing from a preserved release
position to the first real NAV_WAYPOINT in a QGC WPL 110 mission file, so
HIL bench tooling (sr75_jsbsim_pixhawk_hil_bridge.py) can use ONE
mission-derived heading source instead of independently hardcoded/manual
yaw values (see HIL-F23-D audit).

This module is pure computation: it never opens a serial port, never
sends MAVLink, never touches ArduPlane/RATO/mission/aero/TECS logic, and
never modifies a validated JSBSim IC file in place (generate_runtime_ic_copy
always writes a NEW file).

Run directly for a no-hardware, read-only demonstration:

    python3 Tools/autotest/sr75_hil_layer2/bridge/sr75_mission_heading.py \\
      --mission-file Tools/autotest/sr75_hil_layer2/closed_loop/SR75_F22_EXTENDED_ALTITUDE_PROFILE_V2.waypoints \\
      --release-lat 32.5378147 --release-lon 74.3661871 \\
      --release-groundspeed-mps 43.85
"""
import argparse
import math
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_DELAY = 93
MAV_CMD_DO_CHANGE_SPEED = 178


class MissionHeadingError(Exception):
    """Raised when a mission file cannot be parsed or has no usable waypoint."""


@dataclass(frozen=True)
class MissionItem:
    seq: int
    current: int
    frame: int
    command: int
    p1: float
    p2: float
    p3: float
    p4: float
    lat: float
    lon: float
    alt: float
    autocontinue: int


@dataclass(frozen=True)
class MissionHeadingResult:
    item: MissionItem
    release_lat: float
    release_lon: float
    bearing_0_360_deg: float
    bearing_signed_deg: float
    distance_m: float
    source: str

    def vn_ve(self, groundspeed_mps: float):
        """North/east velocity components for the computed bearing.

        Uses groundspeed, not airspeed -- callers must not assume TAS
        equals groundspeed when wind is present (see release_groundspeed
        argument in the bridge and __main__ below).
        """
        rad = math.radians(self.bearing_0_360_deg)
        return groundspeed_mps * math.cos(rad), groundspeed_mps * math.sin(rad)


def parse_qgc_wpl(path) -> List[MissionItem]:
    """Parse a QGC WPL 110 mission file into MissionItem rows."""
    path = Path(path)
    items = []
    with path.open() as f:
        header = f.readline().strip()
        if not header.startswith("QGC WPL"):
            raise MissionHeadingError(f"{path}: not a QGC WPL mission file (got header {header!r})")
        for line_no, raw_line in enumerate(f, start=2):
            line = raw_line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != 12:
                raise MissionHeadingError(f"{path}:{line_no}: expected 12 fields, got {len(parts)}: {raw_line!r}")
            seq, current, frame, command = (int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3]))
            p1, p2, p3, p4, lat, lon, alt = (float(x) for x in parts[4:11])
            autocontinue = int(parts[11])
            items.append(MissionItem(seq, current, frame, command, p1, p2, p3, p4, lat, lon, alt, autocontinue))
    return items


def select_first_nav_waypoint(items: List[MissionItem]) -> MissionItem:
    """Skip seq0/home and any non-NAV_WAYPOINT item (NAV_TAKEOFF,
    DO_CHANGE_SPEED, NAV_DELAY, or anything else); return the first
    MAV_CMD_NAV_WAYPOINT item.
    """
    for item in items:
        if item.seq == 0:
            continue  # home item, always skipped regardless of its listed command
        if item.command != MAV_CMD_NAV_WAYPOINT:
            continue  # e.g. NAV_TAKEOFF(22), DO_CHANGE_SPEED(178), NAV_DELAY(93), etc.
        return item
    raise MissionHeadingError("no NAV_WAYPOINT(16) item found after seq0")


def normalize_0_360(deg: float) -> float:
    return deg % 360.0


def normalize_signed_180(deg: float) -> float:
    return ((deg + 180.0) % 360.0) - 180.0


def wrap_angle_error_deg(target_deg: float, actual_deg: float) -> float:
    """Signed angular error (target - actual), wrapped to -180..180."""
    return normalize_signed_180(target_deg - actual_deg)


# HIL-F23-F2A: MAVLink GPS_INPUT.yaw wire encoding (message id 232, a
# MAVLink2-only extension field -- see modules/mavlink/message_definitions
# /v1.0/common.xml). uint16_t centidegrees, valid range 1..36000, where
# 36000 means true north and 0 is reserved by the spec to mean "not
# available" -- 0 is NOT a valid encoding of a real north heading.
GPS_INPUT_YAW_UNAVAILABLE_CDEG = 0
GPS_INPUT_YAW_NORTH_CDEG = 36000


def encode_gps_input_yaw_cdeg(bearing_deg):
    """Encode a heading (any range/sign, degrees) as a GPS_INPUT.yaw wire
    value. Pass None (or a non-finite value) to explicitly encode "not
    available" (0). A real heading that rounds to 0 after quantization
    (e.g. bearing_deg=0.0 or 360.0) is remapped to 36000, since 0 is
    reserved for "not available" by the MAVLink spec, not for north.
    """
    if bearing_deg is None or not math.isfinite(bearing_deg):
        return GPS_INPUT_YAW_UNAVAILABLE_CDEG
    cdeg = round(normalize_0_360(bearing_deg) * 100.0) % 36000
    if cdeg == 0:
        cdeg = GPS_INPUT_YAW_NORTH_CDEG
    return int(cdeg)


def decode_gps_input_yaw_cdeg(yaw_cdeg):
    """Inverse of encode_gps_input_yaw_cdeg. Returns None for 0 ("not
    available"), else a heading in degrees, normalized 0..360 (36000/north
    decodes to 0.0).
    """
    if yaw_cdeg == 0:
        return None
    return normalize_0_360(yaw_cdeg / 100.0)


def great_circle_bearing_deg(lat1, lon1, lat2, lon2) -> float:
    """Initial bearing (0..360, clockwise from true North) from point 1 to point 2."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    return normalize_0_360(math.degrees(math.atan2(y, x)))


def great_circle_distance_m(lat1, lon1, lat2, lon2) -> float:
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def compute_mission_heading(mission_path, release_lat: float, release_lon: float) -> MissionHeadingResult:
    items = parse_qgc_wpl(mission_path)
    item = select_first_nav_waypoint(items)
    bearing = great_circle_bearing_deg(release_lat, release_lon, item.lat, item.lon)
    dist = great_circle_distance_m(release_lat, release_lon, item.lat, item.lon)
    return MissionHeadingResult(
        item=item,
        release_lat=release_lat,
        release_lon=release_lon,
        bearing_0_360_deg=bearing,
        bearing_signed_deg=normalize_signed_180(bearing),
        distance_m=dist,
        source=f"mission:{Path(mission_path).name}:seq{item.seq}",
    )


_PSI_TAG_RE = re.compile(r'(<psi\s+unit="DEG">)[^<]*(</psi>)')


def generate_runtime_ic_copy(source_xml_path, psi_deg: float, dest_dir: Optional[str] = None) -> Path:
    """Write a NEW copy of an IC XML file with <psi> replaced by
    psi_deg. The source file is opened read-only and is never modified.

    Uses a targeted regex substitution (not an XML rewrite/re-serialize)
    so the source file's existing comments and formatting are preserved
    byte-for-byte apart from the one <psi> value -- this avoids
    reintroducing the JSBSim "--" -in-comment parse-error class of bug
    that a full XML re-serialization could risk.
    """
    source_xml_path = Path(source_xml_path)
    content = source_xml_path.read_text()
    new_content, n = _PSI_TAG_RE.subn(rf'\g<1>{psi_deg:.6f}\g<2>', content)
    if n != 1:
        raise MissionHeadingError(
            f"{source_xml_path}: expected exactly one <psi unit=\"DEG\">...</psi> tag, found {n}"
        )
    dest_dir_path = Path(dest_dir) if dest_dir else Path("/tmp")
    dest_dir_path.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    dest_path = dest_dir_path / f"{source_xml_path.stem}_runtime_{stamp}.xml"
    dest_path.write_text(new_content)
    return dest_path


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mission-file", required=True)
    parser.add_argument("--release-lat", type=float, required=True)
    parser.add_argument("--release-lon", type=float, required=True)
    parser.add_argument("--release-groundspeed-mps", type=float, default=None,
                         help="Groundspeed for the vn/ve example (NOT assumed equal to TAS)")
    parser.add_argument("--generate-runtime-ic", default=None,
                         help="Optional source IC XML path; writes a new runtime copy with mission-derived <psi> and prints its path")
    parser.add_argument("--runtime-ic-dir", default=None, help="Directory for --generate-runtime-ic output (default /tmp)")
    return parser


def main():
    args = build_arg_parser().parse_args()
    try:
        result = compute_mission_heading(args.mission_file, args.release_lat, args.release_lon)
    except MissionHeadingError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print(f"selected mission item: seq={result.item.seq} command={result.item.command} (NAV_WAYPOINT)")
    print(f"  lat={result.item.lat:.7f} lon={result.item.lon:.7f} alt={result.item.alt:.1f}m")
    print(f"release coordinates: lat={result.release_lat:.7f} lon={result.release_lon:.7f}")
    print(f"distance: {result.distance_m/1000.0:.3f} km")
    print(f"bearing (0..360): {result.bearing_0_360_deg:.6f} deg")
    print(f"signed yaw (-180..180): {result.bearing_signed_deg:.6f} deg")
    print(f"source: {result.source}")

    if args.release_groundspeed_mps is not None:
        vn, ve = result.vn_ve(args.release_groundspeed_mps)
        print(
            f"vn/ve at groundspeed={args.release_groundspeed_mps:.2f} m/s "
            f"(groundspeed, NOT assumed == TAS): vn={vn:.4f} m/s ve={ve:.4f} m/s"
        )

    if args.generate_runtime_ic:
        dest = generate_runtime_ic_copy(args.generate_runtime_ic, result.bearing_0_360_deg, args.runtime_ic_dir)
        print(f"runtime IC copy written: {dest}")
        print(f"source IC file untouched: {args.generate_runtime_ic}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
