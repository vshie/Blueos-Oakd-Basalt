"""
Reads the ROV's own autopilot state from mavlink2rest (no tow vehicle: everything comes from the
local autopilot at `vehicle_ip`). Used to label captured images and to drive the transect monitor.
"""

import json
import math
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone

# A message older than this is treated as missing
STALE_AFTER_S = 5.0

# ArduSub custom_mode for AUTO
ARDUSUB_MODE_AUTO = 3
MAV_MODE_FLAG_SAFETY_ARMED = 128

_executor = ThreadPoolExecutor(max_workers=6, thread_name_prefix="m2r")


def base_url(vehicle_ip):
    return f"http://{vehicle_ip}/mavlink2rest"


def get_message(base, name, system_id=1, component_id=1, timeout=0.5):
    """Latest `name` message as mavlink2rest returns it ({"message":..., "status":...}), or None."""
    url = f"{base}/mavlink/vehicles/{system_id}/components/{component_id}/messages/{name}"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            return json.loads(response.read())
    except Exception:
        return None


def message_age_s(message):
    """Seconds since mavlink2rest last updated `message`, or None if it can't be told."""
    try:
        last = datetime.fromisoformat(message["status"]["time"]["last_update"])
    except (KeyError, TypeError, ValueError):
        return None
    if last.tzinfo is None:
        last = last.replace(tzinfo=timezone.utc)
    return (datetime.now(timezone.utc) - last).total_seconds()


def fresh(message):
    """The message body if it exists and isn't stale, else None."""
    if message is None:
        return None
    age = message_age_s(message)
    if age is not None and age > STALE_AFTER_S:
        return None
    return message.get("message")


def counter(message):
    try:
        return message["status"]["time"]["counter"]
    except (KeyError, TypeError):
        return None


def get_messages(base, names):
    """Fetches several messages in parallel. Returns {name: raw mavlink2rest reply or None}."""
    futures = {name: _executor.submit(get_message, base, name) for name in names}
    return {name: future.result() for name, future in futures.items()}


def text_of(field):
    """STATUSTEXT.text arrives as a list of characters (or a string); returns a str."""
    if isinstance(field, list):
        field = "".join(c for c in field if isinstance(c, str))
    return (field or "").rstrip("\u0000")


def read_autopilot_telemetry(base):
    """Position, attitude, depth and environment for labelling one image. Missing values are None."""
    raw = get_messages(base, ["GLOBAL_POSITION_INT", "ATTITUDE", "VFR_HUD",
                              "SCALED_PRESSURE2", "DISTANCE_SENSOR", "MISSION_CURRENT"])
    pos = fresh(raw["GLOBAL_POSITION_INT"])
    att = fresh(raw["ATTITUDE"])
    vfr = fresh(raw["VFR_HUD"])
    press = fresh(raw["SCALED_PRESSURE2"])
    dist = fresh(raw["DISTANCE_SENSOR"])
    mission = fresh(raw["MISSION_CURRENT"])

    t = {"lat": None, "lon": None, "heading_deg": None, "roll_deg": None, "pitch_deg": None,
         "depth_m": None, "temperature_c": None, "altitude_m": None, "mission_seq": None}
    if pos:
        lat, lon = pos["lat"] / 1e7, pos["lon"] / 1e7
        # 0,0 means the EKF has no origin yet (Cockpit sets it)
        if lat != 0 or lon != 0:
            t["lat"], t["lon"] = lat, lon
        if pos.get("hdg", 65535) != 65535:
            t["heading_deg"] = pos["hdg"] / 100.0
        t["depth_m"] = -pos["relative_alt"] / 1000.0
    if att:
        t["roll_deg"] = math.degrees(att["roll"])
        t["pitch_deg"] = math.degrees(att["pitch"])
        if t["heading_deg"] is None:
            t["heading_deg"] = math.degrees(att["yaw"]) % 360
    if vfr:
        # ArduSub reports depth as negative altitude
        t["depth_m"] = -vfr["alt"]
    if press:
        t["temperature_c"] = press["temperature"] / 100.0
    if dist and dist.get("current_distance", 0) > 0:
        t["altitude_m"] = dist["current_distance"] / 100.0
    if mission:
        t["mission_seq"] = mission.get("seq")
    return t
