"""
Checks and sets the autopilot's EKF parameters for the VISION_POSITION_DELTA messages sent by this
extension, and makes sure the EKF has an origin.
"""

import json
import time
import urllib.error
import urllib.request
from loguru import logger

# (name, value, takes effect only after an autopilot reboot)
VISUAL_ODOMETRY_PARAMS = [
    ("AHRS_EKF_TYPE", 3, True),   # EKF3
    ("EK3_ENABLE", 1, True),
    ("VISO_TYPE", 1, True),       # MAVLink visual odometry
    ("EK3_SRC1_POSXY", 6, False),  # ExternalNav
    ("EK3_SRC1_VELXY", 6, False),  # ExternalNav
    ("EK3_SRC1_POSZ", 1, False),   # Baro
]

MAV_MODE_FLAG_SAFETY_ARMED = 128
GPS_GLOBAL_ORIGIN_MSG_ID = 49


class AutopilotUnreachable(Exception):
    """mavlink2rest itself could not be reached."""


class NoHeartbeat(Exception):
    """mavlink2rest is up, but no autopilot HEARTBEAT has been seen."""


class VehicleArmed(Exception):
    """Parameters are never changed while armed."""


class AutopilotParams:
    def __init__(self, vehicle_ip, system_id=1, component_id=1, timeout=3.0):
        self.base_url = f"http://{vehicle_ip}/mavlink2rest"
        self.system_id = system_id
        self.component_id = component_id
        self.timeout = timeout

    def _get(self, path):
        with urllib.request.urlopen(f"{self.base_url}{path}", timeout=self.timeout) as response:
            return json.loads(response.read())

    def _post(self, message):
        payload = {
            "header": {"system_id": 255, "component_id": 0, "sequence": 0},
            "message": message,
        }
        request = urllib.request.Request(
            f"{self.base_url}/mavlink",
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(request, timeout=self.timeout) as response:
            response.read()

    def _vehicle_message(self, name):
        try:
            return self._get(f"/mavlink/vehicles/{self.system_id}/components/{self.component_id}/messages/{name}")
        except Exception:
            return None

    @staticmethod
    def _encode_id(name):
        return list(name.ljust(16, "\u0000"))

    @staticmethod
    def _decode_id(param_id):
        if isinstance(param_id, list):
            param_id = "".join(param_id)
        return param_id.rstrip("\u0000")

    def _message_counter(self, name):
        message = self._vehicle_message(name)
        return message["status"]["time"]["counter"] if message else 0

    def _wait_param_value(self, name, previous_counter):
        """Waits for a PARAM_VALUE for `name` newer than `previous_counter`. Returns (value, type) or None."""
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            message = self._vehicle_message("PARAM_VALUE")
            if message and message["status"]["time"]["counter"] > previous_counter:
                content = message["message"]
                if self._decode_id(content["param_id"]) == name:
                    return content["param_value"], content["param_type"]["type"]
            time.sleep(0.05)
        return None

    def _request_param(self, name, message, attempts=3):
        # mavlink2rest only keeps the latest PARAM_VALUE, so our reply can be overwritten by another one
        for _ in range(attempts):
            counter = self._message_counter("PARAM_VALUE")
            self._post(message)
            result = self._wait_param_value(name, counter)
            if result is not None:
                return result
        return None

    def read(self, name):
        return self._request_param(name, {
            "type": "PARAM_REQUEST_READ",
            "param_index": -1,
            "target_system": self.system_id,
            "target_component": self.component_id,
            "param_id": self._encode_id(name),
        })

    def write(self, name, value, param_type):
        return self._request_param(name, {
            "type": "PARAM_SET",
            "param_value": float(value),
            "target_system": self.system_id,
            "target_component": self.component_id,
            "param_id": self._encode_id(name),
            "param_type": {"type": param_type},
        })

    def is_armed(self):
        """Raises AutopilotUnreachable or NoHeartbeat instead of guessing."""
        path = f"/mavlink/vehicles/{self.system_id}/components/{self.component_id}/messages/HEARTBEAT"
        try:
            heartbeat = self._get(path)
        except urllib.error.HTTPError as e:
            # mavlink2rest answered, it just has no HEARTBEAT from the autopilot yet
            raise NoHeartbeat(f"no autopilot HEARTBEAT in mavlink2rest ({e.code})") from e
        except (urllib.error.URLError, OSError, ValueError) as e:
            raise AutopilotUnreachable(f"Could not reach mavlink2rest at {self.base_url}: {e}") from e
        return bool(heartbeat["message"]["base_mode"]["bits"] & MAV_MODE_FLAG_SAFETY_ARMED)

    def check(self):
        """Returns the parameters that differ from VISUAL_ODOMETRY_PARAMS. Read-only."""
        self.is_armed()  # surfaces AutopilotUnreachable / NoHeartbeat before reading anything
        diffs = []
        for name, wanted, reboot_required in VISUAL_ODOMETRY_PARAMS:
            current = self.read(name)
            if current is None:
                logger.warning(f"{name}: not reported by the autopilot")
                diffs.append({"name": name, "current": None, "wanted": wanted,
                              "reboot_required": reboot_required, "param_type": None})
                continue
            value, param_type = current
            if value == wanted:
                logger.info(f"{name} = {wanted:g} (already set)")
                continue
            diffs.append({"name": name, "current": value, "wanted": wanted,
                          "reboot_required": reboot_required, "param_type": param_type})
        return diffs

    def apply(self, diffs):
        """Writes `diffs` (from check()). Returns (results, needs_reboot). Raises VehicleArmed if armed."""
        if self.is_armed():
            raise VehicleArmed("Vehicle is armed, not changing autopilot parameters")
        results = []
        needs_reboot = []
        for diff in diffs:
            name, wanted = diff["name"], diff["wanted"]
            if diff["param_type"] is None:
                results.append({"name": name, "ok": False, "error": "not reported by the autopilot"})
                continue
            try:
                result = self.write(name, wanted, diff["param_type"])
            except Exception as e:
                logger.error(f"{name}: {e}")
                results.append({"name": name, "ok": False, "error": str(e)})
                continue
            if result is None or result[0] != wanted:
                logger.error(f"{name}: failed to set to {wanted:g} (was {diff['current']:g})")
                results.append({"name": name, "ok": False, "error": "write not confirmed"})
                continue
            logger.info(f"{name}: {diff['current']:g} -> {wanted:g}")
            results.append({"name": name, "ok": True})
            if diff["reboot_required"]:
                needs_reboot.append(name)
        if needs_reboot:
            logger.warning(f"Reboot the autopilot for {', '.join(needs_reboot)} to take effect")
        return results, needs_reboot

    def get_origin(self, request_timeout=2.0):
        """Returns (lat, lon) of the EKF origin, or None if the autopilot has none."""
        message = self._vehicle_message("GPS_GLOBAL_ORIGIN")
        if message is None:
            counter = self._message_counter("GPS_GLOBAL_ORIGIN")
            self._post({
                "type": "COMMAND_LONG",
                "target_system": self.system_id,
                "target_component": self.component_id,
                "command": {"type": "MAV_CMD_REQUEST_MESSAGE"},
                "confirmation": 0,
                "param1": float(GPS_GLOBAL_ORIGIN_MSG_ID),
                "param2": 0.0, "param3": 0.0, "param4": 0.0,
                "param5": 0.0, "param6": 0.0, "param7": 0.0,
            })
            deadline = time.time() + request_timeout
            while time.time() < deadline:
                message = self._vehicle_message("GPS_GLOBAL_ORIGIN")
                if message and message["status"]["time"]["counter"] > counter:
                    break
                message = None
                time.sleep(0.1)
        if message is None:
            return None
        content = message["message"]
        return content["latitude"] / 1e7, content["longitude"] / 1e7

    def set_origin(self, lat, lon):
        self._post({
            "type": "SET_GPS_GLOBAL_ORIGIN",
            "target_system": self.system_id,
            "latitude": int(round(lat * 1e7)),
            "longitude": int(round(lon * 1e7)),
            "altitude": 0,
            "time_usec": 0,
        })
        logger.info(f"Set EKF origin to {lat:.6f}, {lon:.6f}")
