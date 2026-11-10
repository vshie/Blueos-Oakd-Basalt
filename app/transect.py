"""
Auto (transect) capture: records only while the ROV runs a mission, labelling images per leg.

Ported from SubReels TowFish's transect monitor (https://github.com/vshie/SubReels_TowFish, MIT),
but watching this ROV's own autopilot instead of a tow boat:

    disabled  --enable--> waiting
    waiting   --armed + AUTO + navigating--> recording   (label wpNN = MISSION_CURRENT.seq)
    recording --MISSION_CURRENT.seq moves forward--> recording, next leg
    recording --"Mission Complete" STATUSTEXT / seq jumps backward--> waiting
    recording --seen (fresh telemetry) out of AUTO or disarmed for 60 s--> waiting
    *         --disable--> disabled

Once recording, missing or stale telemetry never stops the session; only fresh, positive evidence
does. Each closed leg is appended to `{source_tag}_manifest.ndjson` in the survey-day folder.
"""

import glob
import json
import os
import threading
import time
from datetime import datetime

from loguru import logger

import vehicle_telemetry
from capture_node import survey_day_folder

POLL_INTERVAL_S = 0.33
STOP_GRACE_S = 60.0
# Mission items such as DO_CHANGE_SPEED advance the seq without a new leg; shorter legs are merged
MIN_LEG_DURATION_S = 1.0
NAV_SPEED_THRESHOLD = 0.2  # m/s

MESSAGES = ["HEARTBEAT", "MISSION_CURRENT", "NAV_CONTROLLER_OUTPUT", "VFR_HUD",
            "STATUSTEXT", "GLOBAL_POSITION_INT"]


def make_source_tag(prefix, now=None):
    """`tr`/`tl` + HHMMSS of the session start, embedded in every filename."""
    return f"{prefix}{(now or datetime.now()):%H%M%S}"


class TransectMonitor:
    def __init__(self, vehicle_ip, captures_dir, on_change, get_messages=None,
                 clock=time.monotonic, now=datetime.now):
        """`on_change(recording, source_tag, wp)` is called whenever capture should change."""
        self.base = vehicle_telemetry.base_url(vehicle_ip)
        self.captures_dir = captures_dir
        self.on_change = on_change
        self.get_messages = get_messages or (lambda names: vehicle_telemetry.get_messages(self.base, names))
        self.clock = clock
        self.now = now

        self.lock = threading.RLock()
        self.state = "disabled"
        self.source_tag = None
        self.current_seq = None
        self.leg_count = 0
        self.events = []
        self._stop_evidence_since = None
        self._mission_complete = False
        self._statustext_counter = None
        self._leg = None
        self._manifest_path = None
        self._thread = None

    # -- control ------------------------------------------------------------
    def enable(self):
        with self.lock:
            if self.state != "disabled":
                return
            self.state = "waiting"
            self._note("enabled, waiting for an armed AUTO mission")
        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._run, daemon=True, name="transect")
            self._thread.start()

    def disable(self):
        with self.lock:
            if self.state == "recording":
                self._exit_recording({}, "disabled")
            self.state = "disabled"
            self._note("disabled")

    def status(self):
        with self.lock:
            return {
                "state": self.state,
                "source_tag": self.source_tag,
                "wp": self._wp_label(self.current_seq) if self.current_seq is not None else None,
                "legs": self.leg_count,
                "events": self.events[-10:],
            }

    # -- loop ---------------------------------------------------------------
    def _run(self):
        while True:
            with self.lock:
                if self.state == "disabled":
                    return
            try:
                self.tick()
            except Exception:
                logger.exception("Transect monitor tick failed")
            time.sleep(POLL_INTERVAL_S)

    def _read_state(self):
        raw = self.get_messages(MESSAGES)
        hb = vehicle_telemetry.fresh(raw.get("HEARTBEAT"))
        mission = vehicle_telemetry.fresh(raw.get("MISSION_CURRENT"))
        nav = vehicle_telemetry.fresh(raw.get("NAV_CONTROLLER_OUTPUT"))
        vfr = vehicle_telemetry.fresh(raw.get("VFR_HUD"))
        pos = vehicle_telemetry.fresh(raw.get("GLOBAL_POSITION_INT"))

        state = {"heartbeat_fresh": hb is not None, "armed": None, "auto": None,
                 "mission_seq": None, "wp_dist": None, "groundspeed": None,
                 "lat": None, "lon": None, "heading": None, "statustext": None}
        if hb:
            state["armed"] = bool(hb["base_mode"]["bits"] & vehicle_telemetry.MAV_MODE_FLAG_SAFETY_ARMED)
            state["auto"] = hb["custom_mode"] == vehicle_telemetry.ARDUSUB_MODE_AUTO
        if mission:
            state["mission_seq"] = mission.get("seq")
        if nav:
            state["wp_dist"] = nav.get("wp_dist")
        if vfr:
            state["groundspeed"] = vfr.get("groundspeed")
            state["heading"] = vfr.get("heading")
        if pos and (pos["lat"] or pos["lon"]):
            state["lat"], state["lon"] = pos["lat"] / 1e7, pos["lon"] / 1e7

        status = raw.get("STATUSTEXT")
        count = vehicle_telemetry.counter(status)
        if count is not None and count != self._statustext_counter:
            if self._statustext_counter is not None:
                state["statustext"] = vehicle_telemetry.text_of(status["message"].get("text"))
            self._statustext_counter = count
        return state

    @staticmethod
    def _navigating(state):
        return bool((state["wp_dist"] or 0) > 0
                    or (state["groundspeed"] or 0) > NAV_SPEED_THRESHOLD)

    def tick(self):
        state = self._read_state()
        with self.lock:
            if state["statustext"]:
                self._note(f"statustext: {state['statustext']}")
                if "mission complete" in state["statustext"].lower():
                    self._mission_complete = True

            if self.state == "waiting":
                self._mission_complete = False
                if (state["heartbeat_fresh"] and state["armed"] and state["auto"]
                        and state["mission_seq"] is not None and self._navigating(state)):
                    self._enter_recording(state)
                return

            if self.state != "recording":
                return

            if self._mission_complete:
                self._exit_recording(state, "mission_complete")
                return
            seq = state["mission_seq"]
            if seq is not None and seq < self.current_seq:
                self._exit_recording(state, "seq_backward")
                return

            # Only fresh evidence counts; a telemetry gap leaves the grace timer untouched
            if state["heartbeat_fresh"]:
                if not state["armed"] or not state["auto"]:
                    if self._stop_evidence_since is None:
                        self._stop_evidence_since = self.clock()
                    elif self.clock() - self._stop_evidence_since >= STOP_GRACE_S:
                        self._exit_recording(state, "left_auto" if state["armed"] else "disarmed")
                        return
                else:
                    self._stop_evidence_since = None

            if seq is not None and seq > self.current_seq:
                self._roll_leg(state, seq)

    # -- transitions --------------------------------------------------------
    @staticmethod
    def _wp_label(seq):
        return f"wp{int(seq):02d}"

    def _note(self, text):
        self.events.append(f"{self.now():%H:%M:%S} {text}")
        self.events = self.events[-50:]
        logger.info(f"Transect: {text}")

    def _enter_recording(self, state):
        now = self.now()
        self.source_tag = make_source_tag("tr", now)
        self.current_seq = state["mission_seq"]
        self.leg_count = 1
        self._stop_evidence_since = None
        self._mission_complete = False
        day = survey_day_folder(self.captures_dir, now)
        os.makedirs(day, exist_ok=True)
        self._manifest_path = os.path.join(day, f"{self.source_tag}_manifest.ndjson")
        self._append_manifest({"ts": time.time(), "time": f"{now:%Y-%m-%d %H:%M:%S.%f}"[:-3],
                               "event": "session_start", "source_tag": self.source_tag})
        self._begin_leg(state)
        self.state = "recording"
        self.on_change(True, self.source_tag, self._wp_label(self.current_seq))
        self._note(f"recording started at {self._wp_label(self.current_seq)}")

    def _exit_recording(self, state, reason):
        try:
            self._close_leg(state, reason)
        except Exception:
            logger.exception("Closing the last leg failed")
        self.on_change(False, self.source_tag, None)
        self._note(f"recording stopped: {reason}")
        self.state = "waiting"
        self.current_seq = None
        self._leg = None
        self._manifest_path = None
        self._stop_evidence_since = None
        self._mission_complete = False

    def _roll_leg(self, state, new_seq):
        duration = self.clock() - self._leg["started_mono"]
        if duration >= MIN_LEG_DURATION_S:
            self._close_leg(state, "next_wp")
            self.leg_count += 1
            self._begin_leg(state, new_seq)
            self._note(f"leg {self._wp_label(new_seq)}")
        else:
            # A burst of non-navigation items: relabel without closing a tiny leg
            self._leg["seq"] = new_seq
            self._note(f"seq -> {new_seq} (merged, prior leg {duration:.1f}s)")
        self.current_seq = new_seq
        self.on_change(True, self.source_tag, self._wp_label(new_seq))

    def _begin_leg(self, state, seq=None):
        self._leg = {
            "seq": self.current_seq if seq is None else seq,
            "started_mono": self.clock(),
            "start_time": self.now(),
            "start_lat": state.get("lat"), "start_lon": state.get("lon"),
            "heading_start": state.get("heading"),
            "groundspeed_start": state.get("groundspeed"),
            "wp_dist_start": state.get("wp_dist"),
        }

    def _count_images(self, wp):
        day = os.path.dirname(self._manifest_path)
        pattern = f"*_{self.source_tag}_{wp}_*.jpg"
        return (len(glob.glob(os.path.join(day, "center", pattern))),
                len(glob.glob(os.path.join(day, "stereo", "left", pattern))))

    def _close_leg(self, state, reason):
        if self._leg is None or not self._manifest_path:
            return
        end = self.now()
        wp = self._wp_label(self._leg["seq"])
        center, stereo = self._count_images(wp)
        self._append_manifest({
            "ts": time.time(),
            "event": "leg_close",
            "seq": self._leg["seq"],
            "leg_wp": wp,
            "start_time": f"{self._leg['start_time']:%Y-%m-%d %H:%M:%S.%f}"[:-3],
            "end_time": f"{end:%Y-%m-%d %H:%M:%S.%f}"[:-3],
            "duration_s": round(self.clock() - self._leg["started_mono"], 2),
            "start_lat": self._leg["start_lat"], "start_lon": self._leg["start_lon"],
            "end_lat": state.get("lat"), "end_lon": state.get("lon"),
            "heading_start": self._leg["heading_start"],
            "groundspeed_start": self._leg["groundspeed_start"],
            "wp_dist_start": self._leg["wp_dist_start"],
            "wp_dist_end": state.get("wp_dist"),
            "leg_center_images": center,
            "leg_stereo_pairs": stereo,
            "close_reason": reason,
        })

    def _append_manifest(self, row):
        try:
            with open(self._manifest_path, "a") as f:
                f.write(json.dumps(row) + "\n")
        except OSError:
            logger.exception("Manifest write failed")
