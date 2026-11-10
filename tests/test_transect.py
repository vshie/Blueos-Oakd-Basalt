"""
TransectMonitor state machine against a stubbed mavlink2rest. Run inside the image:
    docker run --rm -v $PWD/tests:/tests blueos-oakd-basalt:local python -m unittest discover -s /tests
"""

import json
import os
import sys
import tempfile
import unittest
from datetime import datetime, timedelta, timezone

sys.path.insert(0, "/app")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "app"))

import transect  # noqa: E402

AUTO, STABILIZE = 3, 0
ARMED = 128


class FakeVehicle:
    """Builds mavlink2rest-shaped replies; every field can be changed between ticks."""

    def __init__(self):
        self.armed = True
        self.mode = AUTO
        self.seq = 1
        self.wp_dist = 12
        self.groundspeed = 0.5
        self.statustext = None
        self.statustext_counter = 0
        self.stale = False

    def _wrap(self, message, counter=1):
        age = timedelta(seconds=30 if self.stale else 0)
        last = (datetime.now(timezone.utc) - age).isoformat()
        return {"message": message, "status": {"time": {"last_update": last, "counter": counter}}}

    def say(self, text):
        self.statustext = text
        self.statustext_counter += 1

    def get_messages(self, names):
        return {
            "HEARTBEAT": self._wrap({"base_mode": {"bits": ARMED if self.armed else 0}, "custom_mode": self.mode}),
            "MISSION_CURRENT": self._wrap({"seq": self.seq}),
            "NAV_CONTROLLER_OUTPUT": self._wrap({"wp_dist": self.wp_dist}),
            "VFR_HUD": self._wrap({"groundspeed": self.groundspeed, "heading": 90}),
            "GLOBAL_POSITION_INT": self._wrap({"lat": 213000000, "lon": -1578000000}),
            "STATUSTEXT": self._wrap({"text": list(self.statustext or "")}, self.statustext_counter),
        }


class TransectTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.vehicle = FakeVehicle()
        self.changes = []
        self.t = 1000.0
        self.monitor = transect.TransectMonitor(
            "127.0.0.1", self.tmp.name, lambda *c: self.changes.append(c),
            get_messages=self.vehicle.get_messages, clock=lambda: self.t)
        # Drive tick() by hand rather than via the background thread
        self.monitor.state = "waiting"

    def tearDown(self):
        self.tmp.cleanup()

    def tick(self, advance=1.0):
        self.t += advance
        self.monitor.tick()

    def manifest(self):
        with open(self.monitor._manifest_path or self.last_manifest) as f:
            return [json.loads(line) for line in f]

    def test_waits_until_armed_auto_and_navigating(self):
        self.vehicle.armed = False
        self.tick()
        self.assertEqual(self.monitor.state, "waiting")
        self.vehicle.armed = True
        self.vehicle.mode = STABILIZE
        self.tick()
        self.assertEqual(self.monitor.state, "waiting")
        self.vehicle.mode = AUTO
        self.vehicle.wp_dist = 0
        self.vehicle.groundspeed = 0
        self.tick()
        self.assertEqual(self.monitor.state, "waiting")
        self.vehicle.wp_dist = 5
        self.tick()
        self.assertEqual(self.monitor.state, "recording")
        self.assertEqual(self.changes[-1][0], True)
        self.assertEqual(self.changes[-1][2], "wp01")
        self.assertTrue(self.changes[-1][1].startswith("tr"))

    def test_rolls_legs_and_merges_short_ones(self):
        self.tick()
        self.vehicle.seq = 2
        self.tick(advance=10)
        self.assertEqual(self.changes[-1][2], "wp02")
        self.assertEqual(self.monitor.leg_count, 2)
        # A non-navigation item advances seq straight away: merged, not a new leg
        self.vehicle.seq = 3
        self.tick(advance=0.3)
        self.assertEqual(self.changes[-1][2], "wp03")
        self.assertEqual(self.monitor.leg_count, 2)
        legs = [r for r in self.manifest() if r["event"] == "leg_close"]
        self.assertEqual([r["leg_wp"] for r in legs], ["wp01"])

    def test_mission_complete_stops(self):
        self.tick()
        self.vehicle.say("Mission Complete")
        self.tick()
        self.assertEqual(self.monitor.state, "waiting")
        self.assertEqual(self.changes[-1][0], False)

    def test_backward_seq_stops(self):
        self.vehicle.seq = 4
        self.tick()
        self.last_manifest = self.monitor._manifest_path
        self.vehicle.seq = 1
        self.tick()
        self.assertEqual(self.monitor.state, "waiting")
        self.assertEqual(self.manifest()[-1]["close_reason"], "seq_backward")

    def test_leaving_auto_needs_grace(self):
        self.tick()
        self.vehicle.mode = STABILIZE
        self.tick()
        self.tick(advance=30)
        self.assertEqual(self.monitor.state, "recording")
        # Back in AUTO resets the grace timer
        self.vehicle.mode = AUTO
        self.tick()
        self.vehicle.mode = STABILIZE
        self.tick()
        self.tick(advance=59)
        self.assertEqual(self.monitor.state, "recording")
        self.tick(advance=2)
        self.assertEqual(self.monitor.state, "waiting")

    def test_stale_telemetry_never_stops(self):
        self.tick()
        self.vehicle.stale = True
        self.vehicle.armed = False
        for _ in range(10):
            self.tick(advance=60)
        self.assertEqual(self.monitor.state, "recording")

    def test_disable_closes_session(self):
        self.tick()
        self.last_manifest = self.monitor._manifest_path
        self.monitor.disable()
        self.assertEqual(self.monitor.state, "disabled")
        self.assertEqual(self.manifest()[-1]["close_reason"], "disabled")


if __name__ == "__main__":
    unittest.main()
