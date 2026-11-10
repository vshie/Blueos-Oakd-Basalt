"""EXIF/XMP written into captured JPEGs: position, depth below sea level, heading, lens."""

import io
import os
import sys
import unittest
from datetime import datetime, timezone

sys.path.insert(0, "/app")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "app"))

import piexif  # noqa: E402
from PIL import Image  # noqa: E402

import photogrammetry_meta as meta  # noqa: E402


def jpeg(width=64, height=48):
    buf = io.BytesIO()
    Image.new("L", (width, height), 128).save(buf, format="JPEG")
    return buf.getvalue()


def dms_to_deg(dms, ref):
    deg = dms[0][0] / dms[0][1] + dms[1][0] / dms[1][1] / 60 + dms[2][0] / dms[2][1] / 3600
    return -deg if ref in (b"S", b"W") else deg


class MetadataTest(unittest.TestCase):
    telemetry = {"lat": 21.3069444, "lon": -157.8583333, "heading_deg": 271.5, "roll_deg": 1.2,
                 "pitch_deg": -3.4, "depth_m": 12.34, "temperature_c": 24.5, "altitude_m": 2.1}

    def stamp(self, telemetry, lens=None):
        now = datetime(2026, 11, 10, 9, 30, 15, 250000)
        return meta.embed_metadata(jpeg(), telemetry, now, now.replace(tzinfo=timezone.utc),
                                   lens=lens, camera_angle_deg=90.0, wp="wp03", vio_tracking=1.0)

    def test_position_depth_heading(self):
        exif = piexif.load(self.stamp(self.telemetry))
        gps = exif["GPS"]
        self.assertAlmostEqual(dms_to_deg(gps[piexif.GPSIFD.GPSLatitude], gps[piexif.GPSIFD.GPSLatitudeRef]),
                               21.3069444, places=5)
        self.assertAlmostEqual(dms_to_deg(gps[piexif.GPSIFD.GPSLongitude], gps[piexif.GPSIFD.GPSLongitudeRef]),
                               -157.8583333, places=5)
        # Depth: below sea level
        self.assertEqual(gps[piexif.GPSIFD.GPSAltitudeRef], 1)
        alt = gps[piexif.GPSIFD.GPSAltitude]
        self.assertAlmostEqual(alt[0] / alt[1], 12.34, places=3)
        direction = gps[piexif.GPSIFD.GPSImgDirection]
        self.assertAlmostEqual(direction[0] / direction[1], 271.5, places=2)
        comment = exif["Exif"][piexif.ExifIFD.UserComment]
        self.assertIn(b"depth=12.34m", comment)
        self.assertIn(b"wp=wp03", comment)

    def test_xmp(self):
        data = self.stamp(self.telemetry)
        self.assertIn(b"<Camera:Yaw>271.50</Camera:Yaw>", data)
        # Camera mounted facing down (90°) with the nose 3.4° down: 3.4° past nadir
        self.assertIn(b"<Camera:Pitch>-3.40</Camera:Pitch>", data)
        self.assertIn(b"<ROV:DepthBelowSurface>12.34</ROV:DepthBelowSurface>", data)
        self.assertIn(b"<ROV:Waypoint>wp03</ROV:Waypoint>", data)

    def test_lens(self):
        # IMX378 at 1920 wide: fx 1000 px at 3.27 um effective pixels = 3.28 mm
        lens = meta.Lens.from_intrinsics(b"MarineSitu C3", b"IMX378", 1000.0, 0.00155, 4056, 1920)
        exif = piexif.load(self.stamp(self.telemetry, lens))
        focal = exif["Exif"][piexif.ExifIFD.FocalLength]
        self.assertAlmostEqual(focal[0] / focal[1], 1000.0 * 0.00155 * 4056 / 1920, places=3)
        self.assertEqual(exif["0th"][piexif.ImageIFD.Model], b"IMX378")

    def test_missing_telemetry_still_writes(self):
        empty = {k: None for k in self.telemetry}
        exif = piexif.load(self.stamp(empty))
        self.assertNotIn(piexif.GPSIFD.GPSLatitude, exif["GPS"])
        self.assertIn(piexif.GPSIFD.GPSDateStamp, exif["GPS"])


if __name__ == "__main__":
    unittest.main()
