"""Photogrammetry metadata: EXIF GPS/depth/orientation + Pix4D XMP for captured JPEGs.

Adapted from SubReels TowFish's ``app/photogrammetry_meta.py``
(https://github.com/vshie/SubReels_TowFish, MIT License), with the tow-specific parts removed:
position, heading and depth all come from this ROV's own autopilot.

Conventions:

* Position is the autopilot's EKF estimate (VIO deltas integrated from the EKF origin).
* ``GPSAltitude`` carries the vehicle **depth**, with ``GPSAltitudeRef = 1`` (below sea level).
  That is the standard EXIF way to express depth, and Metashape, Pix4D and WebODM read it as a
  negative altitude.
* ``Camera:Yaw`` is the vehicle heading (0..360). ``Camera:Pitch`` is measured **from nadir**,
  as Pix4D and Metashape expect: a camera looking straight down is 0. It is derived from the
  camera's mounting angle on the vehicle plus the vehicle's pitch.
* Interior orientation (``FocalLength`` + ``FocalPlane*Resolution``) comes from the camera's own
  in-water calibration, so the solver starts from the real underwater focal length instead of
  Metashape's 50 mm default.
"""
from __future__ import annotations

import io
import logging
import math

import piexif

logger = logging.getLogger(__name__)

SOFTWARE = b"BlueOS OAK-D VIO"
ROV_XMP_NS = "http://bluerobotics.com/oakd-vio/1.0/"

# Depth is a pressure reading; publish it as the Z accuracy prior so solvers weight it properly.
DEPTH_Z_ACCURACY_M = 0.3


def _finite(value):
    """Return ``value`` as a float, or ``None`` if it is not finite.

    A NaN reaching a rational conversion raises inside piexif, which would strip the frame of
    all metadata rather than just the offending field.
    """
    if value is None:
        return None
    try:
        num = float(value)
    except (TypeError, ValueError):
        return None
    return num if math.isfinite(num) else None


class Lens:
    """Interior orientation of one camera at the resolution it is captured at."""

    def __init__(self, make, model, focal_mm, pixel_mm):
        self.make = make
        self.model = model
        self.focal_mm = focal_mm  # fx (pixels, at capture resolution) * pixel_mm
        self.pixel_mm = pixel_mm  # effective pixel size at capture resolution

    @classmethod
    def from_intrinsics(cls, make, model, fx_px, native_pixel_mm, native_width_px, width_px):
        """``fx_px`` must be the focal length in pixels at ``width_px``."""
        pixel_mm = native_pixel_mm * native_width_px / width_px
        return cls(make, model, fx_px * pixel_mm, pixel_mm)


def decimal_deg_to_dms_rationals(deg):
    """Convert decimal degrees -> EXIF-style ((d,1),(m,1),(s*10000,10000))."""
    deg = abs(float(deg))
    d = int(deg)
    m_full = (deg - d) * 60.0
    m = int(m_full)
    s = (m_full - m) * 60.0
    return ((d, 1), (m, 1), (int(round(s * 10000)), 10000))


def camera_pitch_from_nadir(camera_angle_deg, body_pitch_deg=None):
    """Camera pitch in the Pix4D/Metashape sense: degrees off nadir.

    ``camera_angle_deg`` is the camera's downward mounting angle (0 = forward, 90 = down).
    Nose-up body pitch tilts the camera toward the horizon.
    """
    angle = _finite(camera_angle_deg) or 0.0
    body = _finite(body_pitch_deg) or 0.0
    return 90.0 - angle + body


def build_user_comment(lat, lon, depth_m, heading_deg, ts_utc, roll_deg=None, pitch_deg=None,
                       altitude_m=None, temp_c=None, wp=None, focal_mm=None):
    """Human-readable telemetry bundle for UserComment/ImageDescription."""
    parts = []
    lat_f, lon_f = _finite(lat), _finite(lon)
    if lat_f is not None and lon_f is not None:
        parts.append(f"pos={lat_f:.7f},{lon_f:.7f}")
    for label, value, fmt in (
        ("depth", depth_m, "{:.2f}m"),
        ("hdg", heading_deg, "{:.1f}deg"),
        ("roll", roll_deg, "{:+.1f}deg"),
        ("pitch", pitch_deg, "{:+.1f}deg"),
        ("alt", altitude_m, "{:.2f}m"),
        ("temp", temp_c, "{:.1f}C"),
        ("fl", focal_mm, "{:.2f}mm"),
    ):
        num = _finite(value)
        if num is not None:
            parts.append(f"{label}=" + fmt.format(num))
    if wp:
        parts.append(f"wp={wp}")
    parts.append(f"utc={ts_utc.strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3]}Z")
    return " ".join(parts)


def build_exif_bytes(lat, lon, depth_m, heading_deg, ts_local, ts_utc, lens=None,
                     roll_deg=None, pitch_deg=None, altitude_m=None, temp_c=None, wp=None):
    """Build piexif-encoded EXIF bytes. Returns ``None`` only if piexif refuses to serialise."""
    lat = _finite(lat)
    lon = _finite(lon)
    depth_m = _finite(depth_m)
    heading_deg = _finite(heading_deg)

    gps_ifd = {piexif.GPSIFD.GPSVersionID: (2, 3, 0, 0)}
    if lat is not None and lon is not None:
        gps_ifd[piexif.GPSIFD.GPSLatitudeRef] = b'N' if lat >= 0 else b'S'
        gps_ifd[piexif.GPSIFD.GPSLatitude] = decimal_deg_to_dms_rationals(lat)
        gps_ifd[piexif.GPSIFD.GPSLongitudeRef] = b'E' if lon >= 0 else b'W'
        gps_ifd[piexif.GPSIFD.GPSLongitude] = decimal_deg_to_dms_rationals(lon)
    if depth_m is not None:
        # 1 = below sea level; a negative depth (above the surface) flips it back to 0
        gps_ifd[piexif.GPSIFD.GPSAltitudeRef] = 1 if depth_m >= 0 else 0
        gps_ifd[piexif.GPSIFD.GPSAltitude] = (int(round(abs(depth_m) * 1000)), 1000)
    if heading_deg is not None:
        gps_ifd[piexif.GPSIFD.GPSImgDirectionRef] = b'M'  # autopilot heading is compass based
        gps_ifd[piexif.GPSIFD.GPSImgDirection] = (int(round((heading_deg % 360) * 100)), 100)

    # GPS time/date in UTC per spec, regardless of system locale.
    gps_ifd[piexif.GPSIFD.GPSTimeStamp] = (
        (ts_utc.hour, 1),
        (ts_utc.minute, 1),
        (int(round((ts_utc.second + ts_utc.microsecond / 1e6) * 100)), 100),
    )
    gps_ifd[piexif.GPSIFD.GPSDateStamp] = ts_utc.strftime('%Y:%m:%d').encode('ascii')

    dt_str = ts_local.strftime('%Y:%m:%d %H:%M:%S').encode('ascii')
    subsec = f"{int(ts_local.microsecond / 1000):03d}".encode('ascii')
    exif_ifd = {
        piexif.ExifIFD.DateTimeOriginal: dt_str,
        piexif.ExifIFD.DateTimeDigitized: dt_str,
        piexif.ExifIFD.SubSecTimeOriginal: subsec,
        piexif.ExifIFD.SubSecTimeDigitized: subsec,
    }
    image_ifd = {
        piexif.ImageIFD.DateTime: dt_str,
        piexif.ImageIFD.Software: SOFTWARE,
    }
    focal_mm = None
    if lens is not None:
        focal_mm = lens.focal_mm
        ppi = 25.4 / lens.pixel_mm
        image_ifd[piexif.ImageIFD.Make] = lens.make
        image_ifd[piexif.ImageIFD.Model] = lens.model
        exif_ifd[piexif.ExifIFD.FocalLength] = (int(round(focal_mm * 1000)), 1000)
        exif_ifd[piexif.ExifIFD.FocalPlaneXResolution] = (int(round(ppi * 1000)), 1000)
        exif_ifd[piexif.ExifIFD.FocalPlaneYResolution] = (int(round(ppi * 1000)), 1000)
        exif_ifd[piexif.ExifIFD.FocalPlaneResolutionUnit] = 2  # inch

    comment = build_user_comment(lat, lon, depth_m, heading_deg, ts_utc, roll_deg=roll_deg,
                                 pitch_deg=pitch_deg, altitude_m=altitude_m, temp_c=temp_c,
                                 wp=wp, focal_mm=focal_mm)
    comment_bytes = comment.encode('ascii', 'replace')
    # EXIF UserComment needs an 8-byte character-code prefix.
    exif_ifd[piexif.ExifIFD.UserComment] = b"ASCII\x00\x00\x00" + comment_bytes
    image_ifd[piexif.ImageIFD.ImageDescription] = comment_bytes

    try:
        return piexif.dump({"0th": image_ifd, "Exif": exif_ifd, "GPS": gps_ifd})
    except Exception:
        logger.exception("EXIF dump failed")
        return None


def build_xmp(yaw_deg=None, pitch_deg=None, roll_deg=None, z_accuracy_m=None,
              depth_m=None, altitude_m=None, vio_tracking=None, wp=None):
    """XMP packet: Pix4D ``Camera:*`` orientation plus ROV values with no standard tag."""
    camera_tags = []
    for name, value in (("Yaw", yaw_deg), ("Pitch", pitch_deg), ("Roll", roll_deg),
                        ("GPSZAccuracy", z_accuracy_m)):
        num = _finite(value)
        if num is not None:
            camera_tags.append(f"    <Camera:{name}>{num:.2f}</Camera:{name}>")

    rov_tags = []
    for name, value in (("DepthBelowSurface", depth_m), ("AltitudeAboveBottom", altitude_m),
                        ("VioTracking", vio_tracking)):
        num = _finite(value)
        if num is not None:
            rov_tags.append(f"    <ROV:{name}>{num:.2f}</ROV:{name}>")
    if wp:
        rov_tags.append(f"    <ROV:Waypoint>{wp}</ROV:Waypoint>")

    if not camera_tags and not rov_tags:
        return None
    body = "\n".join(camera_tags + rov_tags)
    return (
        '<?xpacket begin="﻿" id="W5M0MpCehiHzreSzNTczkc9d"?>'
        '<x:xmpmeta xmlns:x="adobe:ns:meta/">'
        '<rdf:RDF xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#">'
        '<rdf:Description rdf:about="" '
        'xmlns:Camera="http://pix4d.com/camera/1.0/" '
        f'xmlns:ROV="{ROV_XMP_NS}">\n'
        f'{body}\n'
        '  </rdf:Description>'
        '</rdf:RDF>'
        '</x:xmpmeta>'
        '<?xpacket end="w"?>'
    )


def insert_xmp_app1(jpeg_bytes, xmp_packet):
    """Splice an APP1 XMP segment in just after the JPEG SOI marker, without re-encoding."""
    try:
        if jpeg_bytes[0:2] != b"\xff\xd8":
            return jpeg_bytes
        payload = b"http://ns.adobe.com/xap/1.0/\x00" + xmp_packet.encode("utf-8")
        seg_len = len(payload) + 2
        if seg_len > 0xFFFF:
            return jpeg_bytes
        app1 = b"\xff\xe1" + seg_len.to_bytes(2, "big") + payload
        return jpeg_bytes[:2] + app1 + jpeg_bytes[2:]
    except Exception:
        logger.exception("XMP insert failed")
        return jpeg_bytes


def embed_metadata(jpeg_bytes, telemetry, ts_local, ts_utc, lens=None, camera_angle_deg=0.0,
                   wp=None, vio_tracking=None):
    """Stamp EXIF + XMP into ``jpeg_bytes`` and return the new bytes.

    ``telemetry`` is the dict produced by ``capture_node.read_autopilot_telemetry``. Failure is
    non-fatal: a frame with partial or no metadata still beats losing the frame.
    """
    t = telemetry or {}
    try:
        exif_bytes = build_exif_bytes(
            t.get("lat"), t.get("lon"), t.get("depth_m"), t.get("heading_deg"), ts_local, ts_utc,
            lens=lens, roll_deg=t.get("roll_deg"), pitch_deg=t.get("pitch_deg"),
            altitude_m=t.get("altitude_m"), temp_c=t.get("temperature_c"), wp=wp,
        )
        if exif_bytes is not None:
            out_buf = io.BytesIO()
            piexif.insert(exif_bytes, jpeg_bytes, out_buf)
            jpeg_bytes = out_buf.getvalue()

        have_attitude = _finite(t.get("heading_deg")) is not None
        xmp = build_xmp(
            yaw_deg=t.get("heading_deg"),
            pitch_deg=camera_pitch_from_nadir(camera_angle_deg, t.get("pitch_deg")) if have_attitude else None,
            roll_deg=t.get("roll_deg"),
            z_accuracy_m=DEPTH_Z_ACCURACY_M if _finite(t.get("depth_m")) is not None else None,
            depth_m=t.get("depth_m"),
            altitude_m=t.get("altitude_m"),
            vio_tracking=vio_tracking,
            wp=wp,
        )
        if xmp is not None:
            jpeg_bytes = insert_xmp_app1(jpeg_bytes, xmp)
    except Exception:
        logger.exception("Photogrammetry metadata embed failed")
    return jpeg_bytes
