"""
Survey image capture alongside VIO.

Frames arrive already JPEG-encoded by the OAK (MJPEG VideoEncoder), at the capture rates set on
the camera, so the Pi only labels and writes them. Every image is tagged from the ROV's own
autopilot (position, heading, depth, ...) and the latest VIO pose.

Layout (naming follows SubReels TowFish's survey layout):

    captures/survey_YYYYMMDD/
        center/{seq:06d}_{source_tag}[_{wp}]_{frame:05d}.jpg
        stereo/left/...   stereo/right/...   (identical names for each synced pair)
        telemetry_center.csv
        telemetry_stereo.csv
        {source_tag}_manifest.ndjson         (Auto mode, written by transect.py)

What is recorded is controlled by `capture_control.json`, written by the web server, so recording
can be started, stopped or relabelled without restarting the pipeline.
"""

import csv
import errno
import json
import os
import re
import shutil
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone

import depthai as dai
from loguru import logger

import photogrammetry_meta
import vehicle_telemetry

CSV_HEADER = [
    'timestamp', 'seq', 'filename', 'source_tag', 'wp', 'frame', 'size_bytes',
    'lat', 'lon', 'heading_deg', 'roll_deg', 'pitch_deg', 'depth_m', 'temperature_c',
    'telem_ms', 'sync_skew_ms', 'altitude_above_bottom_m',
    'vio_x', 'vio_y', 'vio_z', 'vio_qw', 'vio_qx', 'vio_qy', 'vio_qz', 'vio_tracking',
]

# Left/right frames from the synced OV9282 pair land within a few ms of each other
STEREO_PAIR_TOLERANCE_S = 0.015
STEREO_PENDING_MAX_AGE_S = 2.0

_SEQ_RE = re.compile(r"^(\d{6})_")


def survey_day_folder(captures_dir, now=None):
    return os.path.join(captures_dir, f"survey_{(now or datetime.now()):%Y%m%d}")


def scan_max_seq(folder):
    """Highest NNNNNN_ prefix in `folder`, so numbering continues across restarts within a day."""
    highest = 0
    try:
        for name in os.listdir(folder):
            match = _SEQ_RE.match(name)
            if match:
                highest = max(highest, int(match.group(1)))
    except FileNotFoundError:
        pass
    return highest


def make_filename(seq, source_tag, wp, frame):
    if wp:
        return f"{seq:06d}_{source_tag}_{wp}_{frame:05d}.jpg"
    return f"{seq:06d}_{source_tag}_{frame:05d}.jpg"


def read_control(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (FileNotFoundError, ValueError):
        return {"mode": "off", "recording": False}


def _fmt(value, spec):
    return format(value, spec) if value is not None else ""


class StreamWriter:
    """Numbering, file writes and CSV rows for one stream ("center" or "stereo")."""

    def __init__(self, name, subdirs):
        self.name = name
        self.subdirs = subdirs  # e.g. ["center"] or ["stereo/left", "stereo/right"]
        self.lock = threading.Lock()
        self.day_folder = None
        self.seq = 0
        self.frame = 0
        self.label = None  # (source_tag, wp) the frame counter belongs to
        self.session_count = 0

    def claim(self, day_folder, source_tag, wp):
        """Atomically reserves (seq, frame) for the next image."""
        with self.lock:
            if day_folder != self.day_folder:
                self.day_folder = day_folder
                self.seq = scan_max_seq(os.path.join(day_folder, self.subdirs[0]))
            if (source_tag, wp) != self.label:
                if self.label is None or self.label[0] != source_tag:
                    self.session_count = 0
                self.label = (source_tag, wp)
                self.frame = 0
            self.seq += 1
            self.frame += 1
            self.session_count += 1
            return self.seq, self.frame


class CaptureNode(dai.node.ThreadedHostNode):
    def __init__(self, data_dir, vehicle_ip, camera_angle, lenses, mavlink_node,
                 min_free_gb=2.0, host_encode_quality=None):
        dai.node.ThreadedHostNode.__init__(self)
        # Non-blocking, so a slow disk can never back-pressure the camera outputs VIO depends on
        self.inputCenter = self.createInput(blocking=False, queueSize=4)
        self.inputLeft = self.createInput(blocking=False, queueSize=4)
        self.inputRight = self.createInput(blocking=False, queueSize=4)

        self.data_dir = data_dir
        self.captures_dir = os.path.join(data_dir, "captures")
        self.control_path = os.path.join(data_dir, "capture_control.json")
        self.base_url = vehicle_telemetry.base_url(vehicle_ip)
        self.camera_angle = camera_angle
        self.lenses = lenses  # {"center": Lens, "left": Lens, "right": Lens}, any may be None
        self.mavlink_node = mavlink_node
        self.min_free_bytes = min_free_gb * 1e9
        # When set, stereo frames arrive as raw GRAY8 and are JPEG-encoded here at this quality
        self.host_encode_quality = host_encode_quality

        self.writers = {
            "center": StreamWriter("center", ["center"]),
            "stereo": StreamWriter("stereo", ["stereo/left", "stereo/right"]),
        }
        self.executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="capture")
        self.control = read_control(self.control_path)
        self._control_mtime = None
        self.pending_left = []
        self.pending_right = []
        self.stats_lock = threading.Lock()
        self.dropped = 0
        self.last_error = None
        self.disk_full = False
        self.free_gb = None

    # -- control ------------------------------------------------------------
    def _refresh_control(self):
        try:
            mtime = os.path.getmtime(self.control_path)
        except FileNotFoundError:
            mtime = None
        if mtime != self._control_mtime:
            self._control_mtime = mtime
            self.control = read_control(self.control_path)
            if not self.control.get("recording"):
                self.disk_full = False

    def _recording(self):
        return bool(self.control.get("recording")) and not self.disk_full

    # -- frame helpers ------------------------------------------------------
    def _jpeg_bytes(self, frame):
        if isinstance(frame, dai.EncodedFrame):
            return frame.getData().tobytes()
        # Raw GRAY8 fallback: encode on the Pi
        import io
        import numpy as np
        from PIL import Image
        width, height = frame.getWidth(), frame.getHeight()
        data = np.asarray(frame.getData(), dtype=np.uint8)
        stride = frame.getStride() or width
        pixels = data[:stride * height].reshape(height, stride)[:, :width]
        buf = io.BytesIO()
        Image.fromarray(pixels, mode="L").save(buf, format="JPEG", quality=self.host_encode_quality or 90)
        return buf.getvalue()

    @staticmethod
    def _capture_time(frame):
        """(local, utc) wall-clock time the frame was exposed, from its host-synced timestamp."""
        age = dai.Clock.now() - frame.getTimestamp()
        local = datetime.now() - age
        utc = datetime.now(timezone.utc) - age
        return local, utc

    def _check_space(self):
        try:
            os.makedirs(self.captures_dir, exist_ok=True)
            free = shutil.disk_usage(self.captures_dir).free
        except OSError:
            return True
        self.free_gb = free / 1e9
        if free < self.min_free_bytes:
            if not self.disk_full:
                logger.error(f"Capture stopped: only {self.free_gb:.1f} GB free")
                with self.stats_lock:
                    self.last_error = f"disk nearly full ({self.free_gb:.1f} GB free)"
            self.disk_full = True
            return False
        return True

    # -- writing ------------------------------------------------------------
    def _write_job(self, stream, frames, control):
        """Runs on the writer pool: label, stamp and write one image (or stereo pair)."""
        try:
            if not self._check_space():
                return
            ts_local, ts_utc = self._capture_time(frames[0])
            t0 = time.monotonic()
            telemetry = vehicle_telemetry.read_autopilot_telemetry(self.base_url)
            telem_ms = (time.monotonic() - t0) * 1000.0
            sync_skew_ms = (dai.Clock.now() - frames[0].getTimestamp()).total_seconds() * 1000.0

            vio = self.mavlink_node.snapshot() if self.mavlink_node else {}
            pose = vio.get("pose")
            quality = vio.get("quality")
            vio_tracking = (1.0 if quality["tracking"] else 0.0) if quality else None

            source_tag = control.get("source_tag") or "tl000000"
            wp = control.get("wp")
            day_folder = survey_day_folder(self.captures_dir, ts_local)
            writer = self.writers[stream]
            seq, frame_no = writer.claim(day_folder, source_tag, wp)
            filename = make_filename(seq, source_tag, wp, frame_no)

            cameras = ["center"] if stream == "center" else ["left", "right"]
            total_bytes = 0
            written = []
            for camera, subdir, frame in zip(cameras, writer.subdirs, frames):
                jpeg = self._jpeg_bytes(frame)
                jpeg = photogrammetry_meta.embed_metadata(
                    jpeg, telemetry, ts_local, ts_utc, lens=self.lenses.get(camera),
                    camera_angle_deg=self.camera_angle, wp=wp, vio_tracking=vio_tracking)
                folder = os.path.join(day_folder, subdir)
                os.makedirs(folder, exist_ok=True)
                path = os.path.join(folder, filename)
                try:
                    with open(path, "wb") as f:
                        f.write(jpeg)
                except OSError as e:
                    # A frame cut short by a full disk would trip up the photogrammetry run
                    if e.errno == errno.ENOSPC:
                        self.disk_full = True
                        for p in written + [path]:
                            try:
                                os.unlink(p)
                            except OSError:
                                pass
                    raise
                written.append(path)
                total_bytes += len(jpeg)

            csv_path = os.path.join(day_folder, f"telemetry_{stream}.csv")
            new_file = not os.path.exists(csv_path)
            with open(csv_path, "a", newline="") as f:
                w = csv.writer(f)
                if new_file:
                    w.writerow(CSV_HEADER)
                p = pose or (None,) * 7
                w.writerow([
                    ts_local.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3], seq, filename, source_tag,
                    wp or "", frame_no, total_bytes,
                    _fmt(telemetry["lat"], ".7f"), _fmt(telemetry["lon"], ".7f"),
                    _fmt(telemetry["heading_deg"], ".1f"), _fmt(telemetry["roll_deg"], ".1f"),
                    _fmt(telemetry["pitch_deg"], ".1f"), _fmt(telemetry["depth_m"], ".2f"),
                    _fmt(telemetry["temperature_c"], ".2f"), f"{telem_ms:.1f}", f"{sync_skew_ms:.1f}",
                    _fmt(telemetry["altitude_m"], ".2f"),
                    *[_fmt(v, ".4f") for v in p],
                    _fmt(vio_tracking, ".0f"),
                ])
        except Exception as e:
            logger.exception(f"Capture write failed ({stream})")
            with self.stats_lock:
                self.last_error = str(e)
                self.dropped += 1

    def _submit(self, stream, frames):
        if not self._recording():
            return
        # Snapshot the control now, so a label change can't land half-way through this frame
        self.executor.submit(self._write_job, stream, frames, dict(self.control))

    def _pair_stereo(self):
        now = dai.Clock.now()
        self.pending_left = [f for f in self.pending_left
                             if (now - f.getTimestamp()).total_seconds() < STEREO_PENDING_MAX_AGE_S]
        self.pending_right = [f for f in self.pending_right
                              if (now - f.getTimestamp()).total_seconds() < STEREO_PENDING_MAX_AGE_S]
        for left in list(self.pending_left):
            for right in self.pending_right:
                if abs((left.getTimestamp() - right.getTimestamp()).total_seconds()) <= STEREO_PAIR_TOLERANCE_S:
                    self.pending_left.remove(left)
                    self.pending_right.remove(right)
                    self._submit("stereo", [left, right])
                    break

    def run(self):
        logger.info("Capture node started")
        while self.isRunning():
            try:
                self._refresh_control()
                got = False
                frame = self.inputCenter.tryGet()
                if frame is not None:
                    got = True
                    self._submit("center", [frame])
                frame = self.inputLeft.tryGet()
                if frame is not None:
                    got = True
                    self.pending_left.append(frame)
                frame = self.inputRight.tryGet()
                if frame is not None:
                    got = True
                    self.pending_right.append(frame)
                if self.pending_left and self.pending_right:
                    self._pair_stereo()
                if not got:
                    time.sleep(0.02)
            except Exception as e:
                logger.error(f"Error in capture loop: {e}")
                time.sleep(0.1)
        self.executor.shutdown(wait=True)
        logger.info("Capture node stopped")

    def snapshot(self):
        with self.stats_lock:
            return {
                "mode": self.control.get("mode", "off"),
                "recording": self._recording(),
                "source_tag": self.control.get("source_tag"),
                "wp": self.control.get("wp"),
                "center_images": self.writers["center"].session_count,
                "stereo_pairs": self.writers["stereo"].session_count,
                "day_folder": os.path.basename(self.writers["center"].day_folder
                                               or self.writers["stereo"].day_folder or ""),
                "free_gb": round(self.free_gb, 2) if self.free_gb is not None else None,
                "disk_full": self.disk_full,
                "dropped": self.dropped,
                "error": self.last_error,
            }
