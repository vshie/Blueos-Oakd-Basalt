#!/usr/bin/env python3
"""
BlueOS OAK-D VIO Extension - web configuration, Cockpit widget and process supervision
"""

import io
import ipaddress
import json
import os
import shutil
import subprocess
import sys
import threading
import time
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Literal, Optional

import uvicorn
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from loguru import logger
from pydantic import BaseModel, field_validator, model_validator

import vehicle_telemetry
from autopilot_params import AutopilotParams, AutopilotUnreachable, NoHeartbeat, VehicleArmed
from transect import TransectMonitor, make_source_tag

APP_DIR = Path(__file__).resolve().parent
STATIC_DIR = APP_DIR / "static"
# Persistent across restarts and updates (bind-mounted from /usr/blueos/extensions/oakd-vio)
DATA_DIR = Path(os.environ.get("OAKD_DATA_DIR", "/app/data"))
CONFIG_FILE = DATA_DIR / "config.json"
STATE_FILE = DATA_DIR / "state.json"  # not touched by Save / Reset Defaults
CONTROL_FILE = DATA_DIR / "capture_control.json"
TELEMETRY_FILE = DATA_DIR / "telemetry.json"
IMU_REPORT_FILE = DATA_DIR / "imu_report.json"
CAPTURES_DIR = DATA_DIR / "captures"
EXTENSION_VERSION = "0.0.3"

CENTER_RESOLUTIONS = ("4056x3040", "3840x2160", "1920x1080")
MONO_RESOLUTIONS = ("1280x800", "640x400")

app = FastAPI(title="OAK-D VIO Configuration")

# VIO frame rate each backend starts from. Basalt runs all of VIO on the Pi but fuses the IMU
# tightly, so it copes with a low rate. RTAB-Map loses tracking around 3 fps, and its odometry is
# one Pi thread that manages ~10 fps on a Pi 4; asking the camera for more only sends frames
# the Pi drops. Mirrored in static/index.html (BACKEND_DEFAULT_FPS).
BACKEND_DEFAULT_FPS = {"basalt": 3, "rtabmap": 10}

# Default configuration
DEFAULT_CONFIG = {
    "fps": BACKEND_DEFAULT_FPS["basalt"],
    "camera_angle": 0.0,
    "camera_ip": "",
    "vehicle_ip": "192.168.2.2",
    "quality_rate": 1.0,
    "resolution_width": 320,
    "resolution_height": 200,
    "vio_backend": "basalt",
    # VIO tuning parameters (Basalt)
    "optical_flow_grid_size": 80,
    "optical_flow_max_iterations": 3,
    "optical_flow_levels": 2,
    "vio_max_states": 2,
    "vio_max_kfs": 4,
    "vio_max_iterations": 3,
    "optical_flow_skip_frames": 1,
    # RTAB-Map
    "rtabmap_features": 500,
    "rtabmap_local_map_size": 1000,
    # Image capture
    "center_capture_fps": 2.0,
    "stereo_capture_fps": 2.0,
    "center_resolution": "1920x1080",
    "mono_resolution": "1280x800",
    "jpeg_quality": 90,
    "mono_host_encode": False,
    "capture_min_free_gb": 2.0,
}


class VIOConfig(BaseModel):
    fps: int = 3
    camera_angle: float = 0.0
    camera_ip: str = ""
    vehicle_ip: str = "192.168.2.2"
    quality_rate: float = 1.0
    resolution_width: int = 320
    resolution_height: int = 200
    vio_backend: Literal["basalt", "rtabmap"] = "basalt"
    optical_flow_grid_size: int = 80
    optical_flow_max_iterations: int = 3
    optical_flow_levels: int = 2
    vio_max_states: int = 2
    vio_max_kfs: int = 4
    vio_max_iterations: int = 3
    optical_flow_skip_frames: int = 1
    rtabmap_features: int = 500
    rtabmap_local_map_size: int = 1000
    center_capture_fps: float = 2.0
    stereo_capture_fps: float = 2.0
    center_resolution: Literal[CENTER_RESOLUTIONS] = "1920x1080"
    mono_resolution: Literal[MONO_RESOLUTIONS] = "1280x800"
    jpeg_quality: int = 90
    mono_host_encode: bool = False
    capture_min_free_gb: float = 2.0

    @field_validator("camera_ip")
    @classmethod
    def _camera_ip(cls, value):
        value = value.strip()
        if value:
            ipaddress.ip_address(value)
        return value

    @field_validator("jpeg_quality")
    @classmethod
    def _quality(cls, value):
        if not 1 <= value <= 100:
            raise ValueError("JPEG quality must be 1-100")
        return value

    @field_validator("rtabmap_local_map_size")
    @classmethod
    def _local_map_size(cls, value):
        if not 200 <= value <= 5000:
            raise ValueError("RTAB-Map local map size must be 200-5000")
        return value

    @field_validator("center_capture_fps", "stereo_capture_fps")
    @classmethod
    def _capture_fps(cls, value):
        if value < 0:
            raise ValueError("Capture rate can't be negative (0 turns the stream off)")
        return value

    @model_validator(mode="before")
    @classmethod
    def _backend_default_fps(cls, data):
        # A config posted without a frame rate gets its backend's default, not Basalt's
        if isinstance(data, dict) and data.get("fps") is None:
            data = {**data, "fps": BACKEND_DEFAULT_FPS.get(data.get("vio_backend", "basalt"), 3)}
        return data

    # A stereo capture rate above the VIO frame rate isn't an error: stereo pairs come from the
    # sensors that feed VIO, so the VIO process caps it at the VIO rate when it builds the
    # pipeline. Keeping the user's value means switching backend (and so frame rate) back and
    # forth always saves, and restores the capture rate they chose.


# VIO process handle
vio_process: Optional[subprocess.Popen] = None
vio_lock = threading.RLock()


def load_json(path, default):
    try:
        with open(path) as f:
            return json.load(f)
    except (FileNotFoundError, ValueError):
        return default


def write_json(path, data):
    tmp = Path(str(path) + ".tmp")
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, path)


def load_config() -> dict:
    """Load configuration from file or return defaults"""
    return {**DEFAULT_CONFIG, **load_json(CONFIG_FILE, {})}


def save_config(config: dict):
    """Save configuration to file"""
    write_json(CONFIG_FILE, config)


def load_state() -> dict:
    return load_json(STATE_FILE, {})


def update_state(**changes):
    state = load_state()
    state.update(changes)
    write_json(STATE_FILE, state)


def vio_running():
    return vio_process is not None and vio_process.poll() is None


# ---------------------------------------------------------------------------
# Autopilot parameters: checked every start, written automatically only the first time
# ---------------------------------------------------------------------------
param_status = {"state": "unknown", "diffs": [], "needs_reboot": [], "message": None,
                "checked_at": None, "origin": None}
param_lock = threading.Lock()


def _set_param_status(**changes):
    with param_lock:
        param_status.update(changes, checked_at=time.time())


def check_params(apply_confirmed=False):
    """Checks the EKF parameters; applies them on the first ever run or when the user confirmed."""
    config = load_config()
    autopilot = AutopilotParams(config["vehicle_ip"])
    try:
        diffs = autopilot.check()
    except AutopilotUnreachable as e:
        logger.warning(str(e))
        _set_param_status(state="unreachable", diffs=[], message=str(e))
        return
    except NoHeartbeat as e:
        logger.warning(f"No autopilot HEARTBEAT seen, can't check parameters: {e}")
        _set_param_status(state="no_heartbeat", diffs=[], message="No autopilot HEARTBEAT seen yet")
        return
    except Exception as e:
        logger.exception("Parameter check failed")
        _set_param_status(state="error", diffs=[], message=str(e))
        return

    try:
        origin = autopilot.get_origin()
    except Exception:
        origin = None
    origin = {"lat": origin[0], "lon": origin[1]} if origin else None

    first_run = not load_state().get("params_auto_applied", False)
    if not diffs:
        if first_run:
            update_state(params_auto_applied=True)
        _set_param_status(state="ok", diffs=[], message="All parameters set", origin=origin)
        return
    if not (first_run or apply_confirmed):
        logger.warning("Autopilot parameters differ from what VIO needs; confirm on the configuration page")
        _set_param_status(state="pending", diffs=diffs, origin=origin,
                          message="Parameters differ from what VIO needs. Review and apply.")
        return
    try:
        results, needs_reboot = autopilot.apply(diffs)
    except VehicleArmed as e:
        logger.warning(str(e))
        _set_param_status(state="armed", diffs=diffs, origin=origin, message=str(e))
        return
    failed = [r for r in results if not r["ok"]]
    if first_run and not failed:
        update_state(params_auto_applied=True)
    message = "Parameters written." + (" Reboot the autopilot for " + ", ".join(needs_reboot) + " to take effect."
                                       if needs_reboot else "")
    if failed:
        message = "Some parameters could not be written: " + ", ".join(r["name"] for r in failed)
    _set_param_status(state="error" if failed else "applied", diffs=diffs if failed else [],
                      needs_reboot=needs_reboot, origin=origin, message=message)


def check_params_async(apply_confirmed=False):
    threading.Thread(target=check_params, args=(apply_confirmed,), daemon=True, name="params").start()


# ---------------------------------------------------------------------------
# VIO process
# ---------------------------------------------------------------------------
def start_vio():
    """Start the VIO process"""
    global vio_process
    with vio_lock:
        if imu_job_running():
            raise HTTPException(409, "An IMU tool is using the camera; wait for it to finish")
        config = load_config()

        # Kill existing process if running
        stop_vio()

        center_w, center_h = config["center_resolution"].split("x")
        mono_w, mono_h = config["mono_resolution"].split("x")
        # Build command with config
        cmd = [
            sys.executable, str(APP_DIR / "basalt_configurable.py"),
            "--fps", str(config["fps"]),
            "--camera-angle", str(config["camera_angle"]),
            "--camera-ip", config["camera_ip"],
            "--vehicle-ip", config["vehicle_ip"],
            "--quality-rate", str(config["quality_rate"]),
            "--width", str(config["resolution_width"]),
            "--height", str(config["resolution_height"]),
            "--backend", config["vio_backend"],
            "--grid-size", str(config["optical_flow_grid_size"]),
            "--flow-iterations", str(config["optical_flow_max_iterations"]),
            "--flow-levels", str(config["optical_flow_levels"]),
            "--max-states", str(config["vio_max_states"]),
            "--max-kfs", str(config["vio_max_kfs"]),
            "--vio-iterations", str(config["vio_max_iterations"]),
            "--skip-frames", str(config["optical_flow_skip_frames"]),
            "--rtabmap-features", str(config["rtabmap_features"]),
            "--rtabmap-local-map-size", str(config.get("rtabmap_local_map_size", 1000)),
            "--center-capture-fps", str(config["center_capture_fps"]),
            "--stereo-capture-fps", str(config["stereo_capture_fps"]),
            "--center-width", center_w, "--center-height", center_h,
            "--mono-width", mono_w, "--mono-height", mono_h,
            "--jpeg-quality", str(config["jpeg_quality"]),
            "--min-free-gb", str(config["capture_min_free_gb"]),
        ]
        if config["mono_host_encode"]:
            cmd.append("--mono-host-encode")

        try:
            TELEMETRY_FILE.unlink()
        except FileNotFoundError:
            pass
        vio_process = subprocess.Popen(cmd, cwd=str(APP_DIR))
        print(f"Started VIO process with PID {vio_process.pid}")
    check_params_async()


def stop_vio():
    """Stop the VIO process"""
    global vio_process
    with vio_lock:
        if vio_process and vio_process.poll() is None:
            vio_process.terminate()
            try:
                vio_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                vio_process.kill()
            print("Stopped VIO process")
        vio_process = None


# ---------------------------------------------------------------------------
# Image capture: Record (manual) / Auto (transect) / Stop
# ---------------------------------------------------------------------------
capture_lock = threading.Lock()
capture_mode = "off"
transect_monitor: Optional[TransectMonitor] = None


def write_control(recording, source_tag=None, wp=None):
    write_json(CONTROL_FILE, {"mode": capture_mode, "recording": recording,
                              "source_tag": source_tag, "wp": wp, "updated": time.time()})


def _on_transect_change(recording, source_tag, wp):
    if capture_mode == "auto":
        write_control(recording, source_tag, wp)


def _new_transect_monitor():
    return TransectMonitor(load_config()["vehicle_ip"], str(CAPTURES_DIR), _on_transect_change)


def set_capture_mode(mode):
    global capture_mode, transect_monitor
    with capture_lock:
        if transect_monitor is not None and mode != "auto":
            transect_monitor.disable()
        capture_mode = mode
        if mode == "manual":
            write_control(True, make_source_tag("tl"), None)
        elif mode == "auto":
            write_control(False)
            if transect_monitor is None:
                transect_monitor = _new_transect_monitor()
            transect_monitor.enable()
        else:
            write_control(False)
        # An enabled Auto survives restarts, like TowFish; Record never does
        update_state(transect_enabled=(mode == "auto"))
    logger.info(f"Capture mode: {mode}")


def capture_status():
    status = {"mode": capture_mode, "transect": transect_monitor.status() if transect_monitor else None,
              "control": load_json(CONTROL_FILE, {})}
    try:
        CAPTURES_DIR.mkdir(parents=True, exist_ok=True)
        status["free_gb"] = round(shutil.disk_usage(CAPTURES_DIR).free / 1e9, 2)
    except OSError:
        status["free_gb"] = None
    return status


# ---------------------------------------------------------------------------
# IMU tools (need the camera, so only while VIO is stopped)
# ---------------------------------------------------------------------------
imu_job = {"action": None, "state": "idle", "result": None, "started": None, "log": ""}
imu_lock = threading.Lock()


def imu_job_running():
    return imu_job["state"] == "running"


def _run_imu_tool(action, args, timeout):
    config = load_config()
    cmd = [sys.executable, str(APP_DIR / "imu_tools.py"), action]
    if action != "discover":
        cmd += ["--camera-ip", config["camera_ip"]]
    cmd += args
    proc = subprocess.run(cmd, cwd=str(APP_DIR), capture_output=True, text=True, timeout=timeout)
    lines = [line for line in proc.stdout.strip().splitlines() if line.strip()]
    try:
        result = json.loads(lines[-1])
    except (IndexError, ValueError):
        result = {"ok": False, "error": (proc.stderr.strip().splitlines() or ["no output"])[-1]}
    return result, proc.stderr


def start_imu_job(action, args, timeout):
    with imu_lock:
        if imu_job_running():
            raise HTTPException(409, f"IMU tool '{imu_job['action']}' is already running")
        if vio_running():
            raise HTTPException(409, "Stop VIO first: the camera accepts only one connection")
        imu_job.update(action=action, state="running", result=None, started=time.time(), log="")

    def run():
        try:
            result, log = _run_imu_tool(action, args, timeout)
        except subprocess.TimeoutExpired:
            result, log = {"ok": False, "error": "timed out"}, ""
        if action in ("detect", "static-test") and result.get("ok"):
            report = load_json(IMU_REPORT_FILE, {})
            report[action] = {**result, "time": time.time()}
            write_json(IMU_REPORT_FILE, report)
        imu_job.update(state="done", result=result, log=log[-4000:])

    threading.Thread(target=run, daemon=True, name="imu-tool").start()


# ---------------------------------------------------------------------------
# Pages, BlueOS registration and the Cockpit widget
# ---------------------------------------------------------------------------
@app.get("/")
async def index():
    """Serve the configuration UI"""
    return FileResponse(STATIC_DIR / "index.html")


@app.get("/widget")
async def widget():
    """Compact panel for Cockpit (iframe widget)"""
    return FileResponse(STATIC_DIR / "widget.html")


@app.get("/register_service")
async def register_service():
    return {
        "name": "OAK-D VIO",
        "description": "Visual-inertial odometry and survey image capture for the MarineSitu C3 / OAK-D",
        "icon": "mdi-camera-marker",
        "company": "Blue Robotics",
        "version": EXTENSION_VERSION,
        "webpage": "https://github.com/williangalvani/Blueos-Oakd-Basalt",
        "api": "https://github.com/bluerobotics/BlueOS-docker",
        "extras": {"cockpit": "/cockpit_extras.json"},
    }


@app.get("/cockpit_extras.json")
async def cockpit_extras(request: Request):
    """Offer the widget to a Cockpit connected to this vehicle.

    The widget fetches root paths (/api/...), so it must load from this service's own port rather
    than through BlueOS's /extensionv2/ proxy. Cockpit reached this file on that port, so the
    request's base URL is an address it can also iframe. Cockpit is served from port 80, which
    makes this fetch cross-origin, hence the CORS header.
    """
    base = str(request.base_url).rstrip("/")
    return JSONResponse({
        "targetCockpitApiVersion": "1.0.0",
        "targetSystem": "Cockpit",
        "widgets": [{
            "name": "OAK VIO",
            "iframeUrl": f"{base}/widget",
            "iconUrl": f"{base}/static/icon.svg",
        }],
    }, headers={"Access-Control-Allow-Origin": "*"})


# ---------------------------------------------------------------------------
# Configuration and VIO control
# ---------------------------------------------------------------------------
@app.get("/api/config")
async def get_config():
    """Get current configuration"""
    return JSONResponse(load_config())


@app.post("/api/config")
def set_config(config: VIOConfig):
    """Save configuration and restart VIO"""
    global transect_monitor
    old_ip = load_config()["vehicle_ip"]
    save_config(config.model_dump())
    if config.vehicle_ip != old_ip and transect_monitor is not None:
        with capture_lock:
            enabled = transect_monitor.status()["state"] != "disabled"
            transect_monitor.disable()
            transect_monitor = _new_transect_monitor()
            if enabled:
                transect_monitor.enable()
    start_vio()
    return {"status": "ok", "message": "Configuration saved and VIO restarted"}


@app.get("/api/status")
async def get_status():
    """Get VIO process status"""
    if vio_running():
        return {"running": True, "pid": vio_process.pid}
    exit_code = vio_process.returncode if vio_process is not None else None
    return {"running": False, "pid": None, "exit_code": exit_code}


@app.post("/api/start")
def api_start():
    """Start VIO"""
    start_vio()
    return {"status": "ok"}


@app.post("/api/stop")
def api_stop():
    """Stop VIO"""
    stop_vio()
    return {"status": "ok"}


@app.post("/api/reset")
def api_reset():
    """Reset to default configuration"""
    save_config(DEFAULT_CONFIG.copy())
    start_vio()
    return {"status": "ok"}


# ---------------------------------------------------------------------------
# Autopilot parameters
# ---------------------------------------------------------------------------
@app.get("/api/params")
async def get_params():
    with param_lock:
        return {**param_status, "auto_applied_before": load_state().get("params_auto_applied", False)}


@app.post("/api/params/check")
def api_params_check():
    check_params()
    return get_params_sync()


@app.post("/api/params/apply")
def api_params_apply():
    check_params(apply_confirmed=True)
    status = get_params_sync()
    if status["state"] == "armed":
        raise HTTPException(409, "Vehicle is armed; disarm to change parameters")
    return status


def get_params_sync():
    with param_lock:
        return dict(param_status)


# ---------------------------------------------------------------------------
# Telemetry for the widget
# ---------------------------------------------------------------------------
@app.get("/api/telemetry")
def api_telemetry():
    vio = load_json(TELEMETRY_FILE, None)
    running = vio_running()
    if running and vio is None:
        vio = {"running": True, "starting": True}
    elif running and time.time() - vio.get("time", 0) > 5:
        vio["stale"] = True
    if not running:
        error = vio.get("error") if vio else None
        vio = {"running": False, "error": error,
               "exit_code": vio_process.returncode if vio_process is not None else None}
    config = load_config()
    vehicle = vehicle_telemetry.read_autopilot_telemetry(vehicle_telemetry.base_url(config["vehicle_ip"]))
    return JSONResponse({"time": time.time(), "vio": vio, "params": get_params_sync(),
                         "capture": capture_status(), "vehicle": vehicle,
                         "config": {"vio_backend": config["vio_backend"], "camera_ip": config["camera_ip"]}},
                        headers={"Access-Control-Allow-Origin": "*"})


# ---------------------------------------------------------------------------
# Capture control and downloads
# ---------------------------------------------------------------------------
@app.post("/api/capture/record")
def api_capture_record():
    set_capture_mode("manual")
    return capture_status()


@app.post("/api/capture/auto")
def api_capture_auto():
    set_capture_mode("auto")
    return capture_status()


@app.post("/api/capture/stop")
def api_capture_stop():
    set_capture_mode("off")
    return capture_status()


@app.get("/api/capture/status")
def api_capture_status():
    return capture_status()


def _folder_summary(folder: Path):
    size = 0
    jpgs = 0
    pngs = 0
    for root, _dirs, files in os.walk(folder):
        for name in files:
            try:
                size += os.path.getsize(os.path.join(root, name))
            except OSError:
                continue
            jpgs += name.endswith(".jpg")
            pngs += name.endswith(".png")
    return {"name": folder.name, "size_mb": round(size / 1e6, 1), "jpg": jpgs, "png": pngs,
            "modified": folder.stat().st_mtime}


@app.get("/api/captures")
def api_captures():
    CAPTURES_DIR.mkdir(parents=True, exist_ok=True)
    folders = [_folder_summary(p) for p in CAPTURES_DIR.iterdir() if p.is_dir()]
    return sorted(folders, key=lambda f: f["modified"], reverse=True)


@app.get("/api/captures/{name}/zip")
def api_capture_zip(name: str):
    folder = (CAPTURES_DIR / name).resolve()
    if folder.parent != CAPTURES_DIR.resolve() or not folder.is_dir():
        raise HTTPException(404, "No such capture folder")

    class Pipe(io.RawIOBase):
        """Write-only buffer that zipfile streams into; chunks are drained as they're produced."""
        def __init__(self):
            self.chunks = []

        def writable(self):
            return True

        def write(self, data):
            self.chunks.append(bytes(data))
            return len(data)

    def generate():
        pipe = Pipe()
        # JPEG/PNG don't compress further, so store them and spare the Pi's CPU
        with zipfile.ZipFile(pipe, "w", compression=zipfile.ZIP_STORED) as archive:
            for root, _dirs, files in os.walk(folder):
                for filename in sorted(files):
                    path = os.path.join(root, filename)
                    archive.write(path, os.path.relpath(path, folder.parent))
                    yield from pipe.chunks
                    pipe.chunks.clear()
        yield from pipe.chunks

    return StreamingResponse(generate(), media_type="application/zip",
                             headers={"Content-Disposition": f'attachment; filename="{name}.zip"'})


# ---------------------------------------------------------------------------
# Camera discovery and IMU tools
# ---------------------------------------------------------------------------
@app.get("/api/devices")
def api_devices():
    try:
        result, _log = _run_imu_tool("discover", [], timeout=20)
    except subprocess.TimeoutExpired:
        raise HTTPException(504, "Discovery timed out")
    result["vio_running"] = vio_running()
    return result


@app.get("/api/imu")
def api_imu():
    return {"job": imu_job, "report": load_json(IMU_REPORT_FILE, {}), "vio_running": vio_running()}


@app.post("/api/imu/detect")
def api_imu_detect():
    start_imu_job("detect", [], timeout=30)
    return api_imu()


@app.post("/api/imu/static-test")
def api_imu_static_test(seconds: float = 30.0):
    start_imu_job("static-test", ["--seconds", str(seconds)], timeout=seconds + 30)
    return api_imu()


@app.post("/api/imu/record-euroc")
def api_imu_record_euroc(seconds: float = 90.0):
    out = CAPTURES_DIR / f"calib-{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}"
    start_imu_job("record-euroc", ["--seconds", str(seconds), "--out", str(out)], timeout=seconds + 60)
    return api_imu()


class FlashRequest(BaseModel):
    yaml: str
    confirm: bool = False


@app.post("/api/imu/flash-extrinsics")
def api_imu_flash(request: FlashRequest):
    if not request.confirm:
        raise HTTPException(400, "Confirmation required to write the camera calibration")
    stamp = f"{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}"
    yaml_path = DATA_DIR / f"camchain-imucam-{stamp}.yaml"
    yaml_path.write_text(request.yaml)
    backup = DATA_DIR / f"calib-backup-{stamp}.json"
    start_imu_job("flash-extrinsics", ["--yaml", str(yaml_path), "--backup", str(backup), "--confirm"], timeout=60)
    return api_imu()


# ---------------------------------------------------------------------------
# Startup / shutdown
# ---------------------------------------------------------------------------
@app.on_event("startup")
async def startup_event():
    """Recording is always off at startup; an enabled Auto is restored; VIO starts."""
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    CAPTURES_DIR.mkdir(parents=True, exist_ok=True)
    set_capture_mode("auto" if load_state().get("transect_enabled") else "off")
    start_vio()


@app.on_event("shutdown")
async def shutdown_event():
    """Stop VIO on server shutdown"""
    stop_vio()


DATA_DIR.mkdir(parents=True, exist_ok=True)
CAPTURES_DIR.mkdir(parents=True, exist_ok=True)
app.mount("/captures", StaticFiles(directory=str(CAPTURES_DIR)), name="captures")
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", 8095)))
