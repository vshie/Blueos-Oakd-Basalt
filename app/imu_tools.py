#!/usr/bin/env python3
"""
Camera discovery and IMU checks for the OAK in the C3, run by main.py while VIO is stopped (the
camera accepts only one connection). Each command prints one JSON object on its last stdout line.

    discover                               list cameras on the network / USB
    detect          [--camera-ip IP]       IMU model, firmware and IMU-to-camera extrinsics
    static-test     [--camera-ip IP] [--seconds N]   gyro bias / accel norm / noise, vehicle still
    record-euroc    [--camera-ip IP] [--seconds N] --out DIR   stereo + IMU dataset for Kalibr
    flash-extrinsics [--camera-ip IP] --yaml FILE --backup FILE --confirm
                                           write Kalibr's T_cam_imu to the camera's EEPROM
"""

import argparse
import csv
import json
import math
import os
import sys
import time

import depthai as dai

GRAVITY = 9.80665
GYRO_BIAS_WARN_DPS = 0.5
ACCEL_NORM_WARN = 0.3
# Generous bound for the IMU's position relative to the left camera on an OAK-D W
EXTRINSIC_MAX_TRANSLATION_CM = 10.0


def emit(result):
    print(json.dumps(result))
    sys.stdout.flush()


def progress(text):
    print(text, file=sys.stderr)
    sys.stderr.flush()


def open_device(camera_ip):
    return dai.Device(camera_ip) if camera_ip else dai.Device()


def rotation_to_euler_deg(r):
    """ZYX (yaw, pitch, roll) of a 3x3 rotation matrix, in degrees."""
    pitch = math.asin(max(-1.0, min(1.0, -r[2][0])))
    roll = math.atan2(r[2][1], r[2][2])
    yaw = math.atan2(r[1][0], r[0][0])
    return {"roll": math.degrees(roll), "pitch": math.degrees(pitch), "yaw": math.degrees(yaw)}


def cmd_discover(args):
    devices = []
    for info in dai.Device.getAllAvailableDevices():
        devices.append({"name": info.name, "device_id": info.deviceId,
                        "state": info.state.name, "protocol": info.protocol.name})
    emit({"ok": True, "devices": devices})


def cmd_detect(args):
    with open_device(args.camera_ip) as device:
        result = {"ok": True, "camera": device.getProductName() or device.getDeviceName(),
                  "camera_id": device.getDeviceId()}
        imu = device.getConnectedIMU()
        result["imu"] = imu if imu and imu.upper() != "NONE" else None
        try:
            result["imu_firmware"] = str(device.getIMUFirmwareVersion())
        except Exception:
            result["imu_firmware"] = None

        extrinsics = {"present": False, "warnings": []}
        try:
            m = device.readCalibration().getImuToCameraExtrinsics(dai.CameraBoardSocket.CAM_B)
            r = [row[:3] for row in m[:3]]
            t = [m[0][3], m[1][3], m[2][3]]  # centimetres
            extrinsics.update(present=True, translation_cm=t, rotation_deg=rotation_to_euler_deg(r))
            is_identity = all(abs(r[i][j] - (1.0 if i == j else 0.0)) < 1e-6 for i in range(3) for j in range(3))
            if is_identity and all(abs(v) < 1e-6 for v in t):
                extrinsics["warnings"].append("Identity transform: the IMU extrinsics look unset")
            if math.sqrt(sum(v * v for v in t)) > EXTRINSIC_MAX_TRANSLATION_CM:
                extrinsics["warnings"].append(
                    f"IMU is more than {EXTRINSIC_MAX_TRANSLATION_CM:g} cm from the left camera, which is implausible")
        except Exception as e:
            extrinsics["warnings"].append(f"No IMU extrinsics in the calibration: {e}")
        result["extrinsics"] = extrinsics
        if result["imu"] is None:
            result["warnings"] = ["No IMU detected: Basalt VIO cannot run on this camera"]
        emit(result)


def _imu_pipeline(p):
    imu = p.create(dai.node.IMU)
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)
    return imu


def _mean_std(values):
    n = len(values)
    mean = sum(values) / n
    return mean, math.sqrt(sum((v - mean) ** 2 for v in values) / n)


def cmd_static_test(args):
    with open_device(args.camera_ip) as device:
        imu_type = device.getConnectedIMU()
        with dai.Pipeline(device) as p:
            queue = _imu_pipeline(p).out.createOutputQueue(maxSize=50, blocking=False)
            p.start()
            gyro, accel = [], []
            start = time.time()
            progress(f"Recording IMU for {args.seconds:g} s, keep the vehicle still")
            while time.time() - start < args.seconds:
                data = queue.tryGet()
                if data is None:
                    time.sleep(0.005)
                    continue
                for packet in data.packets:
                    g, a = packet.gyroscope, packet.acceleroMeter
                    gyro.append((g.x, g.y, g.z))
                    accel.append((a.x, a.y, a.z))
            elapsed = time.time() - start

    if len(gyro) < 10:
        emit({"ok": False, "error": "No IMU data received"})
        return
    axes = "xyz"
    result = {"ok": True, "imu": imu_type, "samples": len(gyro),
              "sample_rate_hz": round(len(gyro) / elapsed, 1), "gyro": {}, "accel": {}, "warnings": []}
    for i, axis in enumerate(axes):
        mean, std = _mean_std([s[i] for s in gyro])
        result["gyro"][axis] = {"bias_dps": round(math.degrees(mean), 3), "std_dps": round(math.degrees(std), 3)}
        if abs(math.degrees(mean)) > GYRO_BIAS_WARN_DPS:
            result["warnings"].append(f"Gyro {axis} bias {math.degrees(mean):.2f} deg/s is high")
        mean, std = _mean_std([s[i] for s in accel])
        result["accel"][axis] = {"mean": round(mean, 3), "std": round(std, 4)}
    norm, _ = _mean_std([math.sqrt(x * x + y * y + z * z) for x, y, z in accel])
    result["accel_norm"] = round(norm, 3)
    if abs(norm - GRAVITY) > ACCEL_NORM_WARN:
        result["warnings"].append(f"Accelerometer reads {norm:.2f} m/s² at rest (expected {GRAVITY:.2f})")
    if result["sample_rate_hz"] < 150:
        result["warnings"].append(f"IMU rate {result['sample_rate_hz']} Hz is below the 200 Hz requested")
    result["pass"] = not result["warnings"]
    emit(result)


def _ns(timestamp):
    return int(round(timestamp.total_seconds() * 1e9))


def cmd_record_euroc(args):
    """Stereo PNGs + IMU in EuRoC layout (mav0/cam0, cam1, imu0), as Kalibr's bag creator expects."""
    import numpy as np
    from PIL import Image

    root = os.path.join(args.out, "mav0")
    cams = {}
    for name in ("cam0", "cam1"):
        os.makedirs(os.path.join(root, name, "data"), exist_ok=True)
    os.makedirs(os.path.join(root, "imu0"), exist_ok=True)

    with open_device(args.camera_ip) as device:
        with dai.Pipeline(device) as p:
            queues = {}
            for name, socket in (("cam0", dai.CameraBoardSocket.CAM_B), ("cam1", dai.CameraBoardSocket.CAM_C)):
                cam = p.create(dai.node.Camera).build(socket, sensorFps=float(args.fps))
                out = cam.requestOutput((args.width, args.height), dai.ImgFrame.Type.GRAY8, fps=float(args.fps))
                queues[name] = out.createOutputQueue(maxSize=30, blocking=False)
            imu_queue = _imu_pipeline(p).out.createOutputQueue(maxSize=100, blocking=False)
            p.start()

            for name in ("cam0", "cam1"):
                cams[name] = open(os.path.join(root, name, "data.csv"), "w", newline="")
                cams[name].write("#timestamp [ns],filename\n")
            imu_file = open(os.path.join(root, "imu0", "data.csv"), "w", newline="")
            imu_csv = csv.writer(imu_file)
            imu_file.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                           "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
            counts = {"cam0": 0, "cam1": 0, "imu": 0}
            progress(f"Recording {args.seconds:g} s of stereo + IMU")
            start = time.time()
            try:
                while time.time() - start < args.seconds:
                    idle = True
                    for name, queue in queues.items():
                        frame = queue.tryGet()
                        if frame is None:
                            continue
                        idle = False
                        ts = _ns(frame.getTimestamp())
                        width, height = frame.getWidth(), frame.getHeight()
                        stride = frame.getStride() or width
                        data = np.asarray(frame.getData(), dtype=np.uint8)
                        pixels = data[:stride * height].reshape(height, stride)[:, :width]
                        filename = f"{ts}.png"
                        Image.fromarray(pixels, mode="L").save(
                            os.path.join(root, name, "data", filename), compress_level=1)
                        cams[name].write(f"{ts},{filename}\n")
                        counts[name] += 1
                    data = imu_queue.tryGet()
                    if data is not None:
                        idle = False
                        for packet in data.packets:
                            g, a = packet.gyroscope, packet.acceleroMeter
                            imu_csv.writerow([_ns(g.getTimestamp()), g.x, g.y, g.z, a.x, a.y, a.z])
                            counts["imu"] += 1
                    if idle:
                        time.sleep(0.002)
            finally:
                for f in cams.values():
                    f.close()
                imu_file.close()
    emit({"ok": True, "folder": os.path.basename(args.out.rstrip("/")), "counts": counts})


def cmd_flash_extrinsics(args):
    import yaml

    if not args.confirm:
        emit({"ok": False, "error": "Refusing to write the camera calibration without --confirm"})
        return
    with open(args.yaml) as f:
        camchain = yaml.safe_load(f)
    try:
        t_cam_imu = camchain["cam0"]["T_cam_imu"]
    except (KeyError, TypeError):
        emit({"ok": False, "error": "cam0.T_cam_imu not found; upload Kalibr's camchain-imucam.yaml"})
        return
    rotation = [list(map(float, row[:3])) for row in t_cam_imu[:3]]
    translation_cm = [float(t_cam_imu[i][3]) * 100.0 for i in range(3)]  # Kalibr metres -> depthai cm

    with open_device(args.camera_ip) as device:
        calib = device.readCalibration()
        if not calib.eepromToJsonFile(args.backup):
            emit({"ok": False, "error": f"Could not back up the calibration to {args.backup}; nothing written"})
            return
        progress(f"Backed up calibration to {args.backup}")
        # Keep the board's design (spec) translation; only the measured transform changes
        try:
            spec = calib.getImuToCameraExtrinsics(dai.CameraBoardSocket.CAM_B, True)
            spec_translation = [spec[0][3], spec[1][3], spec[2][3]]
        except Exception:
            spec_translation = translation_cm
        calib.setImuExtrinsics(dai.CameraBoardSocket.CAM_B, rotation, translation_cm, spec_translation)
        device.flashCalibration(calib)
        check = device.readCalibration().getImuToCameraExtrinsics(dai.CameraBoardSocket.CAM_B)
    emit({"ok": True, "backup": os.path.basename(args.backup),
          "translation_cm": [check[0][3], check[1][3], check[2][3]],
          "rotation_deg": rotation_to_euler_deg([row[:3] for row in check[:3]])})


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("discover")
    for name in ("detect", "static-test", "record-euroc", "flash-extrinsics"):
        p = sub.add_parser(name)
        p.add_argument("--camera-ip", default="")
        if name == "static-test":
            p.add_argument("--seconds", type=float, default=30.0)
        if name == "record-euroc":
            p.add_argument("--seconds", type=float, default=90.0)
            p.add_argument("--fps", type=float, default=20.0)
            p.add_argument("--width", type=int, default=640)
            p.add_argument("--height", type=int, default=400)
            p.add_argument("--out", required=True)
        if name == "flash-extrinsics":
            p.add_argument("--yaml", required=True)
            p.add_argument("--backup", required=True)
            p.add_argument("--confirm", action="store_true")
    args = parser.parse_args()
    try:
        {"discover": cmd_discover, "detect": cmd_detect, "static-test": cmd_static_test,
         "record-euroc": cmd_record_euroc, "flash-extrinsics": cmd_flash_extrinsics}[args.command](args)
    except Exception as e:
        emit({"ok": False, "error": str(e)})
        sys.exit(1)


if __name__ == "__main__":
    main()
