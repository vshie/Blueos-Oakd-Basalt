#!/usr/bin/env python3
"""
BlueOS OAK-D VIO with configurable parameters, started and stopped by main.py.

Backends:
- basalt:  Basalt VIO on the Pi, fed by the stereo pair and IMU.
- rtabmap: stereo depth and feature tracking run on the OAK; only RTAB-Map's pose estimation
           runs on the Pi.

Optionally also captures centre-colour and stereo JPEGs (encoded on the OAK) for surveys.
"""

import argparse
import json
import os
import sys
import threading
import time

import depthai as dai
from loguru import logger

from capture_node import CaptureNode
from mavlink_node import MavlinkNode
from photogrammetry_meta import Lens

DATA_DIR = "/app/data"
TELEMETRY_FILE = os.path.join(DATA_DIR, "telemetry.json")

# Native sensor geometry, used to turn calibrated focal lengths into millimetres for EXIF
IMX378_PIXEL_MM, IMX378_NATIVE_WIDTH = 0.00155, 4056
OV9282_PIXEL_MM, OV9282_NATIVE_WIDTH = 0.003, 1280


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='BlueOS OAK-D VIO')

    # Camera settings
    parser.add_argument('--camera-ip', default='')
    parser.add_argument('--fps', type=int, default=3)
    parser.add_argument('--camera-angle', type=float, default=0.0)
    parser.add_argument('--width', type=int, default=320)
    parser.add_argument('--height', type=int, default=200)

    # VIO algorithm settings
    parser.add_argument('--backend', choices=['basalt', 'rtabmap'], default='basalt')
    parser.add_argument('--grid-size', type=int, default=80)
    parser.add_argument('--flow-iterations', type=int, default=3)
    parser.add_argument('--flow-levels', type=int, default=2)
    parser.add_argument('--max-states', type=int, default=2)
    parser.add_argument('--max-kfs', type=int, default=4)
    parser.add_argument('--vio-iterations', type=int, default=3)
    parser.add_argument('--skip-frames', type=int, default=1)
    parser.add_argument('--rtabmap-features', type=int, default=500)
    parser.add_argument('--rtabmap-local-map-size', type=int, default=1000,
                        help="Descriptors kept in RTAB-Map's local map (OdomF2M/MaxSize; RTAB-Map's default is 2000)")

    # MAVLink settings
    parser.add_argument('--vehicle-ip', default='192.168.2.2')
    parser.add_argument('--quality-rate', type=float, default=1.0)

    # Image capture
    parser.add_argument('--center-capture-fps', type=float, default=0.0)
    parser.add_argument('--stereo-capture-fps', type=float, default=0.0)
    parser.add_argument('--center-width', type=int, default=1920)
    parser.add_argument('--center-height', type=int, default=1080)
    parser.add_argument('--mono-width', type=int, default=1280)
    parser.add_argument('--mono-height', type=int, default=800)
    parser.add_argument('--jpeg-quality', type=int, default=90)
    parser.add_argument('--mono-host-encode', action='store_true',
                        help='JPEG-encode stereo frames on the Pi instead of the OAK')
    parser.add_argument('--min-free-gb', type=float, default=2.0)

    return parser.parse_args()


def write_telemetry(data):
    """Atomically replaces telemetry.json, which main.py serves to the widget."""
    tmp = TELEMETRY_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f)
    os.replace(tmp, TELEMETRY_FILE)


def fail(message):
    logger.error(message)
    try:
        write_telemetry({"time": time.time(), "running": False, "error": message})
    except OSError:
        pass
    sys.exit(2)


def build_basalt(p, args, left, right, imu):
    odom = p.create(dai.node.BasaltVIO)
    odom.vioConfig.optical_flow_detection_grid_size = args.grid_size
    odom.vioConfig.optical_flow_max_iterations = args.flow_iterations
    odom.vioConfig.optical_flow_levels = args.flow_levels
    odom.vioConfig.vio_max_states = args.max_states
    odom.vioConfig.vio_max_kfs = args.max_kfs
    odom.vioConfig.vio_max_iterations = args.vio_iterations
    odom.vioConfig.optical_flow_skip_frames = args.skip_frames

    left.requestOutput((args.width, args.height)).link(odom.left)
    right.requestOutput((args.width, args.height)).link(odom.right)
    imu.out.link(odom.imu)
    logger.info("Backend: Basalt (VIO on the Pi)")
    return odom


def build_rtabmap(p, args, left, right, imu):
    """Stereo depth and feature tracking on the OAK; RTAB-Map odometry on the Pi."""
    stereo = p.create(dai.node.StereoDepth)
    feature_tracker = p.create(dai.node.FeatureTracker)
    odom = p.create(dai.node.RTABMapVIO)

    left.requestOutput((args.width, args.height)).link(stereo.left)
    right.requestOutput((args.width, args.height)).link(stereo.right)
    stereo.setExtendedDisparity(False)
    stereo.setLeftRightCheck(True)
    stereo.setRectifyEdgeFillColor(0)
    stereo.enableDistortionCorrection(True)  # the OV9282 lenses on the OAK-D W are wide
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)
    # On the OAK, so free for the Pi: less depth noise for RTAB-Map to reject
    stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_5x5)

    feature_tracker.setHardwareResources(1, 2)
    feature_tracker.initialConfig.setCornerDetector(dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS)
    feature_tracker.initialConfig.setNumTargetFeatures(args.rtabmap_features)

    stereo.rectifiedLeft.link(feature_tracker.inputImage)
    feature_tracker.passthroughInputImage.link(odom.rect)
    stereo.depth.link(odom.depth)
    feature_tracker.outputFeatures.link(odom.features)
    imu.out.link(odom.imu)
    # The odometry is one thread on the Pi and is what limits the frame rate. Frame-to-map
    # matching cost grows with the local map, so it is kept smaller than RTAB-Map's 2000.
    odom.setParams({
        "OdomF2M/MaxSize": str(args.rtabmap_local_map_size),
        "Vis/MaxFeatures": str(args.rtabmap_features),
    })
    logger.info(f"Backend: RTAB-Map (stereo depth and feature tracking on the OAK, "
                f"local map {args.rtabmap_local_map_size})")
    return odom


def lens_for(calib, socket, make, model, width, height, native_pixel_mm, native_width):
    """Lens from the camera's (in-water) calibration at the captured resolution, or None."""
    try:
        fx = calib.getCameraIntrinsics(socket, width, height)[0][0]
        return Lens.from_intrinsics(make, model, fx, native_pixel_mm, native_width, width)
    except Exception as e:
        logger.warning(f"No calibration for {socket}, images will have no focal length: {e}")
        return None


def build_capture(p, args, device, left, right, mavlink_publisher):
    """Adds MJPEG capture outputs and returns the CaptureNode (or None if capture is disabled)."""
    if args.center_capture_fps <= 0 and args.stereo_capture_fps <= 0:
        return None
    calib = device.readCalibration()
    make = b"MarineSitu C3"
    lenses = {}
    capture = CaptureNode(
        data_dir=DATA_DIR, vehicle_ip=args.vehicle_ip, camera_angle=args.camera_angle,
        lenses=lenses, mavlink_node=mavlink_publisher, min_free_gb=args.min_free_gb,
        host_encode_quality=args.jpeg_quality if args.mono_host_encode else None)
    profile = dai.VideoEncoderProperties.Profile.MJPEG

    if args.center_capture_fps > 0:
        fps = args.center_capture_fps
        # The sensor runs a little faster than the capture rate; the output is decimated on the OAK
        center = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, sensorFps=max(fps, 5.0))
        out = center.requestOutput((args.center_width, args.center_height), dai.ImgFrame.Type.NV12, fps=fps)
        encoder = p.create(dai.node.VideoEncoder).build(out, frameRate=fps, profile=profile,
                                                        quality=args.jpeg_quality)
        encoder.out.link(capture.inputCenter)
        lenses["center"] = lens_for(calib, dai.CameraBoardSocket.CAM_A, make, b"IMX378",
                                    args.center_width, args.center_height,
                                    IMX378_PIXEL_MM, IMX378_NATIVE_WIDTH)
        logger.info(f"Capturing centre {args.center_width}x{args.center_height} at {fps} fps")

    if args.stereo_capture_fps > 0:
        fps = min(args.stereo_capture_fps, float(args.fps))
        for name, cam, socket, link in (("left", left, dai.CameraBoardSocket.CAM_B, capture.inputLeft),
                                        ("right", right, dai.CameraBoardSocket.CAM_C, capture.inputRight)):
            # The OAK's VideoEncoder takes 8-bit grey only as YUV400p; it rejects GRAY8 frames
            frame_type = dai.ImgFrame.Type.GRAY8 if args.mono_host_encode else dai.ImgFrame.Type.YUV400p
            out = cam.requestOutput((args.mono_width, args.mono_height), frame_type, fps=fps)
            if args.mono_host_encode:
                out.link(link)
            else:
                encoder = p.create(dai.node.VideoEncoder).build(out, frameRate=fps, profile=profile,
                                                                quality=args.jpeg_quality)
                encoder.out.link(link)
            lenses[name] = lens_for(calib, socket, make, f"OV9282 {name}".encode(),
                                    args.mono_width, args.mono_height,
                                    OV9282_PIXEL_MM, OV9282_NATIVE_WIDTH)
        where = "Pi" if args.mono_host_encode else "OAK"
        logger.info(f"Capturing stereo {args.mono_width}x{args.mono_height} at {fps} fps (JPEG on the {where})")
    return capture


def telemetry_loop(p, device, args, imu_type, mavlink_publisher, capture):
    device_info = {"backend": args.backend, "imu": imu_type, "camera_ip": args.camera_ip or None}
    try:
        device_info["camera"] = device.getProductName() or device.getDeviceName()
        device_info["camera_id"] = device.getDeviceId()
    except Exception:
        pass
    while p.isRunning():
        data = {"time": time.time(), "running": True, "error": None, **device_info,
                "vio": mavlink_publisher.snapshot(),
                "capture": capture.snapshot() if capture else None}
        try:
            data["chip_temp_c"] = round(device.getChipTemperature().average, 1)
        except Exception:
            data["chip_temp_c"] = None
        try:
            write_telemetry(data)
        except OSError as e:
            logger.warning(f"Could not write telemetry: {e}")
        time.sleep(1.0)


def main():
    args = parse_args()

    print(f"🚀 Starting OAK-D VIO ({args.backend})")
    print(f"📷 Camera: {args.width}x{args.height} @ {args.fps} FPS")
    print(f"📐 Camera Angle: {args.camera_angle}°")
    print(f"📡 MAVLink: {args.vehicle_ip}")

    # A PoE OAK stays busy for a few seconds after the previous VIO process is killed (e.g. on a
    # config save), so a restart retries instead of failing on the first X_LINK_DEVICE_NOT_FOUND
    deadline = time.time() + 30.0
    while True:
        try:
            if args.camera_ip:
                logger.info(f"Connecting to camera at {args.camera_ip}")
                pipeline = dai.Pipeline(dai.Device(args.camera_ip))
            else:
                logger.info("Connecting to the first camera found")
                pipeline = dai.Pipeline()
            break
        except Exception as e:
            if time.time() < deadline:
                logger.warning(f"Camera not available yet ({e}), retrying")
                time.sleep(3.0)
                continue
            fail(f"Could not connect to the camera{' at ' + args.camera_ip if args.camera_ip else ''}: {e}")

    with pipeline as p:
        device = p.getDefaultDevice()
        imu_type = device.getConnectedIMU()
        if not imu_type or imu_type.upper() == "NONE":
            fail("This camera reports no IMU; VIO needs one. Check the IMU panel on the configuration page.")
        logger.info(f"Camera IMU: {imu_type}")

        # Define sources
        left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=float(args.fps))
        right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=float(args.fps))
        imu = p.create(dai.node.IMU)
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)

        if args.backend == "rtabmap":
            odom = build_rtabmap(p, args, left, right, imu)
        else:
            odom = build_basalt(p, args, left, right, imu)

        # Create MAVLink publisher
        mavlink_url = f"ws://{args.vehicle_ip}/mavlink2rest/ws/mavlink?filter=SEND_ONLY"
        mavlink_publisher = MavlinkNode(
            mavlink_host=mavlink_url,
            camera_angle=args.camera_angle,
            quality_rate=args.quality_rate
        )
        odom.transform.link(mavlink_publisher.inputTrans)
        # The poses carry no timestamp; the frame each one was computed from does
        mavlink_publisher.link_frame_times(odom.passthroughRect if args.backend == "rtabmap" else odom.passthrough)
        if args.backend == "basalt":
            # RTAB-Map has no quality output; MavlinkNode reads this input with tryGet()
            odom.quality.link(mavlink_publisher.inputQuality)

        capture = build_capture(p, args, device, left, right, mavlink_publisher)

        p.start()
        print("✅ Pipeline started!")
        print("📡 Sending VIO data via MAVLink...")

        threading.Thread(target=telemetry_loop, daemon=True,
                         args=(p, device, args, imu_type, mavlink_publisher, capture)).start()

        while p.isRunning():
            time.sleep(0.1)


if __name__ == "__main__":
    main()
