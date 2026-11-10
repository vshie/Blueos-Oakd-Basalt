#!/usr/bin/env python3
"""
BlueOS OAK-D Basalt VIO with MAVLink output

Publishes the following MAVLink messages:
- VISION_POSITION_DELTA: Position delta data in body frame
- NAMED_VALUE_FLOAT: VIO quality metrics for autopilot monitoring

Quality metrics sent as NAMED_VALUE_FLOAT:
- VIO_QUAL: Average tracking quality (0.0-1.0)
- VIO_FEAT: Normalized number of tracked features (0.0-1.0)
- VIO_LMK: Normalized number of landmarks (0.0-1.0)
- VIO_PTIME: Processing time in seconds
- VIO_TRACK: 1.0 if tracking, 0.0 if lost

Usage examples:
  python basalt.py                                    # Local BlueOS (127.0.0.1:6040)
  python basalt.py --vehicle-ip 192.168.1.100        # Remote BlueOS on LAN
  python basalt.py --ip blueos.local                  # Using hostname
  python basalt.py --ip 192.168.1.100 --port 6041    # Custom port
  python basalt.py --fps 30 --camera-angle 45        # Custom FPS and camera angle
  python basalt.py --quality-rate 2.0                # Send quality data at 2 Hz
"""

import signal
import time
import argparse
import depthai as dai
from mavlink_node import MavlinkNode

def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='BlueOS OAK-D Basalt VIO with MAVLink output')
    parser.add_argument('--vehicle-ip', '--ip', 
                       default='127.0.0.1',
                       help='IP address of the vehicle/BlueOS instance (default: 127.0.0.1)')
    parser.add_argument('--fps',
                       type=int,
                       default=3,
                       help='FPS of the camera (default: 3)')
    parser.add_argument('--camera-angle',
                       type=float,
                       default=0.0,
                       help='Camera angle in degrees (0 = forward, 45 = 45° down, etc.)')
    parser.add_argument('--quality-rate',
                       type=float,
                       default=1.0,
                       help='Rate limit for quality messages in Hz (default: 1.0)')
    return parser.parse_args()

# Parse command line arguments
args = parse_args()

print(f"🚀 Starting BlueOS OAK-D Basalt VIO")
print(f"📡 Vehicle IP: {args.vehicle_ip}")
print(f"📷 Camera FPS: {args.fps}")
print(f"📐 Camera Angle: {args.camera_angle}°")
print(f"📊 Quality Rate: {args.quality_rate} Hz")

print(f"🌐 WebSocket URL: ws://{args.vehicle_ip}/mavlink2rest/ws/mavlink?filter=SEND_ONLY")

# Create pipeline

with dai.Pipeline() as p:
    fps = float(args.fps)
    width = 320
    height = 200
    # Define sources and outputs
    left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
    imu = p.create(dai.node.IMU)
    odom = p.create(dai.node.BasaltVIO)
    
    # Configure VIO for lower CPU usage on Pi
    odom.vioConfig.optical_flow_detection_grid_size = 80  # Fewer features (default 50)
    odom.vioConfig.optical_flow_max_iterations = 3        # Fewer iterations (default 5)
    odom.vioConfig.optical_flow_levels = 2                # Fewer pyramid levels (default 3)
    odom.vioConfig.vio_max_states = 2                     # Fewer states (default 3)
    odom.vioConfig.vio_max_kfs = 4                        # Fewer keyframes (default 7)
    odom.vioConfig.vio_max_iterations = 3                 # Fewer optimization iterations
    odom.vioConfig.optical_flow_skip_frames = 1           # Process every frame
    print("⚡ VIO configured for low-CPU mode")

    # Create MAVLink publisher instead of RerunNode
    mavlink_url = f"ws://{args.vehicle_ip}/mavlink2rest/ws/mavlink?filter=SEND_ONLY"
    mavlink_publisher = MavlinkNode(mavlink_host=mavlink_url, camera_angle=args.camera_angle, quality_rate=args.quality_rate)
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    # Linking
    left.requestOutput((width, height)).link(odom.left)
    right.requestOutput((width, height)).link(odom.right)
    imu.out.link(odom.imu)
    # Link outputs to MAVLink publisher instead of rerun viewer
    odom.transform.link(mavlink_publisher.inputTrans)
    # odom.passthrough.link(mavlink_publisher.inputImg)
    odom.quality.link(mavlink_publisher.inputQuality)
    
    p.start()
    print("✅ Pipeline started successfully!")
    print("📡 Sending VIO data to autopilot via MAVLink...")
    print("📊 Quality metrics will be sent as NAMED_VALUE_FLOAT messages")
    print("   - VIO_QUAL: Average tracking quality (0.0-1.0)")
    print("   - VIO_FEAT: Normalized number of tracked features (0.0-1.0)")
    print("   - VIO_LMK: Normalized number of landmarks (0.0-1.0)")
    print("   - VIO_PTIME: Processing time in seconds")
    print("   - VIO_TRACK: 1.0 if tracking, 0.0 if lost")
    print("")
    print("Press Ctrl+C to stop...")
    
    # Keep the pipeline running
    while p.isRunning():
        time.sleep(0.1)