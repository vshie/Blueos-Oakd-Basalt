# BlueOS OAK-D Visual Odometry

A [BlueOS](https://github.com/bluerobotics/BlueOS) extension that runs stereo visual-inertial odometry (VIO) on a Luxonis OAK-D, such as the **MarineSitu C3 stereo camera** on a BlueROV2, and feeds the result to ArduSub over MAVLink. It can also capture geotagged colour and stereo images for surveys while VIO runs.

It uses [depthai-core](https://github.com/luxonis/depthai-core) v3 with the camera's stereo pair and IMU, and sends the estimated motion as `VISION_POSITION_DELTA` messages through BlueOS's mavlink2rest.

Tested target: Raspberry Pi 4 on **64-bit** BlueOS, ArduSub 4.7, Cockpit, MarineSitu C3 (OAK-D W PoE: IMX378 colour, OV9282 stereo pair).

## What it does on startup

1. Starts the configuration page and Cockpit widget on port **8080**.
2. Starts VIO with the saved settings and begins sending MAVLink.
3. Checks the autopilot's EKF parameters (see below).
4. Image recording is **off**. If Auto mode was enabled before the restart, it is re-armed.

## Setting up the C3

1. **Network.** The C3 gets its IP address by DHCP. In BlueOS, open the network menu (top right), expand **eth0** and choose **Enable DHCP server**. You may need to power-cycle the ROV once with the camera connected.
2. **Disable Madrona.** The camera accepts only one connection, so MarineSitu's Madrona extension must be disabled (BlueOS → Extensions → Installed) while this extension runs.
3. **Camera IP** (optional). Leave it blank to use the first camera found, or press **Discover** on the configuration page and pick the C3. A fixed IP only helps if the address doesn't change; with BlueOS's DHCP it can.
4. **EKF origin.** Set the origin and vehicle position from **Cockpit** at the dive site. The extension never sets the origin; it only shows whether one exists. Without an origin, images have no latitude or longitude.

The C3's link is **100 Mbit/s** on a BlueROV2. The configuration page estimates how much of it the chosen VIO and capture settings use.

## Autopilot parameters

Every time VIO starts, the extension reads these parameters from the autopilot (system 1, component 1):

| Parameter | Value | Meaning | Needs reboot |
|---|---|---|---|
| `AHRS_EKF_TYPE` | 3 | Use EKF3 | yes |
| `EK3_ENABLE` | 1 | Enable EKF3 | yes |
| `VISO_TYPE` | 1 | Visual odometry over MAVLink | yes |
| `EK3_SRC1_POSXY` | 6 | Horizontal position from ExternalNav | no |
| `EK3_SRC1_VELXY` | 6 | Horizontal velocity from ExternalNav | no |
| `EK3_SRC1_POSZ` | 1 | Depth/altitude from the barometer | no |

- **First run:** any parameter that differs is written automatically.
- **After that:** differences are only reported. The **Autopilot Parameters** panel shows a table of current and required values; nothing is written until you press **Apply changes**. This protects an EKF setup you changed on purpose.
- Parameters are only written while the vehicle is **disarmed**; Apply is refused while armed.
- Each write is confirmed against the autopilot's `PARAM_VALUE` reply.
- If a parameter that needs a reboot was changed, the panel and the log say so. The extension never reboots the autopilot itself.
- "Already applied once" is kept in `state.json`, separate from the settings, so **Reset Defaults** does not re-enable automatic writes. Delete `/usr/blueos/extensions/oakd-vio/state.json` to start over.
- If mavlink2rest can't be reached, the log says `Could not reach mavlink2rest at http://<ip>/mavlink2rest: ...`. If it is reachable but no autopilot is talking yet, it reports that no HEARTBEAT has been seen.

Don't run this alongside another source of `VISION_POSITION_DELTA`, such as a Water Linked DVL extension.

## MAVLink output

Messages are sent with component ID 195 (`MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY`) to `ws://<vehicle_ip>/mavlink2rest/ws/mavlink`.

- `VISION_POSITION_DELTA`: position change since the previous frame, in the body frame (front-right-down), one per VIO frame. `angle_delta` is not computed and is always zero; `confidence` is always 100.
- `NAMED_VALUE_FLOAT`, rate-limited by the *Quality Rate* setting (Basalt backend only):

| Name | Meaning | Range | Healthy | Problem |
|---|---|---|---|---|
| `VIO_TRACK` | 1 while tracking, 0 when lost | 0 or 1 | 1 | 0: don't trust position hold; slow down or move to more texture |
| `VIO_QUAL` | Average tracking quality | 0–1 | above about 0.5 | falling toward 0 before tracking is lost |
| `VIO_FEAT` | Tracked features ÷ 1000, capped at 1 | 0–1 | above about 0.05 (50 features) | near 0: featureless scene, turbidity or darkness |
| `VIO_LMK` | Landmarks ÷ 10000, capped at 1 | 0–1 | steady | dropping to 0 with `VIO_TRACK` |
| `VIO_PTIME` | Processing time of the last frame, seconds | ≥ 0 | well below 1 ÷ VIO FPS | close to 1 ÷ FPS: the Pi can't keep up; lower FPS, resolution, or switch backend |

Names are at most 10 characters, as MAVLink requires.

## Using with Cockpit

**Values.** The `VIO_*` values arrive in Cockpit's data lake like any other MAVLink data:

1. Enter **Edit mode**.
2. Add a **Very Generic Indicator** mini-widget to a bar.
3. Open its settings, search the variable list for `VIO_TRACK` (and the others above), and pick an icon and unit.

`VIO_TRACK` and `VIO_PTIME` are the two most useful to keep in view while piloting.

**Widget.** The extension advertises an **OAK VIO** widget to the Cockpit connected to this vehicle (through `/register_service` and `/cockpit_extras.json`). In Edit mode it appears in the widget list with the other extension widgets; drag it onto a view. It shows:

- **REC / AUTO / STOP** buttons for image capture
- capture state, transect leg, image counts and free space
- VIO backend, rate, tracking, quality, features, landmarks and processing time
- the vehicle's estimated position, heading, depth and altitude
- camera, IMU, chip temperature, parameter state and whether the EKF origin is set

For bench work you can also add a plain iframe widget pointing at `http://<vehicle>:8080/widget`.

## VIO backends

| Backend | On the camera | On the Pi | Quality messages |
|---|---|---|---|
| **Basalt** (default) | image capture | optical flow and optimisation (Basalt) | yes |
| **RTAB-Map** | stereo depth, rectification, feature tracking | RTAB-Map odometry only | no |

RTAB-Map moves most of the image processing onto the OAK, so it allows higher frame rates on a Pi 4. Compare the two on your vehicle (CPU load in `htop`, achieved FPS in the widget, and how well tracking holds). Luxonis' `basalt_vio_rtab` example combines Basalt with `RTABMapSLAM`; that mapping runs on the host, so it would add load rather than remove it, and isn't used here.

## Image capture

While VIO runs, the extension can save centre-colour (IMX378) and stereo (OV9282 left + right) JPEGs. The JPEGs are encoded on the OAK, so this costs little Pi CPU. Centre and stereo rates are set separately (0 turns a stream off); the stereo rate can't exceed the VIO FPS.

The capture modes, file naming, CSV sidecars and EXIF layout follow [SubReels TowFish](https://github.com/vshie/SubReels_TowFish)'s survey format, adapted for an ROV: every value comes from **this ROV's own autopilot** via mavlink2rest at the configured Vehicle IP.

### Modes

- **Record:** captures continuously until **Stop**. Session tag `tl` + start time.
- **Auto:** records only while the ROV runs a mission. It starts when the vehicle is armed, in **AUTO** and moving toward a waypoint, and labels images per leg (`wp01`, `wp02`, … from `MISSION_CURRENT.seq`). It stops on *Mission Complete*, when a new mission is loaded (sequence jumps back), or after the vehicle has been seen out of AUTO or disarmed for 60 s. Missing telemetry never stops a session. Session tag `tr` + start time. An enabled Auto survives an extension restart; only Stop clears it.
- **Stop:** no recording. This is always the state at startup, apart from a restored Auto.

### Files

```
captures/survey_YYYYMMDD/
    center/000123_tr093015_wp02_00017.jpg
    stereo/left/000088_tr093015_wp02_00012.jpg
    stereo/right/000088_tr093015_wp02_00012.jpg    (same name as its left image)
    telemetry_center.csv
    telemetry_stereo.csv
    tr093015_manifest.ndjson                      (Auto: one row per leg)
```

- Name: `{seq:06d}_{source_tag}[_{wp}]_{frame:05d}.jpg`. `seq` counts up through the day for each stream, `frame` restarts each leg.
- Left and right images of a pair share one name, so the `stereo/` folders import into Metashape as a multi-camera rig.
- CSV columns: `timestamp, seq, filename, source_tag, wp, frame, size_bytes, lat, lon, heading_deg, roll_deg, pitch_deg, depth_m, temperature_c, telem_ms, sync_skew_ms, altitude_above_bottom_m, vio_x, vio_y, vio_z, vio_qw, vio_qx, vio_qy, vio_qz, vio_tracking`. `sync_skew_ms` is the time between the frame's exposure and the telemetry read.
- Manifest `leg_close` rows: start/end time and position, heading, distance to waypoint, image counts and why the leg ended.

### Metadata in each JPEG

| Tag | Content |
|---|---|
| `GPSLatitude` / `GPSLongitude` | Autopilot EKF position (VIO from the Cockpit-set origin) |
| `GPSAltitude` + `GPSAltitudeRef = 1` | **Vehicle depth**, as an altitude below sea level |
| `GPSImgDirection` | Vehicle heading |
| `DateTimeOriginal` / `GPSDateStamp` / `GPSTimeStamp` | Local system time and UTC |
| `FocalLength`, `FocalPlaneX/YResolution` | From the camera's in-water calibration at the captured resolution |
| XMP `Camera:Yaw/Pitch/Roll` | Heading, pitch from nadir (camera angle + vehicle pitch), roll; read by Metashape, Pix4D, WebODM |
| XMP `ROV:DepthBelowSurface`, `ROV:AltitudeAboveBottom`, `ROV:Waypoint`, `ROV:VioTracking` | Values with no standard tag |
| `UserComment` | Readable summary: position, depth, heading, attitude, temperature, waypoint |

Altitude above the bottom is filled in when a Ping sonar reports `DISTANCE_SENSOR`.

### Getting the images

- The **Image Capture** panel lists each folder with a **Download zip** link.
- On the vehicle they are in `/usr/blueos/extensions/oakd-vio/captures/` (BlueOS File Browser or `scp`).
- Recording stops by itself when free space drops below the configured minimum (default 2 GB).

## IMU tools

Basalt needs the camera's IMU and its position relative to the left camera (the IMU extrinsics). The OAK-D W PoE has a **BNO086**, or a **BMI270** on most units made between Q2 2021 and Q2 2023. The C3 is recalibrated in water, and that calibration may not carry IMU extrinsics. If the camera reports no IMU, VIO won't start and the page says why.

The **IMU** panel works while VIO is stopped:

1. **Detect IMU** shows the IMU model and firmware, and the stored IMU-to-left-camera transform. It warns when the transform is missing, is an identity, or puts the IMU more than 10 cm from the camera.
2. **Static test** (30 s, vehicle still on the bench) measures gyro bias, the accelerometer's reading of gravity, noise and sample rate, and reports pass or check.
3. **Record Kalibr dataset** (90 s) saves stereo images at 20 Hz and IMU at 200 Hz in **EuRoC layout** (`mav0/cam0`, `mav0/cam1`, `mav0/imu0`) under `captures/calib-<time>/`. Download it as a zip.
4. **Apply Kalibr result** uploads `camchain-imucam.yaml` and writes its `cam0.T_cam_imu` to the camera's EEPROM. The current calibration is first saved to `/usr/blueos/extensions/oakd-vio/calib-backup-<time>.json`. Restart VIO afterwards.

Calibrating with [Kalibr](https://github.com/ethz-asl/kalibr) on a PC:

1. Put an AprilGrid target in water (in a tank or at the dive site) and record a dataset while moving the ROV slowly so it rotates around and moves along all three axes, keeping the target in view of both cameras.
2. Convert it: `kalibr_bagcreater --folder calib-<time>/mav0 --output-bag calib.bag`.
3. Calibrate the stereo pair: `kalibr_calibrate_cameras --bag calib.bag --target aprilgrid.yaml --models pinhole-radtan pinhole-radtan --topics /cam0/image_raw /cam1/image_raw`.
4. Calibrate camera–IMU: `kalibr_calibrate_imu_camera --bag calib.bag --target aprilgrid.yaml --cam camchain.yaml --imu imu.yaml`. `imu.yaml` holds the IMU noise densities; take them from the datasheet, or from an Allan-variance run, then inflate them by about 10×.
5. Upload the resulting `camchain-imucam.yaml` with **Apply Kalibr result**.

Only the IMU extrinsics are written; the C3's in-water camera calibration is left as it is.

## Configuration page

Open `http://<vehicle>:8080` (or the extension's entry in the BlueOS menu). Saving restarts VIO with the new values. Settings are stored in `/usr/blueos/extensions/oakd-vio/config.json`, so they survive restarts and updates.

| Setting | Default | Notes |
|---|---|---|
| Camera IP | blank | Blank = first camera found |
| VIO FPS | 3 | Stereo frame rate for VIO. The Raspberry Pi CPU is the limit with Basalt; raise it slowly |
| Camera Angle (°) | 0 | Downward pitch of the camera. 0 = facing forward, 90 = facing down |
| VIO Width × Height | 320 × 200 | Mono resolution fed to VIO |
| Backend | Basalt | Basalt or RTAB-Map, see above |
| Optical Flow Grid Size | 80 | Basalt. Larger means fewer features and less CPU. Basalt default is 50 |
| Flow Iterations | 3 | Basalt. Optical flow iterations per level |
| Pyramid Levels | 2 | Basalt. Optical flow image pyramid levels |
| Skip Frames | 1 | Basalt. Run optical flow on every Nth frame |
| Max States | 2 | Basalt. Sliding window size |
| Max Keyframes | 4 | Basalt. Keyframes kept in the optimisation |
| VIO Iterations | 3 | Basalt. Optimiser iterations per frame |
| Tracked Features | 500 | RTAB-Map. Features tracked on the OAK |
| Centre Capture FPS | 2 | 0 = off |
| Centre Resolution | 1920 × 1080 | 4056 × 3040, 3840 × 2160 or 1920 × 1080 |
| Stereo Capture FPS | 2 | 0 = off; at most the VIO FPS |
| Stereo Resolution | 1280 × 800 | or 640 × 400 |
| JPEG Quality | 90 | |
| Minimum Free Space (GB) | 2 | Recording stops below this |
| Encode Stereo on the Pi | off | Fallback in case the OAK can't JPEG-encode the mono streams |
| Vehicle IP | 192.168.2.2 | This ROV's BlueOS. MAVLink output, parameters and image labels all use it |
| Quality Rate (Hz) | 1 | Rate of the `NAMED_VALUE_FLOAT` quality messages |

The page is backed by a small HTTP API:

| Method | Path | Action |
|---|---|---|
| GET / POST | `/api/config` | Current settings / save settings (JSON body, same keys) and restart VIO |
| GET | `/api/status` | Whether the VIO process is running |
| POST | `/api/start`, `/api/stop`, `/api/reset` | Start, stop, or restore defaults and restart VIO |
| GET | `/api/params` | Parameter check result, including the EKF origin |
| POST | `/api/params/check`, `/api/params/apply` | Re-check / write the parameters (refused while armed) |
| GET | `/api/telemetry` | Everything the widget shows |
| POST | `/api/capture/record`, `/api/capture/auto`, `/api/capture/stop` | Capture mode |
| GET | `/api/captures`, `/api/captures/{folder}/zip` | Capture folders / download one as a zip |
| GET | `/api/devices` | Discover cameras |
| GET | `/api/imu` | IMU tool status and last results |
| POST | `/api/imu/detect`, `/api/imu/static-test`, `/api/imu/record-euroc`, `/api/imu/flash-extrinsics` | IMU tools |
| GET | `/widget`, `/register_service`, `/cockpit_extras.json` | Cockpit widget and BlueOS registration |

## Installing

The image is built for **64-bit ARM (arm64) only**. It won't run on 32-bit BlueOS installs; BlueOS flags the image as incompatible, and System Information shows a kernel ending in `-v7l+`. Flash a 64-bit BlueOS image first.

In BlueOS, go to **Extensions → Installed → +** and fill in:

- **Extension Identifier**: `williangalvani.oakd-basalt`
- **Extension Name**: `Oak-D Basalt VIO`
- **Docker image**: `williangalvani/blueos-oakd-basalt` (or the name of a locally built image, see below)
- **Docker tag**: `arm64-rtabmap`
- **Custom settings**:

```json
{
  "ExposedPorts": {"8080/tcp": {}},
  "HostConfig": {
    "NetworkMode": "host",
    "Binds": ["/dev/:/dev/", "/usr/blueos/extensions/oakd-vio:/app/data"],
    "DeviceCgroupRules": ["c 189:* rmw"]
  }
}
```

Host networking lets the extension find a PoE camera such as the C3. `/dev` and the `c 189:*` cgroup rule give the container access to a USB OAK-D.

### Building a .tar locally

On a computer with Docker (an Apple Silicon Mac or other arm64 machine builds natively; elsewhere Docker uses QEMU):

```bash
./build_tar.sh                 # -> oakd-vio-arm64-local.tar, image blueos-oakd-basalt:local
TAG=test2 ./build_tar.sh       # -> oakd-vio-arm64-test2.tar
```

This builds only the runtime image on top of the prebuilt builder image (`williangalvani/blueos-oakd-basalt-builder:arm64-rtabmap`), so it takes a few minutes. Upload the `.tar` with BlueOS's extension manager (BlueOS versions with Kraken v2), using image `blueos-oakd-basalt`, tag `local` (or your `TAG`), and the identifier, name and custom settings above.

## Building the images

The image is built in two stages so that the slow part only runs once:

1. **Builder image** ([`dockerfile.builder`](dockerfile.builder)): compiles depthai-core with its Python bindings and Basalt, RTAB-Map and PCL support. Built under QEMU for arm64 this takes several hours, mostly compiling vcpkg dependencies. A prebuilt copy is at `williangalvani/blueos-oakd-basalt-builder:arm64-rtabmap`, and `build_tar.sh` uses it.

   The builder needs depthai-core's source in `depthai-core/`. This repository records it as a gitlink but has no `.gitmodules`, so `git submodule update` can't fetch it. Clone [depthai-core](https://github.com/luxonis/depthai-core) into that folder at the recorded commit (`git ls-files -s depthai-core`) with its own submodules before building.

   ```bash
   docker buildx build --platform linux/arm64 -f dockerfile.builder \
     -t williangalvani/blueos-oakd-basalt-builder:arm64-rtabmap --push .
   ```

2. **Runtime image** ([`dockerfile`](dockerfile)): a slim Debian image that copies the Python bindings and shared libraries out of the builder and adds the app.

   ```bash
   docker buildx build --platform linux/arm64 \
     --build-arg BUILDER_IMAGE=williangalvani/blueos-oakd-basalt-builder:arm64-rtabmap \
     -t williangalvani/blueos-oakd-basalt:arm64-rtabmap --push .
   ```

Tests for the transect state machine and image metadata run inside the image:

```bash
docker run --rm -v "$PWD/tests:/tests" blueos-oakd-basalt:local python -m unittest discover -s /tests -v
```

## Code layout

| File | Purpose |
|---|---|
| `app/main.py` | Web API, parameter flow, capture modes, IMU tool jobs; starts and stops the VIO process |
| `app/static/index.html` | Configuration page |
| `app/static/widget.html` | Cockpit widget |
| `app/basalt_configurable.py` | The VIO pipeline: cameras, IMU, Basalt or RTAB-Map, MAVLink output, capture outputs |
| `app/mavlink_node.py` | depthai host node that turns poses into MAVLink messages |
| `app/capture_node.py` | depthai host node that labels and writes captured JPEGs |
| `app/photogrammetry_meta.py` | EXIF/XMP writer (adapted from SubReels TowFish) |
| `app/transect.py` | Auto mode: follows the ROV's mission and labels legs |
| `app/vehicle_telemetry.py` | Reads position, attitude and depth from the local autopilot's mavlink2rest |
| `app/autopilot_params.py` | Checks and sets the EKF parameters; reads the EKF origin |
| `app/imu_tools.py` | Camera discovery, IMU detection, static test, Kalibr dataset, extrinsics flashing |
| `app/basalt.py` | Standalone command-line version of the Basalt pipeline, useful for testing (doesn't touch parameters) |
| `entrypoint.sh` | Container entrypoint; starts the web server |
| `build_tar.sh` | Builds the image locally and saves it as a `.tar` |

## Known limitations

- `VISION_POSITION_DELTA` reports a fixed confidence of 100% and no rotation delta.
- The RTAB-Map backend sends no `VIO_*` quality messages.
- If the camera isn't reachable when the extension starts, VIO exits and isn't retried. Press **Start** on the configuration page once the camera is up, or restart the extension.
- The IMX378 colour camera isn't hardware-synchronised with the stereo pair, so centre and stereo images are labelled independently; match them by timestamp in the CSVs.
- Stopping the container takes about 10 seconds, because the entrypoint doesn't forward the stop signal.
