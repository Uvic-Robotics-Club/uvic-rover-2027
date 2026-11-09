
# FR-3.1.1 — ZED 2i ROS 2 Bring-up

## Objective
Integrate the ZED 2i with ROS 2 Jazzy using Stereolabs'
`zed-ros2-wrapper` to publish synchronized RGB, depth,
and point-cloud data.

## Development Environment
- OS: Windows 11 + Ubuntu 24.04 (WSL2)
- GPU: NVIDIA RTX 5070 Ti
- ROS: Jazzy (Docker)
- ZED SDK: 5.5 (Linux), 5.5.1 (Windows)
- CUDA: 12.8 (Docker)
- Camera: ZED 2i
- Input: Windows SDK network stream to Docker

## Setup
The ZED Docker image was built using Stereolabs'
`zed-ros2-wrapper/docker/build_desktop.sh`, using the
regular SDK 5.5 development image instead of `gl-devel`.

Build the rover's ZED development image:

```bash
docker build -f docker/Dockerfile \
  --target zed-dev -t uvic-rover:zed-dev .
```

Run with GPU access and mount the repository:

```bash
docker run -it --rm --gpus all --network host \
  -v "$PWD:/rover_ws" \
  uvic-rover:zed-dev bash
```

Inside Docker, source ROS and the wrapper workspace:

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
```

## Camera Launch
Start ZED Explorer streaming on Windows, port 30000.

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i \
  stream_address:=WINDOWS_HOST_IP \
  stream_port:=30000
```

Replace WINDOWS_HOST_IP with the reachable Windows host IP.

## ROS Topics
- RGB: `/zed/zed_node/rgb/color/rect/image`
- Depth: `/zed/zed_node/depth/depth_registered`
- Point cloud: `/zed/zed_node/point_cloud/cloud_registered`

## Synchronization Test
Inside the ROS container, with the repository mounted:

```bash
python3 /rover_ws/scripts/verify_zed_sync.py \
  --duration 30 --pointcloud
```

The script compares message header timestamps and reports
observed subscriber frequencies. It is a preliminary test,
not a measurement of USB bandwidth or SDK frame drops.

## Current Results
- ROS 2 Jazzy and ZED wrapper: verified running
- RGB/depth/point-cloud messages: received
- Configured camera grab rate: 30 Hz
- Observed SDK data capture: approximately 18.1 Hz
- Reported frame drops: 63.09%

The high drop rate remains under investigation.
Current tests use Windows-to-WSL streaming, not direct
Jetson USB capture.

## Pending Acceptance Tests
- Resolve performance issues and verify synchronization
- Confirm SDK compatibility with the installed JetPack
- Confirm dedicated USB 3.0 controller on the Jetson
- Measure physical USB throughput at 1080p/30fps
- Verify no USB-contention-related dropped frames

Final hardware acceptance requires the assembled rover.
