# ROS Crawler
Turn a remote controlled car into an autonomous robot via a raspberry pi 4 and ROS2 Jazzy.
Further details: [https://www.moritzboeker.de/](https://www.moritzboeker.de/2022/08/ros-raspi-car/)

![A view of the robot's point cloud and frames with Lichtblick](./docs/01_lichtblick_view.png)

## Parts
- Remote-controlled car: I used the [DF-4J - XXL Crawler](https://www.df-models.info/RC-Cars/BasicLine-XXL), since it's slow and strong
- Raspberry Pi 4B or higher
- Odometry: [SparkFun Optical Tracking Odometry Sensor - PAA5160E1 (Qwiic)](https://www.sparkfun.com/sparkfun-optical-tracking-odometry-sensor-paa5160e1-qwiic.html)
- Laser Scanner: [Vanjee WLR-719 360° 4 layer LiDAR sensor](https://www.vanjee.net/vanjee_products/196223.html)
- Servo Driver: [16-Channel 12-bit PWM/Servo Driver - I2C interface - PCA9685](https://www.adafruit.com/product/815)

## Run in Docker

The image `ghcr.io/moritzboeker/ros_crawler:jazzy` (linux/arm64) contains `foxglove_bridge`. On start it runs `ros2 launch bringup bringup.py`.

### One-time network setup for Vanjee lidar

The Vanjee lidar sits at `192.168.0.2` and streams to the host at the fixed address `192.168.0.64` over ethernet by default. Set this up once (it persists across reboots as a NetworkManager profile named `lidar`):

```bash
cd ~/ros_crawler
sudo ./scripts/setup_lidar_network.sh              # Pi, interface eth0 (default)
sudo ./scripts/setup_lidar_network.sh enp0s31f6    # other machines: pass your ethernet interface
```

The addresses match `host_address`/`lidar_address` in `bringup/config/vanjee_wlr719c_cfg.yaml` — change them together if you ever move the lidar to a different subnet.

### Run / update

```bash
cd ~/ros_crawler
docker compose pull   # fetch the latest image
docker compose up -d  # start; restarts automatically on boot (restart: unless-stopped)
docker compose logs -f
```

The container uses host networking, so topics are visible on the LAN via DDS (matching `ROS_DOMAIN_ID`). For visualization, connect [Lichtblick](https://github.com/lichtblick-suite/lichtblick) on your laptop to `ws://<pi-ip>:8765` (Foxglove WebSocket).

### Build the image manually

CI (`.github/workflows/docker.yml`) builds and pushes the image on every push to `main` using a native arm64 runner. To build and push manually from an x86 machine:

```bash
docker login ghcr.io                  # PAT with write:packages
docker buildx build --platform linux/arm64 -t ghcr.io/moritzboeker/ros_crawler:jazzy-test .
```

## Third-party code

`ros2_vanjee_lidar_driver/` is a git submodule of [ros2_vanjee_lidar_driver](https://github.com/moritzboeker/ros2_vanjee_lidar_driver), a ROS2 port of Vanjee's BSD-3-Clause `vanjee_lidar_sdk` — see that repo's README for details and attribution.
