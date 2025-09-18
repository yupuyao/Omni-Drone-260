# Omni-Drone-260

**Omni-Drone-260** is a UAV autonomy planning project that runs the [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) entirely within a ROS 2 environment. It integrates XRCE-DDS communication to interface with both PX4 flight controller and the K230 platform with different task supporting.

**Update：** Official PX4 image support is provided, with the safety warning that previously required a ground station for takeoff removed, allowing flight without a ground station.

**Note:** This repository uses a monocular camera for localization, so obstacle avoidance relies on metric 3D depth estimation and synchronizes with VINS-Mono via hardware trigger. If you are using a stereo camera, please modify the topic subscriptions accordingly and you can ignore the USB cam-related parts.

## ✨ Features

- Fully ROS 2-based UAV autonomy system
- Real-time 3D trajectory planning with EGO-Planner
- Supports depth estimation for a monocular camera using metric 3D
- Lightweight XRCE-DDS communication with PX4 and K230
- Control the robotic arm or communication module via serial port
- Flight can be controlled using terminal coordinates or predefined waypoints
- Embedded-friendly, low-latency deployment

## 🚀 Getting Started

To run the system, simply execute the `pipeline.sh` script in the utils directory:

```bash
cd utils
./pipeline.sh
```

If you only want to test takeoff, you can run the following script:
```bash
./takeoff.sh
```
After that, you can control the flight direction by inputting coordinates.

**Note:** If you are not using optical flow or GPS for localization, make sure to start the visual localization command in the takeoff script.