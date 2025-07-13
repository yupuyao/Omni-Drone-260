# Omni-Drone-260

**Omni-Drone-260** is a UAV autonomy planning project that runs the [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) entirely within a ROS 2 environment. It integrates XRCE-DDS communication to interface with both PX4 flight controller and the K230 platform.

## ✨ Features

- Fully ROS 2-based UAV autonomy system
- Real-time 3D trajectory planning with EGO-Planner
- Lightweight XRCE-DDS communication with PX4 and K230
- Embedded-friendly, low-latency deployment

## 🚀 Getting Started

To run the system, simply execute the `pipeline.sh` script in the root directory:

```bash
./pipeline.sh
