# Hanford Wire Manipulator + ZED X Mini (ROS 2 Description + Isaac Sim Export)

ROS 2 robot description package for a riser-deployed Hanford wire manipulator with an end-effector **ZED X Mini** camera model. Includes URDF, meshes, and a simple RViz display launch. This repo also documents a reproducible workflow to import the URDF into **NVIDIA Isaac Sim** (URDF → USD) and simulate the **ZED camera stream** using **stereolabs/zed-isaac-sim**.

---

## Demo

<p align="center">
  <img src="media/new_full_assembly_with_camera.gif" width="900" />
</p>

### Videos
GitHub does not reliably play videos embedded via HTML in README files. Use these direct links:

- Isaac Sim environment: **[Simulation_env.mp4](media/Simulation_env.mp4)**
- ZED camera view: **[ZED_camera_view.mp4](media/ZED_camera_view.mp4)**

> If you prefer inline playback, upload the videos to GitHub Releases (or YouTube) and link them here.

---

## Repository Structure

- `urdf/` — robot URDF(s)
- `meshes/` — STL/OBJ meshes (includes `meshes/zed_x_mini/`)
- `launch/display.launch.py` — quick RViz visual check
- `media/` — demo GIF/videos used in this README

---

## Requirements

### ROS 2 / RViz (sanity check)
- ROS 2 (Humble recommended)
- `colcon` + common ROS 2 desktop tools (RViz, robot_state_publisher)

### Isaac Sim + ZED simulation
- NVIDIA Isaac Sim **>= 5.0.0**
- ZED SDK installed (same machine that runs Isaac Sim if you want to stream into ZED tools)
- `stereolabs/zed-isaac-sim` extension (build + enable in Isaac Sim)

---

## 1) Build and View the URDF in RViz (sanity check)

Clone this repo into a ROS 2 workspace:

```bash
mkdir -p ~/sensors_ws/src
cd ~/sensors_ws/src
git clone <THIS_REPO_URL>
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
