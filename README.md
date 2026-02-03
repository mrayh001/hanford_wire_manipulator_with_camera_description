# Hanford Wire Manipulator + ZED X Mini (ROS 2 Description + Isaac Sim Export)

ROS 2 robot description package for a riser-deployed Hanford wire manipulator with an end-effector **ZED X Mini** camera model. Includes URDF, meshes, and a simple RViz display launch. This repo also documents a reproducible workflow to import the URDF into **NVIDIA Isaac Sim** (URDF → USD) and simulate the **ZED camera stream** using **stereolabs/zed-isaac-sim**.

---

## Demo

<p align="center">
  <img src="media/new_full_assembly_with_camera.gif" width="900" />
</p>

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
```

Launch RViz display:

```bash
ros2 launch hanford_wire_manipulator_with_camera_description display.launch.py
```

If the model looks correct in RViz, you’re ready to export/import in Isaac Sim.

---

## 2) Import URDF into Isaac Sim (URDF → USD)

This section converts the URDF into a USD asset that Isaac Sim can simulate efficiently.

### A. Enable the URDF importer
1. Launch Isaac Sim
2. Open **Window → Extensions**
3. Search for and enable: **`isaacsim.asset.importer.urdf`** (URDF Importer)

### B. Import the URDF
1. Go to **File → Import…**
2. Select **URDF** and choose:
   - `urdf/robot_pit_end_effector.urdf`

Recommended import settings (good stability defaults):
- **Links:** Static base (fixed-base arm)  
- **Colliders:** enable *Collision From Visuals* if collision meshes are not provided  
- **Self-collision:** keep **OFF** unless you are sure meshes do not intersect  
- **Joints/Drives:** start conservative, tune later

3. Import, then **File → Save As…** and save the stage as:
   - `usd/hanford_wire_manipulator.usd` (recommended)

> Tip: For reuse across scenes, save the robot as a referenced USD and bring it into environments as needed.

---

## 3) Simulate the ZED X camera using `stereolabs/zed-isaac-sim`

The Stereolabs Isaac Sim extension streams your **virtual ZED camera** output into the **ZED SDK** pipeline, so you can use ZED tools and/or the ZED ROS 2 wrapper with a simulated camera.

### A. Clone + build the extension

```bash
cd ~/projects
git clone https://github.com/stereolabs/zed-isaac-sim.git
cd zed-isaac-sim
./build.sh   # Linux
# build.bat  # Windows
```

### B. Add the extension search path in Isaac Sim
1. Open **Window → Extensions**
2. Open **Settings** (hamburger menu)
3. Under **Extension Search Paths**, click **+**
4. Add:
   - `<path-to-zed-isaac-sim>/exts`
5. Enable the **ZED** extension (usually under Third-Party)

### C. Add a ZED X camera prim to the stage
Two options:

**Option 1 (recommended):** use the ZED X USD from Stereolabs  
- Drag `ZED_X.usd` into the stage (from the Stereolabs repo assets)

**Option 2:** keep your mesh and add an Isaac Sim camera sensor  
- Works for Isaac-only cameras, but ZED streaming is easiest with Option 1.

### D. Attach the ZED camera to the robot end-effector
1. Expand your imported robot prim in the Stage tree
2. Locate the end-effector link prim
3. Parent the ZED camera prim under the end-effector link
4. Adjust pose (XYZ/RPY) to align with your mount

### E. Create the Action Graph to stream to ZED SDK
1. **Window → Visual Scripting → Action Graph**
2. Create a new graph
3. Add nodes:
   - **On Playback Tick**
   - **ZED Camera Helper** (from the ZED extension)
4. Connect:
   - `On Playback Tick` → `ZED Camera Helper`
5. Set properties:
   - **ZED Camera prim**: select your ZED prim
   - **Model**: ZED X (or matching model)
   - **FPS/Resolution/IPC**: tune for performance

Press **Play**. You should see console output indicating the stream is active.

---

## Acknowledgements
- NVIDIA Isaac Sim URDF Importer workflow
- Stereolabs ZED Isaac Sim extension: https://github.com/stereolabs/zed-isaac-sim
- This research was supported by the U.S. Department of Energy (DOE), Office of Environmental Man- agement, under the project ”Digitally Optimized Autonomous Robot System for Hanford Waste Tank Han- dling” (Solicitation No. LAB 23-EM001 Award No. 278709). This work was performed in partnership with Idaho National Laboratory (INL).
