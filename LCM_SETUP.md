# LCM Setup (Lightweight Communication)

This project now includes optional LCM pub/sub tools for object poses.

## 1) Install LCM

Ubuntu/Debian:

```bash
sudo apt-get update
sudo apt-get install -y lcm liblcm-dev
```

After install, these should exist:

```bash
pkg-config --modversion lcm
lcm-gen --help
```

## 2) Configure and Build

```bash
cmake -S . -B build -DOPENARM_BUILD_LCM_TOOLS=ON
cmake --build build -j
```

If LCM is missing, CMake will keep building `openarm_core` and skip LCM tools with a warning.

## 3) Run Demo

Terminal A:

```bash
./build/lcm_pose_subscriber OPENARM_OBJECT_POSES
```

Terminal B:

```bash
./build/lcm_pose_publisher OPENARM_OBJECT_POSES 30
```

You should see subscriber output with 3 object poses (`obj_1`, `obj_2`, `obj_3`).

## 4) Message Schema

Defined in:

`lcm_types/openarm_pose.lcm`

- `ObjectPose`
  - `utime` (int64)
  - `object_id` (string)
  - `position[3]` (x, y, z)
  - `quaternion[4]` (w, x, y, z)
- `ObjectPoseArray`
  - `utime`
  - `num_poses`
  - `poses[num_poses]`
