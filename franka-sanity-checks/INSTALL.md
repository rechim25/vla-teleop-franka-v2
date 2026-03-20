# Installation (Ubuntu 22.04, libfranka 0.9.2)

This guide installs everything needed to build and run this project against Franka robot server version 5.

## 1) Install system dependencies

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git pkg-config \
  libpoco-dev libeigen3-dev
```

## 2) Install libfranka 0.9.2 (required)

```bash
cd ~
git clone --branch 0.9.2 --recursive https://github.com/frankaemika/libfranka.git
cd libfranka
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=ON
cmake --build build -j"$(nproc)"
sudo cmake --install build
sudo ldconfig
```

## 3) Verify libfranka is installed

```bash
ls /usr/local/lib/cmake/Franka/FrankaConfig.cmake
ls /usr/local/include/franka/robot.h
ls /usr/local/lib/libfranka.so*
```

## 4) Build this sanity-check project

```bash
cd /home/radu/vla-teleop-franka-v2/franka-sanity-checks
rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

If CMake still cannot find `FrankaConfig.cmake`, run:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DFranka_DIR=/usr/local/lib/cmake/Franka
cmake --build build -j"$(nproc)"
```

## 5) Run after installation

Read-only connectivity check:

```bash
./build/panda_libfranka_sanity --robot-ip 192.168.2.200 --mode read-only --read-samples 200
```

Tiny motion (single smooth move):

```bash
./build/panda_libfranka_sanity --robot-ip 192.168.2.200 --mode tiny-motion --joint-index 3 --delta-rad 0.01 --duration-s 3.0
```

Tiny motion with one-shot automatic error recovery after abort:

```bash
./build/panda_libfranka_sanity --robot-ip 192.168.2.200 --mode tiny-motion --auto-recover
```

Clear Reflex mode only (no motion):

```bash
./build/panda_libfranka_sanity --robot-ip 192.168.2.200 --mode recover-only
```

Gripper check:

```bash
./build/panda_libfranka_sanity --robot-ip 192.168.2.200 --mode gripper-check
```

## Notes

- Use `libfranka` version `0.9.2` (allowed: `>=0.9.1` and `<0.10.0`).
- Do not use `libfranka >= 0.10.0` with robot server version 5.
- Ignore stale `FrankaConfig.cmake` files generated in other environments (for example paths containing `/workspace/...`), because they may point to invalid absolute paths.
- If motion aborts with `communication_constraints_violation`, the robot may remain in `Reflex` mode; run `recover-only` before retrying motion or `communication_test`.
