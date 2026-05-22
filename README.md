# vla-sweeping-controller
A low-latency C++ simulation workspace built natively on MuJoCo and GLFW for sweeping and sorting robotic tasks.

## Repository Layout
* `/src` - C++ source files
* `/models` - MuJoCo MJCF XML environment definitions
* `/build` - Local compilation binaries (gitignored)

## Set up
### Prerequisites (WSL2 / Ubuntu)
```bash
sudo apt update
sudo apt install -y build-essential cmake libmujoco-dev libglfw3-dev libqhull-dev
```
### Build
```bash
mkdir -p build && cd build
cmake ..
make -j4
```
### Launch
Run the simulator from within the `build` directory:
```bash
./vla_sim_runner
```

## Progress
**Week 1 (17/05/2026):** Reviewed MuJoCo docs and initialized workspace. Custom environment `sweeping_scene.xml` created with dynamic physical pucks (red square, blue circle, green square) and dual target landing zones with target task defined. Integrated base UR5e asset and tested hardcoded joint movement. Developed a multi-threaded architecture to decouple physics calculations from visual rendering.