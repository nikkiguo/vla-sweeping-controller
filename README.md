# vla-sweeping-controller
A low-latency C++ simulation workspace built natively on MuJoCo and GLFW for sweeping and sorting robotic tasks.

## Repository Layout
* `/src` - C++ source files
* `/models` - MuJoCo MJCF XML environment definitions
* `/build` - Local compilation binaries (gitignored)
* `/scripts` - Python utilities for consuming sim data (joint states, camera feed)

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

### Python Consumer Setup
Create and activate a virtual environment, then install dependencies:
```bash
cd scripts
python3 -m venv venv
source venv/bin/activate
pip install -r ../requirements.txt
```

Run the consumer script after launching the simulator:
```bash
python3 consumer.py
```

## Progress
**Week 1 (17/05/2026):** Reviewed MuJoCo docs and initialized workspace. Custom environment `sweeping_scene.xml` created with dynamic physical pucks (red square, blue circle, green square) and dual target landing zones with target task defined. Integrated base UR5e asset and tested hardcoded joint movement. Developed a multi-threaded architecture to decouple physics calculations from visual rendering.

**Week 2 (24/05/2026):** Implemented a shared-memory IPC pipeline allowing the producer (simulation environment) to stream sensor information to the consumer process. Profiled the physics loop to get average latency and max spike and validate it against the intended frequency. Technical deep dive #1 WIP.