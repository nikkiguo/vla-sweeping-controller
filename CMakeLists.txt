cmake_minimum_required(VERSION 3.16)
project(vla_sweeping_controller CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find MuJoCo and GLFW libraries locally
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)

# Define compilation target executable
add_executable(vla_sim_runner src/main.cpp)

# Link the physics engine and graphics rendering dependencies
target_link_libraries(vla_sim_runner PRIVATE mujoco::mujoco glfw)