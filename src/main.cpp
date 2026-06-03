#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include "controller.h"
#include "ipc_common.h"

// Shared resources
std::atomic<bool> exit_simulation{false};

// Handle keyboard input for teleoperation
void handleKeyboardInput(GLFWwindow* window, SweeperController& controller, double& target_x, double& target_y, double& target_z) {
    const double step = 0.02;

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) target_y += step;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) target_y -= step;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) target_x -= step;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) target_x += step;
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) target_z -= step;
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) target_z += step;

    controller.setTarget(target_x, target_y, target_z);
}

// Physics thread function
void physics_thread(mjModel* m, mjData* d, SharedDataStruct* shm, SweeperController* controller) {
    // Match the timestep defined in sweeping_scene.xml (0.002s = 2ms = 500 Hz)
    auto timestep = std::chrono::milliseconds(2);

    static double max_latency = 0.0;
    static double total_latency = 0.0;
    static uint64_t frame_count = 0;

    while (!exit_simulation) {
        auto start_time = std::chrono::steady_clock::now();
        auto next_tick = start_time + timestep;

        // Perform physics stepping using IK target following
        controller->compute(m, d);
        mj_step(m, d);

        // Update shared memory with new state (positions and camera pixels)
        shm->frame_index.fetch_add(1, std::memory_order_relaxed);
        mju_copy(shm->joint_pos, d->qpos, 6);
        mju_copy(shm->joint_vel, d->qvel, 6);
        shm->frame_index.fetch_add(1, std::memory_order_release);

        // Measure physics step latency
        auto end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end_time - start_time;

        double current_latency = elapsed.count();
        max_latency = std::max(max_latency, current_latency);
        total_latency += current_latency;
        frame_count++;

        if (frame_count % 5000 == 0) {
            std::cout << "[main]: Physics profiler | Avg: " << (total_latency / frame_count) << "ms  |  max spike: " << max_latency << "ms" << std::endl;
        }

        // Enforce 500Hz loop rate (2ms per step)
        std::this_thread::sleep_until(next_tick);
    }
}

int main() {
    char error[1000];
    mjModel* m = mj_loadXML("../models/sweeping_scene.xml", 0, error, 1000);
    
    // Check if XML loading failed
    if (!m) {
        std::cerr << "[main]: Failed to load XML model: " << error << std::endl;
        return 1;
    }
    
    mjData* d_render = mj_makeData(m);
    mjData* d_physics = mj_makeData(m);

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "[main]: Failed to initialize GLFW" << std::endl;
        mj_deleteModel(m);
        return 1;
    }
    
    GLFWwindow* window = glfwCreateWindow(640, 480, "VLA Sweeping Sandbox", NULL, NULL);
    if (!window) {
        std::cerr << "[main]: Failed to create GLFW window" << std::endl;
        mj_deleteModel(m);
        mj_deleteData(d_render);
        mj_deleteData(d_physics);
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);

    // Initialize MuJoCo visualization objects
    mjvCamera cam; mjv_defaultCamera(&cam);
    mjvOption opt; mjv_defaultOption(&opt);
    mjvScene scn; mjv_defaultScene(&scn);
    mjrContext con; mjr_defaultContext(&con);

    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // Setup shared memory for IPC
    int shm_fd = shm_open("/robot_data_shm", O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd, sizeof(SharedDataStruct));
    SharedDataStruct* shm = (SharedDataStruct*)mmap(NULL, sizeof(SharedDataStruct), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

    // Create the controller
    SweeperController controller(90.0, 10.0);
    controller.setTarget(0.3, 0.0, 0.35);

    // Teleoperation state
    double target_x = 0.3, target_y = 0.0, target_z = 0.35;

    // Start the physics thread
    std::thread physics_worker(physics_thread, m, d_physics, shm, &controller);

    while (!glfwWindowShouldClose(window)) {
        // Copy physics state (positions and velocities) to render state
        mju_copy(d_render->qpos, d_physics->qpos, m->nq);
        mju_copy(d_render->qvel, d_physics->qvel, m->nv);
        d_render->time = d_physics->time;

        // Update kinematics
        mj_forward(m, d_render);

        // Render the scene
        mjrRect viewport = {0, 0, 640, 480};
        mjv_updateScene(m, d_render, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // Capture framebuffer and write to shared memory (every 10 frames to match consumer display rate)
        static int frame_count = 0;
        if (frame_count++ % 10 == 0) {
            static uint8_t rgb_buffer[640 * 480 * 3];
            mjr_readPixels(rgb_buffer, NULL, viewport, &con);
            shm->frame_index.fetch_add(1, std::memory_order_relaxed);
            memcpy(shm->camera_pixels, rgb_buffer, sizeof(rgb_buffer));
            shm->frame_index.fetch_add(1, std::memory_order_release);
        }

        // Handle keyboard teleoperation
        handleKeyboardInput(window, controller, target_x, target_y, target_z);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Clean up
    exit_simulation = true; // Signal physics thread to exit
    physics_worker.join(); // Wait for physics thread to finish

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d_render);
    mj_deleteData(d_physics);
    mj_deleteModel(m);
    munmap(shm, sizeof(SharedDataStruct));
    shm_unlink("/robot_data_shm");
    glfwTerminate();
    return 0;
}