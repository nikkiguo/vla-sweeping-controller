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
std::mutex mtx;
std::atomic<bool> exit_simulation{false};

// Physics thread function
void physics_thread(mjModel* m, mjData* d, SharedDataStruct* shm) {
    // Match the timestep defined in sweeping_scene.xml (0.002s = 2ms = 500 Hz)
    auto timestep = std::chrono::milliseconds(2);

    SweeperController controller(90.0, 10.0);

    while (!exit_simulation) {
        auto next_tick = std::chrono::steady_clock::now() + timestep;

        // Perform physics stepping
        {
            // Lock only for the math
            std::lock_guard<std::mutex> lock(mtx);
            double target[6] = {0.0};
            target[0] = sin(d->time);               // Base swings left/right
            target[1] = -0.5 + 0.2*sin(d->time);    // Shoulder moves up/down gently
            target[2] = 1.0;                        // Elbow stays bent
            controller.compute(m, d, target);
            mj_step(m, d);

            // Update shared memory with new state (positions and camera pixels)
            shm->frame_index.fetch_add(1, std::memory_order_relaxed);
            mju_copy(shm->joint_states, d->qpos, 6);
            shm->frame_index.fetch_add(1, std::memory_order_release);
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
        std::cerr << "Failed to load XML model: " << error << std::endl;
        return 1;
    }
    
    mjData* d_render = mj_makeData(m);
    mjData* d_physics = mj_makeData(m);

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        mj_deleteModel(m);
        return 1;
    }
    
    GLFWwindow* window = glfwCreateWindow(640, 480, "VLA Sweeping Sandbox", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
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

    // Start the physics thread
    std::thread physics_worker(physics_thread, m, d_physics, shm);

    while (!glfwWindowShouldClose(window)) {
        {
            // Lock mjData only long enough to copy state to the visual scene
            std::lock_guard<std::mutex> lock(mtx);
            // Copy physics state (positions and velocities) to render state
            mju_copy(d_render->qpos, d_physics->qpos, m->nq);
            mju_copy(d_render->qvel, d_physics->qvel, m->nv);
            d_render->time = d_physics->time;
        }

        // Update kinematics
        mj_forward(m, d_render);

        // Render the scene
        mjrRect viewport = {0, 0, 640, 480};
        mjv_updateScene(m, d_render, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // Capture framebuffer and write to shared memory (every 10 frames to match consumer display rate)
        {
            static int frame_count = 0;
            if (frame_count++ % 10 == 0) {
                static uint8_t rgb_buffer[640 * 480 * 3];
                mjr_readPixels(rgb_buffer, NULL, viewport, &con);
                std::lock_guard<std::mutex> lock(mtx);
                memcpy(shm->camera_pixels, rgb_buffer, sizeof(rgb_buffer));
            }
        }

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