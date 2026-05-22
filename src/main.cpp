#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic>

// Shared resources
std::mutex mtx;
std::atomic<bool> exit_simulation{false};

// Physics thread function
void physics_thread(mjModel* m, mjData* d) {
    // Match the timestep defined in sweeping_scene.xml (0.002s = 2ms = 500 Hz)
    auto timestep = std::chrono::milliseconds(2);

    while (!exit_simulation) {
        auto next_tick = std::chrono::steady_clock::now() + timestep;

        // Perform physics stepping
        {
            // Lock only for the math
            std::lock_guard<std::mutex> lock(mtx);
            
            // Apply a hardcoded sine wave to the first joint to see it move
            d->ctrl[0] = 1.5 * sin(d->time); 
            d->ctrl[1] = -1.0 + 0.5 * sin(d->time * 0.5); // Dynamic torque profile to simulate reach and sweep motion
            mj_step(m, d);
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
    
    GLFWwindow* window = glfwCreateWindow(800, 600, "VLA Sweeping Sandbox", NULL, NULL);
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

    // Start the physics thread
    std::thread physics_worker(physics_thread, m, d_physics);

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
        mjrRect viewport = {0, 0, 800, 600};
        mjv_updateScene(m, d_render, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

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
    glfwTerminate();
    return 0;
}