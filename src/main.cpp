#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>

int main() {
    char error[1000];
    mjModel* m = mj_loadXML("../models/sweeping_scene.xml", 0, error, 1000);
    
    // Check if XML loading failed
    if (!m) {
        std::cerr << "Failed to load XML model: " << error << std::endl;
        return 1;
    }
    
    mjData* d = mj_makeData(m);

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
        mj_deleteData(d);
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

    while (!glfwWindowShouldClose(window)) {
        // Apply a hardcoded sine wave to the first joint to see it move
        d->ctrl[0] = 1.5 * sin(d->time); 
        d->ctrl[1] = -1.0; // Constant torque to help lift the arm

        double start_time = d->time;
        while (d->time - start_time < 0.01) { // Only allow 10ms of physics work per frame
            mj_step(m, d);
        }

        // Update scene and render
        mjrRect viewport = {0, 0, 800, 600};
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Clean up
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
    return 0;
}