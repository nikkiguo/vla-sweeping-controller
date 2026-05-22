#pragma once
#include <mujoco/mujoco.h>

class SweeperController {
public:
    SweeperController(double kp, double kd);
    
    // Calculates and applies torques based on a target coordinate
    void compute(const mjModel* m, mjData* d, double target_pos[7]);

private:
    double Kp;
    double Kd;
};