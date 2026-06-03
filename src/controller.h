#pragma once
#include <mujoco/mujoco.h>
#include <vector>

class SweeperController {
public:
    SweeperController(double kp, double kd);

    // Calculates and applies torques to track a desired joint velocity
    // Joint velocities are computed directly from task-space end effector error
    void compute(const mjModel* m, mjData* d);

private:
    double Kp;
    double Kd;

    int end_site_id;
    double current_target[3];
    double next_target_time;

    std::vector<double> jacp;

    void initialize(const mjModel* m, mjData* d);
    void sampleRandomTarget(double current_time);
};