#include "controller.h"

SweeperController::SweeperController(double kp, double kd) : Kp(kp), Kd(kd) {}

void SweeperController::compute(const mjModel* m, mjData* d, double target_pos[6]) {
    // Simple PD control for each joint
    for (int i = 0; i < 6; ++i) {
        // PD Control law: Force = Kp * (error) + Kd * (velocity_error)
        // target_pos[i] is the desired joint angle
        double error = target_pos[i] - d->qpos[i];
        double vel_error = 0 - d->qvel[i]; // Aim for 0 velocity at the target
        
        d->ctrl[i] = (Kp * error) + (Kd * vel_error);
    }
}