#include "controller.h"
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>

SweeperController::SweeperController(double kp, double kd): Kp(kp), Kd(kd), end_site_id(-1), next_target_time(0.0) {
    std::srand(static_cast<unsigned>(std::time(nullptr)));
    current_target[0] = 0.3;
    current_target[1] = 0.0;
    current_target[2] = 0.35;
}

void SweeperController::initialize(const mjModel* m, mjData* d) {
    if (end_site_id >= 0) {
        return;
    }

    // Find the end effector site ID by name in the robot model
    end_site_id = mj_name2id(m, mjOBJ_SITE, "attachment_site");
    if (end_site_id < 0) {
        std::cerr << "[SweeperController]: cannot find end effector site 'attachment_site' in model" << std::endl;
        return;
    }

    // Resize Jacobian storage based on model dimensions
    jacp.resize(3 * m->nv);
    sampleRandomTarget(d->time);
}

// Sample a random double in a specified range
static double randomDouble(double min_value, double max_value) {
    return min_value + (max_value - min_value) * (std::rand() / static_cast<double>(RAND_MAX));
}

// Sample a new random target and set the next target time to a random interval in the future
void SweeperController::sampleRandomTarget(double current_time) {
    current_target[0] = randomDouble(0.15, 0.45);
    current_target[1] = randomDouble(-0.25, 0.25);
    current_target[2] = randomDouble(0.15, 0.5);
    next_target_time = current_time + randomDouble(3.0, 5.0);

    std::cout << "[SweeperController]: New random IK target: [" << current_target[0] << ", " << current_target[1] << ", " << current_target[2] << "]" << std::endl;
}

void SweeperController::compute(const mjModel* m, mjData* d) {
    initialize(m, d);
    if (end_site_id < 0) {
        return;
    }

    // Get latest kinematic state
    mj_forward(m, d);

    int nv = m->nv;
    int nu = m->nu;

    // Get error in 3D space (error = target_position - current_end_effector_position)
    double* site_pos = d->site_xpos + 3 * end_site_id;
    double error[3] = {current_target[0] - site_pos[0], current_target[1] - site_pos[1], current_target[2] - site_pos[2]};

    // Refresh target if needed (when close enough to current target or after a timeout)
    if (mju_norm3(error) < 0.03 || d->time >= next_target_time) {
        sampleRandomTarget(d->time);
        error[0] = current_target[0] - site_pos[0];
        error[1] = current_target[1] - site_pos[1];
        error[2] = current_target[2] - site_pos[2];
    }

    // Get Jacobian for the end effector site
    // The Jacobian is made up of partial derivatives that tell us how changes in joint angles affect the position of the end effector in 3D space
    mj_jacSite(m, d, jacp.data(), NULL, end_site_id);

    // Map Cartesian error to joint space via Jacobian Transpose (dq = J^T * e * gain)
    // This computes the joint velocities required to move the end-effector toward the target
    const double gain = 5.0;
    std::vector<double> qdot(nv);
    for (int i = 0; i < nv; ++i) {
        double dq = 0.0;
        for (int j = 0; j < 3; ++j) {
            dq += jacp[j * nv + i] * error[j] * gain;
        }
        qdot[i] = dq;
    }

    // Apply torque proportional to the difference between desired joint velocity (qdot) and actual joint velocity (d->qvel)
    const double vel_gain = 15.0;
    for (int i = 0; i < nu; ++i) {
        // Torque = K_v * (Desired_Velocity - Current_Velocity)
        d->ctrl[i] = vel_gain * (qdot[i] - d->qvel[i]);
    }
}