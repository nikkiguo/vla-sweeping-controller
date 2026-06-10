#include "controller.h"
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>

SweeperController::SweeperController(double kp, double kd): Kp(kp), Kd(kd), end_site_id(-1), next_target_time(0.0), teleop_enabled(true),
    auto_sweep_enabled(false), sweep_phase(SweepPhase::Lift), sweep_task(0), sweep_phase_start(0.0) {
    std::srand(static_cast<unsigned>(std::time(nullptr)));
    current_target[0] = 0.3;
    current_target[1] = 0.0;
    current_target[2] = 0.35;

    // Initialize puck body IDs and goal zone positions for auto-sweep
    for (int i = 0; i < 3; ++i) {
        puck_body_ids[i] = -1;
        goal_xy[i][0] = 0.0;
        goal_xy[i][1] = 0.0;
    }
}

void SweeperController::setTeleopEnabled(bool enabled) {
    teleop_enabled = enabled;
}

void SweeperController::setAutoSweepEnabled(bool enabled) {
    auto_sweep_enabled = enabled;
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

    // Match puck to goal zone by name
    // Order of sweeping: outer corner pucks -> blue puck toward the robot base
    const char* puck_names[3] = {"green_puck", "red_box", "blue_puck"};
    const char* zone_names[3] = {"zone_green", "zone_red", "zone_blue"};
    for (int i = 0; i < 3; ++i) {
        puck_body_ids[i] = mj_name2id(m, mjOBJ_BODY, puck_names[i]);
        int zone_geom_id = mj_name2id(m, mjOBJ_GEOM, zone_names[i]);
        if (puck_body_ids[i] < 0 || zone_geom_id < 0) {
            std::cerr << "[SweeperController]: cannot find '" << puck_names[i] << "' or '" << zone_names[i] << "' in model" << std::endl;
            continue;
        }
        goal_xy[i][0] = m->geom_pos[3 * zone_geom_id + 0];
        goal_xy[i][1] = m->geom_pos[3 * zone_geom_id + 1];
    }

    // Sample an initial random target only when teleoperation is disabled
    if (!teleop_enabled && !auto_sweep_enabled) {
        sampleRandomTarget(d->time);
    }
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

// Get behind the active puck, descend, and push it toward its zone, re-aiming from live puck positions every cycle
void SweeperController::updateAutoSweep(const mjModel* m, mjData* d) {
    const double travel_z = 0.25;     // safe height for moving between pucks
    const double push_z = 0.08;       // pushing height
    const double behind_dist = 0.16;  // EE standoff behind the puck
    const double goal_tol = 0.06;     // success radius in goal zone
    const double reach_tol = 0.04;    // how close the EE must get before the next phase starts

    if (sweep_phase == SweepPhase::Done || sweep_task >= 3 || puck_body_ids[sweep_task] < 0) {
        // Move back to initial position (parking)
        current_target[0] = 0.3;
        current_target[1] = 0.0;
        current_target[2] = 0.35;
        return;
    }

    const double* ee = d->site_xpos + 3 * end_site_id;
    const double* puck = d->xpos + 3 * puck_body_ids[sweep_task];
    const double* goal = goal_xy[sweep_task];

    // Log Status every 5s
    static long status_counter = 0;
    if (status_counter++ % 2500 == 0) {
        const char* phase_names[] = {"Lift", "Approach", "Descend", "Push", "Retreat", "Done"};
        std::cout << "[SweeperController]: task=" << sweep_task << " phase=" << phase_names[(int)sweep_phase]
                  << " ee=[" << ee[0] << ", " << ee[1] << ", " << ee[2] << "]"
                  << " puck=[" << puck[0] << ", " << puck[1] << "]"
                  << " goal=[" << goal[0] << ", " << goal[1] << "]" << std::endl;
    }

    // Direction the puck travels to the goal zone (xy plane)
    double to_goal[2] = {goal[0] - puck[0], goal[1] - puck[1]};
    double dist_to_goal = std::sqrt(to_goal[0] * to_goal[0] + to_goal[1] * to_goal[1]);
    double dir[2] = {to_goal[0] / std::max(dist_to_goal, 1e-6), to_goal[1] / std::max(dist_to_goal, 1e-6)};

    // Puck linear speed
    int jid = m->body_jntadr[puck_body_ids[sweep_task]];
    const double* puck_vel = d->qvel + m->jnt_dofadr[jid];
    double puck_speed = std::sqrt(puck_vel[0] * puck_vel[0] + puck_vel[1] * puck_vel[1]);

    // Change phase and stamp its start time (used for stall timeouts)
    auto setPhase = [&](SweepPhase next) {
        sweep_phase = next;
        sweep_phase_start = d->time;
    };
    double phase_elapsed = d->time - sweep_phase_start;

    // Check if the puck is in the goal zone
    if (sweep_phase != SweepPhase::Retreat && dist_to_goal < goal_tol && puck_speed < 0.05) {
        std::cout << "[SweeperController]: Puck " << sweep_task << " delivered to its zone" << std::endl;
        setPhase(SweepPhase::Retreat);
    }

    // Standoff point behind the puck, on the line through the goal
    double behind[2] = {puck[0] - dir[0] * behind_dist, puck[1] - dir[1] * behind_dist};

    switch (sweep_phase) {
        case SweepPhase::Lift: {
            // Rise straight up before any horizontal travel
            current_target[0] = ee[0];
            current_target[1] = ee[1];
            current_target[2] = travel_z;
            if (ee[2] > travel_z - reach_tol || phase_elapsed > 6.0) {
                setPhase(SweepPhase::Approach);
            }
            break;
        }
        case SweepPhase::Approach: {
            // Hover above the standoff point behind the puck
            current_target[0] = behind[0];
            current_target[1] = behind[1];
            current_target[2] = travel_z;
            double dx = ee[0] - behind[0], dy = ee[1] - behind[1];
            if (std::sqrt(dx * dx + dy * dy) < reach_tol || phase_elapsed > 8.0) {
                setPhase(SweepPhase::Descend);
            }
            break;
        }
        case SweepPhase::Descend: {
            // Drop down to pushing height behind the puck
            current_target[0] = behind[0];
            current_target[1] = behind[1];
            current_target[2] = push_z;
            if (ee[2] < push_z + reach_tol || phase_elapsed > 6.0) {
                setPhase(SweepPhase::Push);
            }
            break;
        }
        case SweepPhase::Push: {
            // Push by tracking a point slightly past the puck center toward the goal
            current_target[0] = puck[0] + dir[0] * 0.04;
            current_target[1] = puck[1] + dir[1] * 0.04;
            current_target[2] = push_z;

            // Re-approach if the EE is no longer behind the puck (deflection/overshoot)
            double to_puck[2] = {puck[0] - ee[0], puck[1] - ee[1]};
            double to_puck_norm = std::sqrt(to_puck[0] * to_puck[0] + to_puck[1] * to_puck[1]);
            double alignment = (to_puck[0] * dir[0] + to_puck[1] * dir[1]) / std::max(to_puck_norm, 1e-6);
            if (alignment < 0.5 || phase_elapsed > 15.0) {
                setPhase(SweepPhase::Lift);
            }
            break;
        }
        case SweepPhase::Retreat: {
            // Lift clear then move on to the next puck
            current_target[0] = ee[0];
            current_target[1] = ee[1];
            current_target[2] = travel_z;
            if (ee[2] > travel_z - reach_tol || phase_elapsed > 6.0) {
                sweep_task++;
                if (sweep_task >= 3) {
                    std::cout << "[SweeperController]: All pucks delivered, parking" << std::endl;
                    setPhase(SweepPhase::Done);
                } else {
                    std::cout << "[SweeperController]: Sweeping puck " << sweep_task << std::endl;
                    setPhase(SweepPhase::Lift);
                }
            }
            break;
        }
        case SweepPhase::Done:
            break;
    }
}

// Set the current target explicitly (used for keyboard teleoperation)
void SweeperController::setTarget(double x, double y, double z) {
    current_target[0] = x;
    current_target[1] = y;
    current_target[2] = z;
    next_target_time = 0.0;
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

    // Update the IK target from the sweep state machine
    if (auto_sweep_enabled) {
        updateAutoSweep(m, d);
    }

    // Get error in 3D space (error = target_position - current_end_effector_position)
    double* site_pos = d->site_xpos + 3 * end_site_id;
    double error[3] = {current_target[0] - site_pos[0], current_target[1] - site_pos[1], current_target[2] - site_pos[2]};

    // Refresh target if needed (when close enough to current target or after a timeout)
    // Only run auto-switching in random-target mode
    if (!teleop_enabled && !auto_sweep_enabled) {
        if (mju_norm3(error) < 0.03 || d->time >= next_target_time) {
            sampleRandomTarget(d->time);
            error[0] = current_target[0] - site_pos[0];
            error[1] = current_target[1] - site_pos[1];
            error[2] = current_target[2] - site_pos[2];
        }
    }

    // Get Jacobian for the end effector site
    // The Jacobian is made up of partial derivatives that tell us how changes in joint angles affect the position of the end effector in 3D space
    mj_jacSite(m, d, jacp.data(), NULL, end_site_id);

    // Cap movement speed to ensure smooth and controlled approach
    double err_cmd[3] = {error[0], error[1], error[2]};
    double err_norm = mju_norm3(err_cmd);
    const double err_max = 0.10;
    if (err_norm > err_max) {
        mju_scl3(err_cmd, err_cmd, err_max / err_norm); // Normalize and scale
    }

    // Map Cartesian error to joint space via Jacobian Transpose (dq = J^T * e * gain)
    // This computes the joint velocities required to move the end-effector toward the target
    const double gain = 25.0;
    std::vector<double> qdot(nv);
    for (int i = 0; i < nv; ++i) {
        double dq = 0.0;
        for (int j = 0; j < 3; ++j) {
            dq += jacp[j * nv + i] * err_cmd[j] * gain;
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