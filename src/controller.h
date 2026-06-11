#pragma once
#include <mujoco/mujoco.h>
#include <vector>

class SweeperController {
public:
    SweeperController(double kp, double kd);

    // Set the desired end-effector target position (for teleoperation)
    void setTarget(double x, double y, double z);

    // Calculates and applies torques to track a desired joint velocity
    // Joint velocities are computed directly from task-space end effector error
    void compute(const mjModel* m, mjData* d);

    // Enable or disable keyboard teleoperation (when disabled, controller will sample random targets)
    void setTeleopEnabled(bool enabled);

    // Enable autonomous sweeping (pushes each puck into its matching goal zone)
    void setAutoSweepEnabled(bool enabled);

    // True once all pucks are delivered and the arm is parking
    bool isDone();

    // Restart the sweeping state machine for a fresh episode
    void resetEpisode();

private:
    double Kp;
    double Kd;

    int end_site_id;
    double current_target[3];
    double smoothed_target[3];
    bool smoothed_target_valid;
    double next_target_time;
    bool teleop_enabled;

    // state machine for sweeping pucks into goal zones autonomously
    enum class SweepPhase { Lift, Approach, Descend, Push, Retreat, Done };
    bool auto_sweep_enabled;
    SweepPhase sweep_phase;
    int sweep_task;
    double sweep_phase_start;
    int puck_body_ids[3];
    double goal_xy[3][2];

    std::vector<double> jacp;

    void initialize(const mjModel* m, mjData* d);
    void sampleRandomTarget(double current_time);
    void updateAutoSweep(const mjModel* m, mjData* d);
};