/**
 * quadruped_driver.cpp
 *
 * Topics:
 *   Subscribe: /gait_command  → "trot" | "bound" | "pace" | "walk"
 *   Publish:   /joint_states
 *   Publish:   /gait_status
 *
 * Controller: Model-Based SMC (LegController2)
 */

#include "quadruped_smc_ros2/quadruped_driver.hpp"
#include <cmath>
#include <cctype>

namespace quadruped_smc_ros2
{

static inline double clamp(double val, double lo, double hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

static const char* MOTOR_NAMES[8] = {
    "motor_FL_hip",  "motor_FL_knee",
    "motor_FR_hip",  "motor_FR_knee",
    "motor_BR_hip",  "motor_BR_knee",
    "motor_BL_hip",  "motor_BL_knee"
};

static const char* SENSOR_NAMES[8] = {
    "sensor_FL_hip",  "sensor_FL_knee",
    "sensor_FR_hip",  "sensor_FR_knee",
    "sensor_BR_hip",  "sensor_BR_knee",
    "sensor_BL_hip",  "sensor_BL_knee"
};

static const std::vector<std::string> JOINT_NAMES = {
    "FL_hip", "FL_knee",
    "FR_hip", "FR_knee",
    "BR_hip", "BR_knee",
    "BL_hip", "BL_knee"
};

// ─────────────────────────────────────────────────────────────
//  init()
// ─────────────────────────────────────────────────────────────
void QuadrupedDriver::init(
    webots_ros2_driver::WebotsNode* node,
    std::unordered_map<std::string, std::string>& /*parameters*/)
{
    node_      = node;
    time_step_ = (int)wb_robot_get_basic_time_step();
    dt_        = time_step_ / 1000.0;

    for (int i = 0; i < 8; ++i) {
        motors_[i]  = wb_robot_get_device(MOTOR_NAMES[i]);
        sensors_[i] = wb_robot_get_device(SENSOR_NAMES[i]);

        if (sensors_[i] != 0)
            wb_position_sensor_enable(sensors_[i], time_step_);

        if (motors_[i] != 0) {
            wb_motor_set_position(motors_[i], 0.0);
            wb_motor_set_velocity(motors_[i], wb_motor_get_max_velocity(motors_[i]));
        }
    }

    for (int i = 0; i < 8; ++i) {
        q_[i] = q_prev_[i] = dq_[i]      = 0.0;
        q_ref_[i] = q_ref_prev_[i]       = 0.0;
        q_ref_prev2_[i] = dq_ref_[i]     = 0.0;
        ddq_ref_[i]                       = 0.0;
    }

    planner_.get_Trot(0.0, q_ref_);

    // Publishers
    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS());

    gait_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/gait_status", 10);

    // Subscriber
    gait_cmd_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/gait_command", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            if (!msg->data.empty()) {
                char c = static_cast<char>(std::toupper(
                    static_cast<unsigned char>(msg->data[0])));
                if (c == 'T' || c == 'B' || c == 'P' || c == 'W') {
                    current_gait_ = c;
                    RCLCPP_INFO(node_->get_logger(),
                        "[QuadrupedDriver] Gait → %c", current_gait_);
                }
            }
        });

    start_time_set_ = false;
    first_step_     = true;

    RCLCPP_INFO(node_->get_logger(),
        "[QuadrupedDriver] Initialized | dt=%.4fs | Controller: Model-Based SMC",
        dt_);
}

// ─────────────────────────────────────────────────────────────
//  step()
// ─────────────────────────────────────────────────────────────
void QuadrupedDriver::step()
{
    double now = wb_robot_get_time();
    if (!start_time_set_) {
        start_time_     = now;
        start_time_set_ = true;
    }
    double t_elapsed = now - start_time_;

    // ── 1. Gait Planner ──────────────────────────────────────
    for (int i = 0; i < 8; ++i) {
        q_ref_prev2_[i] = q_ref_prev_[i];
        q_ref_prev_[i]  = q_ref_[i];
    }

    double gait_time = (t_elapsed < STANDUP_DURATION)
                       ? 0.0 : (t_elapsed - STANDUP_DURATION);

    switch (current_gait_) {
        case 'T': planner_.get_Trot (gait_time, q_ref_); break;
        case 'B': planner_.get_Bound(gait_time, q_ref_); break;
        case 'P': planner_.get_Pace (gait_time, q_ref_); break;
        case 'W': planner_.get_Walk (gait_time, q_ref_); break;
        default:  planner_.get_Trot (gait_time, q_ref_); break;
    }

    // ── 2. Đọc cảm biến & tính vận tốc / gia tốc ────────────
    for (int i = 0; i < 8; ++i) {
        dq_ref_[i] = (q_ref_[i] - q_ref_prev_[i]) / dt_;
        dq_ref_[i] = clamp(dq_ref_[i], -20.0, 20.0);

        ddq_ref_[i] = (q_ref_[i] - 2.0 * q_ref_prev_[i] + q_ref_prev2_[i])
                      / (dt_ * dt_);
        ddq_ref_[i] = clamp(ddq_ref_[i], -200.0, 200.0);

        if (sensors_[i] != 0)
            q_[i] = wb_position_sensor_get_value(sensors_[i]);

        if (first_step_) q_prev_[i] = q_[i];

        double raw_dq = (q_[i] - q_prev_[i]) / dt_;
        dq_[i] = ALPHA_FILTER * raw_dq + (1.0 - ALPHA_FILTER) * dq_[i];
        dq_[i] = clamp(dq_[i], -30.0, 30.0);

        q_prev_[i] = q_[i];
    }
    first_step_ = false;

    // ── 3. Motor Control (Model-Based SMC) ───────────────────
    for (int leg = 0; leg < 4; ++leg) {
        int h = leg * 2;
        int k = leg * 2 + 1;

        if (t_elapsed < STANDUP_DURATION) {
            if (motors_[h] != 0) wb_motor_set_position(motors_[h], q_ref_[h]);
            if (motors_[k] != 0) wb_motor_set_position(motors_[k], q_ref_[k]);
        } else {
            if (motors_[h] != 0) wb_motor_set_position(motors_[h], INFINITY);
            if (motors_[k] != 0) wb_motor_set_position(motors_[k], INFINITY);

            double q_r_leg[2]   = {q_ref_[h],   q_ref_[k]};
            double dq_r_leg[2]  = {dq_ref_[h],  dq_ref_[k]};
            double ddq_r_leg[2] = {ddq_ref_[h], ddq_ref_[k]};
            double q_leg[2]     = {q_[h],        q_[k]};
            double dq_leg[2]    = {dq_[h],       dq_[k]};
            double tau_out[2]   = {0.0, 0.0};

            leg_controllers_[leg].compute_SMC(
                leg, dt_, q_r_leg, q_leg, dq_r_leg, dq_leg, ddq_r_leg, tau_out);

            if (motors_[h] != 0) wb_motor_set_torque(motors_[h], tau_out[0]);
            if (motors_[k] != 0) wb_motor_set_torque(motors_[k], tau_out[1]);
        }
    }

    // ── 4. Publish ROS2 ──────────────────────────────────────
    publishJointStates();
    publishGaitStatus();
}

// ─────────────────────────────────────────────────────────────
// void QuadrupedDriver::publishJointStates()
// {
//     auto msg = sensor_msgs::msg::JointState();
//     msg.header.stamp = node_->get_clock()->now();
//     msg.name         = JOINT_NAMES;
//     msg.position.assign(q_,    q_ + 8);
//     msg.velocity.assign(dq_,   dq_ + 8);
//     msg.effort.assign(q_ref_,  q_ref_ + 8);
//     joint_state_pub_->publish(msg);
// }

void QuadrupedDriver::publishJointStates()
{
    auto msg = sensor_msgs::msg::JointState();
    double sim_time = wb_robot_get_time();   // đơn vị giây

    int32_t sec = static_cast<int32_t>(sim_time);
    uint32_t nanosec = static_cast<uint32_t>((sim_time - sec) * 1e9);
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;

    msg.name = JOINT_NAMES;
    msg.position.assign(q_, q_ + 8);
    msg.velocity.assign(dq_, dq_ + 8);
    msg.effort.assign(q_ref_, q_ref_ + 8);

    joint_state_pub_->publish(msg);
}

// ─────────────────────────────────────────────────────────────
void QuadrupedDriver::publishGaitStatus()
{
    static char last_pub = '\0';
    if (current_gait_ != last_pub) {
        auto msg = std_msgs::msg::String();
        switch (current_gait_) {
            case 'T': msg.data = "trot";  break;
            case 'B': msg.data = "bound"; break;
            case 'P': msg.data = "pace";  break;
            case 'W': msg.data = "walk";  break;
            default:  msg.data = "trot";  break;
        }
        gait_status_pub_->publish(msg);
        last_pub = current_gait_;
    }
}

}  // namespace quadruped_smc_ros2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    quadruped_smc_ros2::QuadrupedDriver,
    webots_ros2_driver::PluginInterface)