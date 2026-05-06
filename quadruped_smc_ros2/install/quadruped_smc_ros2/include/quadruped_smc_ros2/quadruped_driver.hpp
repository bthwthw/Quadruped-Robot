#ifndef QUADRUPED_SMC_ROS2__QUADRUPED_DRIVER_HPP_
#define QUADRUPED_SMC_ROS2__QUADRUPED_DRIVER_HPP_

#include <unordered_map>
#include <string>
#include <memory>
#include <vector>

#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "quadruped_smc_ros2/GaitPlanner.hpp"
#include "quadruped_smc_ros2/LegController2.hpp"  // chỉ dùng Model-Based

namespace quadruped_smc_ros2
{

class QuadrupedDriver : public webots_ros2_driver::PluginInterface
{
public:
    void init(webots_ros2_driver::WebotsNode* node,
              std::unordered_map<std::string, std::string>& parameters) override;
    void step() override;

private:
    static constexpr double STANDUP_DURATION = 0.5;
    static constexpr double ALPHA_FILTER     = 0.15;

    webots_ros2_driver::WebotsNode* node_{nullptr};
    WbDeviceTag motors_[8]{};
    WbDeviceTag sensors_[8]{};
    int    time_step_{};
    double dt_{};
    double start_time_{0.0};
    bool   start_time_set_{false};

    double q_[8]{},        q_prev_[8]{};
    double dq_[8]{};
    double q_ref_[8]{},    q_ref_prev_[8]{},  q_ref_prev2_[8]{};
    double dq_ref_[8]{},   ddq_ref_[8]{};
    bool   first_step_{true};

    char current_gait_{'T'};
    GaitPlanner  planner_;
    LegController2 leg_controllers_[4];  // Model-Based SMC

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr         gait_status_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr      gait_cmd_sub_;

    void publishJointStates();
    void publishGaitStatus();
};

}  // namespace quadruped_smc_ros2

#endif