//
// Created by stanley on 6/6/22.
//

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "gazebo_msgs/msg/link_states.hpp"
#include <Eigen/Dense>
#include <unordered_map>
#include "ros2_control_test_nodes/Controllers.hpp"

using std::placeholders::_1;

Controllers::Controllers() : Node("test_controller_cpp") {
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controllers/commands", 10);
    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(
            &Controllers::update_joint_states, this, _1));
}

void Controllers::update_joint_states(const sensor_msgs::msg::JointState &msg) {
    std::unordered_map<std::string, int> order_of_names = {{"FL_HAA", 0},
                                                           {"FL_HFE", 1},
                                                           {"FL_KFE", 2},
                                                           {"FR_HAA", 3},
                                                           {"FR_HFE", 4},
                                                           {"FR_KFE", 5},
                                                           {"HL_HAA", 6},
                                                           {"HL_HFE", 7},
                                                           {"HL_KFE", 8},
                                                           {"HR_HAA", 9},
                                                           {"HR_HFE", 10},
                                                           {"HR_KFE", 11}};
    for (int i = 0; i < 12; i++) {
        std::string name = msg.name[i];
        joint_config(order_of_names[name]) = msg.position[i];
        joint_velocity(order_of_names[name]) = msg.velocity[i];
    }
}

void Controllers::update_body_state(const gazebo_msgs::msg::LinkStates &msg) {
    robot_pose(0) = msg.pose[1].position.x;
    robot_pose(1) = msg.pose[1].position.y;
    robot_pose(2) = msg.pose[1].position.z;
    robot_pose(3) = msg.pose[1].orientation.x;
    robot_pose(4) = msg.pose[1].orientation.y;
    robot_pose(5) = msg.pose[1].orientation.z;
    robot_pose(6) = msg.pose[1].orientation.w;
}
