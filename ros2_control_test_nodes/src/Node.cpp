//
// Created by stanley on 6/6/22.
//
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "gazebo_msgs/msg/link_state.hpp"

#include "ros2_control_test_nodes/Controllers.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controllers>());
    rclcpp::shutdown();
    return 0;
}
