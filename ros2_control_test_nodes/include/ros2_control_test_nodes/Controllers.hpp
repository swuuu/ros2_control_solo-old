//
// Created by stanley on 6/6/22.
//

#ifndef ROS2_CONTROL_TEST_NODES_CONTROLLERS_HPP
#define ROS2_CONTROL_TEST_NODES_CONTROLLERS_HPP


class Controllers : public rclcpp::Node {
public:
    Controllers();
private:
    // subscribers, publishers, and services
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // methods
    void update_joint_states(const sensor_msgs::msg::JointState &msg);

    void update_body_state(const gazebo_msgs::msg::LinkStates &msg);

    void timer_callback(const sensor_msgs::msg::JointState &msg) const;

    // fields
    Eigen::VectorXd joint_config = Eigen::VectorXd::Zero(12, 1);
    Eigen::VectorXd joint_velocity = Eigen::VectorXd::Zero(12, 1);
    Eigen::VectorXd robot_pose = Eigen::VectorXd::Zero(7, 1);
    Eigen::VectorXd robot_twist = Eigen::VectorXd::Zero(6, 1);
};

#endif //ROS2_CONTROL_TEST_NODES_CONTROLLERS_HPP
