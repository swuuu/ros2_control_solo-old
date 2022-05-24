import rclpy
from .impedance.node import RobotImpedanceController


def main(args=None):
    rclpy.init(args=args)
    # effort_node = EffortNode()
    # rclpy.spin(effort_node)
    # effort_node.destroy_node()
    impedance_node = RobotImpedanceController()
    rclpy.spin(impedance_node)
    impedance_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
