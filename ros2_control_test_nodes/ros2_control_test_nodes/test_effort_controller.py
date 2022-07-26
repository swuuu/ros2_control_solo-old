import rclpy
from .mim_control.node import RobotImpedanceController


def main(args=None):
    rclpy.init(args=args)
    impedance_node = RobotImpedanceController()
    rclpy.spin(impedance_node)
    impedance_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
