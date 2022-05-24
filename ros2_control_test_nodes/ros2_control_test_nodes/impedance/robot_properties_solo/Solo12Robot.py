from .Solo12Config import Solo12Config


class Solo12Robot:

    def __init__(self):
        self.base_link_name = "base_link"
        self.end_effector_names = []
        controlled_joints = []

        for leg in ["FL", "FR", "HL", "HR"]:
            controlled_joints += [leg + "_HAA", leg + "_HFE", leg + "_KFE"]
            self.end_effector_names.append(leg + "_FOOT")

        self.joint_names = controlled_joints
        self.nb_ee = len(self.end_effector_names)

        pin_robot = Solo12Config.buildRobotWrapper()
        self.nv = pin_robot.nv
        self.nb_dof = self.nv - 6
        self.pin_robot = pin_robot

