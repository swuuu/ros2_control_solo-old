import pinocchio as pin
import numpy as np


class PinocchioHelperFunctions:

    def __init__(self, urdf_model):
        # load the URDF model
        self.model = pin.buildModelFromUrdf(urdf_model)
        self.data = self.model.createData()

    def forward_kinematics(self, q):
        foot_positions = np.zeros((4, 3))  # 4x3
        q_list = np.array(list(q))
        pin.forwardKinematics(self.model, self.data, q_list)
        curr_foot = 0
        for i in range(20):
            print(self.model.names[i])
        print(self.model.names)
        print(self.data.oMi)
        for i, (name, oMi) in enumerate(zip(self.model.names, self.data.oMi)):
            print(("{:<24} : {: .2f} {: .2f} {: .2f}"
                   .format(name, *oMi.translation.T.flat)))
            # TODO: Verify if you get the correct positions of the joints
            # TODO: Figure joint IDs of the 4 ankles
            if i % 5 == 0:
                foot_positions[curr_foot] = oMi.translation.T.flat
                curr_foot += 1
        return foot_positions

    def get_mass_matrix(self, q, v):
        # pin.computeAllTerms(self.model, self.data, q, v)
        M = pin.crba(self.model, self.data, q)
        # return self.data.M
        return M

    def get_h(self, q, v):
        # pin.computeAllTerms(self.model, self.data, q, v)
        h = pin.nonLinearEffects(self.model, self.data, q, v)
        # return self.data.nle
        return h

    def get_jacobian(self):
        return pin.getJointJacobian(self.model, self.data, reference_frame=pin.ReferenceFrame.WORLD)

    def get_jacobian_dot(self, q, v, id_ankle):
        pin.computeJointJacobiansTimeVariation(self.model, self.data, q, v)
        return pin.getJointJacobianTimeVariation(self.model, self.data, id_ankle,
                                                 reference_frame=pin.ReferenceFrame.WORLD)

    
