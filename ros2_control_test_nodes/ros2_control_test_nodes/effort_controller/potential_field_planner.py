import numpy as np


def is_at_goal(pos_goal, pos_fbk):
    error = 0.001
    for pos_goal_el, pos_fbk_el in zip(pos_goal, pos_fbk):
        if not (pos_goal_el - error <= pos_fbk_el <= pos_goal_el + error):
            return False
    return True


def is_HAA_symmetric(pos_fbk):
    error = 0.001
    return ((pos_fbk[0] - error <= -1 * pos_fbk[3] <= pos_fbk[0] + error) and (
                pos_fbk[6] - error <= -1 * pos_fbk[9] <= pos_fbk[6] + error))


class PotentialFieldPlanner:

    def __init__(self, k_attr, delta_t, pos_target):
        self.k_attr = k_attr
        self.delta_t = delta_t
        self.pos_target = pos_target

    def compute_ref_vel(self, pos_fbk):
        return self.k_attr * (self.pos_target - pos_fbk)

    def compute_ref_pos(self, pos_fbk):
        return pos_fbk + self.compute_ref_vel(pos_fbk) * self.delta_t
