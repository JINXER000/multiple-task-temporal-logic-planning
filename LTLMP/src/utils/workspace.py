import numpy as np
import random

import sys
sys.path.append("/home/joseph/yzchen_ws/task_planning/AI-plan/pybullet-planning/")
sys.path.append("/home/joseph/yzchen_ws/task_planning/ltl_mq_opensource/src/pybullet-planning")
sys.path.append("/home/joseph/yzchen_ws/task_planning/ltl_mq_opensource/src/LTLMP/src/utils/")

from pyb_plan_utils import AABB
from misc_func import setup_pybullet


def calc_ur_given_ll(ll_coords, reg_side):
    ur_coords = []
    for ll_crd in ll_coords:
        ur_crd = [ll_crd[i] + reg_side for i in range(len(ll_crd))]
        ur_coords.append(ur_crd)

    return  ur_coords

def calc_reg_center(ll_coord, reg_len):
    return (ll_coord[0]+ 0.5*reg_len, ll_coord[1]+ 0.5*reg_len)




class Workspace3D(object):
    def __init__(self):
        self.length = 7.6
        self.width = 6.2
        self.height = 2.2
        self.workspace_range = (self.length, self.width, self.height)
        self.workspace_ll = (-3.4, -3.0, 0.)
        self.workspace_ur = (self.workspace_ll[0]+ self.length, self.workspace_ll[1]+ self.width, self.workspace_ll[2] + self.height)
        self.ws_AABB = AABB(self.workspace_ll, self.workspace_ur)
        reg_side = 1.2

        # regions
        reg_ll_coords = [(2.0, 1.0, 0.0), (-1.0, -3.0, 1.), (-3.0, 0.0, 0.0)]
        reg_ur_coords = calc_ur_given_ll(reg_ll_coords, reg_side)
        self.regions = {'l1': AABB(reg_ll_coords[0], reg_ur_coords[0]),
                        'l2': AABB(reg_ll_coords[1], reg_ur_coords[1]),
                        'l3': AABB(reg_ll_coords[2], reg_ur_coords[2])}
        
        self.reg_set = {k for k in self.regions}
        self.properties = {'l1': 'grassland', 'l2': 'pond', 'l3': 'grassland'}

        self.reg_centers = {
            'l1': tuple((ll + ur) / 2 for ll, ur in zip(reg_ll_coords[0], reg_ur_coords[0])),
            'l2':tuple((ll + ur) / 2 for ll, ur in zip(reg_ll_coords[1], reg_ur_coords[1])),
            'l3': tuple((ll + ur) / 2 for ll, ur in zip(reg_ll_coords[2], reg_ur_coords[2])),
        }

        # obstacles
        obs_ll_coords = [(-3.4, -3.0, 0.), (2.0, -3.0, 0.), (-1.0, 1.0, 0)]
        obs_ur_coords = [(-2.0, -2.0, 1.0), (4.0, -1.5, 1.0), (0.0, 3.0, 0.2)]
        self.obstacles = {'o1': AABB(obs_ll_coords[0], obs_ur_coords[0]),
                        'o2': AABB(obs_ll_coords[1], obs_ur_coords[1]),
                        'o3': AABB(obs_ll_coords[2], obs_ur_coords[2])}

        # dynamic obstacle
        self.do_ll, self.do_ur = (-50, -30,10), (-15, -20, 10)
        self.dyn_obs = {'d1': AABB(self.do_ll, self.do_ur) }
        self.dynamic_obs_center = ((self.do_ur[0] +self.do_ll[0])* 0.5, (self.do_ur[1] +self.do_ll[1])* 0.5, (self.do_ur[2] +self.do_ll[2])* 0.5)




if __name__ == '__main__':
    wksp = Workspace3D()
    setup_pybullet(wksp, use_gui=False)
    # wksp.test_scene()
