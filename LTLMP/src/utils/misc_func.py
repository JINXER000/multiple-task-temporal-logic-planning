import  random

import pybullet as p
from pyb_plan_utils import connect, get_aabb_extent, load_model, disconnect, wait_if_gui, create_box, set_point, dump_body, \
    TURTLEBOT_URDF, HideOutput, LockRenderer, joint_from_name, set_euler, get_euler, get_point, \
    set_joint_position, get_joint_positions, pairwise_collision, stable_z, wait_for_duration, get_link_pose, \
    link_from_name, get_pose, euler_from_quat, multiply, invert, draw_pose, unit_point, unit_quat, \
    remove_debug, get_aabb, draw_aabb, get_subtree_aabb, ROOMBA_URDF, set_all_static, assign_link_colors, \
    set_camera_pose, RGBA, draw_point, AABB, get_aabb_center, aabb_intersection

# RGBA colors (alpha is transparency)
RED = RGBA(1, 0, 0, 1)
TAN = RGBA(0.824, 0.706, 0.549, 1)

from itertools import product

def get_automaton_dict(automaton, off_set=0):
    aut_nodes_dict = {}
    auto_nodes_ls = list(automaton.nodes)
    for i in range(len(auto_nodes_ls)):
        aut_nodes_dict[auto_nodes_ls[i]] = i + off_set

    return aut_nodes_dict


def swap_dict(dictionary):
    return {value: key for key, value in dictionary.items()}

# AABB defined in pyb_plan_utils

class LineSegment:
    def __init__(self, start_point, end_point):
        self.start = start_point
        self.end = end_point

def LinesegIntersectsAABB(aabb, line):
    tmin = 0.0
    tmax = 1.0

    for i in range(3):
        if abs(line.end[i] - line.start[i]) < 1e-9:  # line segment is parallel to slab in this dimension
            if line.start[i] < aabb.lower[i] or line.start[i] > aabb.upper[i]:  # line segment is outside of slab
                return False
        else:
            ood = 1.0 / (line.end[i] - line.start[i])
            t1 = (aabb.lower[i] - line.start[i]) * ood
            t2 = (aabb.upper[i] - line.start[i]) * ood
            # t1 is the lower, t2 is the upper
            if t1 > t2:
                t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax:
                return False

    return True

# NOTE: if both points are inside obs, it will tell us NO COLLISION!
def pybCheckCollision(p0, p1, dyn_obs_bdr= None):
    if p0 == p1:
        raise NameError("Cannot use it to check a point!")

    result = p.rayTest(p0, p1)
    if result[0][0] == -1:
        return  False
    else:
        return  True

def collisionFreeAABB(p0, p1, workspace, dyn_obs_aabb= None):
    return not pybCheckCollision(p0, p1, dyn_obs_aabb)

    #  not changing b_state if crossing
    for (obs, aabb) in iter(workspace.obstacles.items()):
        is_intersect = LinesegIntersectsAABB(aabb, LineSegment(p0, p1))
        if is_intersect:
            return False

    # check dynamic obstacle
    if dyn_obs_aabb is None:
        return True

    for (d_obs, aabb) in iter(workspace.dyn_obs.items()):
        is_intersect = LinesegIntersectsAABB(aabb, LineSegment(p0, p1))
        if is_intersect:
            return False    
    return True    

def ptInObs(x, workspace):
    # obstacles
    for (obs, obs_AABB) in iter(workspace.obstacles.items()):
        if isWithinAABB(x, obs_AABB):
            return True
    return False


def getLabel(x, workspace, is_hybrid = False):
    ret_label_set = set()

    # obstacles
    if ptInObs(x, workspace):
        ret_label_set.add('o')
        return ret_label_set

    # regions
    for (reg, reg_AABB) in iter(workspace.regions.items()):
        if isWithinAABB(x, reg_AABB):
            ret_lab = workspace.properties[reg]
            ret_label_set.add(ret_lab)

    # for hybrid
    if is_hybrid:
        media_lab = workspace.in_which_media(x)
        ret_label_set.add(media_lab)

    return ret_label_set

def isWithinAABB(point, aabb):
    return all(min_coord <= point_coord <= max_coord for min_coord, point_coord, max_coord in zip(aabb.lower, point, aabb.upper))

def randomPointAABB(aabb):
    """
    Samples a random point within an AABB.

    Args:
    aabb: A tuple of two lists of 3 floats each representing the min and max coordinates of the AABB.

    Returns:
    A list of 3 floats representing the coordinates of the sampled point.
    """
    min_coords, max_coords = aabb
    rand_pt = [random.uniform(min_coord, max_coord) for min_coord, max_coord in zip(min_coords, max_coords)]
    return tuple(rand_pt)


def fovSense(rbt_pos, fov_dist, workspace, reg_ls =[], obs_ls = [] ):
    fov_ll = [rbt_pos[i] - fov_dist * 0.5 for i in range(len(rbt_pos))]
    fov_ur = [rbt_pos[i] + fov_dist * 0.5 for i in range(len(rbt_pos))]
    fov_aabb = AABB(fov_ll, fov_ur)

    if len(reg_ls) and len(obs_ls):
        raise NameError('Cannot deal with them simultaneously!')

    if len(reg_ls):
        for reg_name in reg_ls:
            sensed = aabb_intersection(fov_aabb, workspace.regions[reg_name])
            if sensed:
                return reg_name
    else:
        for obs_name in obs_ls:
            sensed = aabb_intersection(fov_aabb, workspace.dyn_obs[obs_name])
            if sensed:
                return obs_name

    return None

############# pyb drawing 

def pyb_draw_sphere_marker(position, radius, color):
   vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
   marker_id = p.createMultiBody(basePosition=position, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vs_id)
   return marker_id

def pyb_remove_marker(marker_id):
   p.removeBody(marker_id)

def pyb_draw_edges(edges, line_color=[0, 0, 1], line_width=1):
    for edge in edges:
        start = edge[0].pos
        end = edge[1].pos
        p.addUserDebugLine(start, end, line_color, line_width)

def pyb_remove_edges():
    p.removeAllUserDebugItems()



def setup_pybullet(workspace, use_gui = False):
    # Creates a pybullet world and a visualizer for it
    connect(use_gui=use_gui)
    if use_gui:
        set_camera_pose(camera_point=[5, -5, 5], target_point=unit_point())  # Sets the camera's position
    identity_pose = (unit_point(), unit_quat())
    origin_handles = draw_pose(identity_pose,
                                length=1.0)  # Draws the origin coordinate system (x:RED, y:GREEN, z:BLUE)


    # Bodies are described by an integer index
    floor = create_box(w=workspace.width, l=workspace.length, h=0.001, color=TAN) # Creates a tan box object for the floor
    set_point(floor, [0, 0, -0.001 / 2.])  # Sets the [x,y,z] translation of the floor

    for obs_aabb in workspace.obstacles.values():
        center = get_aabb_center(obs_aabb)
        extent = get_aabb_extent(obs_aabb)
        obs = create_box(w = extent[0], l = extent[1], h= extent[2], color=RED)
        set_point(obs, center)
        print('Position:', get_point(obs))
        # wait_for_duration(1.0)

    set_all_static()




# generate pointcloud inside AABB
def genPntCld(aabb, shrink_dis = 0):
    lower, upper = aabb
    ll_x, ll_y, ll_z = lower
    ur_x, ur_y, ur_z = upper
    X_list = range(int(ll_x+shrink_dis), int(ur_x+1-shrink_dis))
    y_list = range(int(ll_y+shrink_dis), int(ur_y+1-shrink_dis))
    z_list = range(int(ll_z+shrink_dis), int(ur_z+1-shrink_dis))
    pnt_cld = list(product(X_list, y_list, z_list))

    return pnt_cld
    

def genWorkspacePntCld(workspace, bad_regs, dynobs_aabb):
    whole_cld = []

    for block_reg in bad_regs:
        bad_aabb = workspace.regions[block_reg]
        cld = genPntCld(bad_aabb)
        whole_cld +=cld

    if dynobs_aabb is not None:
        cld = genPntCld(dynobs_aabb, shrink_dis = 0.1)
        whole_cld +=cld

    return whole_cld


# for unit test
if __name__ =='__main__':
    point = [5,5,5]
    from pyb_plan_utils import AABB

    aabb = AABB([0,0,0], [10,10,10])
    if isWithinAABB(point, aabb):
        print('OK')
    else:
        print('fuck')
    