import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from tf.transformations import  quaternion_from_euler


class rviz_maker(object):
    def __init__(self, start_pos, end_pos, workspace, scale_factor, has_b_state):
        self.start_p = start_pos
        self.end_p = end_pos
        self.workspace = workspace
        self.scale = scale_factor
        self.has_b_state = has_b_state
        self.vertex_vis = Marker()
        self.changed_regs = set()
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=1)
        self.clear_all()

    def clear_all(self):
        edges_vis = Marker()
        edges_vis.header.frame_id = "map"
        edges_vis.header.stamp = rospy.rostime.Time.now()
        edges_vis.type = Marker.LINE_LIST
        edges_vis.action = Marker.DELETEALL
        edges_vis.ns = "edges"
        edges_vis.id = 3
        self.marker_pub.publish(edges_vis)

        vertex = Marker()
        vertex.header.frame_id = "map"
        vertex.header.stamp = rospy.rostime.Time.now()
        vertex.type = Marker.POINTS
        vertex.action = Marker.DELETEALL
        vertex.ns = "leaves"
        vertex.id = 4
        self.marker_pub.publish(vertex)

        rob_vis = Marker()
        rob_vis.header.frame_id = "map"
        rob_vis.header.stamp = rospy.rostime.Time.now()
        rob_vis.type = Marker.CUBE
        rob_vis.action = Marker.DELETEALL
        rob_vis.ns = "rbt"
        rob_vis.id = 1
        self.marker_pub.publish(rob_vis)

        fcs_vis = Marker()
        fcs_vis.header.frame_id = "map"
        fcs_vis.header.stamp = rospy.rostime.Time.now()
        fcs_vis.type = Marker.LINE_STRIP
        fcs_vis.action = Marker.DELETEALL
        fcs_vis.ns = "path"
        fcs_vis.id = 6
        self.marker_pub.publish(fcs_vis)

        vertex_vis = Marker()
        vertex_vis.header.frame_id = "map"
        vertex_vis.header.stamp = rospy.rostime.Time.now()
        vertex_vis.type = Marker.POINTS
        vertex_vis.action = Marker.ADD

        vertex_vis.ns = "rewired"
        vertex_vis.id = 4
        self.marker_pub.publish(vertex_vis)

    def display_edges(self, edge_list):
        edges_vis = Marker()
        edges_vis.header.frame_id = "map"
        edges_vis.header.stamp = rospy.rostime.Time.now()
        edges_vis.type = Marker.LINE_LIST
        edges_vis.action = Marker.ADD

        edges_vis.ns = "edges"
        edges_vis.id = 3

        edges_vis.scale.x = 0.02
        edges_vis.color.g = 0.5
        edges_vis.color.b = 0.5
        edges_vis.color.a = 0.7
        edges_vis.pose.orientation.w = 1

        invalid_edge_vis = Marker()
        invalid_edge_vis.header.frame_id = "map"
        invalid_edge_vis.header.stamp = rospy.rostime.Time.now()
        invalid_edge_vis.type = Marker.LINE_LIST
        invalid_edge_vis.action = Marker.ADD

        invalid_edge_vis.ns = "invalid_edges"
        invalid_edge_vis.id = 3

        invalid_edge_vis.scale.x = 0.02
        invalid_edge_vis.color.g = 0.2
        invalid_edge_vis.color.b = 0.2
        invalid_edge_vis.color.r = 0.5
        invalid_edge_vis.color.a = 0.7
        invalid_edge_vis.pose.orientation.w = 1

        BS_change_vis = Marker()
        BS_change_vis.header.frame_id = "map"
        BS_change_vis.header.stamp = rospy.rostime.Time.now()
        BS_change_vis.type = Marker.LINE_LIST
        BS_change_vis.action = Marker.ADD

        BS_change_vis.ns = "b_change_edges"
        BS_change_vis.id = 3

        BS_change_vis.scale.x = 0.02
        BS_change_vis.color.g = 0.5
        BS_change_vis.color.b = 0.0
        BS_change_vis.color.r = 1.0
        BS_change_vis.color.a = 0.7
        BS_change_vis.pose.orientation.w = 1

        for edge in edge_list:
            pt1 = Point()
            pt1.x = edge[0].pos[0] * self.scale
            pt1.y = edge[0].pos[1] * self.scale
            pt1.z = edge[0].pos[2] * self.scale
            pt2 = Point()
            pt2.x = edge[1].pos[0] * self.scale
            pt2.y = edge[1].pos[1] * self.scale
            pt2.z = edge[1].pos[2] * self.scale

            if self.has_b_state:
                pt1.z = int(edge[0].b_state)
                pt2.z = int(edge[1].b_state)

                if edge[0].cost2Root > 1e5 or edge[1].cost2Root > 1e5:

                    invalid_edge_vis.points.append(pt1)
                    invalid_edge_vis.points.append(pt2)
                    continue
                elif edge[0].b_state is not None and edge[0].b_state != edge[1].b_state:
                    BS_change_vis.points.append(pt1)
                    BS_change_vis.points.append(pt2)
                    continue

            if edge[0].cost2Root > 1e5 or edge[1].cost2Root > 1e5:

                invalid_edge_vis.points.append(pt1)
                invalid_edge_vis.points.append(pt2)
            else:
                edges_vis.points.append(pt1)
                edges_vis.points.append(pt2)

        self.marker_pub.publish(edges_vis)
        self.marker_pub.publish(invalid_edge_vis)
        self.marker_pub.publish(BS_change_vis)

    def draw_empty(self):
        vertex_vis = Marker()
        vertex_vis.header.frame_id = "map"
        vertex_vis.header.stamp = rospy.rostime.Time.now()
        vertex_vis.type = Marker.POINTS
        vertex_vis.action = Marker.ADD

        vertex_vis.ns = "rewired"
        vertex_vis.id = 4

        vertex_vis.scale.x = 0.06
        vertex_vis.color.r = 1.0
        vertex_vis.color.a = 1

        self.marker_pub.publish(vertex_vis)

    def draw_rewired_root(self, rewired_nodes):
        if len(rewired_nodes) == 0:
            return

        vertex_vis = Marker()
        vertex_vis.header.frame_id = "map"
        vertex_vis.header.stamp = rospy.rostime.Time.now()
        vertex_vis.type = Marker.POINTS
        vertex_vis.action = Marker.ADD

        vertex_vis.ns = "rewired"
        vertex_vis.id = 4

        vertex_vis.scale.x = 0.06
        vertex_vis.color.r = 1.0
        vertex_vis.color.a = 1

        for nd in rewired_nodes:
            p1 = nd.pos

            pt1 = Point()
            pt1.x = p1[0] * self.scale
            pt1.y = p1[1] * self.scale
            pt1.z = p1[2] * self.scale

            if self.has_b_state:
                pt1.z = int(self.nd.b_state)

            vertex_vis.points.append(pt1)

        self.marker_pub.publish(vertex_vis)

    def populateRviz(self):
        self.populateStart()
        self.populateEnd()
        self.populateWorkspace(self.workspace.regions, False)
        self.populateWorkspace(self.workspace.obstacles, True)
        self.draw_dyn_obs()

    def draw_ws_water(self):
        water_vis = Marker()
        water_vis.header.frame_id = "map"
        water_vis.header.stamp = rospy.rostime.Time.now()
        water_vis.type = Marker.CUBE
        water_vis.action = Marker.ADD

        water_vis.ns = "water"
        water_vis.id = 1

        water_vis.color.r = 0
        water_vis.color.g = 1
        water_vis.color.b = 1
        water_vis.color.a = 0.1

        water_vis.scale.x = 10
        water_vis.scale.y = 20
        water_vis.scale.z = 0.2
        water_vis.pose.position.x = 50 * self.scale
        water_vis.pose.position.y = 0 * self.scale

        water_vis.pose.position.z = 0

        water_vis.pose.orientation.w = 1
        self.marker_pub.publish(water_vis)

    def populateExt(self, coords):
        bdr = Marker()
        bdr.header.frame_id = "map"
        bdr.header.stamp = rospy.rostime.Time.now()
        bdr.type = Marker.LINE_LIST
        bdr.action = Marker.ADD
        bdr.ns = "ws_ext"
        bdr.id = 0

        bdr.scale.x = 0.2
        bdr.color.r = 0.6
        bdr.color.g = 0.6
        bdr.color.b = 0.9
        bdr.color.a = 0.8
        bdr.pose.orientation.w = 1

        for idx in range(len(coords) - 1):
            pt0 = Point()
            pt1 = Point()
            pt0.x = coords[idx][0] * self.scale
            pt0.y = coords[idx][1] * self.scale
            pt1.x = coords[idx + 1][0] * self.scale
            pt1.y = coords[idx + 1][1] * self.scale
            bdr.points.append(pt1)
            bdr.points.append(pt0)
        self.marker_pub.publish(bdr)

    def populateWorkspace(self, regions, is_obs=True):
        idx_offset = 5
        if is_obs:
            idx_offset += 10
        for key in regions.keys():
            aabb = regions[key]
            lower, upper = aabb
            coords = np.array([(lower[0], lower[1]), (upper[0], lower[1]), (upper[0], upper[1]), (lower[0], upper[1]), (lower[0], lower[1])])
            
            # x, y = poly.exterior.xy
            # coords = np.column_stack((x, y))  # coords = [(0, 0), (1, 1), (1, 0)]

            bdr = Marker()
            bdr.header.frame_id = "map"
            bdr.header.stamp = rospy.rostime.Time.now()
            bdr.type = Marker.LINE_LIST
            bdr.action = Marker.ADD
            bdr.ns = "regs"
            bdr.id = idx_offset
            idx_offset += 1

            bdr.scale.x = 0.2
            if is_obs:
                bdr.color.r = 0.8
                bdr.color.g = 0.3
                bdr.color.b = 0.3
            elif key in self.changed_regs:
                bdr.color.r = 0.6
                bdr.color.g = 0.6
                bdr.color.b = 0.0
            else:
                bdr.color.r = 0.3
                bdr.color.g = 1
                bdr.color.b = 0.3
            bdr.color.a = 0.8
            bdr.pose.orientation.w = 1

            for idx in range(len(coords) - 1):
                pt0 = Point()
                pt1 = Point()
                pt0.x = coords[idx][0] * self.scale
                pt0.y = coords[idx][1] * self.scale
                pt1.x = coords[idx + 1][0] * self.scale
                pt1.y = coords[idx + 1][1] * self.scale
                bdr.points.append(pt1)
                bdr.points.append(pt0)

            self.marker_pub.publish(bdr)

            txt = Marker()
            # compute center
            x_center = sum([pt[0] for pt in coords]) / len(coords)
            y_center = sum([pt[1] for pt in coords]) / len(coords)

            # place text
            txt.header.frame_id = "map"
            txt.header.stamp = rospy.rostime.Time.now()
            txt.type = Marker.TEXT_VIEW_FACING
            txt.ns = "text"
            txt.id = idx_offset
            idx_offset += 1
            txt.action = Marker.ADD
            txt.pose.position.x = x_center * self.scale
            txt.pose.position.y = y_center * self.scale
            txt.pose.orientation.w = 1

            txt.scale.z = 1
            if is_obs:
                txt.color.r = 0
                txt.color.g = 0
                txt.color.b = 0
                txt.text = 'obs'
            else:
                txt.color.r = 0
                txt.color.g = 0
                txt.color.b = 0
                txt.text = key
                # txt.text = properties[key]
            txt.color.a = 1

            # self.txt_vis_list.append(txt)
            self.marker_pub.publish(txt)

    def display_text(self, text):
        txt = Marker()
        # compute center
        x_center = 50
        y_center = 50

        # place text
        txt.header.frame_id = "map"
        txt.header.stamp = rospy.rostime.Time.now()
        txt.type = Marker.TEXT_VIEW_FACING
        txt.ns = "user_text"
        txt.id = 0
        txt.action = Marker.ADD
        txt.pose.position.x = x_center * self.scale
        txt.pose.position.y = y_center * self.scale
        txt.pose.orientation.w = 1

        txt.scale.z = 0.7
        txt.color.r = 0.7
        txt.color.g = 0.8
        txt.color.b = 0.95
        txt.text = text
        txt.color.a = 1.0

        self.marker_pub.publish(txt)

    def populateStart(self):
        if self.start_p is None:
            return

        pt1 = Point()
        pt1.x = self.start_p[0] * self.scale
        pt1.y = self.start_p[1] * self.scale
        pt1.z = self.start_p[2] * self.scale

        vertex = Marker()
        vertex.header.frame_id = "map"
        vertex.header.stamp = rospy.rostime.Time.now()
        vertex.type = Marker.POINTS
        vertex.action = Marker.ADD

        vertex.ns = "start"
        vertex.id = 0

        vertex.scale.x = vertex.scale.y = vertex.scale.z = 0.16

        vertex.color.g = 1.0
        vertex.color.a = 1

        vertex.points.append(pt1)

        self.marker_pub.publish(vertex)

    def populateEnd(self):
        if self.end_p is None:
            return

        vertex = Marker()
        vertex.header.frame_id = "map"
        vertex.header.stamp = rospy.rostime.Time.now()
        vertex.type = Marker.CUBE
        vertex.action = Marker.ADD

        vertex.ns = "end"
        vertex.id = 1

        vertex.scale.x = vertex.scale.y = vertex.scale.z = 0.1
        vertex.color.g = 1.0
        vertex.color.r = 0.7
        vertex.color.a = 1

        vertex.pose.position.x = self.end_p[0] * self.scale
        vertex.pose.position.y = self.end_p[1] * self.scale

        self.marker_pub.publish(vertex)

    def draw_rtroot(self, root):
        if root is None:
            return

        vertex = Marker()
        vertex.header.frame_id = "map"
        vertex.header.stamp = rospy.rostime.Time.now()
        vertex.type = Marker.CUBE
        vertex.action = Marker.ADD

        vertex.ns = "rt_root"
        vertex.id = 1

        vertex.scale.x = vertex.scale.y = vertex.scale.z = 0.1
        vertex.color.g = 1.0
        vertex.color.r = 0.7
        vertex.color.a = 1

        vertex.pose.position.x = root.pos[0] * self.scale
        vertex.pose.position.y = root.pos[1] * self.scale
        vertex.pose.position.z = root.pos[2] * self.scale
        if self.has_b_state:
            vertex.pose.position.z = int(root.b_state)

        self.marker_pub.publish(vertex)

    def drawSample(self, x_rand, near_range):
        vertex = Marker()
        vertex.header.frame_id = "map"
        vertex.header.stamp = rospy.rostime.Time.now()
        vertex.type = Marker.CYLINDER
        vertex.action = Marker.ADD

        vertex.ns = "sample"
        vertex.id = 2

        vertex.scale.x = vertex.scale.y = vertex.scale.z = 0.1
        vertex.color.b = 0.2
        vertex.color.g = 1.0
        vertex.color.r = 1.0
        vertex.color.a = 1

        vertex.pose.position.x = x_rand[0] * self.scale
        vertex.pose.position.y = x_rand[1] * self.scale

        self.marker_pub.publish(vertex)

        circle = Marker()
        circle.header.frame_id = "map"
        circle.header.stamp = rospy.rostime.Time.now()
        circle.type = Marker.SPHERE
        circle.action = Marker.ADD

        circle.ns = "near_range"
        circle.id = 1

        circle.scale.x = circle.scale.y = near_range * self.scale
        circle.scale.z = 0.05
        circle.color.b = circle.color.g = circle.color.r = 0.2
        circle.color.a = 0.2

        circle.pose.position.x = vertex.pose.position.x
        circle.pose.position.y = vertex.pose.position.y

        self.marker_pub.publish(circle)

    def drawFinalPath(self, path):
        # if len(path)<2:
        #     return

        fcs_vis = Marker()
        fcs_vis.header.frame_id = "map"
        fcs_vis.header.stamp = rospy.rostime.Time.now()
        fcs_vis.type = Marker.LINE_STRIP
        fcs_vis.action = Marker.ADD
        fcs_vis.ns = "path"
        fcs_vis.id = 6

        fcs_vis.scale.x = 0.8
        fcs_vis.color.g = 0.9
        fcs_vis.color.b = 0.5
        fcs_vis.color.a = 0.2
        fcs_vis.pose.orientation.w = 1

        for p1 in path:
            pt1 = Point()
            pt1.x = p1.pos[0] * self.scale
            pt1.y = p1.pos[1] * self.scale
            pt1.z = p1.pos[2] * self.scale

            if self.has_b_state:
                pt1.z = int(p1.b_state)

            fcs_vis.points.append(pt1)

        self.marker_pub.publish(fcs_vis)

    def drawPrefixPath(self, path, motion_bias=None):
        if len(path) < 2:
            return

        fcs_vis = Marker()
        fcs_vis.header.frame_id = "map"
        fcs_vis.header.stamp = rospy.rostime.Time.now()
        fcs_vis.type = Marker.LINE_STRIP
        fcs_vis.action = Marker.ADD
        fcs_vis.ns = "path"
        fcs_vis.id = 6

        fcs_vis.scale.x = 0.8
        fcs_vis.color.g = 0.9
        fcs_vis.color.b = 0.5
        fcs_vis.color.a = 0.2
        fcs_vis.pose.orientation.w = 1

        for p1 in path:
            pt1 = Point()
            pt1.x = p1[0][0][0] + motion_bias[0]
            pt1.y = p1[0][0][1] + motion_bias[1]
            pt1.z = p1[0][0][2] + motion_bias[1]

            if self.has_b_state:
                pt1.z = int(p1[1])

            fcs_vis.points.append(pt1)

        self.marker_pub.publish(fcs_vis)

    def draw_rbt(self, rbt_pos, cur_heading, fov_range, b_state=None):
        rob_vis = Marker()
        rob_vis.header.frame_id = "map"
        rob_vis.header.stamp = rospy.rostime.Time.now()
        rob_vis.type = Marker.CUBE
        rob_vis.action = Marker.ADD

        rob_vis.ns = "rbt"
        rob_vis.id = 1

        rob_vis.scale.x = rob_vis.scale.y = rob_vis.scale.z = 0.1

        rob_vis.color.r = 1
        rob_vis.color.g = 0.2
        rob_vis.color.b = 1
        rob_vis.color.a = 0.8

        rob_vis.scale.x = 0.5
        rob_vis.scale.y = 0.3
        rob_vis.scale.z = 0.2
        rob_vis.pose.position.x = rbt_pos[0] * self.scale
        rob_vis.pose.position.y = rbt_pos[1] * self.scale
        rob_vis.pose.position.z = rbt_pos[2] * self.scale
        if b_state:
            rob_vis.pose.position.z = int(b_state)
        yaw = np.arctan2(cur_heading[1], cur_heading[0])
        quat = quaternion_from_euler(0, 0, yaw)
        rob_vis.pose.orientation.x = quat[0]
        rob_vis.pose.orientation.y = quat[1]
        rob_vis.pose.orientation.z = quat[2]
        rob_vis.pose.orientation.w = quat[3]
        self.marker_pub.publish(rob_vis)

        circle = Marker()
        circle.header.frame_id = "map"
        circle.header.stamp = rospy.rostime.Time.now()
        circle.type = Marker.CUBE
        circle.action = Marker.ADD

        circle.ns = "fov_range"
        circle.id = 1

        circle.scale.x = circle.scale.y = fov_range * self.scale
        circle.scale.z = 0.1
        circle.color.b = circle.color.g = 0.2
        circle.color.r = 0.5
        circle.color.a = 0.1

        circle.pose.position.x = rob_vis.pose.position.x
        circle.pose.position.y = rob_vis.pose.position.y
        self.marker_pub.publish(circle)

    def draw_leaves(self, node_ls):

        vertex = Marker()
        vertex.header.frame_id = "map"
        vertex.header.stamp = rospy.rostime.Time.now()
        vertex.type = Marker.POINTS
        vertex.action = Marker.ADD

        vertex.ns = "leaves"
        vertex.id = 4

        vertex.scale.x = vertex.scale.y = vertex.scale.z = 0.05

        vertex.color.b = 1.0
        vertex.color.a = 1

        for nd in node_ls:
            pt1 = Point()
            pt1.x = nd.pos[0] * self.scale
            pt1.y = nd.pos[1] * self.scale
            pt1.z = nd.pos[2] * self.scale

            if self.has_b_state:
                pt1.z = int(nd.b_state)

            vertex.points.append(pt1)

        self.marker_pub.publish(vertex)

    def draw_dyn_obs(self):
        vertex = Marker()
        vertex.header.frame_id = "map"
        vertex.header.stamp = rospy.rostime.Time.now()
        vertex.type = Marker.CUBE
        vertex.action = Marker.ADD

        vertex.ns = "dyn_obs"
        vertex.id = 2

        vertex.scale.x = (self.workspace.do_ur[0] - self.workspace.do_ll[0]) * self.scale
        vertex.scale.y = (self.workspace.do_ur[1] - self.workspace.do_ll[1]) * self.scale
        vertex.scale.z = 0.8
        vertex.color.b = 0.5
        vertex.color.g = 0.5
        vertex.color.r = 0.8
        vertex.color.a = 0.8

        vertex.pose.position.x = self.workspace.dynamic_obs_center[0] * self.scale
        vertex.pose.position.y = self.workspace.dynamic_obs_center[1] * self.scale

        self.marker_pub.publish(vertex)


if __name__ == '__main__':
    from workspace import Workspace3D

    rospy.init_node('visualizer', anonymous=False)
    wksp = Workspace3D()
    rviz_mgr = rviz_maker(None, None, wksp, scale_factor=1, has_b_state= False)

    # ws_ext_coords = getAABB(wksp.workspace_ll, ws_ur)
    # ws_ext_coords.append(wksp.workspace_ll)

    while not rospy.is_shutdown():
        rviz_mgr.populateWorkspace(rviz_mgr.workspace.regions, False)
        rviz_mgr.populateWorkspace(rviz_mgr.workspace.obstacles, True)
        rviz_mgr.draw_dyn_obs()
        # rviz_mgr.populateExt(ws_ext_coords)

        rospy.sleep(0.1)

    # rospy.spin()