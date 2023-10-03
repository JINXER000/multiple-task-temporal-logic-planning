
import numpy as np

from  utils.misc_func import  genWorkspacePntCld, fovSense, setup_pybullet
from  utils.tree_visualizer import rviz_maker
import threading

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import  quaternion_from_euler

from ltl3.msg import activity_pose

from enum import Enum
DiscreteMode = Enum('DiscreteMode', ('notInit', 'planning', 'executing'))

robot_name = ''

class gazebo_HMI_base(object):
    def __init__(self,  workspace, para):
        rospy.init_node('HMI_LTL', anonymous=False)

        self.node_init_common(para, workspace)
 
        self.exeInfo_pub = rospy.Publisher("/HMI/exe_info", String, queue_size=1)
        self.trace_pub = rospy.Publisher("/HMI/plan_skeleton", String, queue_size=1)

         # customized initialize
        self.MP_initialized = False
        # wait until got rbt pose
        init_pos = self.rbt_pos
        while init_pos is None:
            init_pos = self.rbt_pos
        
        goal = None
        self.rviz_mgr = rviz_maker(init_pos, goal, workspace, 1, True)

        self.automaton = None
        rospy.Timer(rospy.Duration(1.0), self.rqt_monitor)
        rospy.set_param('cur_task', 0)

        setup_pybullet(workspace, use_gui = False)

        # wait for formula
        while self.mode == DiscreteMode.notInit or self.automaton == 0:
            pass


        self.on_initialize(init_pos, workspace)

        self.MP_initialized = True

        rospy.Timer(rospy.Duration(0.1), self.rbt_routine)

        self.start_time = rospy.Time.now()
        rospy.spin()


    # MUST IMPLEMENT THIS
    def translate_LTL(self, formula):
        raise NotImplementedError('LTLf2DFA or LTL2BA?')

    def on_reset_task(self):
        raise NotImplementedError('This planner does not support multi-query')

    def rqt_monitor(self, event):
        

        if self.mode == DiscreteMode.notInit:
            self.mutex.acquire()

            self.rviz_mgr.populateRviz()
            formula = rospy.get_param('cur_task')

            automaton = self.translate_LTL(formula)
            if automaton != 0:

                self.automaton = automaton

                print ('get task: '+ formula)
                # for multi query
                if self.MP_initialized:
                    self.on_reset_task()

                self.start_time = rospy.Time.now()
                self.initplan_start_time = rospy.Time.now()

                self.mode = DiscreteMode.planning
                rospy.set_param('cur_task', 0)

            self.mutex.release()
        else:  # abort
            pass

        

    def pub_msg_rqt(self, text, puber):
        info_msg = String()
        info_msg.data = text
        puber.publish(info_msg)




    def node_init_common(self, para, workspace):
            self.para = para

            self.mutex = threading.Lock()

            self.mode = DiscreteMode.notInit
            self.doing_replan = False
            self.replan_start_time = None
            self.replan_total_secs = 0
            self.replan_total_times = 0
            self.start_time = rospy.Time.now()
            self.initplan_start_time = rospy.Time.now()

            self.pynt_stamp_idx = 0
            self.pynt_pub = rospy.Publisher("/forbid_reg_cloud", PointCloud2, queue_size=1)

            self.cmd_header = 0
            self.temp_goal_pub = rospy.Publisher(robot_name+"/move_act", activity_pose)
            self.path_pub = rospy.Publisher('/cur_path', Path, queue_size=1)


            # moving obs related
                    # not for real test!
            self.obs_move_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
            self.obs_initialize()
            self.new_goal_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.obs_move_CB, queue_size=1)
            self.new_dynamic_obs = workspace.dynamic_obs_center

            # rospy.Timer(rospy.Duration(0.2), self.move_obs_fake)

            self.rbt_pos = None
            self.rbt_odom_sub = rospy.Subscriber(robot_name+'/odom', Odometry, self.rbt_odom_CB, queue_size=1) #/iris_0



    # MUST IMPLEMENT THIS
    def on_initialize(self, init_pos, workspace):
        raise NotImplementedError('must implement initialize')

    def rbt_routine(self, event):
        # perception
        # self.LTLMP.workspace.update_dynamic_obstacle(self.new_dynamic_obs)
        self.do_perception()
        self.pub_pntcld()

        # planning
        self.on_update()

    # MUST IMPLEMENT THIS
    def on_update(self):
        raise NotImplementedError('must implement update')

    def rbt_odom_CB(self, msg):
        # tuples are immutable therefore a = sign does not create a reference but a copy as expected.

        rbt_pos_x = msg.pose.pose.position.x 
        rbt_pos_y = msg.pose.pose.position.y 
        rbt_pos_z = msg.pose.pose.position.z
        self.rbt_pos = (rbt_pos_x, rbt_pos_y, rbt_pos_z)


        
    # same condition for each planner
    def do_perception(self):
        reg_name = fovSense(self.rbt_pos, self.para['fov_dist'], self.LTLMP.workspace, self.para['reg_change_ls'])
        if reg_name is not None and len(self.rviz_mgr.changed_regs)==0:

            self.LTLMP.workspace.properties[reg_name] = ''

            self.LTLMP.detect_TS_change = True
            self.LTLMP.changed_regs.add(reg_name)
            self.rviz_mgr.changed_regs.add( reg_name)

        self.obs_detection()

    def obs_detection(self):
        sensed_dynamic_obs = self.circle_sense_obs(self.rbt_pos, self.LTLMP.workspace.dynamic_obs_center)

        # sensed_dynamic_obs = sense_obs(self.rbt_pos, self.para['fov_dist'], self.LTLMP.workspace, 'd3')
        if sensed_dynamic_obs:
            cur_dynobs_pos = self.LTLMP.workspace.dynamic_obs_center
            obs_diff = self.LTLMP.evaluator.euclideanDist(self.LTLMP.old_dynamic_obs, cur_dynobs_pos)
            if obs_diff>3:
                self.LTLMP.detect_obs_move = True
                print('OBS move detected')

                self.LTLMP.old_dynamic_obs = cur_dynobs_pos
                self.LTLMP.old_dynobs_aabb = self.LTLMP.workspace.dyn_obs['d3']


    def obs_initialize(self):
        self.dyn_obs_mover_msg = PoseWithCovarianceStamped()
        self.obs_move_dir = -1
        self.dyn_obs_mover_msg.header.stamp= rospy.Time.now()
        self.dyn_obs_mover_msg.header.frame_id = 'map'
        self.dyn_obs_mover_msg.pose.pose.position.x = -2.5
        self.dyn_obs_mover_msg.pose.pose.position.y = -0.5
        self.dyn_obs_mover_msg.pose.pose.position.z = -1
        self.obs_move_pub.publish(self.dyn_obs_mover_msg)

    def move_obs_fake(self, event):
        move_speed = 0.02
        cur_obs_axis =  self.dyn_obs_mover_msg.pose.pose.position.x
        if cur_obs_axis>3.5:
            self.obs_move_dir = -1
        elif cur_obs_axis<-1:
            self.obs_move_dir = 1

        self.dyn_obs_mover_msg.pose.pose.position.x +=self.obs_move_dir*move_speed
        self.obs_move_pub.publish(self.dyn_obs_mover_msg)

    def obs_move_CB(self, msg): # covairance have pose.pose
        # self.mutex.acquire()

        new_obs_x= msg.pose.position.x/self.rviz_mgr.scale
        new_obs_y = msg.pose.position.y/self.rviz_mgr.scale

        # new_dynamic_obs = (new_obs_x, new_obs_y)
        # self.LTLMP.workspace.update_dynamic_obstacle(new_dynamic_obs)
        # print('received new dyn obs pos: '+new_dynamic_obs)

        # self.mutex.release()


    def circle_sense_obs(self, rbt_pos, obs_pos):
        obs_dist = self.LTLMP.evaluator.euclideanDist(rbt_pos, obs_pos)
        if obs_dist <  self.para['fov_dist']:
            return True
        else:
            return False




    def pub_pntcld(self):
        # if pynt changes, then 
        # self.handle_pynt_changes()

        cld_pynts = genWorkspacePntCld(self.LTLMP.workspace, self.LTLMP.bad_regs, self.LTLMP.old_dynobs_aabb)


        #header
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        # header.frame_id = str(self.pynt_stamp_idx)
        #create pcl from points
        scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cld_pynts)

        self.pynt_pub.publish(scaled_polygon_pcl)

    def calc_heading(self, pt1, pt2):
        pt_diff_x = pt2[0] - pt1[0]
        pt_diff_y = pt2[1] - pt1[1]
        yaw = np.arctan2(pt_diff_y, pt_diff_x)
        quat = quaternion_from_euler(0, 0, yaw)
        return quat


    def prepare_path_msg(self, path_seg):
        path_out = Path()
        path_out.header.stamp = rospy.Time.now()
        path_out.header.frame_id = "map"

        start_pos = path_seg[0]
        for nd_pos in path_seg:
            nd_pos_msg = PoseStamped()
            nd_pos_msg.pose.position.x = nd_pos[0]
            nd_pos_msg.pose.position.y = nd_pos[1]
            nd_pos_msg.pose.position.z = 1
            # calc heading
            if start_pos != nd_pos:
                quat = self.calc_heading(start_pos, nd_pos)
                nd_pos_msg.pose.orientation.w = 1
                nd_pos_msg.pose.orientation.x = 0
                nd_pos_msg.pose.orientation.y = 0
                nd_pos_msg.pose.orientation.z = 0 
            else:              
                nd_pos_msg.pose.orientation.w = 1
                nd_pos_msg.pose.orientation.x = 0
                nd_pos_msg.pose.orientation.y = 0
                nd_pos_msg.pose.orientation.z = 0

            path_out.poses.append(nd_pos_msg)

        return path_out

       

    def pub_temp_goal(self, nxt_pos = None):
        act_pos = activity_pose()
        act_pos.header = self.cmd_header

        if nxt_pos is None:
            # in case arrive target
            if self.LTLMP.next_wp is None:
                return
            nxt_pos = self.LTLMP.next_wp.pos

        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = 'map'


        goal_msg.pose.orientation.w = 1
        goal_msg.pose.orientation.x = 0
        goal_msg.pose.orientation.y = 0
        goal_msg.pose.orientation.z = 0

        if type(nxt_pos) is  str:
            act_pos.type = nxt_pos
            goal_msg.pose.position.x = self.rbt_pos[0]
            goal_msg.pose.position.y = self.rbt_pos[1]
            goal_msg.pose.position.z = self.rbt_pos[2]

        else:
            act_pos.type = "pass_by"
            goal_msg.pose.position.x = nxt_pos[0]
            goal_msg.pose.position.y = nxt_pos[1]
            goal_msg.pose.position.z = nxt_pos[2]


        self.cmd_header+=1

        act_pos.tgt_pose = goal_msg
        self.temp_goal_pub.publish(act_pos)
