import numpy as np
from utils.workspace import Workspace3D
from utils.misc_func import  getLabel
from RTLTL import RT_LTL_planner
from utils.naive_evaluator import evaluator
from formula_parser.ltlf_parser import getDFA
from ros_gazebo_HMI_base import gazebo_HMI_base, DiscreteMode

import rospy

import os


root_dir = r'/home/joseph/yzchen_ws/task_planning/ltl_translate/ltl-amdp/language/models/torch/rqt_IO'
buchi_path = os.path.join(root_dir, 'dfa_remote.gv')

class motionA2B_HMI_RTMQ_ros(gazebo_HMI_base):
    def __init__(self,  workspace, para):

        super(motionA2B_HMI_RTMQ_ros, self).__init__(workspace, para)


    def translate_LTL(self, formula):
        # invalid input (get from rosparam)
        if formula == 0 or formula[0] == '0':
            return 0

        DFA = getDFA(buchi_path, formula)
        self.pub_msg_rqt('Automata constructed', self.exeInfo_pub)
        return DFA

    def on_initialize(self, init_pos, workspace):
        init_label = getLabel(init_pos, workspace)

        root = RT_LTL_planner.Node(0, init_pos, 0, self.automaton.graph['initial'][0], init_label)
        root.same_pos_nodes.add(root)
        dist_eval = evaluator()
        self.LTLMP = RT_LTL_planner(root, None, workspace, dist_eval, self.para, self.automaton, 'prefix')

        rospy.Timer(rospy.Duration(1), self.checkShortCut)


    def on_update(self):
        # in case during planning, the tree is reset by  rqt_monitor()
        self.mutex.acquire()

        if self.mode == DiscreteMode.planning and self.LTLMP.goalFound:
            self.mode = DiscreteMode.executing
            initplan_time = (rospy.Time.now() - self.initplan_start_time).to_sec()
            print('initplan_time is ' + str(initplan_time))
            self.pub_msg_rqt('initplan_time is ' + str(initplan_time) + 's', self.exeInfo_pub)

            plan_skeleton = self.LTLMP.wrap_plan()
            self.pub_msg_rqt(plan_skeleton, self.trace_pub)

        ready_obs = False
        if self.LTLMP.detect_obs_move:
            ready_obs = True

        ready_TS = False
        if self.LTLMP.detect_TS_change:
            ready_TS = True

        finished, obs_no_affect = self.LTLMP.plan_iteration(self.rbt_pos, self.rviz_mgr)

        # if need replan
        if ((ready_obs and obs_no_affect == False) or ready_TS) and self.doing_replan == False:
            self.doing_replan = True
            self.replan_start_time = rospy.Time.now()

            log_text = "Prior knowledge incorrect, replanning!"
            if not ready_TS:
                log_text = "Blocked by obstacles, replanning! "
            self.pub_msg_rqt(log_text, self.exeInfo_pub)

        # if the replan succeed
        if self.doing_replan == True and self.LTLMP.goalFound == True and obs_no_affect == False:
            self.doing_replan = False
            self.replan_total_times += 1
            replan_time = (rospy.Time.now() - self.replan_start_time).to_sec()
            self.replan_total_secs += replan_time
            print('replan using ' + str(replan_time) + ' s')
            self.pub_msg_rqt('replan using ' + str(replan_time) + 's', self.exeInfo_pub)

            plan_skeleton = self.LTLMP.wrap_plan()
            self.pub_msg_rqt(plan_skeleton, self.trace_pub)

        if finished:
            finish_time = rospy.Time.now()
            total_time = (finish_time - self.start_time).to_sec()
            print('elapse time is ' + str(total_time))
            if self.replan_total_times > 0:
                print('avg replan is ' + str(self.replan_total_secs / self.replan_total_times))

            self.mode = DiscreteMode.notInit
            self.pub_msg_rqt('elapse time is ' + str(total_time) + 's', self.exeInfo_pub)

        if self.LTLMP.goalFound:
            ## use astar
            path_seg = []
            # # if cur_pose is not the 1st wp
            for nd in self.LTLMP.cur_path:
                path_seg.append(nd.pos)

                if len(self.LTLMP.rt_root.label):
                    if len(nd.label) and list(nd.label)[0] != list(self.LTLMP.rt_root.label)[0]:
                        break
                else:
                    if len(nd.label):
                        break
            # for fast forward motion
            self.LTLMP.seg_num = len(path_seg)
            if len(path_seg):
                self.pub_temp_goal(path_seg[-1])

        self.mutex.release()


    def on_reset_task(self):
        # reset the planner
        self.LTLMP.reset_task(self.automaton)
        self.LTLMP.supplement_nodes()
        self.LTLMP.reset_tree(self.rviz_mgr)

    def checkShortCut(self, event):
        if not self.doing_replan and self.mode != DiscreteMode.planning:
            return
        self.LTLMP.check_all_nodes_to_goal()


if __name__ == '__main__':
    # temp for test
    rospy.set_param('workspace_choice', 'Vicon room')

    workspace = Workspace3D()

    para = dict()
    para['step_size'] = np.inf
    para['real_time_plot'] = False
    para['goal_radius'] = 1
    para['time_for_rewire'] = 0.2

    # tell if to change root
    para['changeRoot_radius'] = 0.4
    # same  solution radius
    para['sameSol_radius'] = 2
    # region bias sampling
    para['reg_sample_ratio'] = 0.24
    #  near dist
    para['min_near_dist'] = 3
    para['max_near_dist'] = 5
    para['k_max'] = 100

    para['bias_sample'] = True

    para['fov_dist'] = 2
    para['reg_change_ls'] = ['l1', 'l3']

    node = motionA2B_HMI_RTMQ_ros(workspace, para)

