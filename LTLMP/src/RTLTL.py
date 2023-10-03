## TODO: rename
## rename all function name as AaBb(), not aa_bb()
## make the FSA (idle, planning, acting) into the planner! delete the condition about goal_found

import networkx as nx
import numpy as np
from networkx.classes.digraph import DiGraph
from utils.misc_func import get_automaton_dict, LineSegment, LinesegIntersectsAABB, getLabel, \
    isWithinAABB, randomPointAABB, pybCheckCollision, collisionFreeAABB, ptInObs

import pybullet as p
from random import uniform
import  rospy

class RT_LTL_planner():
    class Node():
        def __init__(self, node_id, pos, cost2Root, b_state, label):
            self.node_id = node_id
            self.pos = pos
            self.cost2Root = cost2Root
            self.b_state = b_state
            self.label = label
            self.same_pos_nodes =set()
            self.visited = False

        def __hash__(self):
            return hash(self.node_id)

        def __eq__(self, other):
            return (self.__class__ == other.__class__ and self.node_id == other.node_id)

    def __init__(self, root, goal, workspace, evaluator, para, automaton, segment, is_hybrid = False):
        self.para = para

        self.tree = DiGraph(init_pose = root.pos)
        self.tree.add_node(root)
        self.init_state = root
        self.goal = goal
        self.nodesAroundGoal = []
        self.goalFound = False
        self.evaluator = evaluator
        # workspace
        self.workspace = workspace
        self.dim = len(self.workspace.workspace_range)

        # variables for real-time
        self.rt_root = self.init_state
        self.cur_target = None
        self.sparse_r = 5
        self.cur_path = []
        self.rand_near_nbrs = set()
        self.rewireRand = set()
        self.visited_set = set()
        self.rewireRoot = set()
        self.pushedToRewireRoot = set()  # has been pushed to rewireRoot within current iter
        self.next_wp = None
        self.old_dynamic_obs = (-10, -10)
        self.old_dynobs_aabb = None

        # variables for LTL
        aut_dict = get_automaton_dict(automaton)
        self.rt_root.b_state = aut_dict[self.rt_root.b_state]
        self.automaton = self.unifyAutomata(automaton, aut_dict)
        self.accept = self.automaton.graph['accept']
        self.segment = segment
        self.unreachable_b_states = self.preprocessDFA()


        self.possible_paths = []
        self.timeKeeper = None
        self.detect_obs_move = False
        self.detect_TS_change = False
        # for node_id, only increase. The reason for designing this parameter is that
        # if using len(self.tree.nodes()), some nodes will be deleted, and id will be duplicated.
        self.added_nodes_cnt = 1
        self.task_initialized = True
        self.rbt_pos = self.rt_root.pos
        self.root_just_changed = False

        self.bad_regs = set()
        self.dist_root2next = 0
        self.dist_root2rbt = 0
        self.dist_next2rbt = 0

        self.is_hybrid = is_hybrid
        self.changed_regs = set()
        self.nodes_pos_dict = {root.pos: root}

        self.old_dynobs_AABB = None

        

    def unifyAutomata(self, automaton, aut_nodes_dict):

        renamed_graph = automaton.copy()
        renamed_graph = nx.relabel_nodes(renamed_graph, aut_nodes_dict)

        initial_set = set()
        for nd in renamed_graph.graph['initial']:
            initial_set.add(aut_nodes_dict[nd])
        renamed_graph.graph['initial'] = list(initial_set)

        accept_set = set()
        for nd in renamed_graph.graph['accept']:
            accept_set.add(aut_nodes_dict[nd])
        renamed_graph.graph['accept'] = list(accept_set)

        return renamed_graph

    def reverseVisit(self, b_node, reachable_set):
        reachable_set.add(b_node)

        for b_parent in self.automaton.pred[b_node]:
            if b_parent == b_node:
                continue
            if b_parent in reachable_set:
                continue
            self.reverseVisit(b_parent, reachable_set)

    # mark unreachable nodes
    def preprocessDFA(self):
        for ac_node in self.accept:

            reachable_set = set()
            self.reverseVisit(ac_node, reachable_set)

            all_b_nodes = set(self.automaton.nodes())
            unreachable = all_b_nodes.difference(reachable_set)

            return unreachable




    def checkCrossLabel(self, node, x_new, new_label):
        #  not changing b_state if crossing
        for (region, aabb) in iter(self.workspace.regions.items()):
            j1 = LinesegIntersectsAABB(aabb, LineSegment(node.pos, x_new))

            inspect_label = getLabel(self.workspace.reg_centers[region], self.workspace, self.is_hybrid)
            j2 = inspect_label!= new_label
            j3 = inspect_label!= node.label
            if j1 and j2 and j3:
                return False
        return True


    def checkBuchiTran(self, q_b, x_label, q_b_new):

        if q_b is None or q_b_new is None or\
             q_b not in self.automaton.nodes or q_b_new not in self.automaton.nodes:
            return False

        b_state_succ = self.automaton.succ[q_b]
        if q_b_new not in b_state_succ:
            return False

        tran_valid = self.automaton.edges[q_b, q_b_new]['guard'].check(x_label)
        # bad states
        is_astray = q_b_new in self.unreachable_b_states or q_b in self.unreachable_b_states

        ok = tran_valid and not is_astray
        return ok

    def near_succ(self, inspect_node):
        near_nodes = set()
        checked_set = set([inspect_node.pos])
        cost_dict = {(inspect_node.pos, inspect_node.pos): 0}
        radius = self.near_dist

        for node in self.tree.nodes:
            if self.evaluator.euclideanDist(inspect_node.pos, node.pos) > radius:
                continue

            if node.b_state not in self.automaton.succ[inspect_node.b_state]:
                continue

            # check trans
            trans_ok = self.checkBuchiTran(inspect_node.b_state, inspect_node.label, node.b_state)
            if not trans_ok:
                continue

            # for DFA, once state is accepting, we don't care its outward edge
            acc2other = inspect_node.b_state in self.accept and node.b_state not in self.accept
            if acc2other:
                continue

            if node.pos not in checked_set:
                # check obs
                no_collision = collisionFreeAABB(node.pos, inspect_node.pos, self.workspace, self.old_dynobs_aabb)
                safe_label = self.checkCrossLabel(node, inspect_node.pos, inspect_node.label)
                if not (no_collision and safe_label):
                    continue

                cost_dict[(inspect_node.pos, node.pos)] = self.evaluator.getCost(node, inspect_node.pos)
                checked_set.add(node.pos)

            near_nodes.add(node)

        return near_nodes, cost_dict


    def extend(self, new_node, near_nodes, cost_dict):
        added = False
        min_cost = 1e7
        best_nbr = ()

        # near_nodes are nodes that are collision-free, and have distinct position
        for diff_x_node in near_nodes:
            # iterate over S^{rcd}
            for node in diff_x_node.same_pos_nodes:
                if node.node_id == new_node.node_id:
                    continue

                j2 = self.checkBuchiTran(node.b_state, node.label, new_node.b_state)
                # don't go from accept to non-accept
                j3 = node.b_state in self.accept and new_node.b_state not in self.accept

                if j2 and not j3:
                    cost = self.calc_cost2Root(node) + cost_dict[(new_node.pos, node.pos)]
                    if cost < min_cost:
                        added = True
                        best_nbr = node
                        min_cost = cost

        if added:
            new_node.cost2Root=min_cost
            self.addNode(new_node, best_nbr)

        return added

    def rewire(self, new_node, near_nodes,  cost_dict):
        rewired_nodes = set()
        for node in near_nodes:
            if node.node_id == new_node.node_id:
                continue
            # if b_state is None, assign new
            if node.cost2Root == np.inf and node.b_state is None:
                raise NotImplementedError()
            else:
                # note: as near_nodes are produced by near_succ(), there is no need to do checkBuchiTran()

                cost = self.calc_cost2Root(new_node) + cost_dict[(new_node.pos, node.pos)]
                delta_c = self.calc_cost2Root(node) - cost

                if delta_c > 1e-3:
                    pred_node_list = list(self.tree.pred[node].keys())
                    if len(pred_node_list):
                        pred_node = pred_node_list[0]
                        self.tree.remove_edge(pred_node, node)
                    # else: isolated node
                    node.cost2Root = cost
                    self.tree.add_edge(new_node, node)

                    rewired_nodes.add(node)
        return rewired_nodes

    # one step further on automaton.
    def checkNode2Goal(self, node, is_hybrid, approx=False):
        if self.segment == 'suffix':
            raise 'Not Implemented'

            # filter out nodes blocked by dyn obs and isolated nodes
        nd_cost = self.calc_cost2Root(node)
        if nd_cost == np.inf:
            return False

        goal_updated = False

        if node.b_state in self.accept:
            pred_nodes = list(self.tree.pred[node].keys())
            # if root satisfy LTL
            if len(pred_nodes) == 0:
                goal_updated = self.update_target(node)
                return goal_updated
            # else use the previous as the goal
            else:
                q_n = pred_nodes[0]
                if q_n.b_state not in self.accept:
                    goal_updated = self.update_target(q_n)
                    # accept -->(1)--> accept
                    return goal_updated

        # node.b_state not in self.accept
        for b_accept in self.accept:
            nd_label = getLabel(node.pos, self.workspace, is_hybrid)
            if self.checkBuchiTran(node.b_state, nd_label, b_accept):
                goal_updated = self.update_target(node)

        return goal_updated

    def pos_near(self, pos1, pos2, dist):
        if self.evaluator.euclideanDist(np.array(pos1), np.array(pos2)) < dist:
            return True
        else:
            return False

    # similar to  homotopy path
    def is_same_solution(self, sol1, sol2):
        if not len(sol1) or not len(sol2):
            raise 'invalid input'
        #1 if the ending point is different
        if not self.pos_near(sol1[-1].pos,sol2[-1].pos, 20):
            return False

        #2 check the changing point list: different length or shape
        if len(sol1) != len(sol2):
            return False

        for i in range(len(sol1)):
            if sol1[i].label != sol2[i].label or not self.pos_near(sol1[i].pos,sol2[i].pos, self.para['sameSol_radius']):
                return False

        return True

    def retrace_path(self, goal_node):
        path = [goal_node]
        pred_ls = list(self.tree.pred[goal_node].keys())
        s = goal_node
        while len(pred_ls)!=0:
            s = self.tree.pred[s].keys()[0]
            path.insert(0, s)
            pred_ls = list(self.tree.pred[s].keys())
        return path

    # find similar path; if found, update it; if not found, put the tgt into pq.
    # if root.automaton changed, try to rewire and check reachability.
    def update_target(self, node):
        updated = False
        if node.b_state is None:
            raise KeyError()

        node_path = self.retrace_path(node)
        node_trace = self.extract_trace(node_path)
        if len(node_trace) == 0:
            return False

        if self.cur_target is None:
            self.cur_target = node
            self.goalFound = True
            self.possible_paths.append(node_path)
            return True

        node_cost = self.calc_cost2Root(node)

        is_similar = False
        for i in range(len(self.possible_paths)):
            path_cmp = self.possible_paths[i]
            trace_cmp = self.extract_trace(path_cmp)
            is_similar = self.is_same_solution(trace_cmp, node_trace)

            if is_similar:
                # compare cost
                end_node = trace_cmp[-1]
                if node_cost < self.calc_cost2Root(end_node):
                    # assign address?
                    self.possible_paths[i] = node_path
                    updated = True

                # can be similar but not that good
                break

        if not is_similar:
            self.possible_paths.append(node_path)
            updated = True

        if updated:
            self.possible_paths = sorted(self.possible_paths, key=lambda trace: self.calc_cost2Root(trace[-1]))
            self.cur_target = self.possible_paths[0][-1]

        return updated

    def calc_cost2Root(self, node):
        # next_wp is the R_aux
        is_pass_nxt = False
        is_bad_node = node.cost2Root == np.inf
        cost = 0
        pred_ls = list(self.tree.pred[node].keys())
        s = node
        while len(pred_ls) != 0:
            if  self.next_wp is not None and self.next_wp == s:
                is_pass_nxt = True

            parent = self.tree.pred[s].keys()[0]
            # check parent is valid
            if parent.cost2Root == np.inf:
                is_bad_node = True

            # check automaton tran
            parent_label = parent.label
            parent_bs = parent.b_state
            child_bs = s.b_state

            tran_valid = self.checkBuchiTran(parent_bs, parent_label, child_bs)
            if not tran_valid:
                s.cost2Root = np.inf
                is_bad_node = True

            cost += self.evaluator.euclideanDist(s.pos, parent.pos)
            s = parent
            pred_ls = list(self.tree.pred[s].keys())

        if is_bad_node:
            node.cost2Root = np.inf
            return np.inf
        else:
            if is_pass_nxt:
                cost -= self.dist_root2next

            node.cost2Root = cost
            return cost

    # check the trace satisfy the specification
    def examine_trace(self, path):

        # implicitely check transition
        path_cost = self.calc_cost2Root(path[-1])

        if path_cost == np.inf:
            return False

        last_bs = path[-1].b_state
        last_label = path[-1].label
        for b_accept in self.accept:
            if self.checkBuchiTran(last_bs, last_label, b_accept):
                return True

        return False

    # get event-driven trace
    def extract_trace(self, path):
        if len(path) <2:
            return path

        trace = [] # no starting point
        for id in range(0,len(path)-1):
            if path[id].label!= path[id+1].label:
                trace.append(path[id])

        trace.append(path[-1])
        return trace

    def ntr_node(self, better_node, worse_node):

        # rewire worse node
        succ_worse_nodes = list(self.tree.succ[worse_node])
        for succ_worse_node in succ_worse_nodes:
            # rewire then do dfs on tree2
            self.tree.remove_edge(worse_node, succ_worse_node)
            self.tree.add_edge(better_node, succ_worse_node)

        prev_worse_node = list(self.tree.pred[worse_node])[0]
        self.tree.remove_edge(prev_worse_node, worse_node)
        worse_node.cost2Root = np.inf
        self.safe_remove_node(worse_node)

        # replace worse nodes in possible path
        for path in self.possible_paths:
            if path[-1] == worse_node:
                path[-1] = better_node

    def safe_remove_node(self, node):

        if len(node.same_pos_nodes):
            node.same_pos_nodes.remove(node)

        same_pos_nd_remain = None
        if len(node.same_pos_nodes):
            same_pos_nd_remain = list(node.same_pos_nodes)[0]

        # take care of pos_node_dict
        if self.nodes_pos_dict[node.pos] == node:
    
            if same_pos_nd_remain is None:
                # delete the entry
                self.nodes_pos_dict.pop(node.pos)
            else:
                self.nodes_pos_dict[node.pos] = same_pos_nd_remain

        self.tree.remove_node(node)

    def addNode(self, node, parent = None):
        self.tree.add_node(node)
        self.nodes_pos_dict[node.pos] = node
        self.added_nodes_cnt +=1

        if parent:
            self.tree.add_edge(parent, node)

    def isIsolated(self, node):
        if self.tree.in_degree(node) ==0 and self.tree.out_degree(node) ==0:
            return True
        else:
            return False

    def mergeNode(self, s_m, depth):
        better_node = s_m

        # pre-order
        same_state_nodes = [nd for nd in s_m.same_pos_nodes \
            if nd.b_state == s_m.b_state and nd != s_m] #  and len(list(self.tree.pred[nd]))!=0

        same_cpy = same_state_nodes[:]
        # delete isolated
        for nd in same_cpy:
            if self.isIsolated(nd):
                same_state_nodes.remove(nd)
                self.safe_remove_node(nd)

        same_state_num = len(same_state_nodes)

        if same_state_num >=1:
            # deal with father-son relation first. father cannot be bad.
            same_state_node = same_state_nodes[0]
            if same_state_node == list(self.tree.pred[s_m].keys())[0]:
                worse_node = s_m
                better_node = same_state_node
            else:
                worse_node = same_state_node

            if self.calc_cost2Root(s_m) > self.calc_cost2Root(same_state_node) and self.calc_cost2Root(same_state_node) <1e6:
                better_node = same_state_node
                worse_node = s_m

            self.ntr_node(better_node, worse_node)
            

            worse_node.visited = True

        better_node.visited = True

        better_succ_list = list(self.tree.succ[better_node])
        # NOTE: better_succ_list can contain worse nodes which is deleted by its siblings. visited flag for the worse nodes are true.
        for child in better_succ_list:
            if child == self.rt_root:
                continue
                            
            # child was visited as a worse node. no succ already.
            if child.visited:
                continue

            self.mergeNode(child, depth+1)


    # 2 step: 1, dfs b_state; 2. do the merging
    def merge_branch(self, subtree_root, parent):

        trans_valid = self.propagate_b_state(parent, subtree_root)
        if not trans_valid:
            return

        print('merge branch rooted at '+ str(subtree_root.pos))

        self.propagateState(subtree_root, 0)

        # DFS post-order on subtree_root. 
        self.mergeNode(subtree_root, 0)


    def propagateState(self, subroot, depth):

        succ_list = list(self.tree.succ[subroot])
        # for merge_branch
        subroot.visited = False

        # NOTE: succ_list can contain worse nodes which is deleted by its siblings
        for child in succ_list:
            if child == self.rt_root:
                continue
            
            # can revise b_satate here
            trans_valid = self.propagate_b_state(subroot, child)
            if not trans_valid:
                continue

            self.propagateState(child, depth+1)

    def propagate_b_state(self, parent, child):


        child_bs_valid = self.checkBuchiTran(parent.b_state, parent.label, child.b_state)
        if  child_bs_valid:
            return True

        buchi_succ = self.automaton.succ[parent.b_state]
        buchi_candidates = []

        for b_state in buchi_succ:
            if b_state != child.b_state and self.checkBuchiTran(parent.b_state, parent.label, b_state):
                buchi_candidates.append(b_state)
             #  TODO: break here
        if len(buchi_candidates)>1: # not a DFA!
            raise NameError('too many candidates')
        elif len(buchi_candidates) ==1:

            # add old state, in case of rewire it back
            same_state_nodes =[nd for nd in child.same_pos_nodes 
                            if nd.b_state== child.b_state and  nd != child]
            if len(same_state_nodes)==0:
                old_id = self.added_nodes_cnt
                old_state_node = self.Node(old_id, child.pos, np.inf, child.b_state, child.label)
                self.addNode(old_state_node)

                child.same_pos_nodes.add(old_state_node)
                old_state_node.same_pos_nodes = child.same_pos_nodes # addr of a list

            child.b_state = buchi_candidates[0]
            child_bs_valid = True
        else: # if DFA, won't enter. for safety LTL only. 
            child.cost2Root = np.inf
        
        return child_bs_valid


    def updateBestPath(self):
        updated = False
        
        # has been sorted in validate_update_sols()
        possible_new_tgt = self.possible_paths[0][-1]
        if possible_new_tgt!= self.cur_target:
            updated = True
            self.cur_target = possible_new_tgt

        self.cur_path = self.retrace_path(self.cur_target)
        if len(self.cur_path)>1:
            self.next_wp = self.cur_path[1]
            self.dist_root2next = self.evaluator.euclideanDist(self.next_wp.pos, self.rt_root.pos)
        else:
            self.next_wp = self.cur_path[0]
            self.dist_root2next = 0.0
        return updated

    def validate_update_sols(self):
        # check if the all candidates are valid and leading to accept (cur_path should be the 1st of candidates)
        resume_path =  False
        
        valid_paths = []
        for path in self.possible_paths:
            updated_path= self.retrace_path(path[-1])
            if self.examine_trace(updated_path):
                valid_paths.append(updated_path)

        if len(valid_paths)==0: 
            self.codeRestart()
            return resume_path

        self.possible_paths = sorted(valid_paths, key=lambda trace: self.calc_cost2Root(trace[-1]))
        # update cur_target, cur_path
        self.updateBestPath()
        self.goalFound = True

        resume_path = True        

        return resume_path

    def changeRoot(self, new_root):
        require_prop = False
        self.tree.remove_edge(self.rt_root, new_root)
        self.rt_root.cost2Root = new_root.cost2Root
        new_root.cost2Root = 0

        self.tree.add_edge(new_root, self.rt_root)

        reverse_trans_valid = self.checkBuchiTran(new_root.b_state, new_root.label, self.rt_root.b_state)
        if not reverse_trans_valid:
            require_prop = True

        self.rt_root = new_root

        return require_prop



    def get_bad_regs(self):
        self.bad_regs.clear()
        # TODO: root_b + root_lab = cur_b
        buchi_succ = self.automaton.succ[self.rt_root.b_state]
        cur_b_state = self.rt_root.b_state
        for b_state in buchi_succ:
            if self.checkBuchiTran(self.rt_root.b_state, self.rt_root.label, b_state):
                cur_b_state = b_state
                break

        b_state_succ = self.automaton.succ[cur_b_state]
        for bad_b in self.unreachable_b_states:
            if bad_b not in b_state_succ:
                continue

            for reg in self.workspace.regions:
                prop = self.workspace.properties[reg]
                lead_to_bad = self.automaton.edges[cur_b_state, bad_b]['guard'].check(prop)
                if lead_to_bad:
                    self.bad_regs.add(reg)




    def sudden_stop(self):
        require_prop = False
        
        exactly_on_root = self.dist_root2rbt< self.para['changeRoot_radius']
        exactly_on_nxt = self.goalFound and self.dist_next2rbt < self.para['changeRoot_radius']
        if exactly_on_root:
            add_node_pos = self.rt_root.pos
            self.dist_root2rbt = 0
        elif exactly_on_nxt: # if meet obs and no valid sols, don't enter this condition
            add_node_pos = self.next_wp.pos
            require_prop = self.changeRoot(self.next_wp)
            self.dist_root2rbt = 0
        else:
            add_node_pos = tuple(self.rbt_pos)
            
        # if property change no affect, then will not merge branch. 
        curpos_id = self.added_nodes_cnt
        cur_label = getLabel(add_node_pos, self.workspace, self.is_hybrid)

        if self.goalFound:
            prev_b_state = self.next_wp.b_state
        else:
            prev_b_state = self.rt_root.b_state

        curpos_node = self.Node(curpos_id, add_node_pos, self.dist_root2rbt, prev_b_state, cur_label)
        if not exactly_on_root and not exactly_on_nxt:
            curpos_node.same_pos_nodes.add(curpos_node)

        self.addNode(curpos_node, self.rt_root)


        old_root = self.rt_root
        require_prop = self.changeRoot(curpos_node)
        if not self.detect_obs_move:
            self.merge_branch(old_root, curpos_node)

        
        self.visited_set.clear()
        self.rewireRoot.clear()
        self.pushedToRewireRoot.clear()

        if  exactly_on_root:
            print("happen to be on root")
            # delete added dummy root
            self.tree.remove_edge(curpos_node, old_root)
            self.safe_remove_node(curpos_node)
            self.rt_root = old_root
        elif exactly_on_nxt:
            print("happen to be on next_wp")
            # delete added dummy root
            self.tree.remove_edge(curpos_node, old_root)
            self.safe_remove_node(curpos_node)
            self.rt_root = old_root

        return require_prop

    def change_labels(self, reg_name):
        for node in self.tree.nodes:
            if isWithinAABB(node.pos, self.workspace.regions[reg_name]):
                node.label = getLabel(node.pos, self.workspace, is_hybrid=self.is_hybrid)

    def codeRestart(self):
            # disable current plan
            self.cur_target = None
            self.goalFound = False
            self.cur_path = []
            self.next_wp = None
            self.dist_root2next = 0
            self.possible_paths = []

            self.check_all_nodes_to_goal()

    # caution: must assure that the cost is not inf
    def check_all_nodes_to_goal(self, is_hybrid = False):
        if self.goalFound:
            return

        # check rt_root first
        goal_updated = self.checkNode2Goal(self.rt_root, is_hybrid, approx=True)
        if goal_updated:
            print('Already fulfill the task!')
            self.updateBestPath()
            return

        for node in list(self.tree.nodes()):

            goal_updated = self.checkNode2Goal(node, is_hybrid, approx=True)
            if goal_updated:
                print('got shortcut')
                self.updateBestPath()
                break

    # NOTE: invalid_set is deleted
    def blockNodes(self):
        move_obs_AABB = self.old_dynobs_AABB
        for child in list(self.tree.nodes()):
            if child == self.rt_root:
                continue

            pred_ls = list(self.tree.pred[child].keys())
            if len(pred_ls) ==0:
                continue
            parent = pred_ls[0]

            # make move_obs_bdr an AABB,
            is_intersect = LinesegIntersectsAABB(move_obs_AABB, LineSegment(parent.pos, child.pos))
            if is_intersect:
                if isWithinAABB(parent.pos, move_obs_AABB):
                    # in case parent is root
                    if parent.cost2Root !=0:
                        parent.cost2Root = np.inf

                    child.cost2Root = np.inf


    def handle_perception(self, reg_name):
        obs_no_affect = False

        if self.detect_obs_move:        
                self.blockNodes()

                resume_path = self.validate_update_sols()
                if not resume_path:
                    self.sudden_stop()
                    self.root_just_changed = True
                else:
                    obs_no_affect = True
            
        # if resume_path is false, then self.codeRestart() is already called in self.validate_update_sols()
        resume_path = True
        # deal with move obs and new observation 
        if self.detect_TS_change:

            self.change_labels(reg_name)

            self.sudden_stop()
            self.root_just_changed = True

            # if cur plan is invalid on automaton, or blocked by obs (redandent checking) 
            #  or (self.cur_target and self.calc_cost2Root(self.cur_target)== np.inf)
            resume_path = self.validate_update_sols()

        return obs_no_affect

    def sample(self):
        if self.para['bias_sample']:
            x_rand = None
            rand_num = uniform(0,1)

            reg_num = len(self.workspace.regions)
            rand_sect = self.para['reg_sample_ratio'] /reg_num


            i=0
            for reg_name, reg_bbx in self.workspace.regions.iteritems():
                if rand_num < 1-  i*rand_sect and rand_num >= 1- (i+1)*rand_sect:
                    x_rand = randomPointAABB(reg_bbx)

                    return x_rand
                i +=1

        x_rand = randomPointAABB(self.workspace.ws_AABB)

        return x_rand

    def calc_near_dist(self):
        # self.near_dist = 50
        # follow the way in RT-RRT*
        self.near_dist = np.sqrt(self.workspace.length*self.workspace.width*self.para['k_max']/np.pi / self.tree.number_of_nodes())

    def nearest_samp(self, x_rand):
        # combine old near() and nearest_samp()
        min_dis = np.inf
        node_nearest = []
        checked_set= set()
        cost_dict = {}
        rand_label = getLabel(x_rand, self.workspace, self.is_hybrid)

        # TODO: put it into kdtree
        for node in self.tree.nodes:
            if self.isIsolated(node) and node!= self.rt_root:
                continue

            x= node.pos
            dis = self.evaluator.euclideanDist(x_rand,x)

            # get the nearest: at least one exists
            if dis < min_dis:
                node_nearest = [node]
                min_dis = dis
            elif dis == min_dis: # no needs
                node_nearest.append(node)
            # near()
            if dis > self.near_dist:
                continue

            if node.pos not in checked_set:
                # check obs
                no_collision = collisionFreeAABB(node.pos, x_rand, self.workspace, self.old_dynobs_aabb)
                safe_label = self.checkCrossLabel(node, x_rand, rand_label)
                if not (no_collision and safe_label):
                    continue

                cost_dict[(x_rand, node.pos)] = self.evaluator.getCost(node, x_rand)
                checked_set.add(node.pos)

            # this is why nearest_samp() is useful in simple case
            self.rand_near_nbrs.add(node)

       
        return node_nearest[0], cost_dict



    # naive steer
    def steer(self, x_rand, x_nearest):
        if self.evaluator.euclideanDist(x_rand, x_nearest) <= self.para['step_size']:
            return x_rand
        else:
            diff = np.subtract(x_rand, x_nearest)
            x_new = np.asarray(x_nearest) + self.para['step_size']* diff / np.linalg.norm(diff)
            return tuple(x_new)

    def expandAndRewire(self):
        self.calc_near_dist()
        #  min < dist < max
        self.near_dist = min(self.near_dist, self.para['max_near_dist'])
        self.near_dist = max(self.near_dist, self.para['min_near_dist'])

        x_rand = self.sample()

        require_extend = True
        node_nearest, cost_dict = self.nearest_samp(x_rand)
        x_nearest = node_nearest.pos
        # if sampled at the existed node
        if self.evaluator.euclideanDist(x_nearest, x_rand) < 1e-2:
            require_extend = False
        # NOTE: rand_near_nodes are nodes around x_rand, not x_new!
        x_new = self.steer(x_rand, node_nearest.pos)
        new_label = getLabel(x_new, self.workspace, self.is_hybrid)
        
        if ptInObs(x_new, self.workspace):
            require_extend = False

        if require_extend and len(self.rand_near_nbrs) < self.para['k_max']: # or self.evaluator.euclideanDist(x_new, node_nearest) > self.sparse_r:
            # do rrt* things

            added_nodes = set()
            not_added_nodes = set()
            for b_state in self.automaton.nodes:

                # NOTE: In prefix, this job is done by checkNode2Goal
                if b_state in self.accept:
                    continue
                # think about until expression
                if b_state in self.unreachable_b_states:
                    continue

                new_node_id = self.added_nodes_cnt
                new_node = self.Node(new_node_id, x_new, np.inf, b_state,new_label)

                added = self.extend(new_node, self.rand_near_nbrs,  cost_dict)
                if added:

                    added_nodes.add(new_node)
                    self.rewireRand.add(new_node)

                    # check reach
                    goal_updated = self.checkNode2Goal(new_node,self.is_hybrid)
                    if goal_updated:
                        print('find better goal')
                else:
                    not_added_nodes.add(new_node)

            # add nodes from not_added_nodes to G
            for nd in not_added_nodes:
                nd.node_id= self.added_nodes_cnt
                self.addNode(nd)
                added_nodes.add(nd)

            for nd in added_nodes:
                nd.same_pos_nodes = added_nodes

        else:
            self.rewireRand.add(node_nearest)
            for nd in node_nearest.same_pos_nodes:
                if nd != node_nearest and self.calc_cost2Root(nd)!=np.inf:
                    self.rewireRand.add(nd)
          
        self.rewireRandNode()

        self.rewireFromRoot()

        return x_rand


    def rewireRandNode(self):
        extendRewireRand = set()
        while len(self.rewireRand):
            Xr = self.rewireRand.pop()
            # NOTE: rewireRand is never cleared, nodes may be deleted by merge_branch
            if Xr.cost2Root == np.inf:
                continue

            near_nodes, cost_dict = self.near_succ(Xr)
            rewired_nodes = self.rewire(Xr, near_nodes,  cost_dict)
            extendRewireRand = extendRewireRand.union(rewired_nodes)

        idx = 0
        while len(extendRewireRand) and idx<3: #self.allowRewiring(0.5*self.para['time_for_rewire']): #
            idx +=1
            Xr = extendRewireRand.pop()
            # NOTE: rewireRand is never cleared, nodes may be deleted by merge_branch
            if Xr.cost2Root == np.inf:
                continue

            near_nodes, cost_dict = self.near_succ(Xr)
            rewired_nodes = self.rewire(Xr, near_nodes,  cost_dict)
            extendRewireRand = extendRewireRand.union(rewired_nodes)


    def allowRewiring(self, threth):
        cur_time = rospy.Time.now()
        if (cur_time- self.timeKeeper).to_sec() < threth:
            return True
        else:
            return False


    def rewireFromRoot(self):
        if len(self.rewireRoot)== 0:
            self.rewireRoot.add(self.rt_root)

        idx = 0
        while len(self.rewireRoot) and self.allowRewiring(self.para['time_for_rewire']): # idx <=5: #
            idx +=1
            Xs = self.rewireRoot.pop()
            # deleted
            if Xs.visited == True:
                continue

            near_nodes, cost_dict = self.near_succ(Xs)
            # delete the parent node of Xs
            if len(list(self.tree.pred[Xs].keys())):
                Xs_parent = list(self.tree.pred[Xs].keys())[0]
                if Xs_parent in near_nodes:
                    near_nodes.remove(Xs_parent)
            obs_check_dict = None
            rewired_nodes = self.rewire(Xs, near_nodes, cost_dict)
            # add nodes which are not pushed into rewireRoot in current iter
            self.rewireRoot = self.rewireRoot.union(rewired_nodes.difference(self.pushedToRewireRoot))
            self.pushedToRewireRoot = self.pushedToRewireRoot.union(rewired_nodes)

    def handle_self_state(self):
        if len(self.cur_path) ==1 and self.evaluator.euclideanDist(self.rbt_pos, np.array(self.rt_root.pos))< self.para['goal_radius']:
            self.task_initialized = False
            print('TASK COMPLETE!')

            return True

        if self.seg_num >=2:
            updated_root_id= -1
            for i in range(1, self.seg_num):

                pos_to_arrive = self.cur_path[i].pos
                if  self.evaluator.euclideanDist(self.rbt_pos, np.array(pos_to_arrive))< self.para['changeRoot_radius']:
                    updated_root_id = i
                    break

            if updated_root_id >1:
                print('Fast forward motion takes over!')
                
            # should change root
            if updated_root_id> 0:
                for j in range(0, updated_root_id):
                    self.cur_path.pop(0) #actually rt_root
                    new_root = self.cur_path[0]

                    old_root = self.rt_root
                    require_prop = self.changeRoot(new_root)
                    if require_prop:
                        self.merge_branch(old_root, new_root)

                    self.root_just_changed = True

                resume_path = self.validate_update_sols()
                if not resume_path:
                    print('current path invalid')

                self.visited_set.clear()
                self.rewireRoot.clear()
                self.pushedToRewireRoot.clear()
                
        return False

    def plan_iteration(self, rbt_pos, rviz_mgr):
        finished = False

        # in case self.detect_obs_move is set true during iteration.
        # if obs_to_proc == 1, then set self.detect_obs_move as 0.
        obs_to_proc = self.detect_obs_move

        self.rbt_pos = rbt_pos
        if not self.task_initialized:
            # rviz_mgr.clear_all()
            return False, False

        self.dist_root2rbt = self.evaluator.euclideanDist(self.rbt_pos, np.array(self.rt_root.pos))

        if self.next_wp is not None:
            self.dist_next2rbt = self.evaluator.euclideanDist(self.rbt_pos, np.array(self.next_wp.pos))

        if len(self.changed_regs):
            reg = self.changed_regs.pop()
        else:
            reg = None
        obs_no_affect = self.handle_perception(reg)

        if not self.root_just_changed and self.goalFound:
            finished = self.handle_self_state()

        self.timeKeeper = rospy.Time.now()
        x_rand = self.expandAndRewire()

        if len(self.possible_paths):
            self.validate_update_sols()

        # post process
        self.get_bad_regs()

        if rviz_mgr:
            rviz_mgr.populateRviz()

            rviz_mgr.draw_rtroot(self.rt_root)
            edges_list = list(self.tree.edges())
            rviz_mgr.display_edges(edges_list)
            nd_ls = list(self.tree.nodes())
            leaf_ls = [nd for nd in nd_ls if self.tree.out_degree(nd) == 0]
            rviz_mgr.draw_leaves(leaf_ls)
            rviz_mgr.draw_empty()
            # rviz_mgr.draw_rewired_root(self.pushedToRewireRoot)
            rviz_mgr.drawFinalPath(self.cur_path)

            
        self.rand_near_nbrs.clear()
        self.detect_goal_changed = False
        if obs_to_proc:
            self.detect_obs_move = False
        self.detect_TS_change = (len(self.changed_regs) != 0)
        self.root_just_changed = False


        return finished, obs_no_affect


    def unify_automata(self, automaton, aut_nodes_dict):



        renamed_graph = automaton.copy()
        renamed_graph = nx.relabel_nodes(renamed_graph, aut_nodes_dict)

        initial_set = set()
        for nd in renamed_graph.graph['initial']:
            initial_set.add(aut_nodes_dict[nd])
        renamed_graph.graph['initial'] = list(initial_set)

        accept_set = set()
        for nd in renamed_graph.graph['accept']:
            accept_set.add(aut_nodes_dict[nd])      
        renamed_graph.graph['accept'] = list(accept_set)  

        return renamed_graph

    def reset_task(self, new_buchi):
        aut_dict = get_automaton_dict(new_buchi)
        self.automaton = self.unify_automata(new_buchi, aut_dict)
        self.accept = self.automaton.graph['accept']
        self.unreachable_b_states = self.preprocessDFA()

        self.possible_paths = []
        self.detect_TS_change = False
        self.bad_regs = set()
        self.dist_root2next = 0
        self.dist_root2rbt = 0
        self.dist_next2rbt = 0

        self.cur_target = None
        self.cur_path = []
        self.rand_near_nbrs = set()
        self.rewireRand = set()
        self.visited_set = set()
        self.rewireRoot = set()
        self.pushedToRewireRoot = set()  # has been pushed to rewireRoot within current iter
        self.detect_goal_changed = False
        self.detect_obs_move = False
        self.invalid_set = set()
        self.next_wp = None

        self.goalFound = False
        self.nodesAroundGoal = []

        self.task_initialized = True


    def wrap_plan(self):
        plan_skeleton = ''
        for nd in self.cur_path:
            node_text = '-> '
            node_text += '({:.2f},{:.2f}) '.format(nd.pos[0], nd.pos[1])
            if len(nd.label):
                node_text+= '['+ list(nd.label)[0] +']'
            plan_skeleton += node_text

        return plan_skeleton


    def supplement_nodes(self):
        # in case 1st time plan
        if len(self.tree.nodes)<=1:
            return

        new_b_set = set(self.automaton.nodes())
        new_b_set = new_b_set.difference(self.accept)
        for pos, node in self.nodes_pos_dict.items():
            existed_b_list = [nd.b_state for nd in node.same_pos_nodes]
            existed_b_set = set(existed_b_list)
            lack_b_set = new_b_set.difference(existed_b_set)

            # add lost nodes
            for b_state in lack_b_set:
                lack_id = self.added_nodes_cnt
                lack_node = self.Node(lack_id, pos, np.inf, b_state, node.label)
                self.tree.add_node(lack_node)
                self.added_nodes_cnt +=1

                node.same_pos_nodes.add(lack_node)
                lack_node.same_pos_nodes = node.same_pos_nodes # addr of a list
                    
    def reset_tree(self, rviz_mgr):
        # in case 1st time plan
        if len(self.tree.nodes)<=1:
            return

        if len(self.automaton.graph['initial'])!=1:
            raise NotImplementedError('more initials')

        # dummy_init 
        dummy_id = self.added_nodes_cnt
        dummy_label = getLabel(self.rt_root.pos, self.workspace)
        init_b_state = self.automaton.graph['initial'][0]
        dummy_node = self.Node(dummy_id, self.rt_root.pos, 0, init_b_state, dummy_label)
        dummy_node.same_pos_nodes.add(dummy_node)
        self.addNode(dummy_node, self.rt_root)
        self.added_nodes_cnt +=1
        
        old_root = self.rt_root
        require_prop = self.changeRoot(dummy_node)
        self.merge_branch(old_root, dummy_node)

        # delete added dummy root
        self.tree.remove_edge(dummy_node, old_root)
        self.safe_remove_node(dummy_node)
        self.rt_root = old_root
