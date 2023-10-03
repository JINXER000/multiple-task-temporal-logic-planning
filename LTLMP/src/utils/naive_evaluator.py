import numpy as np

class evaluator:
    def __init__(self):
        pass

    def euclideanDist(self, p0, p1):
        return np.linalg.norm(np.subtract(p0,p1))

    def getCost(self, near_node, x_new):
        return self.euclideanDist(near_node.pos, x_new)

    def getHeuristic(self, x_new, goal_pos):
        return self.euclideanDist(goal_pos, x_new)