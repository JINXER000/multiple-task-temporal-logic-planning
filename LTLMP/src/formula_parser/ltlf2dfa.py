#!/home/joseph/yzchen_ws/env/py3torch/bin/python3
#/home/joseph/yzchen_ws/task_planning/ltl_ros_ws/src/src/P_MAS_TG/rrt_collection/formula_parser/ltlf2dfa.py
import sys
sys.path.append('/home/joseph/yzchen_ws/env/LTLf2DFA/')
sys.path.append('/home/joseph/yzchen_ws/env/LTLf2DFA/ltlf2dfa/')
from ltlf2dfa.parser.ltlf import LTLfParser


if __name__ == '__main__':
    parser = LTLfParser()
    # formula_str = "F (pond && F grassland) && FG base"
    # formula_str = "(!grassland U pond) && F grassland "
    formula_str = "F (pond && F grassland)"
    formula = parser(formula_str)

    # dfa = formula.to_dfa()

    # path = "until.gv"
    # fp= open(path, "w")
    # test_gv = fp.write(dfa)
    # fp.close()