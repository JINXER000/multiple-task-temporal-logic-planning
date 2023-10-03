The software for IROS2023 paper: An interactive system for multiple-task linear temporal logic path planning. 

# functionality
- [x] Sampling-based LTL path planning 
- [x] Task switching
- [ ] Human-machine interface

# install
## 3rd party dependancies
```
pip install numpy networkx itertools matplotlib ply pybullet
```
## ltl2dfa
```
sudo apt-get install flex -y
sudo apt-get install bison -yx
sudo apt-get install byacc -y
git clone https://github.com/whitemech/LTLf2DFA.git
cd ltlf2dfa
pip install .
```
## ltl2ba
follow $(PROJECT_PATH)/src/formula_parser/install_ltl2ba/README.txt.


# Folder configuration
Replace folder name in workspace.py and gazebo_HMI_plugin.py. A quick way is to search '/home/joseph/yzchen_ws'.


# Experiment with the whole system

## [optional when doing simulation] launch a gazebo environment
## run modules like localization, mapping, and motion planning
```
./start nndp
```
## launch HMI
```
cd /home/joseph/yzchen_ws/task_planning/ltl_interface/
source devel/setup.bash
rqt
```
Then open the plugin in rqt: 'LTL task planning'
## launch the teleoperation service
```
cd /home/joseph/yzchen_ws/task_planning/ltl_interface/
source devel/setup.bash
python /home/joseph/yzchen_ws/task_planning/ltl_interface/src/remote_server/src/remote_launch_service.py
```
## launch the LTL planner
```
python gazebo_HMI_plugin.py
```
## Get ready for the task (e.g., taking off, do system selection in HMI)
## Input the LTL specification by voice or text

# credit
The body of code in pyb_plan_utils.py comes from https://github.com/caelan/pybullet-planning.git.