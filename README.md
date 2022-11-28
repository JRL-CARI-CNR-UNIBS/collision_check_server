# Compute IK solutions using RosDyn #



## Description
This package computes all the IK solutions for a TF using RosDyn. __No collision checking are provided__

Services are described [here](https://github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs).


## Usage

run the node _node_ providing the following parameter
### required (local) params
```yaml
base_frame: world # base frame of the chain
flange_frame: tool0 # end frame of the chain
tool_frame: open_tip # destination frame of the IK (it should be rigid attached to flange_frame)
desired_solutions: 32 # number of desired solution
```
Example in [this launcher](launch/test.launch)

## Test
run the node _get_tf_ik.py_ providing the name of the IK server and the name of the desired tf. Example:
```
rosrun rosdyn_ik_solver get_tf_ik.py [server_name] [tf_name]
```
The node publish a _moveit_msgs/DisplayRobotState_ topic called _ik_solution_. The robot state shows cyclically the IK solution.
