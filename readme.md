# Mission Coordination TP
Autnonomous mobile robot navigation task. 

## About
Repository containing solutions to the final project realised as a lab activity on Mission Coordination course, at Université d'Évry. Fall 2022. 

## Solutions

The realisation of the tasks concerned development of navigation and control strategies for autonomous robots. The issue concerned mixed-level control directives for said robots but not from a swarm-like perspective of robot coordination, but rather from POV of individual unit - adaptive response to the environment while trying to achieve its prescribed goal. 

Presented robust navigation strategies were developed in accordance to the presented assumptions on which data do we have at our disposal. The developed solutions did not involve 'bending' the rules of the game, and by extension any major interference with provided robot control interfaces (in fact - we only changed the velocity constraint at one point, to allow the robot to move a tiny bit faster).

These strategies are: 
1. *"Parabolic trajectory modification by means of temporary following of alternate goal"*. The core idea of this strategy is for the robot to move in the general direction of a main goal (flag), and in case of detecting the obstacle - the robot determines a temporary virtual goal point. The robot then follows the path to the new goal for a time proportional inversely proportional to the refresh rate of the control node, after which it continues to follow the path to the main goal. The resulting motion is a parabolic deflection from the main goal. The procedure of obstacle deflection maneuver can be preempted by detection of new obstacle, in case of which the robot only continues to follow the main goal after there are no obstacles detected. 

2. "Quasi-APF method of adjusting the the robot-path trajectory". The inspiration for this strategy is taken from the concept of the domain of path-planning. The robot is deflecting its course based on the virtual repulsive field computed around the detected obstacle. Until the robot is sufficiently far away from the obstacle, it is acted upon by its repulsive field. This is not the 1:1 implementation of the APF method. The adjustment concerns only the heading angle, based on the distance from the obstacle and the angle by the vector pointing from the object to the obstacle at any given time it does not change the robot's velocity. It is like influencing the orientation of a sailboat heading with an upside-down tornado. 

## Outlines for running the solutions
The workspace is based on the repository provided by the instructor during the classes, so it will work in the same environment (`ROS noetic` is required, the rest should work out-of-the-box). To test the proposed solutions one has to perform the following steps:

1. Create a dedicated folder:
```bash
mkdir MC_TP_Grp3
```
Then clone the repository:

```bash
cd MC_TP_Grp3
git clone https://github.com/miodine/mission_coordination_tp.git
```
2. Re-build the workspace and preprare the environment: 
```bash 
cd mission_coordination_tp/catkin_ws/
catkin_make clean
catkin_make
source devel/setup.bash
```
3. Testing procedures: 
To make sure that the routines work as intended, for each test: 
```bash 
roslaunch evry_project_description simu_robot.launch 
```
In other terminal, in the same directory, run:
```bash
roslaunch evry_project_strategy agent_str_one.launch
```
... `agent_str_one.launch` is the launch file for the first strategy (deflection), to test second one run `agent_str_two.launch` (quasi-APF). After finished test, close gazebo, and kill the process.
```bash 
killall gzclient
killall gzserver
```
This is important, for some reason on local machines the gazebo server does not work properly, and the service for returing the distance to the flag breaks. This service fails at initial stage of the simulation too, that's why in the strategies - for flag coordinate estimation routine the robot is gets the instruction to move a bit and stop before executing the trajectory. This rought patch makes it work good.

After that, repeat the entire third point  for the other strategy (after replacing the launch file name in the second to last step -  `roslaunch evry_project_strategy agent_str_two.launch`).

## Remarks 
1. The simulations were tested on a local machine, not in TheConstructSim. 

2. Make sure to always close gazebo and kill the orphaned processes related to it after finished tests, if testing outside TheConstructSim. 

