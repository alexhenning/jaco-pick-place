Jaco Pick and Place
===================

Description
-----------

This repository includes the software necessary for running simple
cylinder pick and place with the Jaco arm in a simple world. It relies
on the [wpi_jaco](https://github.com/RIVeR-Lab/wpi_jaco) package for
simulation support of the Jaco arm.

Usage
-----
1. Open two terminals
   - Run "cd <workspace> && source devel/setup.bash"
2. Run `roslaunch jaco_pick_place cylinder_world.launch` in the first terminal
3. Wait for Gazebo and RViz to launch
4. Run `rosrun jaco_pick_place cylinder_grasps` 
