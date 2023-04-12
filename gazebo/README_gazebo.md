# Gazebo simulation

## Downloading world and launch files
Download the 'tiago_dual_gazebo' folder and replace your existing folder :

- /tiago_dual_public_ws/src/tiago_dual_simulation/tiago_dual_gazebo

Download 'pal_gazebo2.launch' file and move to the 'pal_gazebo_worlds/launch' folder:

- /tiago_dual_public_ws/src/pal_gazebo_worlds/launch/pal_gazebo2.launch

## Running the simulation
To run, replace command to run gazebo with the the following:

- roslaunch tiago_dual_gazebo tiago_dual_gazebo2.launch public_sim:=true end_effector_left:=pal-gripper end_effector_right:=pal-gripper base_type:=omni_base

