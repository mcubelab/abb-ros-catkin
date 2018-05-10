# abb-ros-catkin for Yumi

ABB ROS node forked from MLab, customized for the dual arm 7-DOF Yumi robot. 

## Workspace Setup

We have to set up a catkin workspace, clone the repo and build it. 
```
$ mkdir my-workspace && cd my-workspace
$ catkin init
$ catkin config --extend /opt/ros/indigo
$ catkin config -a --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
$ cd src
$ git clone https://github.com/mcubelab/rut.git
$ cd ../..
$ catkin_make
```

## Connect with controller

Using the teach pendent, place the robot in `auto`. Then navigate to the `Production Window` and click `PP to Main`. Finally, press the physical `Play` button.

Inside the catkin workspace created above, launch the controller:
```
$ source devel/setup.bash
$ roslaunch robot_node mcubeSystem.launch
```

## ROS Services and Topics Notes

You can view the possible services and topics with `rosservice list` and `rostopic list` respectively. We will detail a few commonly used services.   

### Execute Trajectories
We can execute a joint space trajectory by adding several poses to a buffer, and then executing that buffer 
```
rosservice call /robot1_ClearJointPosBuffer
rosservice call -- /robot1_AddJointPosBuffer 0 0 0 0 90 0 0
rosservice call -- /robot1_AddJointPosBuffer 0 0 0 0 91 0 0
rosservice call -- /robot1_AddJointPosBuffer 0 0 0 0 89 0 0
rosservice call /robot1_ExecuteJointPosBuffer
```

We can do a similar series of commands for a cartesian space trajectory: 
```
rosservice call /robot1_ClearBuffer
rosservice call /robot1_SetSpeed 50 50 # apply to the following knot points until the next set speed.
rosservice call -- /robot1_AddBuffer 300 0 300 1 0 0 0    # x y z (mm) q0 qx qy qz
rosservice call -- /robot1_AddBuffer 300 0 301 1 0 0 0
rosservice call /robot1_SetSpeed 50 100
rosservice call -- /robot1_AddBuffer 300 0 300 1 0 0 0
rosservice call /robot1_ExecuteBuffer  # go through the whole trajectory
```
Note: Too small spacing between points may cause jerky motions. Try SetZone to higher value.
