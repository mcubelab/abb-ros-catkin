abb-ros
=======

ABB ROS node forked from MLab

Make it
------
clone it under catkin_ws and use ```catkin_make```

Connect with controller
------
```
roslaunch robot_node mcubeSystem.launch
```


Run a series of joint configurations:
------

```
rosservice call /robot1_ClearJointPosBuffer
rosservice call -- /robot1_AddJointPosBuffer 0 0 0 0 90 0
rosservice call -- /robot1_AddJointPosBuffer 0 0 0 0 91 0
rosservice call -- /robot1_AddJointPosBuffer 0 0 0 0 89 0
rosservice call /robot1_ExecuteJointPosBuffer
```

Run a series of cartesian configurations:
------

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

