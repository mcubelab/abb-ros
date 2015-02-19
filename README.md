abb-ros
=======

ABB ROS node forked from MLab

Connect with controller
------
```
roslaunch launch mcubeSystem.launch
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
