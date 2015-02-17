float64 timeStamp    # ROS time-stamp
float64 logtimeStamp # Time since the beginning of last logStart.
int64 id             # Id of file begin logged.
float64 x            # x-coordinate of the robot.
float64 y            # y-coordinate of the robot.
float64 z            # z-coordinate of the robot.
float64 q0 	     # q0-coordinate of the robot.
float64 qx           # qx-coordinate of the robot.
float64 qy	     # qy-coordinate of the robot.
float64 qz	     # qz-coordinate of the robot.
float64 j1	     # joint 1 of the robot.
float64 j2	     # joint 2 of the robot.
float64 j3	     # joint 3 of the robot.
float64 j4	     # joint 4 of the robot.
float64 j5	     # joint 5 of the robot.
float64 j6	     # joint 6 of the robot.
float64 fx	     # force in the x direction.
float64 fy	     # force in the y direction.
float64 fz	     # force in the z direction.
float64 tx	     # torque in the x direction.
float64 ty	     # torque in the y direction.
float64 tz	     # torque in the z direction.
int64 encMotor	     # motor encoder.
int64[] encFinger    # array of finger encoders.
float64 angleMotor   # motor angle.
float64[] angle	     # array of finger angles.
int64[] rawForces	   # hand palm raw forces.
float64[] forces	   # hand palm calibrated forces.
