// Type of grasp: GM_OPENLOOP
// Open loop grasp. Defined by the parameters
// (All inside the structure GM_InputParams)
//   double x;                   // X coordinate of the center of the grasp procedue (50mm)
//   double y;                   // Y coordinate of the center of the grasp procedure (-50mm)
//   double z_lim;               // Absolute z limit that the motion should not pass (150mm)
//   double up;                  // How much to go up from the detected surface. (32mm)
//   double down;                // How much to go down in the final grasp. (41mm)
//   int nFlips;                 // Number of oscillation during the grasp. (4)
//   double oscillationAmplitude;// Initial oscillation amplitude. (PI/3rad)
//   int id;                     // Experiment id. Integer to be written at the beginning of the log file.

#include "grasp_openloop_detect.h"

bool grasp_OpenLoop(grasp_comm::grasp_OpenLoop::Request& req, grasp_comm::grasp_OpenLoop::Response&res)
{
  // For safety we first set the workobject and tool.
  // The default values are specified in grasp_comm.h located in grasp_comm ROS package
  // and are shared between all grasp procedures 
  robot.SetWorkObject(GM_WORK_X, GM_WORK_Y, GM_WORK_Z, 
      GM_WORK_Q0, GM_WORK_QX, GM_WORK_QY, GM_WORK_QZ);
  robot.SetTool(GM_TOOL_X, GM_TOOL_Y, GM_TOOL_Z, 
      GM_TOOL_Q0, GM_TOOL_QX, GM_TOOL_QY, GM_TOOL_QZ);

  //NOTE!!
  //Here we should check that the parameters are within safe ranges
  //Set parameters for open loop grasp
  GM_InputParams graspParams;
  graspParams.x = req.x;
  graspParams.y = req.y;
  graspParams.z_lim = req.z_lim;
  graspParams.up = req.up;
  graspParams.down = req.down;
  graspParams.id = req.id;
  graspParams.handSpeed = req.handSpeed;
  graspParams.nFlips = req.nFlips;
  graspParams.oscillationAmplitude = req.oscillationAmplitude;

  //Initial state.
  int currState = GM_INIT;

  //State Machine.
  ros::Rate SMRate(GM_STATE_MACHINE_RATE);
  while(currState!=GM_END && currState!=GM_END_COL && currState!=GM_ERROR)
  {
    currState = stepStateMachine(currState, graspParams);
    ros::spinOnce();
    SMRate.sleep();
  }

  // Check for correct ending of the state machine.
  if(currState == GM_END)
  {
    res.error = GM_ERROR_NONE;
    res.logFileName = logFileName;
    res.ret = 1;
    res.msg = "GRASP_OPENLOOP: OK.";
    return true;
  }
  else if (currState == GM_END_COL)
  {
    res.error = GM_ERROR_COLLISION;
    res.logFileName = logFileName;
    res.ret = 0;
    res.msg = "GRASP_OPENLOOP: Collision detected, but OK.";
    return true;
  }

  res.error = GM_ERROR_UNKNOWN;
  res.ret = 0;
  res.msg = "GRASP_OPENLOOP: There was an error in the grasp_OpenLoop state machine.";
  ROS_ERROR("%s",res.msg.c_str());
  return false;
}

int stepStateMachine(int currState, GM_InputParams graspParams)
{
  int nextState = GM_ERROR;
  switch (currState)
  {
    case GM_INIT:
			  
				ROS_INFO("Initializing grasp...");

        //Set robot communication
        if (!robot.SetComm(1))
        {
          nextState = GM_ERROR;
          ROS_ERROR("GRASP_OPENLOOP: Could not set the Robot communication mode to blocking.");
          break;
        }
        // Set robot Zone
        if (!robot.SetZone(GM_ZONE))
        {
          nextState = GM_ERROR;
          ROS_ERROR("GRASP_OPENLOOP: Could not set the Robot zone.");
          break;
        }
        // Set robot Speed
        if (!robot.SetSpeed(GM_FLY_SPEED_TCP, GM_FLY_SPEED_ORI))
        {
          nextState = GM_ERROR;
          ROS_ERROR("GRASP_OPENLOOP: Could not set the Robot zone.");
          break;
        }

      nextState = GM_FIND_SURFACE;
			// Get robotic hand out of the way for vision.
			// First get the robot out of singularity.
      hand.SetAngle(40.0);
			robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
			robot.SetCartesian(10, 572.57, 658.88, 0.707, 0.707, 0.002, 0.001);

      break;

    case GM_INIT_GRASP:
      {
        // Set hand position (Make sure we always come from the same direction)
        hand.SetForce(HAND_OPEN_FORCE);
        hand.SetSpeed(HAND_OPEN_SPEED);
				ROS_INFO("Opening hand.");
        hand.SetAngle(80.0);

        // Command robot to go to initial grasp position
        if (!robot.SetCartesian(graspParams.x, graspParams.y, GM_HOME_Z, 
              GM_HOME_Q0, GM_HOME_QX, GM_HOME_QY, GM_HOME_QZ))
        {
          nextState = GM_ERROR;
          ROS_ERROR("GRASP_OPENLOOP: Could not command robot to initial grasp position.");
          break;
        } 

        // Wait for our initial hand move to complete.
        hand.WaitRest(0.0);

        nextState = GM_GO_HOME_GRASP;
        break;
      }

    case GM_FIND_SURFACE:
      {
  			// Use container_comm to find average height in an area.

				ROS_INFO("Finding surface height at <%f, %f>...",
				  graspParams.x, graspParams.y); 

				// 10 is 1 cm
				if (container.GetAverageHeight(graspParams.x, graspParams.y, 50.0, surface))
				{
				  ROS_INFO("Surface is %f.", surface);
          nextState = GM_INIT_GRASP;
				}
				else 
				{
				  ROS_INFO("No surface found. Trying again.");
					nextState = GM_FIND_SURFACE;
				}

        break;
      }

    case GM_GO_HOME_GRASP:
      {
        ROS_INFO("Going to home position after surface detection!");
        // Reset robot communication mode and speed
        robot.SetComm(1);
        robot.SetSpeed(GM_FLY_SPEED_TCP, GM_FLY_SPEED_ORI);

        // Meanwhile command to the robot to Home_Grasp position
        robot.SetCartesian(graspParams.x, graspParams.y, surface + graspParams.up,
            GM_HOME_Q0, GM_HOME_QX, GM_HOME_QY, GM_HOME_QZ);

        // Open the hand again
        hand.SetSpeed(HAND_OPEN_SPEED);
        hand.SetForce(HAND_OPEN_FORCE);
        hand.SetAngle(-10.0);
        hand.WaitRest(0.0);

        nextState = GM_GRASP;
        break;
      }

    case GM_GRASP:
      {
        ROS_INFO("Beginning grasp maneuver!");
        // Prepare the speed of the robot and hand for the grasp motion
        // Set blocking communication with the robot
        robot.SetComm(1);
        robot.SetSpeed(GM_GRASP_FAST_SPEED_TCP, GM_GRASP_FAST_SPEED_ORI);
        hand.SetSpeed(graspParams.handSpeed);
        hand.SetForce(HAND_CLOSE_FORCE);

        //////////////////////////////////////
        // Open loop grasp motion
        // The hand goes down while it oscillates around axis z folowing 
        // a linearly decrasing sinusoidal

        // 0-Start logging the signature of the grasp
        string aux;
        logger.StartGrasp(logFileName, graspParams.id);

        // 1-Close hand to grasp 
        hand.SetAngle(HAND_CLOSE_ANGLE);

        Vec auxVec(3);
        Quaternion auxQuat;
        robot.GetCartesian(auxVec[0], auxVec[1], auxVec[2],
            auxQuat[0], auxQuat[1], auxQuat[2], auxQuat[3]);
				// Move the robot downward, while closing the hand.
				bool ismoving = true;
				robot.SetCartesian(auxVec[0], auxVec[1], auxVec[2] - graspParams.down,
            auxQuat[0], auxQuat[1], auxQuat[2], auxQuat[3]);
				// Check if the hand is moving.
        hand.IsMoving(ismoving);
				while(ismoving)
				{
					// If it is, keep waiting until it's not anymore. Then move on.
					ros::Duration(0.01).sleep();
					hand.IsMoving(ismoving);
				}

/*
        // 2-Get the initial robot position
        HomogTransf iniGraspPose;
        Vec auxVec(3);
        Quaternion auxQuat;
        robot.GetCartesian(auxVec[0], auxVec[1], auxVec[2],
            auxQuat[0], auxQuat[1], auxQuat[2], auxQuat[3]);
        iniGraspPose = HomogTransf(auxQuat.getRotMat(),auxVec);

        // 3-Go down half the way following a staight line
        HomogTransf incGraspPose, currentGraspPose;
        RotMat incRot;
        Vec incTrans;
        incRot.rotZ(0.0);
        incTrans = Vec("0.0 0.0 1.0",3)*(graspParams.down*0.5); 
        incGraspPose = HomogTransf(incRot,incTrans);
        currentGraspPose = iniGraspPose*incGraspPose;

        auxVec = currentGraspPose.getTranslation();
        auxQuat = currentGraspPose.getRotation().getQuaternion();
        if(!robot.SetCartesian(auxVec[0], auxVec[1], auxVec[2],
              auxQuat[0], auxQuat[1], auxQuat[2], auxQuat[3]))
        {
          ROS_INFO("Got stuck on initial descent!");
          nextState = GM_COLLISION;
          break;
        }

        bool moving;
        bool collision = false;
        // 4-Sinusoidal oscillations while going down
        for(int i=1; i<=graspParams.nFlips ; i++)
        {
          double rotMag = (graspParams.oscillationAmplitude)/(double)i;
          // Oscillation of amplitude +rotMag
          incRot.rotZ(-rotMag);
          incTrans = Vec("0.0 0.0 1.0",3) * (graspParams.down * (0.5 + (double)i/(double)graspParams.nFlips)); 
          incGraspPose = HomogTransf(incRot,incTrans);
          currentGraspPose = iniGraspPose * incGraspPose;

          auxVec = currentGraspPose.getTranslation();
					ROS_INFO("Opening hand.");
          auxQuat = currentGraspPose.getRotation().getQuaternion();

          if(!robot.SetCartesian(auxVec[0], auxVec[1], auxVec[2],
                auxQuat[0], auxQuat[1], auxQuat[2], auxQuat[3]))
          {
            collision = true;
            break;
          }

          // If the hand is no longer moving, there's no need to dig deeper
          hand.IsMoving(moving);
          if (!moving)
          {
            ROS_INFO("STOPPING! %d", i);
            break;
          }

          // Oscillation of magnitude -rotMag
          incRot.rotZ(rotMag);
          incTrans = Vec("0.0 0.0 1.0",3) * (graspParams.down * (0.5 + (2*(double)i + 1.0)/(2.0*(double)graspParams.nFlips))); 
          incGraspPose = HomogTransf(incRot,incTrans);
          currentGraspPose = iniGraspPose * incGraspPose;

          auxVec = currentGraspPose.getTranslation();
          auxQuat = currentGraspPose.getRotation().getQuaternion();

          if(!robot.SetCartesian(auxVec[0], auxVec[1], auxVec[2],
                auxQuat[0], auxQuat[1], auxQuat[2], auxQuat[3]))
          {
            ROS_INFO("Got stuck while grasping!");
            collision = true;
            break;
          }

          // If the hand is no longer moving, there's no need to dig deeper
          hand.IsMoving(moving);
          if (!moving)
          {
            ROS_INFO("STOPPING!!! %d", i);
            break;
          }
        }

        // If there was a collision, immediately go to the collision state
        if (collision)
        {
          nextState = GM_COLLISION;
          break;
        }
				*/

        ROS_INFO("Going back to home position after grasp!");

        // 5-Go back up to home position and wait for hand to end up closing
        robot.SetCartesian(graspParams.x, graspParams.y, GM_HOME_Z,
            GM_HOME_Q0, GM_HOME_QX, GM_HOME_QY, GM_HOME_QZ);
        hand.WaitRest(0.0);
        //hand.SetRest();

        // 6-Stop logging the grasp signature
        logger.Stop();

        // End of open loop grasp motion
        //////////////////////////////////

        //Reset robot and hand speed
        robot.SetSpeed(GM_FLY_SPEED_TCP, GM_FLY_SPEED_ORI);
        hand.SetSpeed(0.5);
        hand.SetForce(0.5);
        //hand.SetRest();

        nextState = GM_END;
        break;
      }

    case GM_COLLISION:
      // If we encountered a collision while grasping. Turn off the 
      //  hand motor and return the arm back up to the home position.
      hand.SetRest();

      ROS_INFO("In grasp collision state!");

      // Make sure we also stop our log
      logger.Stop();

      logger.Append(logFileName, "GRASP_COLLISION_DETECTED");

      if (!robot.SetCartesian(graspParams.x, graspParams.y, GM_HOME_Z,
            GM_HOME_Q0, GM_HOME_QX, GM_HOME_QY, GM_HOME_QZ))
      {
        // If we can't get back to the home position, something really 
        //  bad happened, so go to the error state
        nextState = GM_ERROR;
        break;
      }

      nextState = GM_END_COL;
      break;

    default:
      ROS_WARN("GRASP_OPENLOOP: Error in grasp_openloop state machine. State number out of bounds...");
      nextState = GM_ERROR;
      break;
  }
  return nextState;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, GM_OPENLOOP_NAME);
  ros::NodeHandle node;
  nodePtr = &node;

  // Subscribe to all of the services we will be using
  ROS_INFO("GRASP_OPENLOOP: Subscribing to ROS services...");
  robot.subscribe(nodePtr);
  hand.subscribe(nodePtr);
  logger.subscribe(nodePtr);
	container.subscribe(nodePtr);

  //Wait until Robot and Hand are active
  ROS_INFO("GRASP_OPENLOOP: Waiting for Robot...");
  while(!robot.Ping()) ;
  ROS_INFO("GRASP_OPENLOOP: Waiting for Hand...");
  while(!hand.Ping()) ;
	ROS_INFO("GRASP_OPENLOOP: Waiting for Container...");
	while (!container.Ping()) ;

  //Advertising ROS services
  ROS_INFO("GRASP_OPENLOOP: Advertising ROS services...");
  handle_grasp_OpenLoop = node.advertiseService("/grasp_OpenLoop_Detect",grasp_OpenLoop);

  //Main ROS loop
  ROS_INFO("GRASP_OPENLOOP: Running node /grasp_openloop_detect...");
  ros::MultiThreadedSpinner spinner(2); // Use 4 threads
  spinner.spin(); // spin() will not return until the node has been shutdown
  ROS_INFO("GRASP_OPENLOOP: Shutting down node /grasp_openloop_detect...");

  ROS_INFO("GRASP_OPENLOOP: Shutting down services ...");
  handle_grasp_OpenLoop.shutdown();
  robot.shutdown();
  hand.shutdown();
	container.shutdown();
  logger.shutdown();
}
