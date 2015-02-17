// Type of grasp: GM_ABORT
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

#include "grasp_abort.h"

bool grasp_Abort(grasp_comm::grasp_Abort::Request& req, grasp_comm::grasp_Abort::Response&res)
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
  graspParams.nFlips = req.nFlips;
  graspParams.oscillationAmplitude = req.oscillationAmplitude;
  graspParams.id = req.id;

  //Initial state.
  int currState = GM_INIT;

  //State Machine.
  ros::Rate SMRate(GM_STATE_MACHINE_RATE);
  while(currState!=GM_END && currState!=GM_ERROR)
    {
      currState = stepStateMachine(currState, graspParams);
      ros::spinOnce();
      SMRate.sleep();
    }

  // Check for correct ending of the state machine.
  if(currState == GM_END)
    {
      res.logFileName = logFileName;
      res.ret = 1;
      res.msg = "GRASP_ABORT: OK.";
      return true;
    }
  res.ret = 0;
  res.msg = "GRASP_ABORT: There was an error in the grasp_Abort state machine.";
  ROS_ERROR("%s",res.msg.c_str());
  return false;
}

int stepStateMachine(int currState, GM_InputParams graspParams)
{
  int nextState;
  switch (currState)
    {
      case GM_INIT:
        nextState = GM_GO_HOME;
        break;

      case GM_GO_HOME:
        {
          //Set robot communication
          if (!robot.SetComm(1))
            {
              nextState = GM_ERROR;
              ROS_ERROR("GRASP_ABORT: Could not set the Robot communication mode to blocking.");
	      break;
            }
          // Set robot Zone
          if (!robot.SetZone(GM_ZONE))
            {
              nextState = GM_ERROR;
              ROS_ERROR("GRASP_ABORT: Could not set the Robot zone.");
	      break;
            }
          // Set robot Speed
          if (!robot.SetSpeed(GM_FLY_SPEED_TCP, GM_FLY_SPEED_ORI))
            {
              nextState = GM_ERROR;
              ROS_ERROR("GRASP_ABORT: Could not set the Robot zone.");
	      break;
            }
          // Joint move to home position
          if (!robot.SetJoints(GM_HOME_J1, GM_HOME_J2, GM_HOME_J3,
          	               GM_HOME_J4, GM_HOME_J5, GM_HOME_J6))
            {
	      nextState = GM_ERROR;
	      ROS_ERROR("GRASP_ABORT: Could not command robot to home position.");
              break;
            }
          nextState = GM_INIT_GRASP;
          break;
        }
      case GM_INIT_GRASP:
        {        
          // Command robot to go to initial grasp position
          if (!robot.SetCartesian(graspParams.x, graspParams.y, GM_HOME_Z, 
                                  GM_HOME_Q0, GM_HOME_QX, GM_HOME_QY, GM_HOME_QZ))
            {
              nextState = GM_ERROR;
	      ROS_ERROR("GRASP_ABORT: Could not command robot to initial grasp position.");
              break;
            } 
          // Set hand position
          hand.SetSpeed(0.75);
          hand.SetAngle(35.0);

          // Fast approach to the bin.
          robot.SetCartesian(graspParams.x, graspParams.y, graspParams.z_lim/2.0,
                             GM_HOME_Q0, GM_HOME_QX, GM_HOME_QY, GM_HOME_QZ);

          // We wait until the hand is prepared for surface detection.
          hand.WaitRest(0.0);
	  nextState = GM_GO_SURFACE;
	  break;
       }
     case GM_GO_SURFACE:
       {
          // Set non blocking communication with the robot
          robot.SetComm(0);

          // Set the tracking distance.
          // We control the stop distance when finding the surface.
          robot.SetTrackDist(GM_GRASP_TRACK_DIST_TCP, GM_GRASP_TRACK_DIST_ORI);
        
          //Get initial finger angles
          double mot_ang;
          iniFingerAngles = Vec(4);
          hand.GetAngles(mot_ang, iniFingerAngles);

          //Command to the Maximum Low Position to find surface
          robot.SetCartesian(graspParams.x, graspParams.y, graspParams.z_lim,
                             GM_HOME_Q0, GM_HOME_QX, GM_HOME_QY, GM_HOME_QZ);

          nextState = GM_FIND_SURFACE;
          break;
      }	

    case GM_FIND_SURFACE:
      {
        // Check finger encoders
        double mot_ang;
        Vec fingerAngles(4);
        hand.GetAngles(mot_ang, fingerAngles);
        double deflection = (fingerAngles - iniFingerAngles).abs().max();
 
	///////////////
        // For debugging purposes
	// double auxX,auxY,auxZ,auxQ0,auxQx,auxQy,auxQz;
	// robot.GetCartesian(auxX,auxY,auxZ,auxQ0,auxQx,auxQy,auxQz);
        // ROS_INFO("GRASP_ABORT: (debug) Robot at z %lf",auxZ);
        ///////////////

        // Stop if deflection is large enough
        if(deflection > MAX_FINGER_DEFLECTION) 
          { 	
            // Save value of z as surface
 	    double auxX,auxY,auxQ0,auxQx,auxQy,auxQz;
	    robot.GetCartesian(auxX,auxY,surface,auxQ0,auxQx,auxQy,auxQz);
            robot.Stop();
            nextState = GM_GO_HOME_GRASP;
            break;
          }
	nextState = GM_FIND_SURFACE;
        break;
      }

    case GM_GO_HOME_GRASP:
      {
        // Reset robot communication mode and speed
        robot.SetComm(1);
        robot.SetSpeed(GM_FLY_SPEED_TCP, GM_FLY_SPEED_ORI);
  	
        // Meanwhile command to the robot to Home_Grasp position
        robot.SetCartesian(graspParams.x, graspParams.y, surface - graspParams.up,
                           GM_HOME_Q0, GM_HOME_QX, GM_HOME_QY, GM_HOME_QZ);
      
        // Open the hand again
        hand.SetSpeed(0.75);
        hand.SetAngle(-10.0);
        hand.WaitRest(0.0);

        nextState = GM_GRASP;
        break;
      }

    case GM_GRASP:
      {
        // Prepare the speed of the robot and hand for the grasp motion
        // Set blocking communication with the robot
        robot.SetComm(1);
        robot.SetSpeed(GM_GRASP_FAST_SPEED_TCP, GM_GRASP_FAST_SPEED_ORI);
        hand.SetSpeed(0.45);
        hand.SetForce(0.10);

        //////////////////////////////////////
        // Open loop grasp motion
        // The hand goes down while it oscillates around axis z folowing 
        // a linearly decrasing sinusoidal
        
        // 0-Start logging the signature of the grasp
        string aux;
        logger.StartGrasp(logFileName, graspParams.id);

	// 1-Close hand to grasp 
        hand.SetAngle(90.0);

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
        currentGraspPose = incGraspPose*iniGraspPose;

        auxVec = currentGraspPose.getTranslation();
        auxQuat = currentGraspPose.getRotation().getQuaternion();
        robot.SetCartesian(auxVec[0], auxVec[1], auxVec[2],
                           auxQuat[0], auxQuat[1], auxQuat[2], auxQuat[3]);

	// 4-Sinusoidal oscillations while going down
        for(int i=1; i<=graspParams.nFlips ; i++)
        {
          double rotMag = -(graspParams.oscillationAmplitude)/(double)i;
          // Oscillation of amplitude +rotMag
          incRot.rotZ(rotMag);
          incTrans = Vec("0.0 0.0 1.0",3) * (graspParams.down * (0.5 + (double)i/(double)graspParams.nFlips)); 
          incGraspPose = HomogTransf(incRot,incTrans);
          currentGraspPose = iniGraspPose * incGraspPose;

          auxVec = currentGraspPose.getTranslation();
          auxQuat = currentGraspPose.getRotation().getQuaternion();

          robot.SetCartesian(auxVec[0], auxVec[1], auxVec[2],
                             auxQuat[0], auxQuat[1], auxQuat[2], auxQuat[3]);

          // Oscillation of magnitude -rotMag
          incRot.rotZ(-rotMag);
          incTrans = Vec("0.0 0.0 1.0",3) * (graspParams.down * (0.5 + (2*(double)i + 1.0)/(2.0*(double)graspParams.nFlips))); 
          incGraspPose = HomogTransf(incRot,incTrans);
          currentGraspPose = iniGraspPose * incGraspPose;

          auxVec = currentGraspPose.getTranslation();
          auxQuat = currentGraspPose.getRotation().getQuaternion();

          robot.SetCartesian(auxVec[0], auxVec[1], auxVec[2],
                             auxQuat[0], auxQuat[1], auxQuat[2], auxQuat[3]);
        }

        // 5-Go back up to home position and wait for hand to end up closing
        robot.SetCartesian(graspParams.x, graspParams.y, GM_HOME_Z,
                           GM_HOME_Q0, GM_HOME_QX, GM_HOME_QY, GM_HOME_QZ);
        hand.WaitRest(0.0);
	hand.SetRest();

        // 6-Stop logging the grasp signature
        logger.Stop();
         
        // End of open loop grasp motion
        //////////////////////////////////
        
        //Reset robot and hand speed
        robot.SetSpeed(GM_FLY_SPEED_TCP, GM_FLY_SPEED_ORI);
        hand.SetSpeed(0.5);
        hand.SetForce(0.6);
	hand.SetRest();
        
        nextState = GM_END;
        break;
      }
      
    default:
      ROS_WARN("GRASP_ABORT: Error in grasp_abort state machine. State number out of bounds...");
      nextState = GM_ERROR;
      break;
  }
  return nextState;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp_abort");
  ros::NodeHandle node;
  nodePtr = &node;

  // Subscribe to all of the services we will be using
  ROS_INFO("GRASP_ABORT: Subscribing to ROS services...");
  robot.subscribe(nodePtr);
  hand.subscribe(nodePtr);
  logger.subscribe(nodePtr);

  //Wait until Robot and Hand are active
  ROS_INFO("GRASP_ABORT: Waiting for Robot...");
  while(!robot.Ping()) ;
  ROS_INFO("GRASP_ABORT: Waiting for Hand...");
  while(!hand.Ping()) ;

  //Advertising ROS services
  ROS_INFO("GRASP_ABORT: Advertising ROS services...");
  handle_grasp_Abort = node.advertiseService("grasp_Abort",grasp_Abort);
  
  //Main ROS loop
  ROS_INFO("GRASP_ABORT: Running node /grasp_abort...");
  ros::spin();
  ROS_INFO("GRASP_ABORT: Shutting down node /grasp_abort...");
  
  ROS_INFO("GRASP_ABORT: Shutting down services ...");
  handle_grasp_Abort.shutdown();
  robot.shutdown();
  hand.shutdown();
  logger.shutdown();
}
