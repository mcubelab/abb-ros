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

#include "drop_openloop.h"

bool drop_OpenLoop(place_comm::drop_OpenLoop::Request& req, place_comm::drop_OpenLoop::Response&res)
{
  // For safety we first set the workobject and tool.
  // The default values are specified in place_comm.h located in place_comm ROS package
  // and are shared between all grasp procedures 
  robot.SetWorkObject(PM_WORK_X, PM_WORK_Y, PM_WORK_Z, 
      PM_WORK_Q0, PM_WORK_QX, PM_WORK_QY, PM_WORK_QZ);
  robot.SetTool(PM_TOOL_X, PM_TOOL_Y, PM_TOOL_Z, 
      PM_TOOL_Q0, PM_TOOL_QX, PM_TOOL_QY, PM_TOOL_QZ);

  // TODO: Check if parameters are safe
  // Set parameters for open loop place
  PM_InputParams dropParams;
  dropParams.angle = req.angle;
  dropParams.h_dist = req.h_dist;

  // While the angle we receive is correct, we need to make sure we move the correct
  //  direction for horizontal distance. If we are in the second or third quadrant, 
  //  we need to move in the negative direction, otherwise positive. Also note that
  //  we only want to move 90 degrees at most to get the marker straight, so if we
  //  are in the second or third quadrant, we change the angle by 180 degrees
  if (dropParams.angle > PI/2)
  {
    dropParams.h_dist *= -1;
    dropParams.angle -= PI;
  }
  else if (dropParams.angle < -PI/2)
  {
    dropParams.h_dist *= -1;
    dropParams.angle += PI;
  }

  // Now, we make sure that we're far enough away from all fingers
  for(int i=0; i<3; i++)
  {
    double ta = tan(dropParams.angle);
    double ca = cos(dropParams.angle);
    double rr = dropParams.h_dist;
    double d = fabs(f_locs[i][0] + ta*f_locs[i][1] - rr / ca) / sqrt(1+ta*ta);
    ROS_INFO("d%d = %f", i, d);

    // If our marker is close to a finger
    if (d < MIN_FING_DIST)
    {
      // Now let's figure out if we're actually going to drop it over that finger
      double ang_d = fabs(f_locs[i][2] - dropParams.angle);

      ROS_INFO("dropAng = %f, fingAng = %f", dropParams.angle, f_locs[i][2]);
      
      // If so, let's drop it the other way
      if (ang_d < MIN_FING_ANG)
      {
        if (dropParams.angle > 0)
        {
          dropParams.angle -= PI;
          dropParams.h_dist *= -1;
        }
        else
        {
          dropParams.angle += PI;
          dropParams.h_dist *= -1;
        }
        break;
      }
    }
  }


  //Initial state.
  PM_State curState = PM_INIT;

  //State Machine.
  ros::Rate SMRate(PM_STATE_MACHINE_RATE);
  while(curState != PM_COMPLETE && curState != PM_ERROR)
  {
    curState = stepStateMachine(curState, dropParams);
    ros::spinOnce();
    SMRate.sleep();
  }

  // Check for correct ending of the state machine.
  if(curState == PM_ERROR)
  {
    res.ret = 0;
    res.logFileName = log_file;
    res.picFileName = picName;
    res.success = drop_success;
    res.msg = "DROP_OPENLOOP: There was an error in the drop_OpenLoop state machine.";
    ROS_ERROR("%s",res.msg.c_str());
    return false;
  }
  else if (drop_success)
  {
    res.logFileName = log_file;
    res.picFileName = picName;
    res.ret = 1;
    res.success = true;
    res.msg = "DROP_OPENLOOP: OK.";
    return true;
  }
  else
  {
    res.logFileName = log_file;
    res.picFileName = picName;
    res.ret = 1;
    res.success = false;
    res.msg = "DROP_OPENLOOP: OK.";
    return true;
  }
}

PM_State stepStateMachine(PM_State curState, PM_InputParams dropParams)
{
  PM_State nextState = PM_ERROR;
  switch (curState)
  {
    case PM_INIT:
      nextState = PM_GOTO_DROP;
      drop_success = false;
      break;

    case PM_GOTO_DROP:
      ////////////////////////////////////////////////////////////////////////
      // GO TO PLACE STATE:
      // Move close to the placing area. Note that we should be a bit above
      // it so that we can rotate without our hand or marker hitting the 
      // platform we want to place on
      ////////////////////////////////////////////////////////////////////////
      if (!robot.SetComm(BLOCKING))
        nextState = PM_ERROR;
      else if (!robot.SetZone(P_ZONE))
        nextState = PM_ERROR;
      else if (!robot.SetSpeed(P_TCP, P_ORI))
        nextState = PM_ERROR;
      else if (!robot.SetCartesian(DROPPING_X, DROPPING_Y, DROPPING_Z, 
                                DROPPING_Q0, DROPPING_QX, DROPPING_QY, DROPPING_QZ))
        nextState = PM_ERROR;
      else
        nextState = PM_ROTATE;
      break;

    case PM_ROTATE:
      {
      ////////////////////////////////////////////////////////////////////////
      // ROTATE STATE:
      // Rotate our marker so that it is vertical. In addition, move 
      // the arm over so the marker is centered on the placing platform
      ////////////////////////////////////////////////////////////////////////
        RotMat r;
        Quaternion orient;
        orient[0] = DROPPING_Q0;
        orient[1] = DROPPING_QX;
        orient[2] = DROPPING_QY;
        orient[3] = DROPPING_QZ;

        //Rotate to marker down based on its measured position
        r.rotY(dropParams.angle);
        goalQuat = r.getQuaternion() ^ orient;

        goalPos[0] = DROPPING_X - dropParams.h_dist;
        goalPos[1] = DROPPING_Y;
        goalPos[2] = DROPPING_Z;

        // Command the cartesian move
        if (robot.SetCartesian(goalPos[0], goalPos[1], goalPos[2],
              goalQuat[0], goalQuat[1], goalQuat[2], goalQuat[3]))
        {
          nextState = PM_DROP_MARKER;
        }
        else
        {
          nextState = PM_ERROR;
        }
        break;
      }

    case PM_DROP_MARKER:

      ////////////////////////////////////////////////////////////////////////
      // RELEASE STATE:
      // The marker is now in the right place. All we need to do is
      // release the marker from our grasp, and it should be balanced.
      ////////////////////////////////////////////////////////////////////////
      logger.StartPlace(log_file);

      hand.SetSpeed(OPEN_SPEED);
      hand.SetForce(OPEN_FORCE);
      hand.SetAngle(HAND_OPEN);

      hand.WaitRest(0.0);

      logger.Stop();

      nextState = PM_DROP_CHECK;
      break;

    case PM_DROP_CHECK:
      {
      ////////////////////////////////////////////////////////////////////////
      // DROP CHECK STATE
      // Checks to see if dropping was successful, and determines if it has to
      // knock down a marker or not. If so, then it moves to a positon right
      // above where it needs to be to knock it down
      ////////////////////////////////////////////////////////////////////////

        bool success;

        // Now, get in position to knock marker over. Note that we do this
        //  even if we were unsuccessful, on the off chance that our vision
        //  system failed, and there actually is a marker there
        hand.SetAngle(HAND_WIDE);

        if (robot.SetCartesian(RM_OUT_X, RM_OUT_Y, RM_OUT_Z, 
              goalQuat[0], goalQuat[1], goalQuat[2], goalQuat[3]))
          nextState = PM_RM_DOWN;
        else
          nextState = PM_ERROR;

        // Don't take a picture until we're sure everything has settled down
        //usleep(PM_SETTLE_TIME);

        // Now take a picture to detect wether we 
        //  successfully dropped a marker or not
        vision.CaptureImage(PLACE_DETECT, picName);
        if (!vision.DropDetect(picName, success))
        {
          nextState = PM_ERROR;
        }
        else
        {
          // Record whether or not placing was successful
          if (success)
          {
            ROS_INFO("Dropping Successful!");
            logger.Append(log_file, "Dropping Successful");
            drop_success = true;
          }
          else
          {
            ROS_INFO("Dropping Unsuccessful");
            logger.Append(log_file, "Dropping Unsuccessful");
            drop_success = false;
          }
        }
        break;
      }

    case PM_RM_DOWN:
      if (robot.SetJoints(RM_DOWN_J1, RM_DOWN_J2, RM_DOWN_J3, 
            RM_DOWN_J4, RM_DOWN_J5, RM_DOWN_J6))
        nextState = PM_RM_IN;
      else
        nextState = PM_COLLISION;

      // Make sure we wait here until our hand is completely open
      hand.WaitRest(0.0);

      break;
 
    case PM_RM_IN:
      if (robot.SetCartesian(RM_IN_X, RM_IN_Y, RM_IN_Z, 
            RM_IN_Q0, RM_IN_QX, RM_IN_QY, RM_IN_QZ))
        nextState = PM_RM_OUT2;
      else
        nextState = PM_COLLISION;
      break;

    case PM_RM_OUT2:
      if (!robot.SetCartesian(RM_IN_X, RM_OUT2_Y, RM_OUT2_Z, 
            RM_OUT2_Q0, RM_OUT2_QX, RM_OUT2_QY, RM_OUT2_QZ))
        nextState = PM_COLLISION;
      else if (robot.SetCartesian(RM_OUT2_X, RM_OUT2_Y, RM_OUT2_Z, 
            RM_OUT2_Q0, RM_OUT2_QX, RM_OUT2_QY, RM_OUT2_QZ))
        nextState = PM_RM_IN2;
      else
        nextState = PM_COLLISION;
      break;

    case PM_RM_IN2:
      if (robot.SetCartesian(RM_IN2_X, RM_IN2_Y, RM_IN2_Z, 
            RM_IN2_Q0, RM_IN2_QX, RM_IN2_QY, RM_IN2_QZ))
        nextState = PM_RM_OVER;
      else
        nextState = PM_COLLISION;
      break;


    case PM_RM_OVER:
      if (robot.SetCartesian(RM_OVER_X, RM_OVER_Y, RM_OVER_Z, 
            RM_OVER_Q0, RM_OVER_QX, RM_OVER_QY, RM_OVER_QZ))
        nextState = PM_RM_ROTATE;
      else
        nextState = PM_COLLISION;
      break;

    case PM_RM_ROTATE:
      if (robot.SetCartesian(RM_ROTATE_X, RM_ROTATE_Y, RM_ROTATE_Z, 
            RM_ROTATE_Q0, RM_ROTATE_QX, RM_ROTATE_QY, RM_ROTATE_QZ))
        nextState = PM_RM_UP;
      else
        nextState = PM_COLLISION;

      // Now grasp the marker
      hand.SetSpeed(CLOSE_SPEED);
      hand.SetAngle(HAND_CLOSE);
      hand.WaitRest(0.0);

      break;      

    case PM_RM_UP:
      if (robot.SetCartesian(RM_UP_X, RM_UP_Y, RM_UP_Z, 
            RM_UP_Q0, RM_UP_QX, RM_UP_QY, RM_UP_QZ))
        nextState = PM_COMPLETE;
      else
        nextState = PM_ERROR;
      break;


    case PM_COMPLETE:
      nextState = PM_COMPLETE;
      break;

    case PM_COLLISION:
      // If we have collided, let's back up and try again.
      
      ROS_WARN("Collision in drop routine detected. Attempting to recover.");

      hand.SetAngle(HAND_WIDE);

      if (robot.SetCartesian(RM_OUT_X, RM_OUT_Y, RM_OUT_Z, 
            RM_OUT_Q0, RM_OUT_QX, RM_OUT_QY, RM_OUT_QZ))
        nextState = PM_RM_DOWN;
      else
        nextState = PM_ERROR;

      hand.WaitRest(0.0);

      break;

    case PM_ERROR:
    case PM_NUM_STATES:
      ROS_WARN("Error in Drop Marker...");
      nextState = PM_ERROR;
      break;
  }
  return nextState;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, PM_DROP_OPENLOOP_NAME);
  ros::NodeHandle node;
  nodePtr = &node;

  // Subscribe to all of the services we will be using
  ROS_INFO("DROP_OPENLOOP: Subscribing to ROS services...");
  robot.subscribe(nodePtr);
  hand.subscribe(nodePtr);
  vision.subscribe(nodePtr);
  logger.subscribe(nodePtr);

  //Wait until Robot and Hand are active
  ROS_INFO("DROP_OPENLOOP: Waiting for Robot...");
  while(!robot.Ping()) ;
  ROS_INFO("DROP_OPENLOOP: Waiting for Hand...");
  while(!hand.Ping()) ;
  ROS_INFO("DROP_OPENLOOP: Waiting for Vision...");
  while(!vision.Ping()) ;

  //Advertising ROS services
  ROS_INFO("DROP_OPENLOOP: Advertising ROS services...");
  handle_drop_OpenLoop = node.advertiseService("drop_OpenLoop",drop_OpenLoop);

  //Main ROS loop
  ROS_INFO("DROP_OPENLOOP: Running node /drop_openloop...");
  ros::spin();
  ROS_INFO("DROP_OPENLOOP: Shutting down node /drop_openloop...");

  ROS_INFO("DROP_OPENLOOP: Shutting down services ...");
  handle_drop_OpenLoop.shutdown();
  robot.shutdown();
  hand.shutdown();
  logger.shutdown();
}
