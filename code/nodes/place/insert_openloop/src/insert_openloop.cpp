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

#include "insert_openloop.h"

bool insert_OpenLoop(place_comm::insert_OpenLoop::Request& req, place_comm::insert_OpenLoop::Response&res)
{
  // For safety we first set the workobject and tool.
  // The default values are specified in place_comm.h located in place_comm ROS package
  // and are shared between all grasp procedures 
  robot.SetWorkObject(PM_WORK_X, PM_WORK_Y, PM_WORK_Z, 
      PM_WORK_Q0, PM_WORK_QX, PM_WORK_QY, PM_WORK_QZ);
  robot.SetTool(PM_TOOL_X, PM_TOOL_Y, PM_TOOL_Z, 
      PM_TOOL_Q0, PM_TOOL_QX, PM_TOOL_QY, PM_TOOL_QZ);

  // TODO: Check if parameters are safe
  // Set parameters for open loop insert
  PM_InputParams insertParams;
  insertParams.angle = req.angle;
  insertParams.h_dist = req.h_dist;

  // While the angle we receive is correct, we need to make sure we move the correct
  //  direction for horizontal distance. If we are in the second or third quadrant, 
  //  we need to move in the negative direction, otherwise positive. Also note that
  //  we only want to move 90 degrees at most to get the marker straight, so if we
  //  are in the second or third quadrant, we change the angle by 180 degrees
  if (insertParams.angle > PI/2)
  {
    insertParams.h_dist *= -1;
    insertParams.angle -= PI;
  }
  else if (insertParams.angle < -PI/2)
  {
    insertParams.h_dist *= -1;
    insertParams.angle += PI;
  }

  /*

  // Now, we make sure that we're far enough away from all fingers
  for(int i=0; i<3; i++)
  {
    double ta = tan(insertParams.angle);
    double ca = cos(insertParams.angle);
    double rr = insertParams.h_dist;
    double d = fabs(f_locs[i][0] + ta*f_locs[i][1] - rr / ca) / sqrt(1+ta*ta);
    ROS_INFO("d%d = %f", i, d);

    // If our marker is close to a finger
    if (d < MIN_FING_DIST)
    {
      // Now let's figure out if we're actually going to insert it near that finger
      double ang_d = min(fabs(f_locs[i][2] - insertParams.angle), fabs(f_locs[i][3] - insertParams.angle));

      ROS_INFO("dropAng = %f, fingAng1 = %f, fingAng2 = %f", insertParams.angle, f_locs[i][2], f_locs[i][3]);
      
      // If so, we will not insert this marker, but just return failure
      if (ang_d < MIN_FING_ANG)
      {
        res.ret = 1;
        res.success = false;
        res.abort = true;
        res.msg = "INSERT_OPENLOOP: Marker in unsafe location to insert";
        ROS_WARN("%s",res.msg.c_str());
        return true;
      }
    }
  }

  */


  res.abort = false;

  //Initial state.
  PM_State curState = PM_INIT;

  //State Machine.
  ros::Rate SMRate(PM_STATE_MACHINE_RATE);
  while(curState != PM_COMPLETE && curState != PM_ERROR)
  {
    curState = stepStateMachine(curState, insertParams);
    ros::spinOnce();
    SMRate.sleep();
  }

  // Check for correct ending of the state machine.
  if(curState == PM_ERROR)
  {
    res.ret = 0;
    res.logFileName = log_file;
    res.picFileName = picName;
    res.success = insert_success;
    res.msg = "INSERT_OPENLOOP: There was an error in the insert_OpenLoop state machine.";
    ROS_ERROR("%s",res.msg.c_str());
    return false;
  }
  else if (insert_success)
  {
    res.logFileName = log_file;
    res.picFileName = picName;
    res.ret = 1;
    res.success = true;
    res.msg = "INSERT_OPENLOOP: OK.";
    return true;
  }
  else
  {
    res.logFileName = log_file;
    res.picFileName = picName;
    res.ret = 1;
    res.success = false;
    res.msg = "INSERT_OPENLOOP: OK.";
    return true;
  }
}

PM_State stepStateMachine(PM_State curState, PM_InputParams insertParams)
{
  PM_State nextState = PM_ERROR;
  switch (curState)
  {
    case PM_INIT:
      nextState = PM_GOTO_PLACE;
      insert_success = false;
      break;

    case PM_GOTO_PLACE:
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
      else if (!robot.SetCartesian(INITIAL_X, INITIAL_Y, INITIAL_Z,
                                INITIAL_Q0, INITIAL_QX, INITIAL_QY, INITIAL_QZ))
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
        orient[0] = INITIAL_Q0;
        orient[1] = INITIAL_QX;
        orient[2] = INITIAL_QY;
        orient[3] = INITIAL_QZ;

        //Rotate to marker down based on its measured position
        r.rotY(insertParams.angle);
        goalQuat = r.getQuaternion() ^ orient;

        goalPos[0] = INITIAL_X - insertParams.h_dist;
        goalPos[1] = INITIAL_Y;
        goalPos[2] = INITIAL_Z;

        // Command the cartesian move
        if (robot.SetCartesian(goalPos[0], goalPos[1], goalPos[2],
              goalQuat[0], goalQuat[1], goalQuat[2], goalQuat[3]))
          nextState = PM_PRESS_DOWN;
        else
        {
          nextState = PM_ERROR;
        }
        break;
      }

    case PM_PRESS_DOWN:
      ////////////////////////////////////////////////////////////////////////
      // PRESS DOWN STATE:
      // Now that the marker is vertical, slowly press down to guarantee
      // that the marker is above the rim of the hand on the other side. 
      // Also, this is the beginning of our placing operation, so start log
      ////////////////////////////////////////////////////////////////////////
      robot.SetSpeed(SLOW_TCP, SLOW_ORI);

      goalPos[2] = PRESS_DOWN_Z;

      logger.StartPlace(log_file);

      if (robot.SetCartesian(goalPos[0], goalPos[1], goalPos[2],
            goalQuat[0], goalQuat[1], goalQuat[2], goalQuat[3]))
      {
        nextState = PM_MOVE_OUT;
      }
      else
      {
        nextState = PM_COLLISION;
      }
      break;

    case PM_MOVE_OUT:
      ////////////////////////////////////////////////////////////////////////
      // MOVE UP STATE:
      // Now move up enough so that we can flip 180 degrees
      ////////////////////////////////////////////////////////////////////////
      robot.SetSpeed(P_TCP, P_ORI);

      goalPos[2] = MOVE_OUT_Z;
      
      if (robot.SetCartesian(goalPos[0], goalPos[1], goalPos[2],
            goalQuat[0], goalQuat[1], goalQuat[2], goalQuat[3]))
      {
        nextState = PM_PREP_INSERT;
      }
      else
      {
        nextState = PM_ERROR;
      }
      break;

    case PM_PREP_INSERT:
      {
      ////////////////////////////////////////////////////////////////////////
      // FLIP STATE:
      // Flip 180 degrees so that the end of the marker that is sticking out
      // is now facing straight down. Also, move the marker over 2 times its
      // distance from center so we can place it in the center of the 
      // placing platform. Make sure we take 2 rotation steps to ensure that 
      // we turn in the correct direction
      ////////////////////////////////////////////////////////////////////////
        RotMat r;
        r.rotY(-PI/2.0);

        goalQuat = r.getQuaternion() ^ goalQuat;

        goalPos[0] = MOVE_OUT_X;
        goalPos[1] = MOVE_OUT_Y;
        goalPos[2] = MOVE_OUT_Z - insertParams.h_dist;

        if (robot.SetCartesian(goalPos[0], goalPos[1], goalPos[2],
              goalQuat[0], goalQuat[1], goalQuat[2], goalQuat[3]))
        {
          nextState = PM_INSERT;
        }
        else
        {
          nextState = PM_ERROR;
        }
        break;
      }

    case PM_INSERT:
      ////////////////////////////////////////////////////////////////////////
      // FINAL DOWN STATE:
      // Now we are ready to place the marker. Move all the way down, and 
      // expect the marker to slide up a bit in our hand as we do this
      ////////////////////////////////////////////////////////////////////////
      robot.SetSpeed(SLOW_TCP, SLOW_ORI);

      goalPos[0] = INSERT_X;
      goalPos[1] = INSERT_Y;

      if (robot.SetCartesian(goalPos[0], goalPos[1], goalPos[2],
            goalQuat[0], goalQuat[1], goalQuat[2], goalQuat[3]))
      {
        nextState = PM_CHECK;
      }
      else
      {
        nextState = PM_COLLISION;
      }
      logger.Stop();
      break;

    case PM_CHECK:
      {
        bool success;

        // Don't take a picture until we're sure everything has settled down
        usleep(PM_SETTLE_TIME);

        vision.CaptureImage(PLACE_DETECT, picName);
        if (!vision.InsertDetect(picName, success))
        {
          nextState = PM_ERROR;
        }
        else
        {
          // Record whether or not insertion was successful
          if (success)
          {
            ROS_INFO("Insertion Successful!");
            logger.Append(log_file, "Insertion Successful");
            insert_success = true;
          }
          else
          {
            ROS_INFO("Insertion Unsuccessful");
            logger.Append(log_file, "Insertion Unsuccessful");
            insert_success = false;
          }

          robot.SetSpeed(P_TCP, P_ORI);

          goalPos[0] = MOVE_OUT_X;
          goalPos[1] = MOVE_OUT_Y;

          // Now, retract away hand from insertion ring
          if (robot.SetCartesian(goalPos[0], goalPos[1], goalPos[2],
            goalQuat[0], goalQuat[1], goalQuat[2], goalQuat[3]))
            nextState = PM_GO_SAFE;
          else
            nextState = PM_ERROR;
        }
        break;
      }

    case PM_GO_SAFE:
      if (robot.SetJoints(SAFE_J1, SAFE_J2, SAFE_J3, 
            SAFE_J4, SAFE_J5, SAFE_J6))
          nextState = PM_COMPLETE;
      else
        nextState = PM_ERROR;
      break;

    case PM_COMPLETE:
      nextState = PM_COMPLETE;
      break;

    case PM_COLLISION:
      ROS_WARN("In Insertion collision state!");
      logger.Stop();

      insert_success = false;

      if (robot.SetCartesian(MOVE_OUT_X, MOVE_OUT_Y, MOVE_OUT_Z, 
            goalQuat[0], goalQuat[1], goalQuat[2], goalQuat[3]))
      {
        if (robot.SetJoints(SAFE_J1, SAFE_J2, SAFE_J3, 
              SAFE_J4, SAFE_J5, SAFE_J6))
        {
          nextState = PM_COMPLETE;
        }
        else
        {
          nextState = PM_ERROR;
        }
      }
      else
      {
        nextState = PM_ERROR;
      }
      break;

    case PM_ERROR:
    case PM_NUM_STATES:
      ROS_WARN("Error in Insert Marker...");
      nextState = PM_ERROR;
      break;
  }
  return nextState;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, PM_INSERT_OPENLOOP_NAME);
  ros::NodeHandle node;
  nodePtr = &node;

  // Subscribe to all of the services we will be using
  ROS_INFO("INSERT_OPENLOOP: Subscribing to ROS services...");
  robot.subscribe(nodePtr);
  hand.subscribe(nodePtr);
  vision.subscribe(nodePtr);
  logger.subscribe(nodePtr);

  //Wait until Robot and Hand are active
  ROS_INFO("INSERT_OPENLOOP: Waiting for Robot...");
  while(!robot.Ping()) ;
  ROS_INFO("INSERT_OPENLOOP: Waiting for Hand...");
  while(!hand.Ping()) ;
  ROS_INFO("INSERT_OPENLOOP: Waiting for Vision...");
  while(!vision.Ping()) ;

  //Advertising ROS services
  ROS_INFO("INSERT_OPENLOOP: Advertising ROS services...");
  handle_insert_OpenLoop = node.advertiseService("insert_OpenLoop",insert_OpenLoop);

  //Main ROS loop
  ROS_INFO("INSERT_OPENLOOP: Running node /insert_openloop...");
  ros::spin();
  ROS_INFO("INSERT_OPENLOOP: Shutting down node /insert_openloop...");

  ROS_INFO("INSERT_OPENLOOP: Shutting down services ...");
  handle_insert_OpenLoop.shutdown();
  robot.shutdown();
  hand.shutdown();
  logger.shutdown();
}
