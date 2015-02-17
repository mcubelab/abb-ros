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

#include "place_openloop.h"

bool place_OpenLoop(place_comm::place_OpenLoop::Request& req, place_comm::place_OpenLoop::Response&res)
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
  PM_InputParams placeParams;
  placeParams.angle = req.angle;
  placeParams.h_dist = req.h_dist;
  placeParams.v1_dist = req.v1_dist;
  placeParams.v2_dist = req.v2_dist;
  placeParams.test = req.test;

  if (placeParams.test)
    ROS_WARN("PLACE: TEST");
  else
    ROS_WARN("PLACE:TRAIN");

  // While the angle we receive is correct, we need to make sure we move the correct
  //  direction for horizontal distance. If we are in the second or third quadrant, 
  //  we need to move in the negative direction, otherwise positive. Also note that
  //  we only want to move 90 degrees at most to get the marker straight, so if we
  //  are in the second or third quadrant, we change the angle by 180 degrees
  if (placeParams.angle > PI/2)
  {
    placeParams.h_dist *= -1;
    placeParams.angle -= PI;
  }
  else if (placeParams.angle < -PI/2)
  {
    placeParams.h_dist *= -1;
    placeParams.angle += PI;
  }

  //Initial state.
  PM_State curState = PM_INIT;

  //State Machine.
  ros::Rate SMRate(PM_STATE_MACHINE_RATE);
  while(curState != PM_COMPLETE && curState != PM_ERROR)
  {
    curState = stepStateMachine(curState, placeParams);
    ros::spinOnce();
    SMRate.sleep();
  }

  // Check for correct ending of the state machine.
  if(curState == PM_ERROR)
  {
    res.ret = 0;
    res.success = false;
    res.msg = "PLACE_OPENLOOP: There was an error in the place_OpenLoop state machine.";
    ROS_ERROR("%s",res.msg.c_str());
    return false;
  }
  else if (placeParams.test)
  {
    res.logFileName = log_file;
    res.picFileName = "no_picture";
    res.ret = 1;
    res.success = true;
    res.msg = "PLACE_OPENLOOP: OK.";
    return true;
  }
  else if (place_success)
  {
    res.logFileName = log_file;
    res.picFileName = picName;
    res.ret = 1;
    res.success = true;
    res.msg = "PLACE_OPENLOOP: OK.";
    return true;
  }
  else
  {
    res.logFileName = log_file;
    res.picFileName = picName;
    res.ret = 1;
    res.success = false;
    res.msg = "PLACE_OPENLOOP: OK.";
    return true;
  }
}

PM_State stepStateMachine(PM_State curState, PM_InputParams placeParams)
{
  PM_State nextState = PM_ERROR;
  switch (curState)
  {
    case PM_INIT:
      nextState = PM_GOTO_PLACE;
      place_success = false;
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
      else if (placeParams.test)
      {
        if(!robot.SetCartesian(PLACING_TEST_X, PLACING_TEST_Y, PLACING_TEST_Z, 
                                PLACING_Q0, PLACING_QX, PLACING_QY, PLACING_QZ))
          nextState = PM_ERROR;
        else
          nextState = PM_ROTATE;
      }
      else
      {
        if(!robot.SetCartesian(PLACING_TRAIN_X, PLACING_TRAIN_Y, PLACING_TRAIN_Z, 
              PLACING_Q0, PLACING_QX, PLACING_QY, PLACING_QZ))
          nextState = PM_ERROR;
        else
          nextState = PM_ROTATE;
      }
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
        orient[0] = PLACING_Q0;
        orient[1] = PLACING_QX;
        orient[2] = PLACING_QY;
        orient[3] = PLACING_QZ;

        //Rotate to marker down based on its measured position
        r.rotY(placeParams.angle);
        goalQuat = r.getQuaternion() ^ orient;

        if (placeParams.test)
        {
          goalPos[0] = PLACING_TEST_X - placeParams.h_dist;
          goalPos[1] = PLACING_TEST_Y;
          goalPos[2] = PLACING_TEST_Z;
        }
        else
        {
          goalPos[0] = PLACING_TRAIN_X - placeParams.h_dist;
          goalPos[1] = PLACING_TRAIN_Y;
          goalPos[2] = PLACING_TRAIN_Z;
        }

        // Command the cartesian move
        if (robot.SetCartesian(goalPos[0], goalPos[1], goalPos[2],
              goalQuat[0], goalQuat[1], goalQuat[2], goalQuat[3]))
        {
          nextState = PM_PRESS_DOWN;
        }
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

      goalPos[2] -= placeParams.v1_dist;

      logger.StartPlace(log_file);

      if (robot.SetCartesian(goalPos[0], goalPos[1], goalPos[2],
            goalQuat[0], goalQuat[1], goalQuat[2], goalQuat[3]))
      {
        nextState = PM_MOVE_UP;
      }
      else
      {
        nextState = PM_COLLISION;
      }
      break;

    case PM_MOVE_UP:
      ////////////////////////////////////////////////////////////////////////
      // MOVE UP STATE:
      // Now move up enough so that we can flip 180 degrees
      ////////////////////////////////////////////////////////////////////////
      robot.SetSpeed(P_TCP, P_ORI);
      goalPos[2] += placeParams.v1_dist;
      
      if (robot.SetCartesian(goalPos[0], goalPos[1], goalPos[2],
            goalQuat[0], goalQuat[1], goalQuat[2], goalQuat[3]))
      {
        nextState = PM_FLIP;
      }
      else
      {
        nextState = PM_ERROR;
      }
      break;

    case PM_FLIP:
      {
      ////////////////////////////////////////////////////////////////////////
      // FLIP STATE:
      // Flip 180 degrees so that the end of the marker that is sticking out
      // is now facing straight down. Also, move the marker over 2 times its
      // distance from center so we can place it in the center of the 
      // placing platform. Make sure we take 2 rotation steps to ensure that 
      // we turn in the correct direction
      ////////////////////////////////////////////////////////////////////////
        Quaternion flip_quat;

        goalPos[0] += 2 * placeParams.h_dist;

        // Assign our rotation step
        flip_quat[0] = FS_Q0;
        flip_quat[1] = FS_QX;
        flip_quat[2] = FS_QY;
        flip_quat[3] = FS_QZ;

        // Rotate by a specific step amount
        goalQuat = flip_quat ^ goalQuat;

        if (robot.SetCartesian(goalPos[0], goalPos[1], goalPos[2],
              goalQuat[0], goalQuat[1], goalQuat[2], goalQuat[3]))
        {
          goalQuat = flip_quat ^ goalQuat;
          if (robot.SetCartesian(goalPos[0], goalPos[1], goalPos[2],
              goalQuat[0], goalQuat[1], goalQuat[2], goalQuat[3]))
          {
            nextState = PM_FINAL_DOWN;
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
      }

    case PM_FINAL_DOWN:
      ////////////////////////////////////////////////////////////////////////
      // FINAL DOWN STATE:
      // Now we are ready to place the marker. Move all the way down, and 
      // expect the marker to slide up a bit in our hand as we do this
      ////////////////////////////////////////////////////////////////////////
      robot.SetSpeed(SLOW_TCP, SLOW_ORI);

      goalPos[2] -= placeParams.v2_dist;

      if (robot.SetCartesian(goalPos[0], goalPos[1], goalPos[2],
            goalQuat[0], goalQuat[1], goalQuat[2], goalQuat[3]))
      {
        nextState = PM_RELEASE;
      }
      else
      {
        nextState = PM_COLLISION;
      }
      break;

    case PM_RELEASE:
      ////////////////////////////////////////////////////////////////////////
      // RELEASE STATE:
      // The marker is now in the right place. All we need to do is
      // release the marker from our grasp, and it should be balanced.
      ////////////////////////////////////////////////////////////////////////
      hand.SetSpeed(OPEN_SPEED);
      hand.SetForce(OPEN_FORCE);
      hand.SetAngle(HAND_OPEN);

      hand.WaitRest(0.25);

      nextState = PM_OUT_AND_UP;
      break;

    case PM_OUT_AND_UP:
      ////////////////////////////////////////////////////////////////////////
      // OUT AND UP STATE:
      // Now the marker is balanced and sitting there. We need to move our
      // hand up and out so that we don't hit the marker as we go back to
      // the home position. Also, this is the end of our placing operation, 
      // so stop the log.
      ////////////////////////////////////////////////////////////////////////

      robot.SetSpeed(P_TCP, P_ORI);

      goalPos[0] += OUT_X_CHG;
      goalPos[1] += OUT_Y_CHG;
      goalPos[2] += UP_Z_CHG;

      if (robot.SetCartesian(goalPos[0], goalPos[1], goalPos[2],
            goalQuat[0], goalQuat[1], goalQuat[2], goalQuat[3]))
      {
        nextState = PM_KN_CHECK;
      }
      else
      {
        nextState = PM_ERROR;
      }

      logger.Stop();

      break;

    case PM_KN_CHECK:
      {
      ////////////////////////////////////////////////////////////////////////
      // KNOCKDOWN CHECK STATE
      // Checks to see if placing was successful, and determines if it has to
      // knock down a marker or not. If so, then it moves to a positon right
      // above where it needs to be to knock it down
      ////////////////////////////////////////////////////////////////////////

        if (!placeParams.test)
        {
          bool success;

          // Don't take a picture until we're sure everything has settled down
          usleep(PM_SETTLE_TIME);

          vision.CaptureImage(PLACE_DETECT, picName);
          if (!vision.PlaceDetect(picName, success))
          {
            nextState = PM_ERROR;
          }
          else
          {
            // Record whether or not placing was successful
            if (success)
            {
              ROS_INFO("Placing Successful!");
              logger.Append(log_file, "Placing Successful");
              place_success = true;
            }
            else
            {
              ROS_INFO("Placing Unsuccessful");
              logger.Append(log_file, "Placing Unsuccessful");
              place_success = false;
            }

            // Now, get in position to knock marker over. Note that we do this
            //  even if we were unsuccessful, on the off chance that our vision
            //  system failed, and there actually is a marker there
            if (robot.SetCartesian(KN_TRAIN_X, KN_TRAIN_Y, KN_TRAIN_Z, 
                  KN_TRAIN_Q0, KN_TRAIN_QX, KN_TRAIN_QY, KN_TRAIN_QZ))
              nextState = PM_KN_DOWN;
            else
              nextState = PM_ERROR;
          }
        }
        else
        {
          // If we are in test mode, simply knock over the marker
            if (robot.SetCartesian(KN_TEST_X, KN_TEST_Y, KN_TEST_Z, 
                  KN_TEST_Q0, KN_TEST_QX, KN_TEST_QY, KN_TEST_QZ))
              nextState = PM_KN_DOWN;
            else
              nextState = PM_ERROR;
        }
        break;
      }

    case PM_KN_DOWN:
      ////////////////////////////////////////////////////////////////////////
      // KNOCKDOWN DOWN STATE
      // In this state, we move down and put our fingers in a position to 
      // knock the marker over
      ////////////////////////////////////////////////////////////////////////
      if (placeParams.test)
      {
        if (robot.SetCartesian(KN_TEST_X, KN_TEST_Y, KN_DOWN_TEST_Z, 
                  KN_TEST_Q0, KN_TEST_QX, KN_TEST_QY, KN_TEST_QZ))
          nextState = PM_KN_OVER;
        else
          nextState = PM_ERROR;
      }
      else
      {
        if (robot.SetCartesian(KN_TRAIN_X, KN_TRAIN_Y, KN_DOWN_TRAIN_Z, 
                  KN_TRAIN_Q0, KN_TRAIN_QX, KN_TRAIN_QY, KN_TRAIN_QZ))
          nextState = PM_KN_OVER;
        else
          nextState = PM_ERROR;
      }
      break;
      
    case PM_KN_OVER:
      ////////////////////////////////////////////////////////////////////////
      // KNOCKDOWN OVER STATE
      // In this state, we move over and knock the marker down with one of
      // our fingers
      ////////////////////////////////////////////////////////////////////////
      if (placeParams.test)
      {
        if (robot.SetCartesian(KN_OVER_TEST_X, KN_TEST_Y, KN_DOWN_TEST_Z, 
              KN_TEST_Q0, KN_TEST_QX, KN_TEST_QY, KN_TEST_QZ))
        {
          nextState = PM_SAFE_UP;
        }
        else
          nextState = PM_ERROR;
      }
      else
      {
        if (robot.SetCartesian(KN_OVER_TRAIN_X, KN_TRAIN_Y, KN_DOWN_TRAIN_Z, 
              KN_TRAIN_Q0, KN_TRAIN_QX, KN_TRAIN_QY, KN_TRAIN_QZ))
        {
          nextState = PM_SAFE_UP;
        }
        else
          nextState = PM_ERROR;
      }
      break;

    case PM_SAFE_UP:
      ////////////////////////////////////////////////////////////////////////
      // KNOCKDOWN OVER STATE
      // After knocking the marker over, we now move up, so we don't get 
      // hooked on anything moving back to the bin
      ////////////////////////////////////////////////////////////////////////
      if (placeParams.test)
      {
        if (robot.SetCartesian(KN_OVER_TEST_X, KN_TEST_Y, SAFE_TEST_Z, 
              KN_TEST_Q0, KN_TEST_QX, KN_TEST_QY, KN_TEST_QZ))
        {
          nextState = PM_COMPLETE;
        }
        else
          nextState = PM_ERROR;
      }
      else
      {
        if (robot.SetCartesian(KN_OVER_TRAIN_X, KN_TRAIN_Y, SAFE_TRAIN_Z, 
              KN_TRAIN_Q0, KN_TRAIN_QX, KN_TRAIN_QY, KN_TRAIN_QZ))
        {
          nextState = PM_COMPLETE;
        }
        else
          nextState = PM_ERROR;
      }
      break;


      case PM_COMPLETE:
      nextState = PM_COMPLETE;
      break;

      case PM_COLLISION:
      /*
        ROS_INFO("IN PLACE MARKER COLLISION STATE");
        
        // Retract up to the start, and release in case 
        //  we're still grasping a marker
        goalPos[2] = KN_Z;
        if (!robot.SetCartesian(goalPos[0], goalPos[1], goalPos[2],
              goalQuat[0], goalQuat[1], goalQuat[2], goalQuat[3]))
        {
          nextState = PM_ERROR;
        }
        else
        {
          // Open the hand so the marker will drop
          hand.SetSpeed(OPEN_SPEED);
          hand.SetForce(OPEN_FORCE);
          hand.SetAngle(HAND_OPEN);
          hand.WaitRest(0.25);
         
          // We clearly fail at placing the marker, so we're done
          nextState = PM_FAIL;
        }
        */
      place_success = false;
      nextState = PM_COMPLETE;
        break;

    case PM_ERROR:
    case PM_NUM_STATES:
      ROS_WARN("Error in Place Marker...");
      nextState = PM_ERROR;
      break;
  }
  return nextState;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, PM_PLACE_OPENLOOP_NAME);
  ros::NodeHandle node;
  nodePtr = &node;

  // Subscribe to all of the services we will be using
  ROS_INFO("PLACE_OPENLOOP: Subscribing to ROS services...");
  robot.subscribe(nodePtr);
  hand.subscribe(nodePtr);
  vision.subscribe(nodePtr);
  logger.subscribe(nodePtr);

  //Wait until Robot and Hand are active
  ROS_INFO("PLACE_OPENLOOP: Waiting for Robot...");
  while(!robot.Ping()) ;
  ROS_INFO("PLACE_OPENLOOP: Waiting for Hand...");
  while(!hand.Ping()) ;
  ROS_INFO("PLACE_OPENLOOP: Waiting for Vision...");
  while(!vision.Ping()) ;

  //Advertising ROS services
  ROS_INFO("PLACE_OPENLOOP: Advertising ROS services...");
  handle_place_OpenLoop = node.advertiseService("place_OpenLoop",place_OpenLoop);

  //Main ROS loop
  ROS_INFO("PLACE_OPENLOOP: Running node /place_openloop...");
  ros::spin();
  ROS_INFO("PLACE_OPENLOOP: Shutting down node /place_openloop...");

  ROS_INFO("PLACE_OPENLOOP: Shutting down services ...");
  handle_place_OpenLoop.shutdown();
  robot.shutdown();
  hand.shutdown();
  logger.shutdown();
}
