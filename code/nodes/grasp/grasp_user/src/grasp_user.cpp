#include "grasp_user.h"

bool grasp_User(grasp_comm::grasp_User::Request& req, grasp_comm::grasp_User::Response&res)
{
  // For safety we first set the workobject and tool.
  // The default values are specified in grasp_comm.h located in grasp_comm ROS package 
  // and are shared between all grasp procedures 
  robot.SetWorkObject(GM_WORK_X, GM_WORK_Y, GM_WORK_Z, 
      GM_WORK_Q0, GM_WORK_QX, GM_WORK_QY, GM_WORK_QZ);
  robot.SetTool(GM_TOOL_X, GM_TOOL_Y, GM_TOOL_Z, 
      GM_TOOL_Q0, GM_TOOL_QX, GM_TOOL_QY, GM_TOOL_QZ);

  //Initial state.
  int currState = GM_INIT;

  //Grasp parameters
  GM_InputParams graspParams;
  graspParams.id = req.id;

  //State Machine.
  while(currState!=GM_END && currState!=GM_ERROR)
  {
    currState = stepStateMachine(currState, graspParams);
    ros::spinOnce();
  }

  // Check for correct ending of the state machine.
  if(currState == GM_END)
  {
    res.logFileName = logFileName;
    res.ret = 1;
    res.msg = "GRASP_USER: OK.";
    return true;
  }
  res.ret = 0;
  res.msg = "GRASP_USER: There was an error in the grasp_User state machine.";
  ROS_ERROR("%s",res.msg.c_str());
  return false;
}

int stepStateMachine(int currState, GM_InputParams graspParams)
{
  int nextState = GM_ERROR;
  switch (currState)
  {
    case GM_INIT:
      nextState = GM_GRASP_MARKER;
      break;

    case GM_GRASP_MARKER:
      {	
        hand.SetSpeed(GM_OPEN_SPEED);
        hand.SetForce(GM_OPEN_FORCE);
        hand.SetAngle(GM_HAND_HOME);

        //Set blocking robot communication
        if (!robot.SetComm(1))
        {
          nextState = GM_ERROR;
          ROS_ERROR("GRASP_USER: Could not set the Robot communication mode.");
        }
        // Set robot Zone
        else if (!robot.SetZone(GM_ZONE))
        {
          nextState = GM_ERROR;
          ROS_ERROR("GRASP_USER: Could not set the Robot zone.");
        }
        // Set robot Speed
        else if (!robot.SetSpeed(GM_SPEED_TCP, GM_SPEED_ORI))
        {
          nextState = GM_ERROR;
          ROS_ERROR("GRASP_USER: Could not set the Robot speed.");
        }
        // Command robot to go to home position
        else if (!robot.SetJoints(GM_GRASP_J1, GM_GRASP_J2, GM_GRASP_J3, 
              GM_GRASP_J4, GM_GRASP_J5, GM_GRASP_J6))
        {
          nextState = GM_ERROR;
          ROS_ERROR("GRASP_USER: Could not set the Robot position.");
        }
        else
        {
          // The hand has been running during this time, so wait for it to finish opening
          hand.WaitRest(0.25);

          // Prompt user for a marker
          ROS_INFO("GRASP_USER: Ready to receive an object. A large deflection on finger will trigger closing the hand.");

          // Get initial pose of the hand
          int motEnc;
          hand.GetEncoders(motEnc, iniEncoders);

          nextState = GM_FEED_MARKER;
        }
        break;
      }
    case GM_FEED_MARKER:
      {
        // Check for finger encoder values every 250 ms
        int motEnc;
        Vec currEncoders(NUM_FINGERS);
        hand.GetEncoders(motEnc, currEncoders);
        double deflection = (currEncoders-iniEncoders).abs().max();

        // Trigger closing the hand if large enough deflection 
        if (deflection > GM_MIN_ENCODER_DEFLECTION)
        {
          logger.StartGrasp(logFileName,graspParams.id);

          // Close hand around object
          hand.SetSpeed(GM_CLOSE_SPEED);
          hand.SetForce(GM_CLOSE_FORCE);
          hand.SetAngle(GM_HAND_CLOSE);

          // Wait until end of grasp
          hand.WaitRest(0.25);
          hand.SetRest();
          logger.Stop();

          // We now successfully have a marker. We are done.
          nextState = GM_END;
        }
        else
        {
          ros::Duration(0.25).sleep(); // sleep for 250 ms
          nextState = GM_FEED_MARKER;
        }
        break;
      }

    default:
      ROS_WARN("GRASP_USER: Error in Grasp_user state machine. State number out of bounds...");
      nextState = GM_ERROR;
      break;
  }
  return nextState;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, GM_USER_NAME);
  ros::NodeHandle node;
  nodePtr = &node;

  // Subscribe to all of the services we will be using
  ROS_INFO("GRASP_USER: Subscribing to ROS services...");
  robot.subscribe(nodePtr);
  hand.subscribe(nodePtr);
  logger.subscribe(nodePtr);

  //Wait until Robot and Hand are active
  ROS_INFO("GRASP_USER: Waiting for Robot...");
  while(!robot.Ping()) ;
  ROS_INFO("GRASP_USER: Waiting for Hand...");
  while(!hand.Ping()) ;

  //Advertising ROS services
  ROS_INFO("GRASP_USER: Advertising ROS services...");
  handle_grasp_User = node.advertiseService("grasp_User",grasp_User);

  //Main ROS loop
  ROS_INFO("GRASP_USER: Running node /grasp_user...");
  ros::spin();
  ROS_INFO("GRASP_USER: Shutting down node /grasp_user...");

  ROS_INFO("GRASP_USER: Shutting down services ...");
  handle_grasp_User.shutdown();
  robot.shutdown();
  hand.shutdown();
}
