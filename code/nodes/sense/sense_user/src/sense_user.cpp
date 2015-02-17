#include "sense_user.h"

bool sense_User(sense_comm::sense_User::Request& req, sense_comm::sense_User::Response&res)
{

  //Initial state.
  int currState = SM_INIT;

  //Grasp parameters
  SM_InputParams senseParams;
  senseParams.logFileName = req.logFileName;
  senseParams.type = SM_TYPE_USER;

  //State Machine.
  while(currState!=SM_END && currState!=SM_ERROR)
  {
    currState = stepStateMachine(currState, senseParams);
    ros::spinOnce();
  }

  // Check for correct ending of the state machine.
  if(currState == SM_END)
  {
    res.logFileName = req.logFileName;
    res.nMarkers = nMarkers;
    if(nMarkers == 1)
      res.singulated = true;
    else
      res.singulated = false;
    res.x = x;
    res.y = y;
    res.theta = theta;
    res.alpha = alpha;
    res.distance = dist;
    res.ret = 1;
    res.msg = "SENSE_USER: OK.";
    return true;
  }
  res.ret = 0;
  res.msg = "SENSE_USER: There was an error in the sense_User state machine.";
  ROS_ERROR("%s",res.msg.c_str());
  return false;
}

int stepStateMachine(int currState, SM_InputParams senseParams)
{
  int nextState;
  switch (currState)
  {
    case SM_INIT:
      nextState = SM_GOTO_CAMERA;
      break;

    case SM_GOTO_CAMERA:

      //Set blocking robot communication
      if (!util.go_vision())
      {
        ROS_ERROR("SENSE_USER: Can't move robot to camera!");
        nextState = SM_ERROR;
      }
      else
      {
        // If the move was successful, we will ask the position of the marker from user
        nextState = SM_ASK_USER;
      }
      break;

    case SM_ASK_USER:
      {	
        cout << "How many markers? ";
        cin >> nMarkers;

        if (nMarkers == 1)
        {
          cout << "Marker Angle? (deg): ";
          cin >> theta;
          theta *= DEG2RAD;
          cout << "X-coordinate of the center of marker? (mm): ";
          cin >> x;
          cout << "Y-coordinate of the center of marker? (mm): ";
          cin >> y;
        }
        double xc = sin(theta)*(-y*cos(theta)+x*sin(theta));
        double yc = cos(theta)*(y*cos(theta)-x*sin(theta));
        alpha = atan2(yc, xc);
        // Compute distance from the axis of the marker to the center of the palm
        dist = fabs(y * cos(theta) - x * sin(theta));
        nextState = SM_LOG;
        break;
      }
    case SM_LOG:
      {
        char buffer[MAX_BUFFER];
        if(nMarkers == 1)
          sprintf(buffer, "#S,%d,1,1.0,%d,%lf,%lf,%lf,%lf,%lf,-",senseParams.type,nMarkers,x,y,theta,alpha,dist);
        else
          sprintf(buffer, "#S,%d,0,1.0,%d,0.0,0.0,0.0,0.0,0.0,-",SM_TYPE_USER,nMarkers);
        string info = buffer;

        // Only append to log file if it exists. Otherwise, we're done
        if (senseParams.logFileName.length() > 0)
        {
          if(logger.Append(senseParams.logFileName,info))
            nextState = SM_END;
          else
          {
            ROS_ERROR("SENSE_USER: Could not append output to log File: %s",
                senseParams.logFileName.c_str());
            nextState = SM_ERROR;
          }
        }
        else
          nextState = SM_END;
        break;
      }
    default:
      ROS_WARN("SENSE_USER: Error in Sense_user state machine. State number out of bounds...");
      nextState = SM_ERROR;
      break;
  }
  return nextState;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "SM_NAME_USER");
  ros::NodeHandle node;
  nodePtr = &node;

  // Subscribe to all of the services we will be using
  ROS_INFO("SENSE_USER: Subscribing to ROS services...");
  //robot.subscribe(nodePtr);
  util.subscribe(nodePtr);
  logger.subscribe(nodePtr);

  //Wait until Robot is active
  //ROS_INFO("SENSE_USER: Waiting for Robot...");
  //while(!robot.Ping()) ;

  //Get camera position
  nodePtr->getParam("/system/cameraJ1",cameraJ1);
  nodePtr->getParam("/system/cameraJ2",cameraJ2);
  nodePtr->getParam("/system/cameraJ3",cameraJ3);
  nodePtr->getParam("/system/cameraJ4",cameraJ4);
  nodePtr->getParam("/system/cameraJ5",cameraJ5);
  nodePtr->getParam("/system/cameraJ6",cameraJ6);

  //Advertising ROS services
  ROS_INFO("SENSE_USER: Advertising ROS services...");
  handle_sense_User = node.advertiseService("sense_User",sense_User);

  //Main ROS loop
  ROS_INFO("SENSE_USER: Running node /sense_user...");
  ros::spin();
  ROS_INFO("SENSE_USER: Shutting down node /sense_user...");

  ROS_INFO("SENSE_USER: Shutting down services ...");
  handle_sense_User.shutdown();
  //robot.shutdown();
  util.shutdown();
  logger.shutdown();
}
