#include "sense_vision.h"

bool sense_Vision(sense_comm::sense_Vision::Request& req, sense_comm::sense_Vision::Response&res)
{

  //Initial state.
  int currState = SM_INIT;

  //Grasp parameters
  SM_InputParams senseParams;
  senseParams.logFileName = req.logFileName;
  senseParams.type = SM_TYPE_VISION;

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
    res.confidence = confidence;
    res.x = x;
    res.y = y;
    res.theta = theta;
    res.alpha = alpha;
    res.distance = dist;
    res.picFileName = picFileName;
    res.ret = 1;
    res.msg = "SENSE_VISION: OK.";
    return true;
  }
  res.ret = 0;
  res.msg = "SENSE_VISION: There was an error in the sense_Vision state machine.";
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
      if(!util.go_safe_vision())
      {
        nextState = SM_ERROR;
        ROS_ERROR("SENSE_VISION: Could not move robot to camera.");
      }
      else
      {
        // If the move was successful, we will take a picture with the camera
        nextState = SM_TAKE_PIC;
      }
      break;

    case SM_TAKE_PIC:
      {	
        // FIXME: Wait a bit to make sure the table is no longer shaking
        ros::Duration(0.25).sleep(); //0.25 seconds
        
        vision.CaptureImage(GRASP_DETECT, picFileName);
        
        // Record values of hand sensors right before the picture is taken. 
        // This will give us an exact match between the image and the sensor 
        // values in terms of marker position. We will save this to the log 
        // file as well, and this will hopefully increase our 
        // pose estimation accuracy
        hand.GetEncoders(motor_enc, &finger_enc[0]);   
        hand.GetRawForces(&raw_forces[0]);   
        
        double aux;
        if (!vision.GetInfo(picFileName, nMarkers, confidence, aux, x, y, theta, alpha, dist))
        {
          nextState = SM_ERROR;
          break;
        }
        //Change orientation of the marker to tool reference frame (may be vision should do it).
        nextState = SM_LOG;
        break;
      }
    case SM_LOG:
      {
        char buffer[MAX_BUFFER];
        if(nMarkers == 1)
          {
            sprintf(buffer, "#S,%d,1,%lf,%d,%lf,%lf,%lf,%lf,%lf,%d",
                  senseParams.type,confidence,
                  nMarkers,
                  x,
                  y,
                  theta,
                  alpha,
                  dist,
                  motor_enc);
            for (int i=0; i<NUM_FINGERS; i++)
              sprintf(buffer,"%s,%d",buffer,finger_enc[i]);
            for (int i=0; i<NUM_RAW_HAND_FORCES; i++)
              sprintf(buffer,"%s,%d",buffer,raw_forces[i]);
            sprintf(buffer,"%s,%s",buffer,picFileName.c_str());
          }
        else
          {
            sprintf(buffer, "#S,%d,0,%lf,%d,0.0,0.0,0.0,0.0,0.0,%d",
                    senseParams.type,
                    confidence,
                    nMarkers,
                    motor_enc);
            for (int i=0; i<NUM_FINGERS; i++)
              sprintf(buffer,"%s,%d",buffer,finger_enc[i]);
            for (int i=0; i<NUM_RAW_HAND_FORCES; i++)
              sprintf(buffer,"%s,%d",buffer,raw_forces[i]);
            sprintf(buffer,"%s,%s",buffer,picFileName.c_str());
          }
        string info = buffer;

        // Only append to log file if it exists. Otherwise, we're done
        if (senseParams.logFileName.length() > 0)
        {
          if(logger.Append(senseParams.logFileName,info))
            nextState = SM_END;
          else
          {
            ROS_ERROR("SENSE_VISION: Could not append output to log File: %s",senseParams.logFileName.c_str());
            nextState = SM_ERROR;
          }
        } 
        else
          nextState = SM_END;

        break;
      }
    default:
      ROS_WARN("SENSE_VISION: Error in Sense_vision state machine. State number out of bounds...");
      nextState = SM_ERROR;
      break;
  }
  return nextState;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, SM_VISION_NAME);
  ros::NodeHandle node;
  nodePtr = &node;

  // Subscribe to all of the services we will be using
  ROS_INFO("SENSE_VISION: Subscribing to ROS services...");
  //robot.subscribe(nodePtr);
  util.subscribe(nodePtr);
  logger.subscribe(nodePtr);
  vision.subscribe(nodePtr);
  hand.subscribe(nodePtr);

  //Wait until Robot is active
  //ROS_INFO("SENSE_VISION: Waiting for Robot...");
  //while(!robot.Ping()) ;

  ROS_INFO("SENSE_VISION: Waiting for Vision System...");
  while(!vision.Ping()) ;

  //Get camera position
  nodePtr->getParam("/system/cameraJ1",cameraJ1);
  nodePtr->getParam("/system/cameraJ2",cameraJ2);
  nodePtr->getParam("/system/cameraJ3",cameraJ3);
  nodePtr->getParam("/system/cameraJ4",cameraJ4);
  nodePtr->getParam("/system/cameraJ5",cameraJ5);
  nodePtr->getParam("/system/cameraJ6",cameraJ6);

  //Advertising ROS services
  ROS_INFO("SENSE_VISION: Advertising ROS services...");
  handle_sense_Vision = node.advertiseService("sense_Vision",sense_Vision);

  //Main ROS loop
  ROS_INFO("SENSE_VISION: Running node /sense_vision...");
  ros::spin();
  ROS_INFO("SENSE_VISION: Shutting down node /sense_vision...");

  ROS_INFO("SENSE_VISION: Shutting down services ...");
  handle_sense_Vision.shutdown();
  //robot.shutdown();
  util.shutdown();
  logger.shutdown();
  vision.shutdown();
}
