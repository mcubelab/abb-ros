#include "sense_abort.h"

bool sense_Abort(sense_comm::sense_Abort::Request& req, sense_comm::sense_Abort::Response&res)
{

  //Initial state.
  int currState = SM_INIT;

  //Grasp parameters
  SM_InputParams senseParams;
  senseParams.logFileName = req.logFileName;
  senseParams.type = SM_TYPE_ABORT;
  senseParams.earlyAbortModelName = req.modelName;

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
    res.singulated = label;
    res.nMarkers = nMarkers;
    res.x = x;
    res.y = y;
    res.theta = theta;
    res.ret = 1;
    res.msg = "SENSE_ABORT: OK.";
    return true;
  }
  res.ret = 0;
  res.msg = "SENSE_ABORT: There was an error in the sense_Abort state machine.";
  ROS_ERROR("%s",res.msg.c_str());
  return false;
}

int stepStateMachine(int currState, SM_InputParams senseParams)
{
  int nextState;
  switch (currState)
  {
    case SM_INIT:
      nextState = SM_LOAD_MODEL;
      break;

    case SM_LOAD_MODEL:
      {	
        // If he requested model is different than the loaded in memory, we load it from memory.
        if(senseParams.earlyAbortModelName.compare(currentModel))
        {
          if(!brain.earlyAbort_LoadModel(senseParams.earlyAbortModelName))
          {
            ROS_ERROR("SENSE_ABORT: Error learning an early abort model.");
            nextState = SM_ERROR;
            break;
          }
          currentModel = senseParams.earlyAbortModelName;
        }	  
        nextState = SM_QUERY_MODEL;
        break;
      }
    case SM_QUERY_MODEL:
      {
        if(!brain.earlyAbort_Test(senseParams.logFileName, label))
        {
          ROS_ERROR("SENSE_ABORT: Error quering against an early abort model.");
          nextState = SM_ERROR;
          break;
        }
        nextState = SM_LOG;
        break;
      }
    case SM_LOG:
      {
        char buffer[MAX_BUFFER];
        if(label == 1)
          sprintf(buffer, "#S,%d,1,1.0,0,0.0,0.0,0.0,0.0,-",senseParams.type);
        else
          sprintf(buffer, "#S,%d,0,1.0,0,0.0,0.0,0.0,0.0,-",senseParams.type);
        string info = buffer;

        // Only append to log file if it exists. Otherwise, we're done
        if (senseParams.logFileName.length() > 0)
        {
          if (logger.Append(senseParams.logFileName,info))
            nextState = SM_END;
          else
          {
            ROS_ERROR("SENSE_ABORT: Could not append output to log File: %s",
                senseParams.logFileName.c_str());
            nextState = SM_ERROR;
          }
        }
        else
          nextState = SM_END;
        break;
      }
    default:
      ROS_WARN("SENSE_ABORT: Error in Sense_abort state machine. State number out of bounds...");
      nextState = SM_ERROR;
      break;
  }
  return nextState;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, SM_ABORT_NAME);
  ros::NodeHandle node;
  nodePtr = &node;

  // Subscribe to all of the services we will be using
  ROS_INFO("SENSE_ABORT: Subscribing to ROS services...");
  brain.subscribe(nodePtr);
  logger.subscribe(nodePtr);

  //We should wait until matlab is active
  //NOTE: Change for something more intelligent
  ros::Duration(2.5).sleep();

  // We have no model by default loaded in memmory
  currentModel = "";
  
  // Advertising ROS services
  ROS_INFO("SENSE_ABORT: Advertising ROS services...");
  handle_sense_Abort = node.advertiseService("sense_Abort",sense_Abort);
  
  // Main ROS loop
  ROS_INFO("SENSE_ABORT: Running node /sense_abort...");
  ros::spin();
  ROS_INFO("SENSE_ABORT: Shutting down node /sense_abort...");
  
  ROS_INFO("SENSE_ABORT: Shutting down services ...");
  handle_sense_Abort.shutdown();
  logger.shutdown();
  brain.shutdown();
}
