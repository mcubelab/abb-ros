#include "sense_data.h"

bool sense_Data(sense_comm::sense_Data::Request& req, sense_comm::sense_Data::Response&res)
{

  //Initial state.
  int currState = SM_INIT;

  //Grasp parameters
  SM_InputParams senseParams;
  senseParams.logFileName = req.logFileName;
  senseParams.type = SM_TYPE_DATA;
  senseParams.singulationModelName = req.singulationModelName;
  senseParams.poseEstimationModelName = req.poseEstimationModelName;

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
    res.distance = dist;
    res.confidence = certainty;
    res.ret = 1;
    res.msg = "SENSE_DATA: OK.";
    return true;
  }
  res.ret = 0;
  res.msg = "SENSE_DATA: There was an error in the sense_Data state machine.";
  ROS_ERROR("%s",res.msg.c_str());
  return false;
}

int stepStateMachine(int currState, SM_InputParams senseParams)
{
  int nextState;
  switch (currState)
  {
    case SM_INIT:
      nextState = SM_LOAD_MODELS;
      break;

    case SM_LOAD_MODELS:
      {	
        ROS_INFO("Current: Singulation Model: %s, PoseEstimationModel: %s", currentSingulationModel.c_str(), currentPoseEstimationModel.c_str());
        ROS_INFO("Requested: Singulation Model: %s, PoseEstimationModel: %s", senseParams.singulationModelName.c_str(), senseParams.poseEstimationModelName.c_str());
        // If the requested model is different than the loaded in memory, we load it from memory.
        if(senseParams.singulationModelName.compare(currentSingulationModel)!=0)
        {
          //ROS_INFO("Loading Model %s...", senseParams.singulationModelName.c_str());
          if(!brain.singulationDetection_LoadModel(senseParams.singulationModelName))
          {
            ROS_ERROR("SENSE_DATA: Error learning a singulation detection model.");
            nextState = SM_ERROR;
            break;
          }
          currentSingulationModel = senseParams.singulationModelName;
        }	  
        if (senseParams.poseEstimationModelName.compare(currentPoseEstimationModel)!=0)
        {
          if(!brain.markerPosEstimation_LoadModel(senseParams.poseEstimationModelName))
          {
            ROS_ERROR("SENSE_DATA: Error learning a Marker Pose Estimation model.");
            nextState = SM_ERROR;
            break;
          }
          currentPoseEstimationModel = senseParams.poseEstimationModelName;
        }
        ROS_INFO("Result: Singulation Model: %s, PoseEstimationModel: %s", currentSingulationModel.c_str(), currentPoseEstimationModel.c_str());
        nextState = SM_QUERY_MODEL;
        break;
      }
    case SM_QUERY_MODEL:
      {
        // First, check to see if we think the current grasp contains 1 marker
        if(!brain.singulationDetection_Test(senseParams.logFileName, label))
        {
          ROS_ERROR("SENSE_DATA: Error quering against a singulation detection model.");
          nextState = SM_ERROR;
          break;
        }
        
        // If so, then determine the pose of that marker
        if (label == 1)
        {
          if (currentPoseEstimationModel.length() == 0)
          {
            nMarkers = 1;
            certainty = 1.0;
          }
          else if(!brain.markerPosEstimation_Test(senseParams.logFileName, theta, dist, theta_s, dist_s))
          {
            ROS_ERROR("SENSE_DATA: Error quering against a marker pose estimation model.");
            nextState = SM_ERROR;
            break;
          }
          else
          {
            nMarkers = 1;
            certainty = 1 - CERT_SCALE * (theta_s * TH_DIST_SCALE + dist_s);
          }
        }
        else
        {
          nMarkers = 0;
          certainty = 1.0;
        }
        nextState = SM_LOG;
        break;
      }
    case SM_LOG:
      {
        char buffer[MAX_BUFFER];
        sprintf(buffer, "#S,%d,%d,%f,%d,%f,%f,%f,%f,-", 
            senseParams.type, label, certainty, label, x, y, theta, dist);
        string info = buffer;

        // Only append to log file if it exists. Otherwise, we're done
        if (senseParams.logFileName.length() > 0)
        {
          if (logger.Append(senseParams.logFileName,info))
            nextState = SM_END;
          else
          {
            ROS_ERROR("SENSE_DATA: Could not append output to log File: %s",
                senseParams.logFileName.c_str());
            nextState = SM_ERROR;
          }
        }
        else
          nextState = SM_END;
        break;
      }
    default:
      ROS_WARN("SENSE_DATA: Error in Sense_data state machine. State number out of bounds...");
      nextState = SM_ERROR;
      break;
  }
  return nextState;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, SM_DATA_NAME);
  ros::NodeHandle node;
  nodePtr = &node;

  // Subscribe to all of the services we will be using
  ROS_INFO("SENSE_DATA: Subscribing to ROS services...");
  brain.subscribe(nodePtr);
  logger.subscribe(nodePtr);

  //We should wait until matlab is active
  //NOTE: Change for something more intelligent
  ros::Duration(2.5).sleep();

  // We have no model by default loaded in memory
  currentSingulationModel = "";
  currentPoseEstimationModel = "";
  
  // Advertising ROS services
  ROS_INFO("SENSE_DATA: Advertising ROS services...");
  handle_sense_Data = node.advertiseService("sense_Data",sense_Data);
  
  // Main ROS loop
  ROS_INFO("SENSE_DATA: Running node /sense_data...");
  ros::spin();
  ROS_INFO("SENSE_DATA: Shutting down node /sense_data...");
  
  ROS_INFO("SENSE_DATA: Shutting down services ...");
  handle_sense_Data.shutdown();
  logger.shutdown();
  brain.shutdown();
}
