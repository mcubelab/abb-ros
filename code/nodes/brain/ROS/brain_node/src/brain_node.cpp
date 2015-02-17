//
// Brain Node
//
// This node is the interface to learning and data driven models of grasping and placing
//
#include "brain_node.h"

Brain::Brain(ros::NodeHandle *n) 
{
  nodePtr = n;
  logging = false;
  logger_SystemLog_present = false;
}

Brain::~Brain() {
  /// Shut down services.
  ROS_INFO("BRAIN: Shutting down services...");
  handle_brain_SingulationDetection_TrainModel.shutdown();
  handle_brain_SingulationDetection_LoadModel.shutdown();
  handle_brain_SingulationDetection_Test.shutdown();
  handle_brain_earlyAbort_TrainModel.shutdown();
  handle_brain_earlyAbort_LoadModel.shutdown();
  handle_brain_earlyAbort_Test.shutdown();
  handle_brain_MarkerPosEstimation_TrainModel.shutdown();
  handle_brain_MarkerPosEstimation_LoadModel.shutdown();
  handle_brain_MarkerPosEstimation_Test.shutdown();
  handle_brain_LearnGrasp_LoadModel.shutdown();
  handle_brain_LearnGrasp_GetNextParams.shutdown();
  handle_brain_LearnGrasp_GetMaxParams.shutdown();
  handle_brain_LearnGrasp_TrainParams.shutdown();
  handle_brain_LearnGrasp_SaveModel.shutdown();

  //Close the MATLAB engine
  ROS_INFO("BRAIN: Closing connection with MATLAB-ROS node.");
  matlab.shutdown();
}

bool Brain::init()
{
  char buffer[MAX_BUFFER];

  ROS_INFO("BRAIN: Subscribing to MATLAB-ROS topics...");
  matlab.subscribe(nodePtr);

  //Do periodic scans of system log topic every 2.0 seconds
  scannerTimer = nodePtr->createTimer(ros::Duration(2.0), &Brain::scannerCallBack,this);

  //Clear Matlab Workspace
  ROS_INFO("BRAIN: Clearing MATLAB workspace...");
  sprintf(buffer, "clear");
  matlab.sendCommand(buffer);

  //Add scripts folder and all subfolders to MATLAB PATH
  ROS_INFO("BRAIN: Adding script folder and subfolders to MATLAB path.");
  std::string folder;
  nodePtr->getParam("/brain/folder", folder);
  sprintf(buffer,"addpath(genpath('%s'));",folder.c_str());
  matlab.sendCommand(buffer);

  //Advertising ROS services
  ROS_INFO("BRAIN: Advertising ROS services...");
  advertiseServices();

  return true;
}

void Brain::scanAndSubscribeTopics()
{
  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);
  for(int i=0; i<(int)topics.size(); i++)
  {
    if(!strcmp(topics[i].name.c_str(),"/logger_SystemLog"))
    {
      handle_logger_SystemLog = nodePtr->subscribe("logger_SystemLog", 100, &Brain::logger_SystemLog_Callback, this);
      logger_SystemLog_present = true;
      ROS_INFO("BRAIN: Topic /logger_SystemLog found.");
      ROS_INFO("BRAIN: Ready to process real time data.");
    }
  }
}

void Brain::advertiseServices()
{
  handle_brain_SingulationDetection_TrainModel = 
    nodePtr->advertiseService("brain_SingulationDetection_TrainModel", &Brain::singulationDetection_TrainModel,this);
  handle_brain_SingulationDetection_LoadModel = 
    nodePtr->advertiseService("brain_SingulationDetection_LoadModel", &Brain::singulationDetection_LoadModel,this);
  handle_brain_SingulationDetection_Test = 
    nodePtr->advertiseService("brain_SingulationDetection_Test", &Brain::singulationDetection_Test,this);

  handle_brain_earlyAbort_TrainModel = 
    nodePtr->advertiseService("brain_earlyAbort_TrainModel", &Brain::earlyAbort_TrainModel,this);
  handle_brain_earlyAbort_LoadModel = 
    nodePtr->advertiseService("brain_earlyAbort_LoadModel", &Brain::earlyAbort_LoadModel,this);
  handle_brain_earlyAbort_Test = 
    nodePtr->advertiseService("brain_earlyAbort_Test", &Brain::earlyAbort_Test,this);

  handle_brain_MarkerPosEstimation_TrainModel = 
    nodePtr->advertiseService("brain_MarkerPosEstimation_TrainModel", &Brain::markerPosEstimation_TrainModel,this);
  handle_brain_MarkerPosEstimation_LoadModel = 
    nodePtr->advertiseService("brain_MarkerPosEstimation_LoadModel", &Brain::markerPosEstimation_LoadModel,this);
  handle_brain_MarkerPosEstimation_Test = 
    nodePtr->advertiseService("brain_MarkerPosEstimation_Test", &Brain::markerPosEstimation_Test,this);

  handle_brain_LearnGrasp_LoadModel = 
    nodePtr->advertiseService("brain_LearnGrasp_LoadModel", &Brain::learnGrasp_LoadModel,this);
  handle_brain_LearnGrasp_GetNextParams = 
    nodePtr->advertiseService("brain_LearnGrasp_GetNextParams", &Brain::learnGrasp_GetNextParams,this);
  handle_brain_LearnGrasp_GetMaxParams = 
    nodePtr->advertiseService("brain_LearnGrasp_GetMaxParams", &Brain::learnGrasp_GetMaxParams,this);
  handle_brain_LearnGrasp_TrainParams = 
    nodePtr->advertiseService("brain_LearnGrasp_TrainParams", &Brain::learnGrasp_TrainParams,this);
  handle_brain_LearnGrasp_SaveModel = 
    nodePtr->advertiseService("brain_LearnGrasp_SaveModel", &Brain::learnGrasp_SaveModel,this);
}

void Brain::scannerCallBack(const ros::TimerEvent& event)
{
  if(!logger_SystemLog_present)
    scanAndSubscribeTopics();
}


void Brain::logger_SystemLog_Callback(const logger_comm::logger_SystemLog& msg)
{
  pthread_mutex_lock(&systemLogMutex);
  //The log begins when logtimeStamp begins to count and ends when it is zero again.
  if(msg.logtimeStamp!=0)
  {
    if(!logging) //transition into logging
    {
      logging = true;
      systemLogVector.clear();
      systemLogVector.reserve(N_TIMESTAMPS_TO_RESERVE);
      systemLogVector.push_back(systemLog);
    }
    else
    {
      if(systemLogVector.size() >systemLogVector.capacity())
        systemLogVector.reserve(systemLogVector.capacity() + 25);
    }
    systemLogVector.push_back(msg);	  
  }
  else
    logging = false;
  //Save the last log
  systemLog = msg;
  pthread_mutex_unlock(&systemLogMutex);
}



bool Brain::singulationDetection_TrainModel(
    brain_comm::brain_SingulationDetection_TrainModel::Request& req,
    brain_comm::brain_SingulationDetection_TrainModel::Response& res)
{
  char logFolderName[MAX_BUFFER];
  char modelName[MAX_BUFFER];
  char command[MAX_BUFFER];
  char buffer[MAX_BUFFER];

  //Train model
  std::string logFolder;
  std:: string scriptFolder;
  nodePtr->getParam("/brain/folder",scriptFolder);
  nodePtr->getParam("/logger/folder",logFolder);

  sprintf(logFolderName,"%s/%s",logFolder.c_str(),req.logFolderName.c_str());
  sprintf(modelName,"%s/%s/learnedModels/%s",scriptFolder.c_str(),SINGULATION_DETECTION,req.modelName.c_str());
  sprintf(command,"error = brain_singulationDetection_trainModel('%s','%s');",logFolderName, modelName);
  bool ok = matlab.sendCommand(command);

  //Recover cross validation error
  if(ok)
  {
    sprintf(buffer,"error");
    if(matlab.getValue(buffer,&res.error))
    {
      res.ret = 1;
      res.msg = "BRAIN_NODE: OK";
      return true;
    }
    res.ret = 0;
    res.msg = "BRAIN_NODE: Error trying to recover the cross-validation error.";
    return false;
  }
  res.ret = 0;
  res.msg = "BRAIN_NODE: Error sending command to MATLAB engine.";
  return false;
}

bool Brain::singulationDetection_LoadModel(
    brain_comm::brain_SingulationDetection_LoadModel::Request& req,
    brain_comm::brain_SingulationDetection_LoadModel::Response& res)
{
  char modelName[MAX_BUFFER];
  char command[MAX_BUFFER];

  //Load model
  std:: string scriptFolder;
  nodePtr->getParam("/brain/folder",scriptFolder);
  sprintf(modelName,"%s/%s/learnedModels/%s",scriptFolder.c_str(),SINGULATION_DETECTION,req.modelName.c_str());
  sprintf(command,"brain_singulationDetection_loadModel('%s');",modelName);
  bool ok = matlab.sendCommand(command);

  if(ok)
  {
    res.ret = 1;
    res.msg = "BRAIN_NODE: OK";
    return true;
  }
  res.ret = 0;
  res.msg = "BRAIN_NODE: Error trying to load the singulation detection model.";
  return false;
}

bool Brain::singulationDetection_Test(
    brain_comm::brain_SingulationDetection_Test::Request& req,
    brain_comm::brain_SingulationDetection_Test::Response& res)
{
  char logFileName[MAX_BUFFER];
  char command[MAX_BUFFER];
  char buffer[MAX_BUFFER];

  //Test log file
  std:: string logFolder;
  nodePtr->getParam("/logger/folder",logFolder);
  sprintf(logFileName,"%s/%s",logFolder.c_str(),req.logFileName.c_str());
  sprintf(command,"label = brain_singulationDetection_test('%s');",logFileName);
  bool ok = matlab.sendCommand(command);

  if(ok)
  {
    //recover class
    double label;
    sprintf(buffer,"label");
    if(matlab.getValue(buffer,&label))
    {

      res.label = (int)label;
      res.ret = 1;
      res.msg = "BRAIN_NODE: OK";
      return true;
    }
    res.ret = 0;
    res.msg = "BRAIN_NODE: Error trying to recover the output from the classifier.";
    return false;
  }
  res.ret = 0;
  res.msg = "BRAIN_NODE: Error trying to classify the example.";
  return false;
}

bool Brain::earlyAbort_TrainModel(
    brain_comm::brain_earlyAbort_TrainModel::Request& req,
    brain_comm::brain_earlyAbort_TrainModel::Response& res)
{
  char logFolderName[MAX_BUFFER];
  char modelName[MAX_BUFFER];
  char command[MAX_BUFFER];
  char buffer[MAX_BUFFER];

  //Train model
  std::string logFolder;
  std:: string scriptFolder;
  nodePtr->getParam("/brain/folder",scriptFolder);
  nodePtr->getParam("/logger/folder",logFolder);

  sprintf(logFolderName,"%s/%s",logFolder.c_str(),req.logFolderName.c_str());
  sprintf(modelName,"%s/%s/learnedModels/%s",scriptFolder.c_str(),EARLY_ABORT,req.modelName.c_str());
  sprintf(command,"error = brain_earlyAbort_trainModel('%s','%s', %d);",logFolderName, modelName, (int)req.nSlices);
ROS_INFO("%s", command);
  bool ok = matlab.sendCommand(command);

  //Recover cross validation error
  if(ok)
  {
    sprintf(buffer,"error");
    if(matlab.getValue(buffer,&res.error))
    {
      res.ret = 1;
      res.msg = "BRAIN_NODE: OK";
      return true;
    }
    res.ret = 0;
    res.msg = "BRAIN_NODE: Error trying to recover the cross-validation error.";
    return false;
  }
  res.ret = 0;
  res.msg = "BRAIN_NODE: Error sending command to MATLAB engine.";
  return false;
}

bool Brain::earlyAbort_LoadModel(
    brain_comm::brain_earlyAbort_LoadModel::Request& req,
    brain_comm::brain_earlyAbort_LoadModel::Response& res)
{
  char modelName[MAX_BUFFER];
  char command[MAX_BUFFER];

  //Load model
  std:: string scriptFolder;
  nodePtr->getParam("/brain/folder",scriptFolder);
  sprintf(modelName,"%s/%s/learnedModels/%s",scriptFolder.c_str(),EARLY_ABORT,req.modelName.c_str());
  sprintf(command,"brain_earlyAbort_loadModel('%s');",modelName);
  bool ok = matlab.sendCommand(command);

  if(ok)
  {
    res.ret = 1;
    res.msg = "BRAIN_NODE: OK";
    return true;
  }
  res.ret = 0;
  res.msg = "BRAIN_NODE: Error trying to load the singulation detection model.";
  return false;
}

bool Brain::earlyAbort_Test(
    brain_comm::brain_earlyAbort_Test::Request& req,
    brain_comm::brain_earlyAbort_Test::Response& res)
{
  char logFileName[MAX_BUFFER];
  char command[MAX_BUFFER];
  char buffer[MAX_BUFFER];

  //Test log file
  std:: string logFolder;
  nodePtr->getParam("/logger/folder",logFolder);
  sprintf(logFileName,"%s/%s",logFolder.c_str(),req.logFileName.c_str());
  sprintf(command,"label = brain_earlyAbort_test('%s');",logFileName);
  bool ok = matlab.sendCommand(command);

  if(ok)
  {
    //recover class
    double label;
    sprintf(buffer,"label");
    if(matlab.getValue(buffer,&label))
    {

      res.label = (int)label;
      res.ret = 1;
      res.msg = "BRAIN_NODE: OK";
      return true;
    }
    res.ret = 0;
    res.msg = "BRAIN_NODE: Error trying to recover the output from the classifier.";
    return false;
  }
  res.ret = 0;
  res.msg = "BRAIN_NODE: Error trying to classify the example.";
  return false;
}

bool Brain::markerPosEstimation_TrainModel(
    brain_comm::brain_MarkerPosEstimation_TrainModel::Request& req,
    brain_comm::brain_MarkerPosEstimation_TrainModel::Response& res)
{
  char logFolderName[MAX_BUFFER];
  char modelName[MAX_BUFFER];
  char command[MAX_BUFFER];
  char buffer[MAX_BUFFER];

  //Train model
  std::string logFolder;
  std:: string scriptFolder;
  nodePtr->getParam("/brain/folder",scriptFolder);
  nodePtr->getParam("/logger/folder",logFolder);

  sprintf(logFolderName,"%s/%s",logFolder.c_str(),req.logFolderName.c_str());
  sprintf(modelName,"%s/%s/learnedModels/%s",scriptFolder.c_str(),MARKER_POS_ESTIMATION,req.modelName.c_str());
  sprintf(command,"nlml = brain_markerPosEstimation_trainModel('%s','%s');",logFolderName, modelName);
  bool ok = matlab.sendCommand(command);

  //Recover negative log liklihood
  if(ok)
  {
    Vec nlml(2);
    sprintf(buffer,"nlml");
    if(!matlab.getVec(buffer,nlml))
    {
      res.ret = 0;
      res.msg = "BRAIN_NODE: Error trying to recover negative log liklihood";
      return false;
    }

    res.theta_nll = nlml[0];
    res.r_nll = nlml[1];

    res.ret = 1;
    res.msg = "BRAIN_NODE: OK";
    return true;
  }
  res.ret = 0;
  res.msg = "BRAIN_NODE: Error sending command to MATLAB engine.";
  return false;
}

bool Brain::markerPosEstimation_LoadModel(
    brain_comm::brain_MarkerPosEstimation_LoadModel::Request& req,
    brain_comm::brain_MarkerPosEstimation_LoadModel::Response& res)
{
  char modelName[MAX_BUFFER];
  char command[MAX_BUFFER];

  //Load model
  std:: string scriptFolder;
  nodePtr->getParam("/brain/folder",scriptFolder);
  sprintf(modelName,"%s/%s/learnedModels/%s",scriptFolder.c_str(),MARKER_POS_ESTIMATION,req.modelName.c_str());
  sprintf(command,"brain_markerPosEstimation_loadModel('%s');",modelName);
  bool ok = matlab.sendCommand(command);

  if(ok)
  {
    res.ret = 1;
    res.msg = "BRAIN_NODE: OK";
    return true;
  }
  res.ret = 0;
  res.msg = "BRAIN_NODE: Error trying to load the marker position estimation model.";
  return false;
}

bool Brain::markerPosEstimation_Test(
    brain_comm::brain_MarkerPosEstimation_Test::Request& req,
    brain_comm::brain_MarkerPosEstimation_Test::Response& res)
{
  char logFileName[MAX_BUFFER];
  char command[MAX_BUFFER];
  char buffer[MAX_BUFFER];

  //Test log file
  std:: string logFolder;
  nodePtr->getParam("/logger/folder",logFolder);
  sprintf(logFileName,"%s/%s",logFolder.c_str(),req.logFileName.c_str());
  sprintf(command,"[y, s] = brain_markerPosEstimation_test('%s');",logFileName);
  bool ok = matlab.sendCommand(command);

  if(ok)
  {
    Vec y(2);
    sprintf(buffer,"y");
    if(!matlab.getVec(buffer,y))
    {
      res.ret = 0;
      res.msg = "BRAIN_NODE: Error trying to recover regressed values";
      return false;
    }

    Vec s(2);
    sprintf(buffer,"s");
    if(!matlab.getVec(buffer,s))
    {
      res.ret = 0;
      res.msg = "BRAIN_NODE: Error trying to recover regressed value variances";
      return false;
    }

    res.theta = y[0];
    res.r = y[1];

    res.theta_s = s[0];
    res.r_s = s[1];

    res.ret = 1;
    res.msg = "BRAIN_NODE: OK";
    return true;
  }
  res.ret = 0;
  res.msg = "BRAIN_NODE: Error trying to classify the example.";
  return false;
}



bool Brain::learnGrasp_LoadModel(
    brain_comm::brain_LearnGrasp_LoadModel::Request& req, 
    brain_comm::brain_LearnGrasp_LoadModel::Response& res)
{
  char modelName[MAX_BUFFER];
  char mins[MAX_BUFFER];
  char maxs[MAX_BUFFER];
  char command[MAX_BUFFER];
  char buffer[MAX_BUFFER];

  // Get the requested model, and find the current optimal parameters
  std:: string scriptFolder;
  nodePtr->getParam("/brain/folder",scriptFolder);

  sprintf(modelName,"%s/%s/learnedModels/%s",scriptFolder.c_str(),LEARN_GRASP,req.modelName.c_str());

  // Keep track of how many parameters our current model has
  learnGrasp_numParams = (int)req.mins.size();

  // Now format our string to load a new model with appropriate bounds
  mins[0] = '\0';
  maxs[0] = '\0';
  for (int i=0; i < learnGrasp_numParams; i++)
  {
    // Get a string of mins and maxs for our parameters
    sprintf(buffer, " %f", req.mins[i]); 
    strcat(mins, buffer);
    sprintf(buffer, " %f", req.maxs[i]); 
    strcat(maxs, buffer);
  }
  sprintf(command,"brain_learnGrasp_loadModel('%s', [%s;%s], %d);", modelName, mins, maxs, (int)req.grid);
  bool ok = matlab.sendCommand(command);

  // Check if loading model was successful
  if(ok)
  {
    res.ret = 1;
    res.msg = "BRAIN_NODE: OK";
    return true;
  }

  res.ret = 0;
  res.msg = "BRAIN_NODE: Error sending command to MATLAB engine.";
  return false;
}

bool Brain::learnGrasp_GetNextParams(
    brain_comm::brain_LearnGrasp_GetNextParams::Request& req, 
    brain_comm::brain_LearnGrasp_GetNextParams::Response& res)
{
  char command[MAX_BUFFER];
  char buffer[MAX_BUFFER];

  // Get the requested model, and find the current optimal parameters
  sprintf(command,"nextP = brain_learnGrasp_getNextParams();");
  bool ok = matlab.sendCommand(command);

  //Recover parameters
  if(ok)
  {
    res.nextP.resize(learnGrasp_numParams);
    Vec nextP(learnGrasp_numParams);
    sprintf(buffer,"nextP");
    if(!matlab.getVec(buffer,nextP))
    {
      res.ret = 0;
      res.msg = "BRAIN_NODE: Error trying to recover next parameters.";
      return false;
    }

    for (int i=0; i < learnGrasp_numParams; i++)
      res.nextP[i] = nextP[i];

    res.ret = 1;
    res.msg = "BRAIN_NODE: OK";
    return true;
  }

  res.ret = 0;
  res.msg = "BRAIN_NODE: Error sending command to MATLAB engine.";
  return false;
}

bool Brain::learnGrasp_GetMaxParams(
    brain_comm::brain_LearnGrasp_GetMaxParams::Request& req, 
    brain_comm::brain_LearnGrasp_GetMaxParams::Response& res)
{
  char command[MAX_BUFFER];
  char buffer[MAX_BUFFER];

  // Get the requested model, and find the current optimal parameters
  sprintf(command,"maxP = brain_learnGrasp_getMaxParams();");
  bool ok = matlab.sendCommand(command);

  //Recover parameters
  if(ok)
  {
    res.maxP.resize(learnGrasp_numParams);
    Vec maxP(learnGrasp_numParams);
    sprintf(buffer,"maxP");
    if(!matlab.getVec(buffer,maxP))
    {
      res.ret = 0;
      res.msg = "BRAIN_NODE: Error trying to recover max parameters.";
      return false;
    }

    for (int i=0; i< learnGrasp_numParams; i++)
      res.maxP[i] = maxP[i];

    res.ret = 1;
    res.msg = "BRAIN_NODE: OK";
    return true;
  }

  res.ret = 0;
  res.msg = "BRAIN_NODE: Error sending command to MATLAB engine.";
  return false;
}

bool Brain::learnGrasp_TrainParams(
    brain_comm::brain_LearnGrasp_TrainParams::Request& req, 
    brain_comm::brain_LearnGrasp_TrainParams::Response& res)
{
  char command[MAX_BUFFER];
  char buffer[MAX_BUFFER];
  char parameters[MAX_BUFFER];

  // Train the requested model

  // Format our string to hold our parameters
  parameters[0] = '\0';
  for (int i=0; i < learnGrasp_numParams; i++)
  {
    sprintf(buffer, " %f", req.params[i]); 
    strcat(parameters, buffer);
  }
  sprintf(command,"brain_learnGrasp_trainParams([%s], %d);", parameters, req.singulated);
  bool ok = matlab.sendCommand(command);

  // Make sure we got that data point
  if(ok)
  {
    res.ret = 1;
    res.msg = "BRAIN_NODE: OK";
    return true;
  }

  res.ret = 0;
  res.msg = "BRAIN_NODE: Error sending command to MATLAB engine.";
  return false;
}

bool Brain::learnGrasp_SaveModel(
    brain_comm::brain_LearnGrasp_SaveModel::Request& req, 
    brain_comm::brain_LearnGrasp_SaveModel::Response& res)
{
  char command[MAX_BUFFER];

  sprintf(command,"brain_learnGrasp_saveModel();");
  bool ok = matlab.sendCommand(command);

  // Make sure we saved our model
  if(ok)
  {
    res.ret = 1;
    res.msg = "BRAIN_NODE: OK";
    return true;
  }

  res.ret = 0;
  res.msg = "BRAIN_NODE: Error sending command to MATLAB engine.";
  return false;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "brain");
  ros::NodeHandle node;

  // Initialize the systemLog mutex
  pthread_mutex_init(&systemLogMutex, NULL);

  ros::Duration(3.0).sleep();
  // Initialize brain
  Brain brain(&node);
  if (!brain.init())
  {
    ROS_ERROR("BRAIN: Problem initializing brain.");
    ROS_ERROR("BRAIN: Shutting down node /brain.");
    exit(-1);
  }

  //Main ROS loop
  ROS_INFO("BRAIN: Running node /brain...");
  ros::spin();
  ROS_INFO("BRAIN: Shutting down node /brain...");

  //Destroy systemLog mutex
  pthread_mutex_destroy(&systemLogMutex);
}
