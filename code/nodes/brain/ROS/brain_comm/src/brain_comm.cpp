#include "brain_comm.h"

BrainComm::BrainComm()
{
}

BrainComm::BrainComm(ros::NodeHandle* np)
{
  subscribe(np);
}

BrainComm::~BrainComm()
{
  shutdown();
}

void BrainComm::subscribe(ros::NodeHandle* np)
{
  handle_brain_SingulationDetection_TrainModel = 
    np->serviceClient<brain_comm::brain_SingulationDetection_TrainModel>("brain_SingulationDetection_TrainModel");
  handle_brain_SingulationDetection_LoadModel = 
    np->serviceClient<brain_comm::brain_SingulationDetection_LoadModel>("brain_SingulationDetection_LoadModel");
  handle_brain_SingulationDetection_Test = 
    np->serviceClient<brain_comm::brain_SingulationDetection_Test>("brain_SingulationDetection_Test");

  handle_brain_MarkerPosEstimation_TrainModel = 
    np->serviceClient<brain_comm::brain_MarkerPosEstimation_TrainModel>("brain_MarkerPosEstimation_TrainModel");
  handle_brain_MarkerPosEstimation_LoadModel = 
    np->serviceClient<brain_comm::brain_MarkerPosEstimation_LoadModel>("brain_MarkerPosEstimation_LoadModel");
  handle_brain_MarkerPosEstimation_Test = 
    np->serviceClient<brain_comm::brain_MarkerPosEstimation_Test>("brain_MarkerPosEstimation_Test");

  handle_brain_earlyAbort_TrainModel = 
    np->serviceClient<brain_comm::brain_earlyAbort_TrainModel>("brain_earlyAbort_TrainModel");
  handle_brain_earlyAbort_LoadModel = 
    np->serviceClient<brain_comm::brain_earlyAbort_LoadModel>("brain_earlyAbort_LoadModel");
  handle_brain_earlyAbort_Test = 
    np->serviceClient<brain_comm::brain_earlyAbort_Test>("brain_earlyAbort_Test");

  handle_brain_LearnGrasp_LoadModel = 
    np->serviceClient<brain_comm::brain_LearnGrasp_LoadModel>("brain_LearnGrasp_LoadModel");
  handle_brain_LearnGrasp_GetNextParams = 
    np->serviceClient<brain_comm::brain_LearnGrasp_GetNextParams>("brain_LearnGrasp_GetNextParams");
  handle_brain_LearnGrasp_GetMaxParams = 
    np->serviceClient<brain_comm::brain_LearnGrasp_GetMaxParams>("brain_LearnGrasp_GetMaxParams");
  handle_brain_LearnGrasp_TrainParams = 
    np->serviceClient<brain_comm::brain_LearnGrasp_TrainParams>("brain_LearnGrasp_TrainParams");
  handle_brain_LearnGrasp_SaveModel = 
    np->serviceClient<brain_comm::brain_LearnGrasp_SaveModel>("brain_LearnGrasp_SaveModel");
}

void BrainComm::shutdown()
{
  handle_brain_SingulationDetection_TrainModel.shutdown();
  handle_brain_SingulationDetection_LoadModel.shutdown();
  handle_brain_SingulationDetection_Test.shutdown();

  handle_brain_MarkerPosEstimation_TrainModel.shutdown();
  handle_brain_MarkerPosEstimation_LoadModel.shutdown();
  handle_brain_MarkerPosEstimation_Test.shutdown();

  handle_brain_earlyAbort_TrainModel.shutdown();
  handle_brain_earlyAbort_LoadModel.shutdown();
  handle_brain_earlyAbort_Test.shutdown();

  handle_brain_LearnGrasp_LoadModel.shutdown();
  handle_brain_LearnGrasp_GetNextParams.shutdown();
  handle_brain_LearnGrasp_GetMaxParams.shutdown();
  handle_brain_LearnGrasp_TrainParams.shutdown();
  handle_brain_LearnGrasp_SaveModel.shutdown();
}

//Client functions to simplify calling Learning routines
//Singulation Detection
bool BrainComm::singulationDetection_TrainModel(std::string logFolderName, std::string modelName, double &cvError)
{
  brain_SingulationDetection_TrainModel_srv.request.logFolderName = logFolderName;
  brain_SingulationDetection_TrainModel_srv.request.modelName = modelName; 
  if(handle_brain_SingulationDetection_TrainModel.call(brain_SingulationDetection_TrainModel_srv))
    {
      cvError = brain_SingulationDetection_TrainModel_srv.response.error;
      return true;
    }
  ROS_ERROR("BRAIN_COMM: Error trainning a singulation detection model.");
  ROS_ERROR("%s",brain_SingulationDetection_TrainModel_srv.response.msg.c_str());
  return false;
}

bool BrainComm::singulationDetection_LoadModel(std::string modelName)
{
  brain_SingulationDetection_LoadModel_srv.request.modelName = modelName;
  if(handle_brain_SingulationDetection_LoadModel.call(brain_SingulationDetection_LoadModel_srv))
      return true;
  ROS_ERROR("BRAIN_COMM: Error loading a singulation detection model.");
  ROS_ERROR("%s",brain_SingulationDetection_LoadModel_srv.response.msg.c_str());
  return false;
}

bool BrainComm::singulationDetection_Test(std::string logFileName, int &label)
{
  brain_SingulationDetection_Test_srv.request.logFileName = logFileName;
  if(handle_brain_SingulationDetection_Test.call(brain_SingulationDetection_Test_srv))
    {
      label = brain_SingulationDetection_Test_srv.response.label;
      return true;
    }
  ROS_ERROR("BRAIN_COMM: Error testing logFile %s against a singulation detection model.",logFileName.c_str());
  ROS_ERROR("%s",brain_SingulationDetection_Test_srv.response.msg.c_str());
  return false;
}


//Client functions to simplify calling Learning routines
//Early Abort
bool BrainComm::earlyAbort_TrainModel(std::string logFolderName, std::string modelName, int nSlices, double &cvError)
{
  ROS_INFO("Here 2");
  brain_earlyAbort_TrainModel_srv.request.logFolderName = logFolderName;
  brain_earlyAbort_TrainModel_srv.request.modelName = modelName; 
  brain_earlyAbort_TrainModel_srv.request.nSlices = nSlices; 
  if(handle_brain_earlyAbort_TrainModel.call(brain_earlyAbort_TrainModel_srv))
    {
      cvError = brain_earlyAbort_TrainModel_srv.response.error;
      return true;
    }
  ROS_ERROR("BRAIN_COMM: Error training an early abort model.");
  ROS_ERROR("%s",brain_earlyAbort_TrainModel_srv.response.msg.c_str());
  return false;
}

bool BrainComm::earlyAbort_LoadModel(std::string modelName)
{
  brain_earlyAbort_LoadModel_srv.request.modelName = modelName;
  if(handle_brain_earlyAbort_LoadModel.call(brain_earlyAbort_LoadModel_srv))
      return true;
  ROS_ERROR("BRAIN_COMM: Error loading a singulation detection model.");
  ROS_ERROR("%s",brain_earlyAbort_LoadModel_srv.response.msg.c_str());
  return false;
}

bool BrainComm::earlyAbort_Test(std::string logFileName, int &label)
{
  brain_earlyAbort_Test_srv.request.logFileName = logFileName;
  if(handle_brain_earlyAbort_Test.call(brain_earlyAbort_Test_srv))
    {
      label = brain_earlyAbort_Test_srv.response.label;
      return true;
    }
  ROS_ERROR("BRAIN_COMM: Error testing logFile %s against a singulation detection model.",logFileName.c_str());
  ROS_ERROR("%s",brain_earlyAbort_Test_srv.response.msg.c_str());
  return false;
}

bool BrainComm::markerPosEstimation_TrainModel(std::string logFolderName, 
    std::string modelName, double &theta_nll, double &r_nll)
{
  brain_MarkerPosEstimation_TrainModel_srv.request.logFolderName = logFolderName;
  brain_MarkerPosEstimation_TrainModel_srv.request.modelName = modelName; 
  if(handle_brain_MarkerPosEstimation_TrainModel.call(brain_MarkerPosEstimation_TrainModel_srv))
    {
      theta_nll = brain_MarkerPosEstimation_TrainModel_srv.response.theta_nll;
      r_nll = brain_MarkerPosEstimation_TrainModel_srv.response.r_nll;
      return true;
    }
  ROS_ERROR("BRAIN_COMM: Error training a marker pose estimation model.");
  ROS_ERROR("%s",brain_MarkerPosEstimation_TrainModel_srv.response.msg.c_str());
  return false;
}

bool BrainComm::markerPosEstimation_LoadModel(std::string modelName)
{
  brain_MarkerPosEstimation_LoadModel_srv.request.modelName = modelName;
  if(handle_brain_MarkerPosEstimation_LoadModel.call(brain_MarkerPosEstimation_LoadModel_srv))
      return true;
  ROS_ERROR("BRAIN_COMM: Error loading a Marker Pose Estimation model.");
  ROS_ERROR("%s",brain_MarkerPosEstimation_LoadModel_srv.response.msg.c_str());
  return false;
}

bool BrainComm::markerPosEstimation_Test(std::string logFileName, 
    double &theta, double &r, double &theta_s, double &r_s)
{
  brain_MarkerPosEstimation_Test_srv.request.logFileName = logFileName;
  if(handle_brain_MarkerPosEstimation_Test.call(brain_MarkerPosEstimation_Test_srv))
    {
      theta = brain_MarkerPosEstimation_Test_srv.response.theta;
      theta_s = brain_MarkerPosEstimation_Test_srv.response.theta_s;
      r = brain_MarkerPosEstimation_Test_srv.response.r;
      r_s = brain_MarkerPosEstimation_Test_srv.response.r_s;
      return true;
    }
  ROS_ERROR("BRAIN_COMM: Error testing logFile %s against a marker pose estimation model.",logFileName.c_str());
  ROS_ERROR("%s",brain_MarkerPosEstimation_Test_srv.response.msg.c_str());
  return false;
}



bool BrainComm::learnGrasp_LoadModel(std::string modelName, 
    std::vector<double> mins, std::vector<double> maxs, int grid)
{
  brain_LearnGrasp_LoadModel_srv.request.modelName = modelName;
  brain_LearnGrasp_LoadModel_srv.request.mins = mins;
  brain_LearnGrasp_LoadModel_srv.request.maxs = maxs;
  brain_LearnGrasp_LoadModel_srv.request.grid = grid;
  if(handle_brain_LearnGrasp_LoadModel.call(brain_LearnGrasp_LoadModel_srv))
    {
      return true;
    }
  ROS_ERROR("BRAIN_COMM: Error loading learning to grasp model: %s", modelName.c_str());
  ROS_ERROR("%s",brain_LearnGrasp_SaveModel_srv.response.msg.c_str());
  return false;
}

bool BrainComm::learnGrasp_GetNextParams(std::vector<double> &nextP)
{
  if(handle_brain_LearnGrasp_GetNextParams.call(brain_LearnGrasp_GetNextParams_srv))
    {
      nextP = brain_LearnGrasp_GetNextParams_srv.response.nextP;
      return true;
    }
  ROS_ERROR("BRAIN_COMM: Error getting parameters for learning to grasp");
  ROS_ERROR("%s",brain_LearnGrasp_GetNextParams_srv.response.msg.c_str());
  return false;
}

bool BrainComm::learnGrasp_GetMaxParams(std::vector<double> &nextP)
{
  if(handle_brain_LearnGrasp_GetMaxParams.call(brain_LearnGrasp_GetMaxParams_srv))
    {
      nextP = brain_LearnGrasp_GetMaxParams_srv.response.maxP;
      return true;
    }
  ROS_ERROR("BRAIN_COMM: Error getting parameters for learning to grasp");
  ROS_ERROR("%s",brain_LearnGrasp_GetMaxParams_srv.response.msg.c_str());
  return false;
}

bool BrainComm::learnGrasp_TrainParams(std::vector<double> params, bool singulated)
{
  brain_LearnGrasp_TrainParams_srv.request.params = params;
  brain_LearnGrasp_TrainParams_srv.request.singulated = singulated;
  if(handle_brain_LearnGrasp_TrainParams.call(brain_LearnGrasp_TrainParams_srv))
    {
      return true;
    }
  ROS_ERROR("BRAIN_COMM: Error training parameters for learning to grasp");
  ROS_ERROR("%s",brain_LearnGrasp_TrainParams_srv.response.msg.c_str());
  return false;
}

bool BrainComm::learnGrasp_SaveModel()
{
  if(handle_brain_LearnGrasp_SaveModel.call(brain_LearnGrasp_SaveModel_srv))
    {
      return true;
    }
  ROS_ERROR("BRAIN_COMM: Error saving learning to grasp model.");
  ROS_ERROR("%s",brain_LearnGrasp_SaveModel_srv.response.msg.c_str());
  return false;
}
