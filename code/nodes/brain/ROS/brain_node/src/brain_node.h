#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <engine.h>
#include <matrix.h>

//ROS specific
#include <ros/ros.h>
#include <matVec/matVec.h>
#include <logger_comm/logger_comm.h>
#include <matlab_comm/matlab_comm.h>
#include <brain_comm/brain_comm.h>

#define MAX_BUFFER 1024
#define N_TIMESTAMPS_TO_RESERVE 100 

//Folder names
#define SINGULATION_DETECTION "singulationDetection"
#define EARLY_ABORT "earlyAbort"
#define MARKER_POS_ESTIMATION "markerPosEstimation"
#define LEARN_GRASP "learnGrasp"

pthread_mutex_t systemLogMutex;                         //Mutex to read and write the systemLog



class Brain
{
 public:
  Brain(ros::NodeHandle*n);
  virtual ~Brain();

  ros::NodeHandle *nodePtr;                             //Pointer to ROS master.
  MatlabComm matlab;                                    //Client class interface to Matlab node.
  logger_comm::logger_SystemLog systemLog;              //Last systemLog published to the network
  bool logging;  					//Is the logger logging to file?
  bool logger_SystemLog_present;                        //Is the systemLog present?
  std::vector<logger_comm::logger_SystemLog> systemLogVector; //Last array of logs when logging to file.
  ros::Timer scannerTimer;                              //Timer to do periodic scans to search for logger
 
  // Initialize
  bool init();
  void advertiseServices();
  void scanAndSubscribeTopics();
  void logger_SystemLog_Callback(const logger_comm::logger_SystemLog& msg);
  void scannerCallBack(const ros::TimerEvent& event);
  
  ////////////////////
  // Service Functions
  // (remember to add the corresponding service handles)
  // 1 - Singulation Detection
  bool singulationDetection_TrainModel(
      brain_comm::brain_SingulationDetection_TrainModel::Request& req, 
      brain_comm::brain_SingulationDetection_TrainModel::Response& res);
  bool singulationDetection_LoadModel(
      brain_comm::brain_SingulationDetection_LoadModel::Request& req, 
      brain_comm::brain_SingulationDetection_LoadModel::Response& res);
  bool singulationDetection_Test(
      brain_comm::brain_SingulationDetection_Test::Request& req, 
      brain_comm::brain_SingulationDetection_Test::Response& res);

  bool earlyAbort_TrainModel(
      brain_comm::brain_earlyAbort_TrainModel::Request& req, 
      brain_comm::brain_earlyAbort_TrainModel::Response& res);
  bool earlyAbort_LoadModel(
      brain_comm::brain_earlyAbort_LoadModel::Request& req, 
      brain_comm::brain_earlyAbort_LoadModel::Response& res);
  bool earlyAbort_Test(
      brain_comm::brain_earlyAbort_Test::Request& req, 
      brain_comm::brain_earlyAbort_Test::Response& res);

  bool markerPosEstimation_TrainModel(
      brain_comm::brain_MarkerPosEstimation_TrainModel::Request& req, 
      brain_comm::brain_MarkerPosEstimation_TrainModel::Response& res);
  bool markerPosEstimation_LoadModel(
      brain_comm::brain_MarkerPosEstimation_LoadModel::Request& req, 
      brain_comm::brain_MarkerPosEstimation_LoadModel::Response& res);
  bool markerPosEstimation_Test(
      brain_comm::brain_MarkerPosEstimation_Test::Request& req, 
      brain_comm::brain_MarkerPosEstimation_Test::Response& res);

  bool learnGrasp_LoadModel(
      brain_comm::brain_LearnGrasp_LoadModel::Request& req, 
      brain_comm::brain_LearnGrasp_LoadModel::Response& res);
  bool learnGrasp_GetNextParams(
      brain_comm::brain_LearnGrasp_GetNextParams::Request& req, 
      brain_comm::brain_LearnGrasp_GetNextParams::Response& res);
  bool learnGrasp_GetMaxParams(
      brain_comm::brain_LearnGrasp_GetMaxParams::Request& req, 
      brain_comm::brain_LearnGrasp_GetMaxParams::Response& res);
  bool learnGrasp_TrainParams(
      brain_comm::brain_LearnGrasp_TrainParams::Request& req, 
      brain_comm::brain_LearnGrasp_TrainParams::Response& res);
  bool learnGrasp_SaveModel(
      brain_comm::brain_LearnGrasp_SaveModel::Request& req, 
      brain_comm::brain_LearnGrasp_SaveModel::Response& res);

  //ROS handles
  ros::ServiceServer handle_brain_SingulationDetection_TrainModel;
  ros::ServiceServer handle_brain_SingulationDetection_LoadModel;
  ros::ServiceServer handle_brain_SingulationDetection_Test;

  ros::ServiceServer handle_brain_earlyAbort_TrainModel;
  ros::ServiceServer handle_brain_earlyAbort_LoadModel;
  ros::ServiceServer handle_brain_earlyAbort_Test;

  ros::ServiceServer handle_brain_MarkerPosEstimation_TrainModel;
  ros::ServiceServer handle_brain_MarkerPosEstimation_LoadModel;
  ros::ServiceServer handle_brain_MarkerPosEstimation_Test;

  ros::ServiceServer handle_brain_LearnGrasp_LoadModel;
  ros::ServiceServer handle_brain_LearnGrasp_GetNextParams;
  ros::ServiceServer handle_brain_LearnGrasp_GetMaxParams;
  ros::ServiceServer handle_brain_LearnGrasp_TrainParams;
  ros::ServiceServer handle_brain_LearnGrasp_SaveModel;

  ros::Subscriber handle_logger_SystemLog;

 private:
  int learnGrasp_numParams;
};
