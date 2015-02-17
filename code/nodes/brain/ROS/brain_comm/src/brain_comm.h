#ifndef BRAIN_COMM_H
#define BRAIN_COMM_H

#define MAX_BUFFER 1024

#include <ros/ros.h>
#include <matVec/matVec.h>
#include <vector>

//Singulation Detecion
#include <brain_comm/brain_SingulationDetection_TrainModel.h>
#include <brain_comm/brain_SingulationDetection_LoadModel.h>
#include <brain_comm/brain_SingulationDetection_Test.h>

//early abort
#include <brain_comm/brain_earlyAbort_TrainModel.h>
#include <brain_comm/brain_earlyAbort_LoadModel.h>
#include <brain_comm/brain_earlyAbort_Test.h>

#include <brain_comm/brain_MarkerPosEstimation_TrainModel.h>
#include <brain_comm/brain_MarkerPosEstimation_LoadModel.h>
#include <brain_comm/brain_MarkerPosEstimation_Test.h>

#include <brain_comm/brain_LearnGrasp_LoadModel.h>
#include <brain_comm/brain_LearnGrasp_GetNextParams.h>
#include <brain_comm/brain_LearnGrasp_GetMaxParams.h>
#include <brain_comm/brain_LearnGrasp_TrainParams.h>
#include <brain_comm/brain_LearnGrasp_SaveModel.h>

class BrainComm
{
  public:
    BrainComm();
    BrainComm(ros::NodeHandle* np);
    ~BrainComm();

    // Subscription
    void subscribe(ros::NodeHandle* np);

    // Call this before program exits so we don't have double freeing issues
    void shutdown();
    
    // Client functions to simplify calling learning routines
    bool singulationDetection_TrainModel(std::string logFolderName, std::string modelName, double &cvError);
    bool singulationDetection_LoadModel(std::string modelName);
    bool singulationDetection_Test(std::string logFileName, int& label);
   
    bool earlyAbort_TrainModel(std::string logFolderName, std::string modelName, int nSlices, double &cvError);
    bool earlyAbort_LoadModel(std::string modelName);
    bool earlyAbort_Test(std::string logFileName, int& label);

    bool markerPosEstimation_TrainModel(std::string logFolderName, 
        std::string modelName, double &nll_theta, double &nll_r);
    bool markerPosEstimation_LoadModel(std::string modelName);
    bool markerPosEstimation_Test(std::string logFileName, 
        double &theta, double &r, double &theta_s, double &r_s);
    
    bool learnGrasp_LoadModel(std::string modelName, 
        std::vector<double> mins, std::vector<double> maxs, int grid);
    bool learnGrasp_GetNextParams(std::vector<double> &nextP);
    bool learnGrasp_GetMaxParams(std::vector<double> &maxP);
    bool learnGrasp_TrainParams(std::vector<double> params, bool singulated);
    bool learnGrasp_SaveModel();

 private:
    // ROS Service Clients
    ros::ServiceClient handle_brain_SingulationDetection_TrainModel;
    ros::ServiceClient handle_brain_SingulationDetection_LoadModel;
    ros::ServiceClient handle_brain_SingulationDetection_Test;

    ros::ServiceClient handle_brain_earlyAbort_TrainModel;
    ros::ServiceClient handle_brain_earlyAbort_LoadModel;
    ros::ServiceClient handle_brain_earlyAbort_Test;

    ros::ServiceClient handle_brain_MarkerPosEstimation_TrainModel;
    ros::ServiceClient handle_brain_MarkerPosEstimation_LoadModel;
    ros::ServiceClient handle_brain_MarkerPosEstimation_Test;

    ros::ServiceClient handle_brain_LearnGrasp_LoadModel;
    ros::ServiceClient handle_brain_LearnGrasp_GetNextParams;
    ros::ServiceClient handle_brain_LearnGrasp_GetMaxParams;
    ros::ServiceClient handle_brain_LearnGrasp_TrainParams;
    ros::ServiceClient handle_brain_LearnGrasp_SaveModel;

    // ROS services
    brain_comm::brain_SingulationDetection_TrainModel brain_SingulationDetection_TrainModel_srv;
    brain_comm::brain_SingulationDetection_LoadModel brain_SingulationDetection_LoadModel_srv;
    brain_comm::brain_SingulationDetection_Test brain_SingulationDetection_Test_srv;

    brain_comm::brain_earlyAbort_TrainModel brain_earlyAbort_TrainModel_srv;
    brain_comm::brain_earlyAbort_LoadModel brain_earlyAbort_LoadModel_srv;
    brain_comm::brain_earlyAbort_Test brain_earlyAbort_Test_srv;

    brain_comm::brain_MarkerPosEstimation_TrainModel brain_MarkerPosEstimation_TrainModel_srv;
    brain_comm::brain_MarkerPosEstimation_LoadModel brain_MarkerPosEstimation_LoadModel_srv;
    brain_comm::brain_MarkerPosEstimation_Test brain_MarkerPosEstimation_Test_srv;

    brain_comm::brain_LearnGrasp_LoadModel brain_LearnGrasp_LoadModel_srv;
    brain_comm::brain_LearnGrasp_GetNextParams brain_LearnGrasp_GetNextParams_srv;
    brain_comm::brain_LearnGrasp_GetMaxParams brain_LearnGrasp_GetMaxParams_srv;
    brain_comm::brain_LearnGrasp_TrainParams brain_LearnGrasp_TrainParams_srv;
    brain_comm::brain_LearnGrasp_SaveModel brain_LearnGrasp_SaveModel_srv;
};

#endif //BRAIN_COMM_H
