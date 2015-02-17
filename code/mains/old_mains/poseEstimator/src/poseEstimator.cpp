#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <ros/ros.h>
#include <matVec/matVec.h>
#include <brain_comm/brain_comm.h>
#include <grasp_comm/grasp_comm.h>
#include <sense_comm/sense_comm.h>
#include <place_comm/place_comm.h>
#include <util_comm/util_comm.h>

#define EXPERIMENT_NAME "singulation_new"

#define MAX_BUFFER 1024

#define N_GRASPS 100

#define VIS_CAL_CNT 10
#define HAND_CAL_CNT 100

int main(int argc, char** argv)
{
  ros::init(argc, argv, "poseEstimator");
  ros::NodeHandle node;
  GraspComm grasper(&node);
  SenseComm sensor(&node);  
  PlaceComm placer(&node);
  UtilComm util(&node);
  BrainComm brain(&node);

  GM_InputParams grasperInParams;
  GM_OutputParams grasperOutParams;

  grasperInParams.type = GM_TYPE_OPENLOOP;
  grasperInParams.z_lim = 150.0;
  grasperInParams.up = 36.0;
  grasperInParams.down = 36.0;
  grasperInParams.nFlips = 4;
  grasperInParams.oscillationAmplitude = 0.8683;
  grasperInParams.handSpeed = 0.5;

  SM_InputParams sensorInParams;
  SM_OutputParams sensorOutParams;

  //sensor_User
  char buffer[MAX_BUFFER];
  std::string logFolder = EXPERIMENT_NAME;
  std::string singulationModelName;
  std::string poseEstimationModelName;

  ROS_INFO("---------------------------------------------------------------");
  ROS_INFO("Hello! I will be running a rudimentary early abort");
  ROS_INFO("algorithm, which will grasp a marker, and then look");
  ROS_INFO("at the output from a classifier to see if it has a single");
  ROS_INFO("marker or not. If so, we're done, if not, it drops the marker");
  ROS_INFO("and tries again! Average time it takes is computed");
  ROS_INFO("-------------------------------------------------------------\n");
  //ROS_INFO("Building classifier...");

  // Learn Model with all files in logFolder
  double finalError;
  sprintf(buffer,"%sSingulationModel.mat",EXPERIMENT_NAME);
  singulationModelName = buffer;
  brain.singulationDetection_TrainModel(logFolder, singulationModelName, finalError);
  ROS_INFO("SINGULATOR: Learned model %s.",singulationModelName.c_str()); 
  ROS_INFO("            Cross-validation error = %.2lf%%.",finalError);

  // Learn Pose Estimation Model with all files in logFolder
  sprintf(buffer,"%sPoseEstimationModel.mat",EXPERIMENT_NAME);
  poseEstimationModelName = buffer;
  double nll_theta, nll_r;
  brain.markerPosEstimation_TrainModel(logFolder, poseEstimationModelName, nll_theta, nll_r);
  ROS_INFO("POSE_ESTIMATOR: Learned model %s.",poseEstimationModelName.c_str()); 
  ROS_INFO("            Negative Log Likelihoods: Theta: %f, distance: %f.",nll_theta, nll_r);
  
  sensorInParams.singulationModelName = singulationModelName;
  sensorInParams.poseEstimationModelName = poseEstimationModelName;

  util.go_home();

  double average = 0;
  double begTime;
  double diffTime;
  double theta, r;

  for (int i=0; i < N_GRASPS && ros::ok(); i++)
  {	
    if (i % HAND_CAL_CNT == 0)
    {
      util.calibrate_hand();
    }
    if (i % VIS_CAL_CNT == 0)
    {
      util.calibrate_vision();
    }

    begTime = ros::Time::now().toSec();
    while (true)
    {
      util.get_random_bin_pos(grasperInParams.x, grasperInParams.y);

      grasperOutParams = grasper.grasp(grasperInParams);

      sensorInParams.type = SM_TYPE_DATA;
      sensorInParams.logFileName = grasperOutParams.logFileName;
      sensorOutParams = sensor.sense(sensorInParams);

      if (sensorOutParams.singulated)
        break;
      else
        util.drop(grasperInParams.x, grasperInParams.y);
    }

    // Remember the angle and distance from center
    theta = sensorOutParams.theta;
    r = sensorOutParams.distance;

    // Now show it to vision and check our answer
    sensorInParams.type = SM_TYPE_VISION;
    sensorOutParams = sensor.sense(sensorInParams);

    // Print out results
    ROS_INFO("Sensing Results:");
    ROS_INFO("--------------------------------------------------------------");
    ROS_INFO("Data: nMarkers: 1, theta: %f, r: %f", theta, r);
    ROS_INFO("Vision: nMarkers: %d, theta: %f, r: %f", sensorOutParams.nMarkers, 
        sensorOutParams.theta, sensorOutParams.distance);
    ROS_INFO("--------------------------------------------------------------");

    // Compute how long it took to get a singulated marker
    diffTime = ros::Time::now().toSec() - begTime;
    average = (average * i + diffTime)/(i+1);
    ROS_INFO("Singulated marker attained in %2.2f sec. Average: %2.2f sec", diffTime, average);

    // Now drop off the marker and start again!
    util.go_home();
    util.random_drop();
  }

  return 0;
}
