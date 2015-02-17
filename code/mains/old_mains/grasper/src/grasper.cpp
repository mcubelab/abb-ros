#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <ros/ros.h>
#include <matVec/matVec.h>
#include <brain_comm/brain_comm.h>
#include <grasp_comm/grasp_comm.h>
#include <sense_comm/sense_comm.h>
#include <util_comm/util_comm.h>

#define EXPERIMENT_NAME "singulation2"

#define MAX_BUFFER 1024

#define N_GRASPS 100

#define VIS_CAL_CNT 10
#define HAND_CAL_CNT 100

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cyclotron");
  ros::NodeHandle node;
  GraspComm grasper(&node);
  SenseComm sensor(&node);  
  UtilComm util(&node);
  BrainComm brain(&node);

  GM_InputParams grasperInParams;
  GM_OutputParams grasperOutParams;

  grasperInParams.type = GM_TYPE_OPENLOOP;
  grasperInParams.z_lim = 150.0;
  grasperInParams.up = 36.0;//32.0;
  grasperInParams.down = 36.0;//41.0;
  grasperInParams.nFlips = 4;
  grasperInParams.oscillationAmplitude = 0.8683;//PI/3.0;
  grasperInParams.handSpeed = 0.5; //0.45;

  SM_InputParams sensorInParams;
  SM_OutputParams sensorOutParams;

  //sensor_User
  char buffer[MAX_BUFFER];
  std::string logFolder = EXPERIMENT_NAME;
  std::string finalModelName;
  double finalError;

  ROS_INFO("---------------------------------------------------------------");
  ROS_INFO("Hello! I will be running a rudimentary early abort");
  ROS_INFO("algorithm, which will grasp a marker, and then look");
  ROS_INFO("at the output from a classifier to see if it has a single");
  ROS_INFO("marker or not. If so, we're done, if not, it drops the marker");
  ROS_INFO("and tries again! Average time it takes is computed");
  ROS_INFO("-------------------------------------------------------------\n");
  ROS_INFO("Building classifier...");

  // Learn Model with all files in logFolder
  sprintf(buffer,"%sCompleteModel.mat",EXPERIMENT_NAME);
  finalModelName = buffer;
  brain.singulationDetection_TrainModel(logFolder, finalModelName, finalError);
  ROS_INFO("SINGULATOR: Learned model %s.",finalModelName.c_str()); 
  ROS_INFO("            Cross-validation error = %.2lf%%.",finalError);

  sensorInParams.singulationModelName = finalModelName;

  util.go_home();

  double average = 0;
  double begTime;
  double diffTime;

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

    util.go_vision();

    diffTime = ros::Time::now().toSec() - begTime;
    average = (average * i + diffTime)/(i+1);

    ROS_INFO("Singulated marker attained in %2.2f sec. Average: %2.2f sec", diffTime, average);

    util.go_home();
    util.random_drop();
  }
}
