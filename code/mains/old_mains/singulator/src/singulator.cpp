#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <ros/ros.h>
#include <matVec/matVec.h>
#include <brain_comm/brain_comm.h>
#include <grasp_comm/grasp_comm.h>
#include <sense_comm/sense_comm.h>
#include <util_comm/util_comm.h>
#include <logger_comm/logger_comm.h>


#define MAX_BUFFER 1024
#define N_GRASPS 1000
#define GRASPS_PER_MODEL 50
#define EXPERIMENT_NAME "singulation_new"//"singulation"

#define HAND_CAL_CNT 50
#define VIS_CAL_CNT 20

#define LEARN_MODE true

int main(int argc, char** argv)
{
  char buffer[MAX_BUFFER];
  ros::init(argc, argv, "singulator");
  ros::NodeHandle node;

  GraspComm grasper(&node);
  SenseComm sensor(&node);  
  UtilComm util(&node);

  BrainComm brain(&node);
  LoggerComm logger(&node); 

  GM_InputParams grasperInParams;
  GM_OutputParams grasperOutParams;

  grasperInParams.type = GM_TYPE_OPENLOOP;
  grasperInParams.z_lim = 150.0;
  grasperInParams.up = 36.0;//32.0;
  grasperInParams.down = 36.0;//41.0;
  grasperInParams.nFlips = 4;
  grasperInParams.oscillationAmplitude = 0.8683;//PI/3.0;
  grasperInParams.handSpeed = 0.5;//0.45;

  SM_InputParams sensorInParams;
  SM_OutputParams sensorOutParams;


  // Calibration
  util.calibrate_hand();
  util.calibrate_vision();

  std::string logFolder = EXPERIMENT_NAME;
  std::string finalModelName;
  double finalError;
  if(LEARN_MODE)
  {
    std::string modelName;	
    ROS_INFO("-------------------------------------------------");
    ROS_INFO("SINGULATOR: Hello, I will run %d grasps and learn",N_GRASPS);
    ROS_INFO("            a singulation detection model.");
    ROS_INFO("-------------------------------------------------");

    int singulated = 0;

    Vec error(round(N_GRASPS/GRASPS_PER_MODEL));
    int n=0;
    for (int i=1; (i<=N_GRASPS) && (ros::ok()); i++)
    {	
      if (i % HAND_CAL_CNT == 0)
        util.calibrate_hand();
      if (i % VIS_CAL_CNT == 0)
        util.calibrate_vision();
      
      // Grasp (grasp_OpenLoop)
      grasperInParams.id = i;
      util.get_random_bin_pos(grasperInParams.x, grasperInParams.y);
      grasperOutParams = grasper.grasp(grasperInParams);

      // Sense (sensor_User)
      sensorInParams.type = SM_TYPE_VISION;
      sensorInParams.logFileName = grasperOutParams.logFileName;
      sensorOutParams = sensor.sense(sensorInParams);

      // Save logFile to folder
      logger.Copy(sensorOutParams.logFileName, logFolder);

      // Check singulation
      if(sensorOutParams.singulated) 
      {
        singulated++;
        ROS_INFO("SINGULATOR: Train grasp number %d: Singulated!!!! (S = %d - N = %d)",i,singulated,i-singulated);
      }
      else 
        ROS_INFO("SINGULATOR: Train grasp number %d: Not Singulated (S = %d - N = %d)",i,singulated,i-singulated);

      util.go_home();
      if (sensorOutParams.nMarkers > 0)
        util.random_drop();

      // Learn Model (Once every GRASPS_PER_MODEL grasps)
      if(i%GRASPS_PER_MODEL == 0)
      {
        sprintf(buffer,"%s%dmodel.mat",EXPERIMENT_NAME,i);
        modelName = buffer;
        brain.singulationDetection_TrainModel(logFolder, modelName, error[n]);
        ROS_INFO("SINGULATOR: Learned model %s.",modelName.c_str()); 
        ROS_INFO("            Cross-validation error = %.2lf%%.",error[n]);
        n++;
      }
    }
    finalError = error[n];
    finalModelName = modelName;
  }

  else
  {
    // Learn Model with all files in logFolder
    sprintf(buffer,"%sCompleteModel.mat",EXPERIMENT_NAME);
    finalModelName = buffer;
    brain.singulationDetection_TrainModel(logFolder, finalModelName, finalError);
    ROS_INFO("SINGULATOR: Learned model %s.",finalModelName.c_str()); 
    ROS_INFO("            Cross-validation error = %.2lf%%.",finalError);
  }

  // Testing
  ROS_INFO("------------------------------------------------------");
  ROS_INFO("SINGULATOR: Moving into testing.");
  ROS_INFO("------------------------------------------------------");

  while (ros::ok())
  {	
    // Grasp (grasp_OpenLoop)
    util.get_random_bin_pos(grasperInParams.x, grasperInParams.y);
    grasperOutParams = grasper.grasp(grasperInParams);

    // Sense (sensor_User)
    sensorInParams.type = SM_TYPE_DATA;
    sensorInParams.logFileName = grasperOutParams.logFileName;
    sensorInParams.singulationModelName = finalModelName;
    sensorOutParams = sensor.sense(sensorInParams);

    // Check singulation
    if(sensorOutParams.singulated) 
      ROS_INFO("SENSOR: Model classifies grasp as singulated.");
    else 
      ROS_INFO("SENSOR: Model classifies grasp as not singulated.");

    // Drop marker in the same location where grasped 
    util.drop(grasperInParams.x, grasperInParams.y);
  }
  brain.shutdown();
  logger.shutdown();
}
