#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <ros/ros.h>
#include <matVec/matVec.h>
#include <brain_comm/brain_comm.h>
#include <matlab_comm/matlab_comm.h>
#include <grasp_comm/grasp_comm.h>
#include <sense_comm/sense_comm.h>
#include <util_comm/util_comm.h>
#include <logger_comm/logger_comm.h>


#define MAX_BUFFER 1024
#define N_GRASPS 200
#define GRASPS_PER_MODEL 10
#define MODELS_PER_GRASP 10
#define EXPERIMENT_NAME "earlyAbort"

#define LEARN_MODE false

int main(int argc, char** argv)
{
  char buffer[MAX_BUFFER];
  ros::init(argc, argv, "early_abort");
  ros::NodeHandle node;

  GraspComm grasper(&node);
  SenseComm sensor(&node);  
  UtilComm util(&node);
  
  BrainComm brain;
  MatlabComm matlab;
  LoggerComm logger; 
  brain.subscribe(&node);
  matlab.subscribe(&node);
  logger.subscribe(&node);

  GM_InputParams grasperInParams;
  GM_OutputParams grasperOutParams;

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
      ROS_INFO("EARLY ABORT: Hello, I will run %d grasps and learn",N_GRASPS);
      ROS_INFO("            an early abort model.");
      ROS_INFO("-------------------------------------------------");

      int singulated = 0;
  
      Vec error(round(N_GRASPS/GRASPS_PER_MODEL));
      int n=0;
      for (int i=1; (i<=N_GRASPS) && (ros::ok()); i++)
        {	
          // Grasp (grasp_OpenLoop)
	  grasperInParams.type = GM_TYPE_OPENLOOP;
	  grasperInParams.z_lim = 150.0;
	  grasperInParams.up = 36.0;//32.0;
	  grasperInParams.down = 36.0;//41.0;
	  grasperInParams.nFlips = 4;
          grasperInParams.oscillationAmplitude = 0.8683;//PI/3.0;
	  grasperInParams.handSpeed = 0.5;//0.45;

          //grasperInParams.x = 50.0;
          //grasperInParams.y = -50.0;
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
	      ROS_INFO("EARLY ABORT: Training grasp number %d: Singulated!!!! (S = %d - N = %d)",i,singulated,i-singulated);
 	    }
          else 
	      ROS_INFO("EARLY ABORT: Training grasp number %d: Not Singulated (S = %d - N = %d)",i,singulated,i-singulated);
   
          util.go_home();
          util.random_drop();

          // Learn Model (Once every GRASPS_PER_MODEL grasps)
     /*     if(i%GRASPS_PER_MODEL == 0)
	    {
	      sprintf(buffer,"%s%dmodel.mat",EXPERIMENT_NAME,i);
	      modelName = buffer;
	      brain.earlyAbort_TrainModel(logFolder, modelName, error[n]);
	      ROS_INFO("EARLY ABORT: Learned model %s.",modelName.c_str()); 
	      ROS_INFO("            Cross-validation error = %.2lf%%.",error[n]);
	      n++;
   	    }
	*/
        }
      finalError = error[n];
      finalModelName = modelName;
    }

  else
    { /*
      // Learn Model with all files in logFolder
      sprintf(buffer,"%sCompleteModel.mat",EXPERIMENT_NAME);
      finalModelName = buffer;
      brain.earlyAbort_TrainModel(logFolder, finalModelName, finalError);
      ROS_INFO("EARLY ABORT: Learned model %s.",finalModelName.c_str()); 
      //ROS_INFO("            Cross-validation error = %.2lf%%.",finalError);*/
    }
      sprintf(buffer,"%sCompleteModel.mat",EXPERIMENT_NAME);
      finalModelName = buffer;
      ROS_INFO("Here");
   //   brain.earlyAbort_TrainModel(logFolder, finalModelName, MODELS_PER_GRASP, finalError);
      ROS_INFO("EARLY ABORT: Learned model %s.",finalModelName.c_str()); 
  // Testing
  ROS_INFO("------------------------------------------------------");
  ROS_INFO("EARLY ABORT: Moving into testing.");
  ROS_INFO("------------------------------------------------------");

  while (ros::ok())
    {	
      // Grasp (grasp_OpenLoop)
      grasperInParams.type = GM_TYPE_OPENLOOP;
    //  grasperInParams.id = 0;
      grasperInParams.z_lim = 150.0;
      grasperInParams.up = 36.0;
      grasperInParams.down = 36.0;
      grasperInParams.nFlips = 4;
      grasperInParams.oscillationAmplitude = 0.8683;
      grasperInParams.handSpeed = 0.5;

      util.get_random_bin_pos(grasperInParams.x, grasperInParams.y);
      grasperOutParams = grasper.grasp(grasperInParams);

      // Sense (sensor_User)
      sensorInParams.type = SM_TYPE_ABORT;
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
  matlab.shutdown();
  logger.shutdown();
}
