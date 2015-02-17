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
#include <logger_comm/logger_comm.h>
#include <matlab_comm/matlab_comm.h>


#define MAX_BUFFER 1024
#define N_GRASPS 1000
#define SINGULATION_FOLDER_NAME "singulation_new"//"singulation"
#define PLACING_FOLDER_NAME "placing_noise_validation"

#define HAND_CAL_CNT 50
#define VIS_CAL_CNT 20

#define TRAIN_CLASSIFIER false
#define TRAINING false

// Standard deviations for spread of angle and center distance
#define STD_THETA 0.25//0.15 // (rad)
#define STD_DIST  20.0//5.0 // (mm)

int main(int argc, char** argv)
{
  char buffer[MAX_BUFFER];
  ros::init(argc, argv, "placer");
  ros::NodeHandle node;

  GraspComm grasper(&node);
  SenseComm sensor(&node);  
  PlaceComm placer(&node);  
  UtilComm util(&node);

  BrainComm brain(&node);
  MatlabComm matlab(&node);
  LoggerComm logger(&node); 

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

  PM_InputParams placerInParams;
  PM_OutputParams placerOutParams;

  placerInParams.type = PM_TYPE_OPENLOOP;
  placerInParams.v1_dist = 70.0;
  placerInParams.v2_dist = 60.0;

  std::string logFolder = SINGULATION_FOLDER_NAME;
  /*
  std::string finalModelName;
  double finalError;
  sprintf(buffer,"%sCompleteModel.mat",SINGULATION_FOLDER_NAME);
  finalModelName = buffer;
  if (TRAIN_CLASSIFIER)
  {
    // Learn Model with all files in logFolder
    brain.singulationDetection_TrainModel(logFolder, finalModelName, finalError);
    ROS_INFO("PLACER: Learned singulation model %s.",finalModelName.c_str()); 
    ROS_INFO("            Cross-validation error = %.2lf%%.",finalError);
  }
  */

  if (TRAINING)
  {
    for (int i=0; i < N_GRASPS && ros::ok(); i++)
    {
      // Calibration
      if (i % HAND_CAL_CNT == 0)
        util.calibrate_hand();
      if (i % VIS_CAL_CNT == 0)
        util.calibrate_vision();
      // Grasp (grasp_OpenLoop)
      util.get_random_bin_pos(grasperInParams.x, grasperInParams.y);
      grasperOutParams = grasper.grasp(grasperInParams);

      // Sense (sensor_User)
      sensorInParams.type = SM_TYPE_VISION;
      sensorInParams.logFileName = grasperOutParams.logFileName;
      sensorOutParams = sensor.sense(sensorInParams);
      logger.Copy(sensorOutParams.logFileName, logFolder);


      if (sensorOutParams.singulated)
      {
        double real_angle = sensorOutParams.theta;
        double real_dist = sensorOutParams.distance;

        Vec rand_nums;
        matlab.sendCommand("rand_nums = rand([1 2]);");
        matlab.getVec("rand_nums", rand_nums);

        double d_theta = 2*(rand_nums[0] - 0.5) * STD_THETA;
        double d_dist = 2*(rand_nums[1] - 0.5) * STD_DIST;

        placerInParams.angle = real_angle + d_theta;
        placerInParams.h_dist = real_dist + d_dist;

        placerOutParams = placer.place(placerInParams);

        if (placerOutParams.success)
        {
          ROS_INFO("Successfully placed marker!");
        }
        else
        {
          ROS_INFO("Placing Unsuccessful.");
        }

        sprintf(buffer, "#Q,%d,%f,%f,%f,%f,%s", placerOutParams.success, real_angle, real_dist, d_theta, d_dist, sensorInParams.logFileName.c_str());
        std::string str = buffer;
        logger.Append(placerOutParams.logFileName, str);
        logger.Copy(placerOutParams.logFileName, PLACING_FOLDER_NAME);
        util.go_home();
      }
      else
      {
        util.go_home();
        ROS_INFO("num markers: %d", sensorOutParams.nMarkers);
        if (sensorOutParams.nMarkers > 0)
        {
          util.random_drop();
        }
      }
    }
  }
  else // testing
  {
    // First, load the marker pose estimation model
    sprintf(buffer,"%sPoseEstimationModel.mat",SINGULATION_FOLDER_NAME);
    std::string poseEstimationModelName = buffer;
    brain.markerPosEstimation_LoadModel(poseEstimationModelName);

    for (int i=0; i < N_GRASPS && ros::ok(); i++)
    {
      // Calibration
      if (i % HAND_CAL_CNT == 0)
        util.calibrate_hand();
      if (i % VIS_CAL_CNT == 0)
        util.calibrate_vision();
      // Grasp (grasp_OpenLoop)
      util.get_random_bin_pos(grasperInParams.x, grasperInParams.y);
      grasperOutParams = grasper.grasp(grasperInParams);

      // Sense (sensor_vision)
      sensorInParams.type = SM_TYPE_VISION;
      sensorInParams.logFileName = grasperOutParams.logFileName;
      sensorOutParams = sensor.sense(sensorInParams);
      logger.Copy(sensorOutParams.logFileName, logFolder);

      if (sensorOutParams.singulated)
      {
        double real_angle = sensorOutParams.theta;
        double real_dist = sensorOutParams.distance;

        double theta, dist, theta_s, dist_s;
        brain.markerPosEstimation_Test(sensorInParams.logFileName, theta, dist, theta_s, dist_s);

        placerInParams.angle = theta;
        placerInParams.h_dist = dist;

        double d_theta = theta - real_angle;
        double d_dist = dist - real_dist;

        ROS_INFO("Real Pose: Theta: %f, R: %f", real_angle, real_dist);
        ROS_INFO("Calculated Pose: Theta: %f, R:%f", theta, dist);

        placerOutParams = placer.place(placerInParams);

        if (placerOutParams.success)
        {
          ROS_INFO("Successfully placed marker!");
        }
        else
        {
          ROS_INFO("Placing Unsuccessful.");
        }

        sprintf(buffer, "#Q,%d,%f,%f,%f,%f,%s", placerOutParams.success, real_angle, real_dist, d_theta, d_dist, sensorInParams.logFileName.c_str());
        std::string str = buffer;
        logger.Append(placerOutParams.logFileName, str);
        logger.Copy(placerOutParams.logFileName, PLACING_FOLDER_NAME);
        util.go_home();
      }
      else
      {
        util.go_home();
        ROS_INFO("num markers: %d", sensorOutParams.nMarkers);
        if (sensorOutParams.nMarkers > 0)
        {
          util.random_drop();
        }
      }
    }
  }
}
