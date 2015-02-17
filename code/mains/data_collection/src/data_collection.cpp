#include <ros/ros.h>
#include <matVec/matVec.h>
#include <grasp_comm/grasp_comm.h>
#include <sense_comm/sense_comm.h>
#include <util_comm/util_comm.h>
#include <logger_comm/logger_comm.h>

#define VIS_CAL_CNT 20
#define HAND_CAL_CNT 20

#define EXPERIMENT_NAME "haptic_collection_newfings"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_collection");
  ros::NodeHandle node;
  GraspComm grasper(&node);
  SenseComm sensor(&node);  
  UtilComm util(&node);  
  LoggerComm logger(&node);  
 
  GM_InputParams grasperInParams;
  GM_OutputParams grasperOutParams;

  grasperInParams.type = GM_TYPE_OPENLOOP;
  grasperInParams.z_lim = 150.0;
  grasperInParams.up = 36.0;
  grasperInParams.down = 36.0;
  grasperInParams.nFlips = 4;
  grasperInParams.oscillationAmplitude = 0.8683;
  grasperInParams.handSpeed = 0.7;

  SM_InputParams sensorInParams;
  SM_OutputParams sensorOutParams;

  sensorInParams.type = SM_TYPE_VISION;

  std::string logFolder = EXPERIMENT_NAME;

  util.go_home();
  util.calibrate_hand();
  util.calibrate_vision();

  int hand_cnt = 0;
  int vis_cnt = 0;
  int i = 1;
  int singulated = 0;

  while (ros::ok())
  {
    if (vis_cnt >= VIS_CAL_CNT)
    {
      util.calibrate_vision();
      vis_cnt = 0;
    }
    if (hand_cnt >= HAND_CAL_CNT)
    {
      util.calibrate_hand(1);
      hand_cnt = 0;
    }

    grasperInParams.id = i;
    util.get_random_bin_pos(grasperInParams.x, grasperInParams.y);

    ROS_INFO("Sensing command to grasp node...");
    grasperOutParams = grasper.grasp(grasperInParams);

    if (grasperOutParams.error != GM_ERROR_NONE)
    {
      ROS_INFO("Grasping Error!");
      if (grasperOutParams.error == GM_ERROR_COLLISION)
      {
        ROS_INFO("Collision occured while grasping. Dropping this marker, recalibrating, and continuing...");
        util.drop();
        util.calibrate_hand();
        hand_cnt = 0;
        continue;
      }
      else
      {
        ROS_INFO("Unexpected error detected!");
        break;
      }
    }
    
    ROS_INFO("Going to sense...");
    sensorInParams.logFileName = grasperOutParams.logFileName;
    sensorOutParams = sensor.sense(sensorInParams);

    // Save logFile to folder
    logger.Copy(sensorOutParams.logFileName, logFolder);

    if (sensorOutParams.singulated)
      singulated++;

    ROS_INFO("DATA_COLLECTION: (S = %d, N = %d, %f%%)", singulated, i-singulated, (100.0*singulated)/i);

    ROS_INFO("Going home...");
    util.go_home_from_vision();
    if (sensorOutParams.nMarkers > 0)
    {
      ROS_INFO("Randomly dropping marker...");
      util.random_drop();
    }

    vis_cnt++;
    hand_cnt++;
    i++;
  }
}
