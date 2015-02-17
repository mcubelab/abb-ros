#include <ros/ros.h>
#include <matVec/matVec.h>
#include <grasp_comm/grasp_comm.h>
#include <sense_comm/sense_comm.h>
#include <place_comm/place_comm.h>
#include <util_comm/util_comm.h>
#include <matlab_comm/matlab_comm.h>

#define VIS_CAL_CNT 10
#define HAND_CAL_CNT 100

#define MIN_CONF 0.5

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_place");
  ros::NodeHandle node;
  GraspComm grasper(&node);
  PlaceComm placer(&node);  
  UtilComm util(&node);  
  MatlabComm matlab(&node);  

  GM_InputParams grasperInParams;
  GM_OutputParams grasperOutParams;

  grasperInParams.type = GM_TYPE_OPENLOOP;
  grasperInParams.z_lim = 150.0;
  grasperInParams.up = 36.0;
  grasperInParams.down = 36.0;
  grasperInParams.nFlips = 4;
  grasperInParams.oscillationAmplitude = 0.8683;
  grasperInParams.handSpeed = 0.5;

  PM_InputParams placerInParams;
  PM_OutputParams placerOutParams;

  placerInParams.type = PM_TYPE_OPENLOOP;
  placerInParams.v1_dist = 70.0;
  placerInParams.v2_dist = 60.0;

  util.go_home();
  util.calibrate_hand();

  int count = 1;

  while (ros::ok())
  {
    if (count >= HAND_CAL_CNT)
    {
      util.calibrate_hand();
      count = 0;
    }

    util.get_random_bin_pos(grasperInParams.x, grasperInParams.y);

    grasperOutParams = grasper.grasp(grasperInParams);

    if (grasperOutParams.error != GM_ERROR_NONE)
    {
      ROS_INFO("Grasping Error!");
      if (grasperOutParams.error == GM_ERROR_COLLISION)
      {
        ROS_INFO("Collision occured while grasping.");
        continue;
      }
      else
      {
        ROS_INFO("Unexpected error detected!");
        break;
      }
    }

    ROS_INFO("%s", (char *)grasperOutParams.logFileName.c_str());

    matlab.sendString("logFile", (char *)grasperOutParams.logFileName.c_str());

    matlab.sendCommand("[quality, r, theta] = PredictGrasp(logFile);");

    if (matlab.getValue("quality") == 0)
    {
      ROS_INFO("NOT 1 MARKER");
      util.drop();
      count++;
      continue;
    }
    else if (matlab.getValue("quality") == 1)
    {
      ROS_INFO("UNCERTAIN ABOUT MARKER POSE");
      util.drop();
      count++;
      continue;
    }
    else
    {
      placerInParams.h_dist = matlab.getValue("r");
      placerInParams.angle = matlab.getValue("theta");

      ROS_INFO("ACCEPTING CURRENT GRASP! Predicted pose: r: %f, theta: %f", placerInParams.h_dist, placerInParams.angle);
      util.go_place();

      placerOutParams = placer.place(placerInParams);
      if (placerOutParams.success)
      {
        ROS_INFO("Successfully placed marker!");
      }
      else
      {
        ROS_INFO("Placing Unsuccessful.");
      }
      util.go_home();
    }

    count++;
  }
}
