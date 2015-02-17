#include <ros/ros.h>
#include <matVec/matVec.h>
#include <grasp_comm/grasp_comm.h>
#include <sense_comm/sense_comm.h>
#include <place_comm/place_comm.h>
#include <util_comm/util_comm.h>

#define VIS_CAL_CNT 10
#define HAND_CAL_CNT 10

#define MIN_CONF 0.5

int main(int argc, char** argv)
{
  ros::init(argc, argv, "insert_demo");
  ros::NodeHandle node;
  GraspComm grasper(&node);
  SenseComm sensor(&node);  
  PlaceComm inserter(&node);  
  UtilComm util(&node);  
 
  GM_InputParams grasperInParams;
  GM_OutputParams grasperOutParams;

  grasperInParams.type = GM_TYPE_OPENLOOP;
  //grasperInParams.type = GM_TYPE_OPENLOOP_EARLYABORT;
  grasperInParams.z_lim = 150.0;
  grasperInParams.up = 36.0;//32.0;
  grasperInParams.down = 36.0;//49.0;//41.0;
  grasperInParams.nFlips = 4;
  grasperInParams.oscillationAmplitude = 0.8683;//0.822622;//PI/3.0;
  grasperInParams.handSpeed = 0.7;//0.5;//0.647368; //0.45;


  SM_InputParams sensorInParams;
  SM_OutputParams sensorOutParams;

  sensorInParams.type = SM_TYPE_VISION;


  PM_InputParams inserterInParams;
  PM_OutputParams inserterOutParams;

  inserterInParams.type = PM_TYPE_INSERT;

  /*

     PM_InputParams placerInParams;
     PM_OutputParams placerOutParams;

     placerInParams.type = PM_TYPE_OPENLOOP;
     placerInParams.v1_dist = 70.0;
     placerInParams.v2_dist = 60.0;

   */

  util.go_home();
  util.calibrate_hand();
  util.calibrate_vision();

  int count = 1;

  while (ros::ok())
  {
    if (count % VIS_CAL_CNT == 0)
    {
      util.calibrate_vision();
    }
    if (count >= HAND_CAL_CNT)
    {
      util.calibrate_hand(1);
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

    sensorInParams.logFileName = grasperOutParams.logFileName;

    sensorOutParams = sensor.sense(sensorInParams);

    ROS_INFO("singulated: %d, confidence: %f", sensorOutParams.singulated, sensorOutParams.confidence);

    //if (sensorOutParams.singulated && sensorOutParams.confidence > MIN_CONF)
    if (sensorOutParams.singulated)
    {
      inserterInParams.angle = sensorOutParams.alpha;
      inserterInParams.h_dist = sensorOutParams.distance;

      inserterOutParams = inserter.place(inserterInParams);
      if (inserterOutParams.abort)
      {
        ROS_INFO("We aborted!");
      }
      else if (inserterOutParams.success)
      {
        ROS_INFO("Successfully inserted marker!");
      }
      else
      {
        ROS_INFO("Insertion Unsuccessful.");
      }
      util.go_home_from_vision();
      util.random_drop();
    }
    else
    {
      util.go_home_from_vision();
      ROS_INFO("num markers: %d", sensorOutParams.nMarkers);
      if (sensorOutParams.nMarkers > 0)
      {
        util.random_drop();
      }
    }

    count++;
  }
}
