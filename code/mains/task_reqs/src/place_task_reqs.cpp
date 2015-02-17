#include <ros/ros.h>
#include <matVec/matVec.h>
#include <grasp_comm/grasp_comm.h>
#include <sense_comm/sense_comm.h>
#include <place_comm/place_comm.h>
#include <util_comm/util_comm.h>
#include <logger_comm/logger_comm.h>
#include <cstdlib>

#define VIS_CAL_CNT 20
#define HAND_CAL_CNT 20

#define GRASP_EXPERIMENT_NAME "data_collection_new_new"
#define PLACE_EXPERIMENT_NAME "place_reqs_new_new"

#define MAX_DALPHA  0.1745  // 10 degrees
#define MAX_DDIST   17.0    // mm

int main(int argc, char** argv)
{
  ros::init(argc, argv, "place_reqs");
  ros::NodeHandle node;
  GraspComm grasper(&node);
  SenseComm sensor(&node);  
  UtilComm util(&node);  
  PlaceComm placer(&node);  
  LoggerComm logger(&node);  

  srand(time(NULL));
 
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

  sensorInParams.type = SM_TYPE_VISION;

  PM_InputParams placerInParams;
  PM_OutputParams placerOutParams;

  placerInParams.type = PM_TYPE_OPENLOOP;
  placerInParams.v1_dist = 70.0;
  placerInParams.v2_dist = 60.0;

  std::string graspFolder = GRASP_EXPERIMENT_NAME;
  std::string placeFolder = PLACE_EXPERIMENT_NAME;

  util.go_home();
  util.calibrate_hand();
  util.calibrate_vision();

  int hand_cnt = 0;
  int vis_cnt = 0;
  int i = 1;
  int singulated = 0;

  double d_alpha, d_dist, real_alpha, real_dist;
  char buffer[1024];

  while (ros::ok())
  {
    if (vis_cnt >= VIS_CAL_CNT)
    {
      util.calibrate_vision();
      vis_cnt = 0;
    }
    if (hand_cnt >= HAND_CAL_CNT)
    {
      util.calibrate_hand();
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
    logger.Copy(sensorOutParams.logFileName, graspFolder);

    if (sensorOutParams.singulated)
    {
      singulated++;

      //////////////////////////////////////////////
      // PERTUBATION
      //////////////////////////////////////////////
      d_alpha = ((double)rand()/(double)RAND_MAX)*(2*MAX_DALPHA)-MAX_DALPHA;
      d_dist = ((double)rand()/(double)RAND_MAX)*(2*MAX_DDIST)-MAX_DDIST;
      real_alpha = sensorOutParams.alpha;
      real_dist = sensorOutParams.distance;

      placerInParams.angle = real_alpha + d_alpha;
      placerInParams.h_dist = real_dist + d_dist;

      ROS_INFO("Real: (%f, %f). Del: (%f, %f)", real_alpha, real_dist, d_alpha, d_dist);

      placerOutParams = placer.place(placerInParams);
      if (placerOutParams.success)
      {
        ROS_INFO("Successfully placed marker!");
      }
      else
      {
        ROS_INFO("Placing Unsuccessful.");
      }

       sprintf(buffer, "#Q,%d,%f,%f,%f,%f,%s", placerOutParams.success, 
           real_alpha, real_dist, d_alpha, d_dist, 
           sensorInParams.logFileName.c_str());
        std::string str = buffer;
        logger.Append(placerOutParams.logFileName, str);
        logger.Copy(placerOutParams.logFileName, placeFolder);

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

    ROS_INFO("DATA_COLLECTION: (S = %d, N = %d, %f%%)", singulated, i-singulated, (100.0*singulated)/i);

    vis_cnt++;
    hand_cnt++;
    i++;
  }

  return 0;
}
