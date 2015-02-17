#include <ros/ros.h>
#include <matVec/matVec.h>
#include <grasp_comm/grasp_comm.h>
#include <hand_comm/hand_comm.h>
#include <sense_comm/sense_comm.h>
#include <place_comm/place_comm.h>
#include <util_comm/util_comm.h>
#include <logger_comm/logger_comm.h>
#include <cstdlib>

#define VIS_CAL_CNT 20
#define HAND_CAL_CNT 1

#define GRASP_EXPERIMENT_NAME "data_collection_05_30_2012"
#define PLACE_EXPERIMENT_NAME "drop_reqs_05_30_2012"

#define MAX_DALPHA  1.047  // 60 degrees
#define MAX_DDIST   30.0    // mm

#define MIN_DALPHA 0.4
#define MIN_DDIST 20.0

#define MIN_FINGER_SUM 4650

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drop_reqs");
  ros::NodeHandle node;
  GraspComm grasper(&node);
  SenseComm sensor(&node);  
  UtilComm util(&node);  
  PlaceComm dropper(&node);  
  LoggerComm logger(&node);  
  HandComm hand(&node);

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

  PM_InputParams dropperInParams;
  PM_OutputParams dropperOutParams;

  dropperInParams.type = PM_TYPE_DROP;

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

  int motor_enc;
  int finger_enc[NUM_FINGERS];
  int sum;

  double begin_time = ros::Time::now().toSec();
  double cur_time;

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
        util.calibrate_hand(0);
        hand_cnt = 0;
        continue;
      }
      else
      {
        ROS_INFO("Unexpected error detected!");
        break;
      }
    }


    // Get the current position of the 3 fingers
    hand.GetEncoders(motor_enc, &finger_enc[0]);

    sum = 0;
    for (int k=0; k<NUM_FINGERS; k++)
      sum += finger_enc[k];

    if (sum < MIN_FINGER_SUM)
    {
      ROS_INFO("Encoder sum = %d. We are not holding any markers.", sum);
      sprintf(buffer, "#S,99,0,0.0,0,0.0,0.0,0.0,0.0,0.0,%d",
          motor_enc);
      for (int i=0; i<NUM_FINGERS; i++)
        sprintf(buffer,"%s,%d",buffer,finger_enc[i]);
      sprintf(buffer,"%s,no_picture",buffer);
      std::string str = buffer;
      logger.Append(grasperOutParams.logFileName, str);
      logger.Copy(grasperOutParams.logFileName, graspFolder);
    }
    else
    {
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
        do
        {
          d_alpha = ((double)rand()/(double)RAND_MAX)*(2*MAX_DALPHA)-MAX_DALPHA;
          d_dist = ((double)rand()/(double)RAND_MAX)*(2*MAX_DDIST)-MAX_DDIST;
        }while((abs(d_alpha) < MIN_DALPHA) && (abs(d_dist) < MIN_DDIST));

        real_alpha = sensorOutParams.alpha;
        real_dist = sensorOutParams.distance;

        dropperInParams.angle = real_alpha + d_alpha;
        dropperInParams.h_dist = real_dist + d_dist;

        ROS_INFO("Real: (%f, %f). Del: (%f, %f)", real_alpha, real_dist, d_alpha, d_dist);

        dropperOutParams = dropper.place(dropperInParams);
        if (dropperOutParams.success)
        {
          ROS_INFO("Successfully dropped marker!");
          util.go_home();       
          util.random_drop();       
        }
        else
        {
          ROS_INFO("Dropping Unsuccessful.");
          util.go_home();       
        }

        sprintf(buffer, "#Q,%d,%f,%f,%f,%f,%s", dropperOutParams.success, 
            real_alpha, real_dist, d_alpha, d_dist, 
            sensorInParams.logFileName.c_str());
        std::string str = buffer;
        logger.Append(dropperOutParams.logFileName, str);
        logger.Copy(dropperOutParams.logFileName, placeFolder);

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
    }

    cur_time = ros::Time::now().toSec();
    double total_time = cur_time - begin_time;

    ROS_INFO("DATA_COLLECTION: (S = %d, N = %d, %2.3f%%), %3.2f s / singulation", singulated, i-singulated, (100.0*singulated)/i,  total_time / singulated);

    vis_cnt++;
    hand_cnt++;
    i++;
  }

  return 0;
}
