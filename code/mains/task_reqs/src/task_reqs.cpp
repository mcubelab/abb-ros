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
#define HAND_CAL_CNT 50

#define GRASP_EXPERIMENT_NAME "insert_reqs_grasp_11_06_2012"
#define PLACE_EXPERIMENT_NAME "insert_reqs_11_06_2012"

//#define MAX_DALPHA  0.5236  // 30 degrees
//#define MAX_DDIST   20.0    // mm
//#define MAX_DALPHA  0.2618  // 15 degrees
//#define MAX_DDIST   20.0    // mm
#define MAX_DALPHA  0.1745  // 10 degrees
#define MAX_DDIST   5.0    // mm

#define MAX_FINGER_SUM 7600

int main(int argc, char** argv)
{
  ros::init(argc, argv, "insert_reqs");
  ros::NodeHandle node;
  GraspComm grasper(&node);
  SenseComm sensor(&node);  
  UtilComm util(&node);  
  PlaceComm inserter(&node);  
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
  grasperInParams.handSpeed = 0.7;

  SM_InputParams sensorInParams;
  SM_OutputParams sensorOutParams;

  sensorInParams.type = SM_TYPE_VISION;

  PM_InputParams inserterInParams;
  PM_OutputParams inserterOutParams;

  inserterInParams.type = PM_TYPE_INSERT;

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
  int raw_forces[NUM_RAW_HAND_FORCES];
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

    ROS_INFO("Sending command to grasp node...");
    grasperOutParams = grasper.grasp(grasperInParams);

    ////////////////////////////////////////////////////////////////
    // If all of our angles are 0, something is wrong. So stop.
    
    
    double angles[NUM_FINGERS], motor_ang;
    hand.GetAngles(motor_ang, &angles[0]);

    double tot=0;
    for (int k=0; k<NUM_FINGERS; k++)
      tot += fabs(angles[k]);

    if (tot < 0.1)
    {
      ROS_ERROR("Hand is not working. Stopping run and sending an email...");
      if (system("/home/simplehands/./sendBrokenEmail.sh \"I am sad. =(\""))
        ROS_ERROR("ok... maybe I can't send an email...");
      break;
    }
    
    //
    ////////////////////////////////////////////////////////////////

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
    hand.GetRawForces(&raw_forces[0]);

    sum = 0;
    for (int k=0; k<NUM_FINGERS; k++)
      sum += finger_enc[k];

    if (sum > MAX_FINGER_SUM)
    {
      ROS_INFO("Encoder sum = %d. We are not holding any markers.", sum);
      sprintf(buffer, "#S,99,0,0.0,0,0.0,0.0,0.0,0.0,0.0,%d",
          motor_enc);
      for (int i=0; i<NUM_FINGERS; i++)
        sprintf(buffer,"%s,%d",buffer,finger_enc[i]);
      for (int i=0; i<NUM_RAW_HAND_FORCES; i++)
        sprintf(buffer, "%s,%d", buffer, raw_forces[i]);
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
        d_alpha = ((double)rand()/(double)RAND_MAX)*(2*MAX_DALPHA)-MAX_DALPHA;
        d_dist = ((double)rand()/(double)RAND_MAX)*(2*MAX_DDIST)-MAX_DDIST;

        real_alpha = sensorOutParams.alpha;
        real_dist = sensorOutParams.distance;

        inserterInParams.angle = real_alpha + d_alpha;
        inserterInParams.h_dist = real_dist + d_dist;

        ROS_INFO("Real: (%f, %f). Del: (%f, %f)", real_alpha, real_dist, d_alpha, d_dist);

        inserterOutParams = inserter.place(inserterInParams);
        if (inserterOutParams.abort)
        {
          ROS_INFO("We aborted!");
        }
        else
        {
          if (inserterOutParams.success)
          {
            ROS_INFO("Successfully inserted marker!");
          }
          else
          {
            ROS_INFO("Insertion Unsuccessful.");
          }

          sprintf(buffer, "#Q,%d,%f,%f,%f,%f,%s", inserterOutParams.success, 
              real_alpha, real_dist, d_alpha, d_dist, 
              sensorInParams.logFileName.c_str());
          std::string str = buffer;
          logger.Append(inserterOutParams.logFileName, str);
          logger.Copy(inserterOutParams.logFileName, placeFolder);
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
