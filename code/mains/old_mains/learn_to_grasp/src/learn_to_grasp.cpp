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

#define MAX_GRASPS 10000
#define MAX_ERR 0
#define EXPERIMENT_NAME "grasp_learner3"
#define MODEL_NAME "grasp_learner.mat"

#define HAND_CAL_FREQ 50
#define VIS_CAL_FREQ 20

//#define MAX_BELOW_SURFACE 13.0

#define UP_STEP 36.0
#define Z_LIM 150.0
#define N_FLIPS 4

#define NUM_PARAMS 3
#define GRIDSIZE 10

#define MIN_DOWN 10.0
#define MAX_DOWN 44.0

#define MIN_OSCILLATION 0.0
#define MAX_OSCILLATION (PI/2.01)

#define MIN_HAND_SPEED 0.3
#define MAX_HAND_SPEED 0.9


int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp_learner");
  ros::NodeHandle node;

  GraspComm grasper(&node);
  SenseComm sensor(&node);  
  UtilComm util(&node);
  
  BrainComm brain(&node);
  MatlabComm matlab(&node);
  LoggerComm logger(&node); 

  GM_InputParams grasperInParams;
  GM_OutputParams grasperOutParams;

  grasperInParams.type = GM_TYPE_OPENLOOP;
  grasperInParams.z_lim = Z_LIM;
  grasperInParams.nFlips = N_FLIPS;
  grasperInParams.up = UP_STEP;

  SM_InputParams sensorInParams;
  SM_OutputParams sensorOutParams;

  ROS_INFO("--------------------------------------------------");
  ROS_INFO("GRASP_LEARNER: Hello, I will attempt to optimize");
  ROS_INFO("               grasping parameters which maximize");
  ROS_INFO("               our chance of singulating a marker.");
  ROS_INFO("--------------------------------------------------");

  // Calibration
  util.calibrate_hand();
  util.calibrate_vision();

  int i = 0;

  vector<double> mins(NUM_PARAMS,0);
  vector<double> maxs(NUM_PARAMS,0);
  vector<double> nextP(NUM_PARAMS, 0);
  vector<double> maxP(NUM_PARAMS, 0);
  
  // Set bounds for down
  mins[0] = MIN_DOWN;
  maxs[0] = MAX_DOWN;
  // Set bounds for oscillation amplitude
  mins[1] = MIN_OSCILLATION;
  maxs[1] = MAX_OSCILLATION;
  // Set bounds for hand speed
  mins[2] = MIN_HAND_SPEED;
  maxs[2] = MAX_HAND_SPEED;

  brain.learnGrasp_LoadModel(MODEL_NAME, mins, maxs, GRIDSIZE);

  while (ros::ok())
  { 
    // Grasp (grasp_OpenLoop)
    grasperInParams.id = i;
    brain.learnGrasp_GetNextParams(nextP);
    brain.learnGrasp_GetMaxParams(maxP);
    grasperInParams.down = nextP[0];
    grasperInParams.oscillationAmplitude = nextP[1];
    grasperInParams.handSpeed = nextP[2];
    util.get_random_bin_pos(grasperInParams.x, grasperInParams.y);

    ROS_INFO("Current Max Params: %f, %f, %f", maxP[0], maxP[1], maxP[2]);
    ROS_INFO("Choosing the following parameters: %f, %f, %f", nextP[0], nextP[1], nextP[2]);

    grasperOutParams = grasper.grasp(grasperInParams);

    // Sense (sensor_User)
    sensorInParams.type = SM_TYPE_VISION;
    sensorInParams.logFileName = grasperOutParams.logFileName;
    sensorOutParams = sensor.sense(sensorInParams);

    // Save logFile to folder
    logger.Copy(sensorOutParams.logFileName, EXPERIMENT_NAME);

    ROS_INFO("Singulated: %d", sensorOutParams.singulated);

    // Record success or failure
    brain.learnGrasp_TrainParams(nextP, sensorOutParams.singulated);

    util.go_home();
    if (sensorOutParams.nMarkers > 0)
      util.random_drop();

    i++;

    if (i >= MAX_GRASPS)
    {
      ROS_INFO("We've reached the maximum number of grasps!");
      brain.learnGrasp_GetMaxParams(nextP);
      ROS_INFO("Current best: %f, %f, %f", nextP[0], nextP[1], nextP[2]);
      break;
    }

    if (i % HAND_CAL_FREQ == 0)
    {
      util.calibrate_hand();
    }
    if (i % VIS_CAL_FREQ == 0)
    {
      util.calibrate_vision();
    }
  }
}
