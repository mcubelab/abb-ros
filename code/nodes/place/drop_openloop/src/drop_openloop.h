#include <stdio.h>
#include <limits>
#include <iostream>
#include <cmath>

//ROS specific
#include <ros/ros.h>
#include <matVec/matVec.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <logger_comm/logger_comm.h>
#include <vision_comm/vision_comm.h>
#include <place_comm/place_comm.h>

// Initial absolute move speeds
#define P_TCP   100
#define P_ORI   50

// Slow move speeds
#define SLOW_TCP 60
#define SLOW_ORI 20

// Zone to use (how much to interpolate)
#define P_ZONE 0

// Initial position to drop marker (cartesian space)
#define DROPPING_X  70
#define DROPPING_Y  445
#define DROPPING_Z  640
#define DROPPING_Q0 0.7071
#define DROPPING_QX 0.7071
#define DROPPING_QY 0.0
#define DROPPING_QZ 0.0

#define RM_OUT_X  70.0//30.0
#define RM_OUT_Y  550.0
#define RM_OUT_Z  640.0
#define RM_OUT_Q0 0.305
#define RM_OUT_QX 0.305
#define RM_OUT_QY -0.638
#define RM_OUT_QZ 0.638

/*
#define RM_DOWN_X  120.0
#define RM_DOWN_Y  550.0
#define RM_DOWN_Z  580.0
#define RM_DOWN_Q0 0.305
#define RM_DOWN_QX 0.305
#define RM_DOWN_QY -0.638
#define RM_DOWN_QZ 0.638
*/

#define RM_DOWN_J1 -79.73
#define RM_DOWN_J2 11.19
#define RM_DOWN_J3 7.93
#define RM_DOWN_J4 93.47
#define RM_DOWN_J5 80.1
#define RM_DOWN_J6 109.55

#define RM_IN_X  120.0//30.0
#define RM_IN_Y  420.0
#define RM_IN_Z  580.0
#define RM_IN_Q0 0.305
#define RM_IN_QX 0.305
#define RM_IN_QY -0.638
#define RM_IN_QZ 0.638

#define RM_OUT2_X  30.0//30.0
#define RM_OUT2_Y  550.0
#define RM_OUT2_Z  580.0
#define RM_OUT2_Q0 0.305
#define RM_OUT2_QX 0.305
#define RM_OUT2_QY -0.638
#define RM_OUT2_QZ 0.638

#define RM_IN2_X  30.0
#define RM_IN2_Y  440.0
#define RM_IN2_Z  580.0
#define RM_IN2_Q0 0.305
#define RM_IN2_QX 0.305
#define RM_IN2_QY -0.638
#define RM_IN2_QZ 0.638


#define RM_OVER_X  100.0
#define RM_OVER_Y  440.0
#define RM_OVER_Z  580.0
#define RM_OVER_Q0 0.305
#define RM_OVER_QX 0.305
#define RM_OVER_QY -0.638
#define RM_OVER_QZ 0.638

#define RM_ROTATE_X  140.0
#define RM_ROTATE_Y  440.0
#define RM_ROTATE_Z  540.0
#define RM_ROTATE_Q0 0.585
#define RM_ROTATE_QX 0.585
#define RM_ROTATE_QY -0.397
#define RM_ROTATE_QZ 0.397

#define RM_UP_X  140.0
#define RM_UP_Y  440.0
#define RM_UP_Z  680.0
#define RM_UP_Q0 0.585
#define RM_UP_QX 0.585
#define RM_UP_QY -0.397
#define RM_UP_QZ 0.397

// Quaternion used to flip the marker 180 degrees
#define FS_Q0 0.7071//0.7071
#define FS_QX 0.0//-0.7071
#define FS_QY -0.7071//0.0
#define FS_QZ 0.0//0.0


// Open hand to this angle when moving away from marker
#define HAND_OPEN 30.0
#define HAND_WIDE -10.0
#define HAND_CLOSE  110.0

// Speed and force to open hand at
#define OPEN_SPEED 0.5//0.75
#define OPEN_FORCE 0.5
#define CLOSE_SPEED 0.5//0.75
#define CLOSE_FORCE 0.5

// useconds to wait before checking if placing was successful
#define PM_SETTLE_TIME 800000

#define NUM_FING 3

#define MIN_FING_DIST 20.0
#define MIN_FING_ANG  0.5236

static const double f_locs[NUM_FING][3] =
{
  {50.0, 0.0, 1.5708},
  {-25.0, -43.3, -0.5236},
  {-25.0, 43.3, -2.618}
};

using namespace std;

// All of the different states in our placing state machine
typedef enum PM_State
{
  PM_INIT,
  PM_GOTO_DROP,
  PM_ROTATE,
  PM_DROP_MARKER,
  PM_DROP_CHECK,
  PM_RM_DOWN,
  PM_RM_IN,
  PM_RM_OUT2,
  PM_RM_IN2,
  PM_RM_OVER,
  PM_RM_ROTATE,
  PM_RM_UP,
  PM_COMPLETE,
  PM_COLLISION,
  PM_ERROR,
  PM_NUM_STATES
} PM_State;

RobotComm robot;
VisionComm vision;
HandComm hand;
LoggerComm logger;

ros::NodeHandle *nodePtr;
ros::ServiceServer handle_drop_OpenLoop;

PM_State stepStateMachine(PM_State currState, PM_InputParams dropParams);
bool drop_OpenLoop(place_comm::drop_OpenLoop::Request& req, place_comm::drop_OpenLoop::Response& res);

// These variables hold our goal positions, so we know where we are
// once our action completes
Quaternion goalQuat;
double goalPos[3];

// Keeps track of log file name in case we want to append any data
string log_file;
string picName;

// Keeps track of whether or not we were successful
bool drop_success;
