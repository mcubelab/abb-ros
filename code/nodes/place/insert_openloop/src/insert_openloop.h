#include <stdio.h>
#include <limits>
#include <iostream>

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

// Platform Location
#define PLATFORM_X  57.5
#define PLATFORM_Y  469.0
#define PLATFORM_Z  443.5

#define INITIAL_X   57.5
#define INITIAL_Y   469.0
#define INITIAL_Z   600.0
#define INITIAL_Q0  0.7071
#define INITIAL_QX  0.7071
#define INITIAL_QY  0.0
#define INITIAL_QZ  0.0

#define PRESS_DOWN_Z  504.0

#define MOVE_OUT_X  57.5
#define MOVE_OUT_Y  473.0//478.0
#define MOVE_OUT_Z  582.0//580.0//583.0

#define INSERT_X    -36.0
#define INSERT_Y    473.0//478.0
#define INSERT_Z    582.0//580.0//583.0

#define SAFE_J1   -83.93
#define SAFE_J2   17.28
#define SAFE_J3   -11.8
#define SAFE_J4   90.6
#define SAFE_J5   83.75
#define SAFE_J6   -5.45

// useconds to wait before checking if placing was successful
#define PM_SETTLE_TIME 800000

using namespace std;

// All of the different states in our placing state machine
typedef enum PM_State
{
  PM_INIT,
  PM_GOTO_PLACE,
  PM_ROTATE,
  PM_PRESS_DOWN,
  PM_MOVE_OUT,
  PM_PREP_INSERT,
  PM_INSERT,
  PM_CHECK,
  PM_GO_SAFE,
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
ros::ServiceServer handle_insert_OpenLoop;

PM_State stepStateMachine(PM_State currState, PM_InputParams insertParams);
bool insert_OpenLoop(place_comm::insert_OpenLoop::Request& req, place_comm::insert_OpenLoop::Response& res);

// These variables hold our goal positions, so we know where we are
// once our action completes
Quaternion goalQuat;
double goalPos[3];

// Keeps track of log file name in case we want to append any data
string log_file;
string picName;

// Keeps track of whether or not we were successful
bool insert_success;


#define NUM_FING 3

#define MIN_FING_DIST 5.0//10.0
#define MIN_FING_ANG  0.5236

// New hand configuration
static const double f_locs[NUM_FING][4] =
{
  {0.0, -50.0, 0.0, 3.1416},
  {43.3, 25.0, -1.0472, 2.0944},
  {-43.3, 25.0, -2.0944, 1.0472}
};

/* Old hand configuration
static const double f_locs[NUM_FING][4] =
{
  {50.0, 0.0, 1.5708, -1.5708},
  {-25.0, -43.3, -0.5236, 2.618},
  {-25.0, 43.3, -2.618, 0.5236}
};
*/
