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

#define PLATFORM_TRAIN_X  57.5
#define PLATFORM_TRAIN_Y  469.0
#define PLATFORM_TRAIN_Z  443.5

#define PLATFORM_TEST_X  1161.0
#define PLATFORM_TEST_Y  469.0
#define PLATFORM_TEST_Z  443.5


// Initial placing position (cartesian space)

#define PLACING_TEST_X  PLATFORM_TEST_X
#define PLACING_TEST_Y  PLATFORM_TEST_Y
#define PLACING_TEST_Z  (PLATFORM_TEST_Z + 130.0)

#define PLACING_TRAIN_X  PLATFORM_TRAIN_X//45.0//159.0
#define PLACING_TRAIN_Y  PLATFORM_TRAIN_Y//476.0//-568.0
#define PLACING_TRAIN_Z  (PLATFORM_TRAIN_Z + 130.0)//525.0//474.8//-226.0//-216.0//-176.0

#define PLACING_Q0 0.7071
#define PLACING_QX 0.7071
#define PLACING_QY 0.0
#define PLACING_QZ 0.0


// Position to prepare to knock marker down from
#define KN_TEST_X  PLATFORM_TEST_X
#define KN_TEST_Y  (PLATFORM_TEST_Y + 10.0)
#define KN_TEST_Z  (PLATFORM_TEST_Z + 210.0)
#define KN_TEST_Q0 0.6533
#define KN_TEST_QX 0.6533
#define KN_TEST_QY 0.2706
#define KN_TEST_QZ -0.2706

#define KN_TRAIN_X  PLATFORM_TRAIN_X//45.0//173.0
#define KN_TRAIN_Y  (PLATFORM_TRAIN_Y + 10.0)//486.0//-568.0
#define KN_TRAIN_Z  (PLATFORM_TRAIN_Z + 210.0)//605.0//-306.0//-266.0
#define KN_TRAIN_Q0 0.6533//-0.6533
#define KN_TRAIN_QX 0.6533//0.2706
#define KN_TRAIN_QY 0.2706//0.6533
#define KN_TRAIN_QZ -0.2706//-0.2706

// Absolute location to move down to when knocking marker over
#define KN_DOWN_TRAIN_Z (PLATFORM_TRAIN_Z + 130.0)//525.0//-230.0//-216.0
#define KN_DOWN_TEST_Z (PLATFORM_TEST_Z + 130.0)//525.0//-230.0//-216.0

// Absolute location to move over to when knocking marker over
//#define KN_OVER_Y -507.0
#define KN_OVER_TRAIN_X (PLATFORM_TRAIN_X + 75.0)
#define KN_OVER_TEST_X (PLATFORM_TEST_X - 75.0)

// Quaternion used to flip the marker 180 degrees
#define FS_Q0 0.7071//0.7071
#define FS_QX 0.0//-0.7071
#define FS_QY -0.7071//0.0
#define FS_QZ 0.0//0.0

// Constants for placing maneuver
//#define PRESS_DOWN_Z  70.0 // Distance to initially press marker down (mm)
//#define MOVE_UP_Z     70.0 // Distance to move up before flipping (mm)
//#define FINAL_DOWN_Z  60.0 // Distance to move down after flipping (mm)
#define OUT_X_CHG     0.0//10.0 // Distance to move away from marker in X (mm)
#define OUT_Y_CHG     10.0//0.0  // Distance to move away from marker in Y (mm)
#define UP_Z_CHG      140.0//-140.0//-300.0 // Distance to move away from marker in Z (mm)

// Absolute Z to move up to so we can safely move back to the bin
#define SAFE_TEST_Z    (PLATFORM_TEST_Z + 200.0)//605.0//-340.0  
#define SAFE_TRAIN_Z    (PLATFORM_TRAIN_Z + 200.0)//605.0//-340.0  

// Open hand to this angle when moving away from marker
#define HAND_OPEN 30.0

// Speed and force to open hand at
#define OPEN_SPEED 0.75
#define OPEN_FORCE 0.6

// Minimum certainty level that the vision information about the
// current marker is correct
#define PM_MIN_CERT 0.30

// Maximum distance (squared) the marker can be from the center of the 
// hand before we reject it for placing because of safety (mm)
#define CENT_THRESH (60*60)

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
  PM_MOVE_UP,
  PM_FLIP,
  PM_FINAL_DOWN,
  PM_RELEASE,
  PM_OUT_AND_UP,
  PM_KN_CHECK,
  PM_KN_DOWN,
  PM_KN_OVER, 
  PM_SAFE_UP,
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
ros::ServiceServer handle_place_OpenLoop;

PM_State stepStateMachine(PM_State currState, PM_InputParams placeParams);
bool place_OpenLoop(place_comm::place_OpenLoop::Request& req, place_comm::place_OpenLoop::Response& res);

// These variables hold our goal positions, so we know where we are
// once our action completes
Quaternion goalQuat;
double goalPos[3];

// Keeps track of log file name in case we want to append any data
string log_file;
string picName;

// Keeps track of whether or not we were successful
bool place_success;
