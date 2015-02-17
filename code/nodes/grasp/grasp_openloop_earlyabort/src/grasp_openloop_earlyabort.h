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
#include <grasp_comm/grasp_comm.h>
#include <matlab_comm/matlab_comm.h>

#define MAX_BUFFER 1024

// Home position (cartesian space)
#define GM_HOME_X 50.0
#define GM_HOME_Y -50.0
#define GM_HOME_Z 0.0
#define GM_HOME_Q0 1.0
#define GM_HOME_QX 0.0
#define GM_HOME_QY 0.0
#define GM_HOME_QZ 0.0

// Home position (joint space)
/*
#define GM_HOME_J1 -6.34
#define GM_HOME_J2 19.07
#define GM_HOME_J3 26.68
#define GM_HOME_J4 0.0
#define GM_HOME_J5 44.24
#define GM_HOME_J6 -6.34
*/

// Zone to use when going to grasp an object
#define GM_ZONE 1

// Speeds
// Long, safe and contact free motions
#define GM_FLY_SPEED_TCP   200.0
#define GM_FLY_SPEED_ORI   150.0
// Fast part of grasp procedure
#define GM_GRASP_FAST_SPEED_TCP 80.0//100.0
#define GM_GRASP_FAST_SPEED_ORI 120.0//150.0
// Slow part of grasp procedure
#define GM_GRASP_SLOW_SPEED_TCP  50.0
#define GM_GRASP_SLOW_SPEED_ORI  50.0

// Tracking distance
#define GM_GRASP_TRACK_DIST_TCP 5.0 //5.0 mm
#define GM_GRASP_TRACK_DIST_ORI 1.0 //1.0 deg

#define MAX_FINGER_DEFLECTION 2.0      // Max finger deflection is surface detection (in degrees).

#define HAND_CLOSE_FORCE 0.5
#define HAND_CLOSE_ANGLE 110.0

typedef enum GM_State
{
  GM_INIT,
  GM_INIT_GRASP,
  GM_GO_SURFACE,
  GM_FIND_SURFACE,
  GM_GO_HOME_GRASP,
  GM_GRASP,
  GM_END,
  GM_ERROR,
  GM_NUM_STATES
} GM_State;

RobotComm robot;
HandComm hand;
LoggerComm logger;
MatlabComm matlab;

ros::NodeHandle *nodePtr;
ros::ServiceServer handle_grasp_OpenLoop_EarlyAbort;

std::string logFileName;  // Name of the log file.
Vec iniFingerAngles; // Initial finger angles in surface detection.
double surface;      // Z of the detected surface.

int stepStateMachine(int currState, GM_InputParams graspParams);
bool grasp_OpenLoop_EarlyAbort(grasp_comm::grasp_OpenLoop_EarlyAbort::Request& req, grasp_comm::grasp_OpenLoop_EarlyAbort::Response& res);
