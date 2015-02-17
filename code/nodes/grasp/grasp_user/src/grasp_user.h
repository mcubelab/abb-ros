#include <stdio.h>
#include <limits>
#include <iostream>
#include <string>

//ROS specific
#include <ros/ros.h>
#include <matVec/matVec.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <logger_comm/logger_comm.h>
#include <grasp_comm/grasp_comm.h>

// Home position (joint space)
#define GM_GRASP_J1 -6.34
#define GM_GRASP_J2 2.23
#define GM_GRASP_J3 11.74
#define GM_GRASP_J4 0.0
#define GM_GRASP_J5 76.03
#define GM_GRASP_J6 -6.34

// Zone to use when moving to home position
#define GM_ZONE 0

// Speed to move to home position at
#define GM_SPEED_TCP 100.0
#define GM_SPEED_ORI 50.0

// Open hand to this absolute location at our home position
#define GM_HAND_HOME 30.0

// Try to close hand to this angle when grasping marker
#define GM_HAND_CLOSE 110.0

// Speed and force to open hand at
#define GM_OPEN_SPEED 0.75
#define GM_OPEN_FORCE 0.6

// Speed and force to close hand at
#define GM_CLOSE_SPEED 0.5 
#define GM_CLOSE_FORCE 0.4

// Minimum finger encoder deflection that trigger closing the hand
#define GM_MIN_ENCODER_DEFLECTION 100

typedef enum GM_State
{
  GM_INIT,
  GM_GRASP_MARKER,
  GM_FEED_MARKER,
  GM_END,
  GM_ERROR,
  GM_NUM_STATES
} GM_State;

RobotComm robot;
HandComm hand;
LoggerComm logger;

ros::NodeHandle *nodePtr;
ros::ServiceServer handle_grasp_User;

std::string logFileName;
Vec iniEncoders(NUM_FINGERS); //Initial finger encoder values when expecting the user to feed an object 

int stepStateMachine(int currState, GM_InputParams graspParams);
bool grasp_User(grasp_comm::grasp_User::Request& req, grasp_comm::grasp_User::Response& res);
