//
// Name: Bowei Tang
//
// File Name: regrasp_node.hpp
//
// Last Modified: 2/24/2013, 06/09/2013-Nikhil
//
// Header file for the regrasp node, which controls P3, a simple regrasp 
// designed in the Manipulation Lab. P3 is controlled by an arduino. This
// program interacts with the arduino over a USB serial port. It records
// and publishes finger, motor, and palm sensor data. It allows the user
// to change the speed and current limit of the motor and then command it
// to move to a certain pose. Since the motor encoder is relative, it also
// implements a calibration routine so we can get consistent motor poses 
// across runs. Please look at the arduino code on P3 for more details with
// communication

#ifndef regrasp_NODE_H
#define regrasp_NODE_H

#include <errno.h>
#include <fcntl.h>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <math.h>

//ROS specific
#include <ros/ros.h>
#include <regrasp_comm/regrasp_comm.h>
#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <matVec/matVec.h>
#include <util_comm/util_comm.h>

// Speed of the robot
#define STCP 45
#define SORI 20
#define FTCP 90
#define FORI 30

// Speed and Force of the hand
#define HAND_SPD .7 //0.8 for P3
#define HAND_FORCE .9 //0.8 for P3

// Opening and closing finger
#define HAND_OPEN 20.0
#define HAND_RELEASE 25// 35//40.0//50.0//60.0  CHANGED BY RPAOLINI FOR NIKHIL; 35 and 30 BY NIKHIL
#define HAND_CLOSE 70 //90.0 70- NIKHIL
#define HAND_CLOSE_STAND 70// //100.0 , reduced by 20; 10 for rubber grip 10 for change in hand code, 80-NIKHIL, 70-Nikhil reduced again for change in hand power
#define HAND_CLOSE_LIE 70 65 //90.0 - 70 NIKHIL, 65-Nikhil reduced it for change in hand power
#define HAND_OPEN_BeforePick 5

// Zone of the robot
#define ZONE 0 //0 means every commanded position is a fine position. (No fly-by)

// Class definition
class RegraspController
{
 public:
  // Constructors and destructors
  RegraspController();
  RegraspController(ros::NodeHandle * n);
  // virtual ~regraspController();
  
  // Advertise topics and services
  void advertiseServices();
  void advertiseTopics();

  // Pointer to ros node
  ros::NodeHandle *node;

  RobotComm robot;
  HandComm hand;

  // Services

  bool regrasp_Ping(regrasp_comm::regrasp_Ping::Request& req, 
    regrasp_comm::regrasp_Ping::Response& res);
  bool regrasp_Initialize(regrasp_comm::regrasp_Initialize::Request& req, 
    regrasp_comm::regrasp_Initialize::Response& res);
  bool regrasp_Rotate(regrasp_comm::regrasp_Rotate::Request& req, 
    regrasp_comm::regrasp_Rotate::Response& res);
  bool regrasp_FreeDrop(regrasp_comm::regrasp_FreeDrop::Request& req, 
    regrasp_comm::regrasp_FreeDrop::Response& res);
  bool regrasp_FramePick(regrasp_comm::regrasp_FramePick::Request& req, 
    regrasp_comm::regrasp_FramePick::Response& res);
  bool regrasp_FrameDrop(regrasp_comm::regrasp_FrameDrop::Request& req, 
    regrasp_comm::regrasp_FrameDrop::Response& res);
  bool regrasp_StandtoLie(regrasp_comm::regrasp_StandtoLie::Request& req, 
    regrasp_comm::regrasp_StandtoLie::Response& res);
  bool regrasp_LietoStand(regrasp_comm::regrasp_LietoStand::Request& req, 
    regrasp_comm::regrasp_LietoStand::Response& res);
  bool regrasp_ThrowCatch(regrasp_comm::regrasp_ThrowCatch::Request& req, 
    regrasp_comm::regrasp_ThrowCatch::Response& res);

 private:
  //regrasp to ROS stuff
  ros::ServiceServer handle_regrasp_Ping;
  ros::ServiceServer handle_regrasp_Initialize;
  ros::ServiceServer handle_regrasp_Rotate;
  ros::ServiceServer handle_regrasp_FreeDrop;
  ros::ServiceServer handle_regrasp_FramePick;
  ros::ServiceServer handle_regrasp_FrameDrop;
  ros::ServiceServer handle_regrasp_StandtoLie;
  ros::ServiceServer handle_regrasp_LietoStand;
  ros::ServiceServer handle_regrasp_ThrowCatch;
};

#endif // REGRASP_NODE_H
