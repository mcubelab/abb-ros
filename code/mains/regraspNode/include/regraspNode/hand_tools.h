#ifndef _HAND_TOOLS_H_ 
#define _HAND_TOOLS_H_

#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <geometry_msgs/Pose.h>
#include "regrasp_test/SetHand.h"

class HandTools
{
  RobotComm robot;
  HandComm hand;

 public:
  HandTools() {};

  void init(RobotComm r, HandComm h);
  bool moveArm(geometry_msgs::Pose pose);
  bool moveArm(float joints);
  bool returnToCenter(void);
  bool closeHand(double close_angle);
  bool openHand(double open_angle);
  bool vibrate(double open_angle, double close_angle, double v0, 
               double v1, double v2, double v3, double v4, double v5);
  bool squeeze(void);
  bool invertHand(void);
  bool moveHandToTop(void);
  bool calcMinFingerAngle(double& min_angle);
  bool setupHand(regrasp_test::SetHand msg);
  bool calcRelease(double& drop_angle);
  

};

#endif
