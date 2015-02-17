#ifndef REGRASP_NODE_H
#define REGRASP_NODE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <geometry_msgs/Pose.h>
#include <regraspComm/regrasp_comm.h>
#include <matlab_comm/matlab_comm.h>
#include "geometry_tools.h"

class RegraspController
{
 public:
  RegraspController();
  RegraspController(ros::NodeHandle * n);
  virtual ~RegraspController();

  // Services
  bool regrasp_Execute(regraspComm::regrasp_Execute::Request &req,
  		       regraspComm::regrasp_Execute::Response &res);

  ros::NodeHandle *node;

  void advertiseServices();

 private:
  RobotComm robot;
  HandComm hand;
  MatlabComm matlab;

  // callbacks
  void init();
  bool pick(regraspComm::Pick msg);
  bool place(regraspComm::Place msg);
  bool move(regraspComm::Move msg);
  bool throwToPalm(regraspComm::ThrowToPalm msg);
  bool pushInEnveloping(regraspComm::PushInEnveloping msg);
  bool vibrate(regraspComm::Vibrate msg);
  bool rollOnGround(regraspComm::RollOnGround msg);
  bool pushInFingers(regraspComm::PushInFingers msg);
  bool rollToFingertip(regraspComm::RollToFingertip msg);
  bool throwAndFlip(regraspComm::ThrowAndFlip msg);
  bool rollToPalm(regraspComm::RollToPalm msg);
  bool rollToGround(regraspComm::RollToGround msg);
  bool droopInFingers(regraspComm::Droop msg);
  bool throwToFingertip(regraspComm::ThrowToFingertip msg);
  bool squeeze(regraspComm::Squeeze msg);
  bool lieToStand(regraspComm::LieToStand msg);
  bool standToLie(regraspComm::StandToLie msg);
  bool longEdgeDroop(regraspComm::LongEdgeDroop msg);
  bool shortEdgeDroop(regraspComm::ShortEdgeDroop msg);
  bool topple(regraspComm::Topple msg);
  bool droopGeneral(regraspComm::Droop msg);

  // tools
  double calcMinFingerAngle();
  double calcMaxFingerAngle();
  bool approach(Vec v, Quaternion q);
  double ellipse(double t);

  std::vector<HomogTransf> calcPushInEnvelopingTransforms(geometry_msgs::Pose safe_pose);
  bool poseToVec(geometry_msgs::Pose pose, Vec& vector);

  // handles to ROS
  ros::ServiceServer handle_regrasp_Execute;

};

#endif
