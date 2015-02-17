#include <iostream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <robot_comm/robot_comm.h>
//#include <robotiq_comm/robotiq_comm.h>
#include <tableVision_comm/tableVision_comm.h>
#include <geometry_msgs/Pose.h>
#include <regraspComm/regrasp_comm.h>
#include <matlab_comm/matlab_comm.h>
#include <matVec/matVec.h>
#include "droop/Polyhedron.h"
#include <geometry_msgs/Pose.h>

using namespace std;

bool ikok(Vec v, Quaternion q)
{
  if (v[2]<90)
    return false;
  else
    return true;
}

bool ikok(Vec v, Quaternion q);
int main (int argc, char** argv) 
{

  ros::init(argc, argv, "DROOP_BITCH");
  ros::NodeHandle node;

  RobotComm robot(&node);
  //  RobotiqComm hand(&node);
  TableVisionComm tv(&node);

  robot.SetTool(0,0,0,1,0,0,0);
  robot.SetJoints(0,0,0,0,90,0);
  // hand.Disable();
  // hand.Enable();
  hand.SetPose(0);
  usleep(2000000);
 
  Vec seenV;// = Vec("554.051 233.93 17.5169",3);
  Quaternion seenQ;// = Quaternion("0.0226087 0.0228433 0.708012 0.705472");

  bool ret = false;
  while (!ret)
    ret = tv.GetPose(RecObj::BIG_TRIANGLE, seenV, seenQ);

  // Polyhedron rect(RecObj::LONG_BLOCK);
  // rect.print();
  // int goal = 2;
  // int please;
  // rect.findSupportFace(rect.getCentroid(),please);
  // cout << "please " << please << endl;

  // Vec orig_grasp;
  // Vec usable_grasp;
  // Quaternion quat;
  // rect.pick(goal, orig_grasp, usable_grasp, quat);
  // cout << "orig grasp " << orig_grasp << endl;
  // cout << "usable " << usable_grasp << endl;

  Polyhedron tri(RecObj::BIG_TRIANGLE);

  cout << "seenV " << seenV << endl;
  cout << "seenQ " << seenQ << endl;

  tri.rotate(seenQ);
  tri.print();

  int goal = 1;// 2;

  Vec orig_grasp;
  Vec usable_grasp;
  Quaternion quat;
  // cout << "pushPick " << tri.pushPick(0, usable_grasp, quat) << endl;
  
  // vector<geometry_msgs::Pose> waypts;
  // tri.push(usable_grasp, quat, waypts);

  tri.pick(goal, orig_grasp, usable_grasp, quat);
  cout << "orig grasp " << orig_grasp << endl;
  cout << "usable grasp " << usable_grasp << endl;
  cout << "quat " << quat << endl;
  
  vector<geometry_msgs::Pose> waypts;
  tri.lift(orig_grasp,usable_grasp, quat, goal, waypts);
 
 //  Vec v = Vec("1.12429 17.4638 -0.063016",3);
 //  Vec tmp;
 //  int gf;

 //  cout << "orig grasp " << v << endl;
 //  bool can = false;  
 //  while (!can)
 //    {
 //      can = tri.testgraspable(v,gf,tmp);
 //      cout << "test " << can << endl;
 //      v = tmp;
 //    }
 //  cout << "usable " << tmp << endl;

  // cout << "orig grasp " << orig_grasp << endl;
  // cout << "usable " << usable_grasp << endl;

 //  vector<geometry_msgs::Pose> waypts;
 //  tri.lift(usable_grasp,usable_grasp, quat, goal, waypts);
 //  tri.lift(orig_grasp,usable_grasp, quat, goal, waypts);

 //  Execute
 for (size_t i=0;i<waypts.size();i++)
    {
      Vec v(3);
      Quaternion q;
      tri.pose2cart(waypts[i], v, q);

      Vec offset;
      tri.findHandOffset(q, offset);
      
      cout << "v " << v << endl;
      cout << "offset " << offset << endl;
      cout << "seenV " << seenV << endl;

      Vec real = v + seenV + offset;
      cout << "waypt " << real << q << endl;
      
      double joints[6];
      HomogTransf h = HomogTransf(q.getRotMat(),real);
      cout << "IK?" << (robot.GetIK(h,joints)) << " toLow? " << (ikok(real,q)) << endl;
      if ((robot.GetIK(h,joints)) && (ikok(real,q)))
      	robot.SetCartesianJ(real[0],real[1],real[2],q[0],q[1],q[2],q[3]);
      else
      	{
      	  cout << "bad waypt " << endl;
      	  break;
      	}
      usleep(1000000); // 1 sec
      // if (i==1) // second step is the grasp (first is approach)
      // 	{
      // 	  int close;
      // 	  tri.findGraspForce(close);
      // 	  cout << "close force " << close << endl;
      // 	  hand.SetPose(close);
      // 	  hand.WaitRest(0.25);
      // 	  usleep(1000000);
      // 	}
    }
  hand.SetPose(0);
  hand.WaitRest(1);
  robot.SetJoints(0,0,0,0,90,0);

  // -0.277872 -0.264346 -0.649469 0.656607
  // 0.279806 0.267812 0.659628 -0.644117 

  /******** For testing *********/
  // Vec seenV = Vec("554.051 233.93 17.5169",3);
  // Quaternion seenQ = Quaternion("0.0226087 0.0228433 0.708012 0.705472");
  // Polyhedron tri(RecObj::BIG_TRIANGLE);

  // cout << "seenV " << seenV << endl;
  // cout << "seenQ " << seenQ << endl;

  // //tri.rotate(seenQ);
  // //tri.translate(seenV);
  // tri.print();
  
  // Vec pt = Vec("17 17 17",3);
  // Vec pt0 = Vec("0 0 0",3);
  // Vec pt1 = Vec("103 0 17",3);
  // Vec pt2 = Vec("-1 0 0",3);
  // cout << "pt " << pt << " in? " << tri.in(pt) << endl;
  // cout << "pt " << pt0 << " in? " << tri.in(pt0) << endl;
  // cout << "pt " << pt1 <<" in? " << tri.in(pt1) << endl;
  // cout << "pt " << pt2 << " in? " << tri.in(pt2) << endl;

  // double angle;
  // cout << "ya? " << tri.angleBetweenSides(0,1, angle) << endl;
  // cout << "angle " << angle << endl;

}

// on face 2
// seenV [ 433.503 176.856 82.0716 ]
// seenQ [ 0.652097 -0.651078 0.27407 0.275279 ]

