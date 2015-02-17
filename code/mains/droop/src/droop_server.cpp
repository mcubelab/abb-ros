#include <iostream>
#include <fstream>
#include <string>
#include <time.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <robot_comm/robot_comm.h>
#include <robotiq_comm/robotiq_comm.h>
#include <tableVision_comm/tableVision_comm.h>
#include <geometry_msgs/Pose.h>
#include <regraspComm/regrasp_comm.h>
#include <matlab_comm/matlab_comm.h>
#include <matVec/matVec.h>
#include "droop/Polyhedron.h"
#include <geometry_msgs/Pose.h>
#include "droop/Pull.h"
#include "droop/Push.h"
#include "droop/Play.h"
#include "droop/Pick.h"
#include "droop/HandMove.h"
#include "droop/Flip.h"
#include "droop/PickPlace.h"

using namespace std;

double Z_THRESH = 90;

class Tester {
public:
  ros::NodeHandle *node;
  RobotComm robot;
  RobotiqComm hand;
  TableVisionComm tv;
  Tester (ros::NodeHandle *n);
  ~Tester();
  
  void advertiseServices();
  bool ikok(Vec v, Quaternion q);
  bool pull(droop::Pull::Request &req, droop::Pull::Response &res);
  bool push(droop::Push::Request &req, droop::Push::Response &res);
  bool play(droop::Play::Request &req, droop::Play::Response &res);
  bool pick(droop::Pick::Request &req, droop::Pick::Response &res);
  bool hand_move(droop::HandMove::Request &req, droop::HandMove::Response &res);
  bool flip(droop::Flip::Request &req, droop::Flip::Response &res);
  bool pp_longshort(droop::PickPlace::Request &req, droop::PickPlace::Response &res);
  bool pp_longlong(droop::PickPlace::Request &req, droop::PickPlace::Response &res);

private:
  ros::ServiceServer handle_droop_pull;
  ros::ServiceServer handle_droop_push;
  ros::ServiceServer handle_droop_play;
  ros::ServiceServer handle_droop_pick;
  ros::ServiceServer handle_droop_hand_move;
  ros::ServiceServer handle_droop_flip;
  ros::ServiceServer handle_droop_pp_longshort;
  ros::ServiceServer handle_droop_pp_longlong;
};

Tester::Tester(ros::NodeHandle *n) : hand(n)
{
  node = n;
  RobotComm r(node);
  //RobotiqComm h(node);
  TableVisionComm tvc(node);

  robot = r;
  //hand = h;
  tv = tvc;

  robot.SetJoints(0,0,0,0,95,0);
  robot.SetTool(0,0,0,1,0,0,0);
  robot.SetJoints(0,0,0,0,90,0);
  hand.Enable();
  usleep(2000000);

  hand.SetPose(10);
}

Tester::~Tester() {
  handle_droop_pull.shutdown();
  handle_droop_push.shutdown();
  handle_droop_play.shutdown();
  handle_droop_pick.shutdown();
  handle_droop_hand_move.shutdown();
  handle_droop_flip.shutdown();
  handle_droop_pp_longshort.shutdown();
  handle_droop_pp_longlong.shutdown();

  hand.Disable();
}

void Tester::advertiseServices()
{
  handle_droop_pull = node->advertiseService("droop_pull", &Tester::pull, this);
  handle_droop_push = node->advertiseService("droop_push", &Tester::push, this);
  handle_droop_play = node->advertiseService("droop_play", &Tester::play, this);
  handle_droop_pick = node->advertiseService("droop_pick", &Tester::pick, this);
  handle_droop_hand_move = node->advertiseService("droop_hand_move", &Tester::hand_move, this);
  handle_droop_flip = node->advertiseService("droop_flip", &Tester::flip, this);
  handle_droop_pp_longshort = node->advertiseService("droop_pp_longshort", &Tester::pp_longshort, this);
  handle_droop_pp_longlong = node->advertiseService("droop_pp_longlong", &Tester::pp_longlong, this);
}

bool Tester::ikok(Vec v, Quaternion q)
{
  if (v[2]<Z_THRESH)
    return false;

  double joints[6];
  HomogTransf h = HomogTransf(q.getRotMat(),v);
  if (!robot.GetIK(h,joints))
    return false;

  return true;
}

bool Tester::pull(droop::Pull::Request &req, droop::Pull::Response &res)
{
  cout << "im here " << endl;
  int goal = req.goal;
  Polyhedron obj(req.obj);

  // Look for the block
  Vec seenV;
  Quaternion seenQ;
  bool ret = false;
  while (!ret)
    ret = tv.GetPose(req.obj, seenV, seenQ);

  cout << "seenV " << seenV << endl;
  cout << "seenQ " << seenQ << endl;
  obj.rotate(seenQ);
  obj.print();

  // Find grasp
  Vec orig_grasp;
  Vec usable_grasp;
  Quaternion quat;
  obj.pick(goal, orig_grasp, usable_grasp, quat);
  cout << "orig grasp " << orig_grasp << endl;
  cout << "usable grasp " << usable_grasp << endl;
  cout << "quat " << quat << endl;

  // Find waypoints to droop
  vector<geometry_msgs::Pose> waypts;
  obj.lift(orig_grasp,usable_grasp, quat, goal, waypts);

  // Execute
  for (size_t i=0;i<waypts.size();i++)
    {
      Vec v(3);
      Quaternion q;
      obj.pose2cart(waypts[i], v, q);

      Vec offset;
      obj.findHandOffset(q, offset);
      Vec real = v + seenV + offset;
      cout << "waypt " << real << q << endl;

      if (ikok(real, q))
  	robot.SetCartesianJ(real[0],real[1],real[2],q[0],q[1],q[2],q[3]);
      else
	{
	  cout << "ERROR: waypoint failed" << endl;
	  break;
	}

      usleep(1000000); // 1 sec
      if (i==1) // second step is the grasp (first is approach)
    	{
    	  int close;
    	  obj.findGraspForce(close);
    	  cout << "close force " << close << endl;
    	  hand.SetPose(close);
    	  hand.WaitRest(0.25);
  	  usleep(1000000); 
  	}
      hand.SetPose(0);
      hand.WaitRest(1);
      robot.SetJoints(0,0,0,0,90,0);
    }

  return true;
}

bool Tester::push(droop::Push::Request &req, droop::Push::Response &res)
{
  // int goal = req.goal;
  // Polyhedron obj(req.obj);

  // // Look for the block
  // Vec seenV;
  // Quaternion seenQ;
  // bool ret = false;
  // while (!ret)
  //   ret = tv.GetPose(req.obj, seenV, seenQ);

  // cout << "seenV " << seenV << endl;
  // cout << "seenQ " << seenQ << endl;
  // obj.rotate(seenQ);
  // obj.print();

  // int support;
  // obj.findSupportFace(obj.getCentroid(),support);
  // if (goal==support)
  //   {
  //     vector<int> new_goals = obj.getSymmetricFaces(goal);
  //     goal = new_goals[0];
  //   }
  // cout << "support " << support << " goal " << goal << endl;

  // // Find grasp
  // Vec usable_grasp;
  // Quaternion quat;
  // obj.pushPick(goal, usable_grasp, quat);
  // cout << "usable grasp " << usable_grasp << endl;
  // cout << "quat " << quat << endl;

  // // Find waypoints to droop
  // vector<geometry_msgs::Pose> waypts;
  // //obj.pushlift(orig_grasp,usable_grasp, quat, goal, waypts);\

  // Vec offset;
  // obj.findHandOffset(quat, offset);
  // cout << "offset " << offset << endl;
  // Vec real = usable_grasp + seenV + offset;
  // cout << "waypt " << real << quat << endl;
  
  // // Execute
  // for (size_t i=0;i<waypts.size();i++)
  //   {
  //     Vec v(3);
  //     Quaternion q;
  //     obj.pose2cart(waypts[i], v, q);

  //     Vec offset;
  //     obj.findHandOffset(q, offset);
  //     Vec real = v + seenV + offset;
  //     cout << "waypt " << real << q << endl;

  //     if (ikok(real, q))
  // 	cout << "execute" << endl; //robot.SetCartesianJ(real[0],real[1],real[2],q[0],q[1],q[2],q[3]);
  //     else
  // 	cout << "ERROR: waypoint failed" << endl;

  //     usleep(1000000); // 1 sec
  //     if (i==1) // second step is the grasp (first is approach)
  //   	{
  //   	  int close;
  //   	  obj.findGraspForce(close);
  //   	  cout << "close force " << close << endl;
  //   	  // hand.SetPose(close);
  //   	  // hand.WaitRest(0.25);
  // 	  usleep(1000000); 
  // 	}
  //     hand.SetPose(0);
  //     hand.WaitRest(1);
  //     robot.SetJoints(0,0,0,0,90,0);
  //   }

  return true;
}

bool Tester::pick(droop::Pick::Request &req, droop::Pick::Response &res)
{
  hand.Open();
  robot.SetCartesian(600,215,272,0,0.707,0.707,0);
  hand.SetPose(req.p);

  return true;
}

bool Tester::hand_move(droop::HandMove::Request &req, droop::HandMove::Response &res)
{
  hand.SetPose(req.p);

  return true;
}

// PICK AND PLACE: rectangle long>short
bool Tester::pp_longshort(droop::PickPlace::Request &req, droop::PickPlace::Response &res)
{
  robot.SetTool(0,0,0,1,0,0,0);
  robot.SetZone(0);
  robot.SetSpeed(350,60);

  // for (int i=0;i<25;i++)
  //   {
  //     ros::Time start = ros::Time::now();

  //     // pick
  //     robot.SetCartesian(600,200,160,0,1,0,0);
  //     hand.SetPose(145);
  //     hand.WaitRest(0.5);
      
  //     robot.SetCartesian(600,200,160,0,1,0,0); 
  //     robot.SetCartesian(600,200,160,0,0.707,0,-0.707); 
  //     robot.SetCartesian(600,200,85,0,0.707,0,-0.707); 
      
  //     hand.SetPose(10);
  //     hand.WaitRest(0.5);
      
  //     ros::Time end = ros::Time::now();
  //     cout << start-end << endl;

  //     robot.SetCartesian(600,200,250,0,0.707,0,-0.707); 
  //     usleep(2000000);
  //   }

  for (int i=0;i<25;i++)
    {
      ros::Time start = ros::Time::now();

      Polyhedron obj(RecObj::LONG_BLOCK);

      Vec seenV = Vec("530 217 25",3);;
      Quaternion seenQ = Quaternion("0.707 0.707 0 0");
      int goal = 0;
      obj.rotate(seenQ);

      // Find grasp
      Vec orig_grasp;
      Vec usable_grasp;
      Quaternion quat = Quaternion("0 1 0 0");
      Quaternion q;
      obj.pick(goal, orig_grasp, usable_grasp, q);
      // cout << "orig grasp " << orig_grasp << endl;
      // cout << "usable grasp " << usable_grasp << endl;
      // cout << "quat " << quat << endl;

      // pick and lift
      Vec offset = Vec("0 0 140",3);
      //obj.findHandOffset(quat, offset);
      Vec real = usable_grasp + seenV + offset;

      // cout << "offset " << offset[2] << endl;
      // cout << "seenV " << seenV[2] << endl;
      // cout << "grasp " << usable_grasp[2] << endl;

      // cout << "real " << real << endl;
      if (ikok(real, quat))
      	robot.SetCartesianJ(real[0],real[1],real[2],quat[0],quat[1],quat[2],quat[3]);
      else
      	{
      	  //cout << "ERROR: waypoint failed" << endl;
      	  return false;
      	}
      int close;
      obj.findGraspForce(close);
      hand.SetPose(close);
      hand.WaitRest(0.5);
      robot.SetSpeed(120,35);
      robot.SetCartesianJ(real[0],real[1],real[2]+120,quat[0],quat[1],quat[2],quat[3]);
      robot.SetCartesianJ(real[0],real[1],real[2]+103,quat[0],quat[1],quat[2],quat[3]);
      robot.SetSpeed(350,60);
      hand.SetPose(20);
      hand.WaitRest(0.5);
      
      ros::Time end = ros::Time::now();
      cout << start-end << endl;

      usleep(2000000);
    }

  return true;
}

bool Tester::pp_longlong(droop::PickPlace::Request &req, droop::PickPlace::Response &res)
{
  robot.SetTool(0,0,0,1,0,0,0);
  robot.SetZone(0);
  robot.SetSpeed(350,60);

  // for (int i=0;i<25;i++)
  //   {
  //     ros::Time start = ros::Time::now();

  //     // pick
  //     robot.SetCartesian(600,200,160,0,1,0,0);
  //     hand.SetPose(145);
  //     hand.WaitRest(0.5);
      
  //     // place
  //     robot.SetCartesian(740,200,200,0,1,0,0); 
  //     robot.SetCartesian(740,200,200,0,0.707,0,-0.707); 
  //     robot.SetCartesian(740,200,85,0,0.707,0,-0.707); 
      
  //     hand.SetPose(10);
  //     hand.WaitRest(0.5);

  //     // second pick
  //     robot.SetCartesian(740,200,250,0,1,0,0); 
  //     robot.SetCartesian(460,200,250,0,1,0,0); 
  //     robot.SetCartesian(460,200,250,0,0.707,0,0.707);
  //     robot.SetCartesian(460,200,85,0,0.707,0,0.707);
  //     hand.SetPose(145);
  //     hand.WaitRest(0.5);

  //     // second place
  //     robot.SetCartesian(600,200,200,0,0.707,0,0.707);
  //     robot.SetCartesian(600,200,160,0,1,0,0);
  //     hand.SetPose(10);
  //     hand.WaitRest(0.5);

  //     ros::Time end = ros::Time::now();
  //     cout << start-end << endl;

  //     robot.SetCartesian(600,200,250,0,1,0,0); 
  //     usleep(2000000);
  //   }


  for (int i=0;i<25;i++)
    {
      ros::Time start = ros::Time::now();

      Polyhedron obj(RecObj::LONG_BLOCK);

      Vec seenV = Vec("530 217 25",3);;
      Quaternion seenQ = Quaternion("0.707 0.707 0 0");
      int goal = 0;
      obj.rotate(seenQ);

      // Find grasp
      Vec orig_grasp;
      Vec usable_grasp;
      Quaternion quat = Quaternion("0 1 0 0");
      Quaternion q;
      obj.pick(goal, orig_grasp, usable_grasp, q);
      // cout << "orig grasp " << orig_grasp << endl;
      // cout << "usable grasp " << usable_grasp << endl;
      // cout << "quat " << quat << endl;

      // pick and lift
      Vec offset = Vec("0 0 140",3);
      //obj.findHandOffset(quat, offset);
      Vec real = usable_grasp + seenV + offset;

      // cout << "offset " << offset[2] << endl;
      // cout << "seenV " << seenV[2] << endl;
      // cout << "grasp " << usable_grasp[2] << endl;

      // cout << "real " << real << endl;
      if (ikok(real, quat))
      	robot.SetCartesianJ(real[0],real[1],real[2],quat[0],quat[1],quat[2],quat[3]);
      else
      	{
      	  //cout << "ERROR: waypoint failed" << endl;
      	  return false;
      	}
      int close;
      obj.findGraspForce(close);
      hand.SetPose(close);
      hand.WaitRest(0.5);
      robot.SetZone(4);
      // robot.SetCartesianJ(real[0]+15,real[1],real[2]+120,quat[0],quat[1],quat[2],quat[3]);
      // robot.SetCartesianJ(real[0]-15,real[1],real[2]+10,quat[0],quat[1],quat[2],quat[3]);

      robot.SetCartesianJ(real[0],real[1],real[2]+120,quat[0],quat[1],quat[2],quat[3]);
      robot.SetCartesianJ(real[0]-30,real[1],real[2]+10,quat[0],quat[1],quat[2],quat[3]);

      usleep(200000);
      hand.SetPose(20);
      hand.WaitRest(0.5);
      
      ros::Time end = ros::Time::now();
      cout << start-end << endl;

      usleep(2000000);
    }



  return true;
}


bool Tester::flip(droop::Flip::Request &req, droop::Flip::Response &res)
{
  double pi = 3.14159;

  hand.SetPose(10);
  robot.SetSpeed(350,60);
  robot.SetZone(4);

  // pick 
  double startX = 600; //440;
  double startY = 200;
  double startZ = 180;
  robot.SetCartesian(startX,startY,startZ,0,1,0,0);  
  hand.SetPose(138);
  usleep(3000000);

  double height = 150; //140*1.7; // offset by start position, scale by height
  //double x [11] = {0,-1.0000,-1.8000,-2.3000,-3.8000,-5.1000,-6.4,-7.9,-8.4,-9.2,-10.2}; 
  //double x [11] = {0,-2.4000,-3.7000,-4.3000,-4.7000,-5.0000,-5.3,-5.7,-6.3,-7.6,-10}; 
  //double x [11] = {0,-0.3000,-0.8000,-1.5000,-3.0000,-5.0000,-7,-8.5,-9.2,-9.7,-10}; 
  double x [9] = {0,0,0,0,0,0,0,0,0}; 
  double z [9] = {0, 0.5, 1, 2.5, 5, 2.5, 1 , 0.5 ,0};

  for (int i=0;i<11;i++)
    {
      robot.SetCartesian(startX+x[i]*(height/5), startY, startZ+z[i]*(height/5),0,1,0,0);
    }

  // double t = (3.0/2.0)*pi;
  // double height = 140*1.7;
  // for (int i=0;i<10;i++)
  //   {
  //     double x = (t-sin(2*t))-4.7124;
  //     double z = -0.5*(1-cos(2*t)) + 1;

  //     robot.SetCartesian(startX + x*(2*height/pi), startY, 
  // 			 startZ + z*height , 0,1,0,0);
  //     t = t + (pi/10.0);
      
  //     cout << "t " << t << endl;
  //     cout << "x " << (t-sin(2*t))-4.7124 << " y " << -0.5*(1-cos(2*t)) + 1 << endl;
  //     cout << "pos " << (startX + x*(2*height/pi)) << " " << startZ + z*height << endl;
  //   }

  // half circle
  //robot.SetSpeed(350,60);

  //robot.SetCartesian(500,200,170,0,1,0,0);

  return true;
}


bool Tester::play(droop::Play::Request &req, droop::Play::Response &res)
{
  double pi = 3.14159;

  //int goal = 2;//req.goal;
  Polyhedron obj(req.obj);

  // // Look for the block
  // Vec seenV;
  // Quaternion seenQ;
  // bool ret = false;
  // while (!ret)
  //   ret = tv.GetPose(req.obj, seenV, seenQ);

  // cout << "seenV " << seenV << endl;
  // cout << "seenQ " << seenQ << endl;
  // obj.rotate(seenQ);
  // obj.print();

  // // Find grasp
  // Vec orig_grasp;
  // Vec usable_grasp;
  // Quaternion quat;
  // obj.pick(goal, orig_grasp, usable_grasp, quat);
  // cout << "orig grasp " << orig_grasp << endl;
  // cout << "usable grasp " << usable_grasp << endl;
  // cout << "quat " << quat << endl;

  // // pick and lift
  // Vec offset;
  // obj.findHandOffset(quat, offset);
  // Vec real = usable_grasp + seenV + offset;
  // if (ikok(real, quat))
  //   robot.SetCartesianJ(real[0],real[1],real[2],quat[0],quat[1],quat[2],quat[3]);
  // else
  //   {
  //     cout << "ERROR: waypoint failed" << endl;
  //     return false;
  //   }
  // int close;
  // obj.findGraspForce(close);
  // cout << "close force " << close << endl;
  // hand.SetPose(close);
  // hand.WaitRest(0.25);
  // usleep(1000000); 
  // robot.SetCartesianJ(real[0],real[1],real[2]+250,quat[0],quat[1],quat[2],quat[3]);

  Vec wksp = Vec("500 200 450",3);
  Quaternion quat = Quaternion("0.007 0.9249 0.0339 -0.3785");

  // find resonant freq of the block
  double speed = 350; // mm/s
  double l = 0.135;
  double resonant_freq = 2*pi*sqrt(l/9.81);
  double circ = speed*resonant_freq; // distance need to travel
  double r = circ/(2*pi*sqrt(5/2)); // the r needed for that travel (where its an ellipse with axes r & 2r
  double scale = r;

  cout << "scale " << scale << endl;

  double ampX = 2;
  double ampY = 1;
  
  robot.SetZone(4);
  robot.SetSpeed(150, 35);
  robot.SetCartesian(wksp[0],wksp[1],wksp[2],quat[0],quat[1],quat[2],quat[3]);
  robot.SetSpeed(350,60);

  hand.SetPose(122);
  //int steps = 30;
  double torig = (3.0/2.0)*pi;
  double tt = torig;
  cout << "tt " << tt << endl;
  for (int i=0;i<18;i++)
    {
      tt = tt-(pi/30.0)*(double)i;
      if (i>=14)
	{
	  cout << "quat " << quat << endl;
	  
	  Quaternion rot = Quaternion("0.707 0.707 0 0");
	  Quaternion newq = Quaternion("0.2726 0.6780 -0.6300 -0.2627");
	  cout << "newq " << newq << endl;
	  hand.SetPose(140);
	  robot.SetCartesianJ(wksp[0]+scale*ampX*cos(tt),wksp[1],wksp[2]+scale*ampY*sin(tt),
			      newq[0],newq[1],newq[2],newq[3]);
	}
      else
	{      
	  robot.SetCartesianJ(wksp[0]+scale*ampX*cos(tt),wksp[1],wksp[2]+scale*ampY*sin(tt),
			      quat[0],quat[1],quat[2],quat[3]);
	}
      // cout << " t " << tt << endl;
      // cout << "x " << ampX*cos(tt) << " y " << ampY*sin(tt) << endl;

    }
  hand.SetPose(140);

  robot.SetSpeed(150, 35);
 
  //robot.SetJoints(0,0,0,0,90,0);
  
  return true;
}

int main (int argc, char** argv) 
{

  ros::init(argc, argv, "droop_server");
  ros::NodeHandle node;

  Tester t (&node);
  t.advertiseServices();
  ROS_INFO("Droop server ready...");
  ros::spin();

  return 0;
}
