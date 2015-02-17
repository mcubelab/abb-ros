#include <ros/ros.h>
#include <ros/package.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <matVec/matVec.h>
#include <util_comm/util_comm.h>
#include <objRec_comm/objRec_comm.h>
#include <matlab_comm/matlab_comm.h>
#include <tableVision_comm/tableVision_comm.h>
#include <regraspComm/regrasp_comm.h>
//#include <planner/Plan.h>
#include <regraspComm/Regrasp.h>
#include <math.h> 
#include <iostream>

#include <ctime>
#include <unistd.h>

#include <stdio.h>
#include <limits.h>

using namespace std;

#define PI 3.14159

#define MOVE_TCP 100.0   // Slow speed mm / s
#define MOVE_ORI 35.0   // degrees / s

#define HAND_FORCE 0.90
#define HAND_SPEED 0.90

int main(int argc, char** argv)
{

  ros::init(argc, argv, "Executioner");
  ros::NodeHandle node;

  RobotComm robot(&node);
  HandComm hand(&node);
  RegraspComm regrasp(&node);
  ObjRecComm objRec(&node);
  TableVisionComm tableVision(&node);

  MatlabComm matlab;
  matlab.subscribe(&node);
  std::string pkg_path = ros::package::getPath("tableVision_node");
  pkg_path += "/matlab_scripts";
  matlab.addPath(pkg_path.c_str());

  // inital setup
  hand.SetForce(HAND_FORCE);
  hand.SetSpeed(HAND_SPEED);
  robot.SetDefaults();
  robot.SetSpeed(MOVE_TCP, MOVE_ORI);

  robot.SetJoints(0,0,0,0,90,0);

  // chose an initial Theta 
  Quaternion baseline = Quaternion("0.0419 0.0591 0.8264 -0.5577");
  double theta_n = -69; //-72.5 is good//21 * (PI/180.0);
  //double theta_new = theta_n;
  // step size of keifer wolfowitz
  double a_n = 10;//5 * (PI/180.0);
  // exploration param
  double c_n = 2;//2 * (PI/180.0);
  cout << "theta_init " << theta_n << endl;
  cout << "a_n " << a_n << " c_n " << c_n << endl;

  // temp
  //for (int j=0;j<1;j++)
  while (true)
    {
      ROS_INFO("theta_n %f", theta_n);

      ROS_INFO("Calculating plus/minus quats");
      matlab.sendVec("baseline", baseline);
      matlab.sendValue("theta_plus", theta_n + c_n);
      matlab.sendCommand("roll_plus = angle2quat(0,0,theta_plus);");
      matlab.sendCommand("quat_plus = quatmultiply(baseline,roll_plus);");
      Quaternion quat_plus = matlab.getQuaternion("quat_plus");
      matlab.sendValue("theta_minus", theta_n - c_n);
      matlab.sendCommand("roll_minus = angle2quat(0,0,theta_minus);");
      matlab.sendCommand("quat_minus = quatmultiply(baseline,roll_minus);");
      Quaternion quat_minus = matlab.getQuaternion("quat_minus");
      double psum = quat_plus[0]+quat_plus[1]+quat_plus[2]+quat_plus[3];
      double msum = quat_minus[0]+quat_minus[1]+quat_minus[2]+quat_minus[3];
      if (psum < 0.0001)
      	quat_plus = Quaternion("0.0008 -0.0017 0.7132 -0.7009");
      if (msum < 0.0001)
      	quat_minus = Quaternion("0.0008 -0.0017 0.7132 -0.7009");


      cout << "quat_plus " << quat_plus << endl;
      cout << "quat_minus " << quat_minus << endl;


      vector<regraspComm::Regrasp> plan_plus;
      plan_plus.push_back(regrasp.getDefaults(PICK));
      regraspComm::Regrasp prtp = regrasp.getDefaults(ROLL_TO_PALM);
 
      prtp.rollToPalm.angle = theta_n+c_n;
      cout << "angle " << prtp.rollToPalm.angle << endl;
      // prtg.rollToGround.flip_pose.orientation.w = quat_plus[0];
      // prtg.rollToGround.flip_pose.orientation.x = quat_plus[1];
      // prtg.rollToGround.flip_pose.orientation.y = quat_plus[2];
      // prtg.rollToGround.flip_pose.orientation.z = quat_plus[3];
      
      plan_plus.push_back(prtp);
      
      // number of trials to create a prob 
      int trials = 5;
      int p_plus[trials];
      for (int i=0; i<trials; i++)
      	{
      	  ROS_INFO("Starting plus trial %d", i);
      	  double mot_ang;
      	  Vec ini_fing_angs(NUM_FINGERS);
      	  Vec cur_fing_angs(NUM_FINGERS);
	  
      	  hand.GetAngles(mot_ang, ini_fing_angs);
	  
      	  cout << "Please push one of my fingers to grasp an object!" << endl;
	  
      	  ros::Rate loop_rate(20);
      	  while(ros::ok())
      	    {
      	      // Get the current finger angles, and if they're significantly 
      	      // different, a finger has been moved!
      	      hand.GetAngles(mot_ang, cur_fing_angs);
      	      if ((ini_fing_angs - cur_fing_angs).norm() > 2.0)
      		break;
      	      loop_rate.sleep();
      	    } 
      	  hand.SetAngle(0);

      	  cout << "plan len " << plan_plus.size() << endl;
      	  cout << "plan " << plan_plus[0].action << " " << plan_plus[1].action << endl;
      	  cout << "angle " << plan_plus[1].rollToPalm.angle << endl;
      	  regrasp.Execute(plan_plus);
      	  ROS_INFO("Done with regrasp");
      	  robot.SetJoints(25,0,0,0,90,0);
	  
      	  // Evaluate
      	  cin.ignore ( std::numeric_limits<std::streamsize>::max(), '\n' );
      	  cin.clear();
      	  cout << "Success(1) or Failure(0)?... " << endl;
      	  char line[100];
      	  cin.get(line, 100);
      	  cout << "you said:" << line << "." << endl;
      	  p_plus[i] = atoi(line);
      	  cout << p_plus[i] << endl;

      	  // int count = 0;
      	  // bool success = false;
      	  // int obj;
      	  // Vec blockVec(3);
      	  // Quaternion blockQuat;
	  
      	  // do 
      	  //   {
      	  //     success = tableVision.GetObjAndPose(RecObj::BIG_TRIANGLE, obj, blockVec, blockQuat);
      	  //     count++;
      	  //   }
      	  // while ((!success) && (count<5));
	  
      	  // cout << "result of trial " << i << " is " << obj << endl;
      	  // if (obj==4) // long triangle
      	  //   p_plus[i] = 1;
      	  // else
      	  //   p_plus[i] = 0; 
      	}

      vector<regraspComm::Regrasp> plan_minus;
      plan_minus.push_back(regrasp.getDefaults(PICK));
      regraspComm::Regrasp mrtp = regrasp.getDefaults(ROLL_TO_PALM);
      
      mrtp.rollToPalm.angle = theta_n-c_n;
      cout << "angle " << mrtp.rollToPalm.angle << endl;
      // mrtp.rollToPalm.flip_pose.orientation.x = quat_minus[1];
      // mrtp.rollToPalm.flip_pose.orientation.y = quat_minus[2];
      // mrtp.rollToPalm.flip_pose.orientation.z = quat_minus[3];
      
      plan_minus.push_back(mrtp);
      int p_minus[trials];
      for (int i=0; i<trials; i++)
      	{
      	  ROS_INFO("Starting minus trial %d", i);

      	  double mot_ang;
      	  Vec ini_fing_angs(NUM_FINGERS);
      	  Vec cur_fing_angs(NUM_FINGERS);
	  
      	  hand.GetAngles(mot_ang, ini_fing_angs);
	  
      	  cout << "Please push one of my fingers to grasp an object!" << endl;
	  
      	  ros::Rate loop_rate(20);
      	  while(ros::ok())
      	    {
      	      // Get the current finger angles, and if they're significantly 
      	      // different, a finger has been moved!
      	      hand.GetAngles(mot_ang, cur_fing_angs);
      	      if ((ini_fing_angs - cur_fing_angs).norm() > 2.0)
      		break;
      	      loop_rate.sleep();
      	    } 
      	  hand.SetAngle(0);
	  
      	  regrasp.Execute(plan_minus);
      	  ROS_INFO("Done with regrasp");
      	  robot.SetJoints(25,0,0,0,90,0);

      	  // Evaluate
      	  cin.ignore ( std::numeric_limits<std::streamsize>::max(), '\n' );
      	  cin.clear();
      	  cout << "Success(1) or Failure(0)?... " << endl;
      	  char line[100];
      	  cin.get(line, 100);
      	  cout << "you said:" << line << "." << endl;
      	  p_minus[i] = atoi(line);
      	  cout << p_minus[i] << endl;

      	  // int count = 0;
      	  // bool success = false;
      	  // int obj;
      	  // Vec blockVec(3);
      	  // Quaternion blockQuat;
	  
      	  // do 
      	  //   {
      	  //     success = tableVision.GetObjAndPose(RecObj::BIG_TRIANGLE, obj, blockVec, blockQuat);
      	  //     count++;
      	  //   }
      	  // while ((!success) && (count<5));
	  
      	  // cout << "result of trial " << i << " is " << obj << endl;
      	  // if (obj==4) // long triangle
      	  //   p_minus[i] = 1;
      	  // else
      	  //   p_minus[i] = 0;
	  
      	} 
      cout << "p_plus " << p_plus[0] << " " << p_plus[1] << " " << p_plus[2] << " " << p_plus[3] << " " << p_plus[4] << endl;
      cout << "p_minus " << p_minus[0] << " " << p_minus[1] << " " << p_minus[2] << " " << p_minus[3] << " " << p_minus[4] << endl;
      // find the p value
      double pp = 0;
      double pm = 0;
      for(int i=0;i<trials;i++)
      	{
      	  pp += p_plus[i];
      	  pm += p_minus[i];
      	}
      
      pp = pp/trials;
      pm = pm/trials;
      
      //kieferwolfowitz update step 
      double theta_new = theta_n + a_n * ((pp - pm)/c_n);
      cout << "theta_new " << theta_new << endl;
      
      if (abs(theta_new-theta_n) < 1)
	break;
      else
	theta_n = theta_new;
	

    }

}

