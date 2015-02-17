#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <matVec/matVec.h>
//#include <stdio.h>
//#include <unistd.h>

// Run the follwing command to start Robotiq node
//rosrun robotiq_c_model_control CModelTcpNode.py 192.168.1.86

// Speed of the robot
#define TCPs 80.0   // Slow speed mm / s
#define ORIs 50.0   // degrees / s

#define TCPss 20.0   // super Slow speed mm / s
#define ORIss 10.0   // degrees / s

// Zone of the robot
#define ZONE 0 //0 means every commanded position is a fine position. (No fly-by)


int main(int argc, char** argv)
{
  ros::init(argc, argv, "linearpushing");
  ros::NodeHandle node;

  RobotComm robot(&node, "1");

  ROS_INFO("Set robot_node configuration");

  // robot.SetWorkObject(WORK_X, WORK_Y, WORK_Z, WORK_Q0, WORK_QX, WORK_QY, WORK_QZ))   // Not necessary
  // robot.SetTool(TOOL_X, TOOL_Y, TOOL_Z, TOOL_Q0, TOOL_QX, TOOL_QY, TOOL_QZ))   // Not necessary

  //Make sure robot communication is blocking
  if (!robot.SetComm(BLOCKING))
    return -1;
  
  // Set default speed limits
  if (!robot.SetSpeed(TCPs, ORIs))
    return -1;
  
  // Set the default "zone" of our robot (amount of interpolation we allow)
  if (!robot.SetZone(ZONE))
    return -1;
  
  ROS_INFO("Prehensile Pushing demo...");

  // All cartesian moves are with respect to the proximal left corner of the table.
  // (where the default work object is located)
  // X-axis along the edge that goes left to right (facing the robot)
  // Y-axis along the edge that goes front to back
  // Z-axis pointing up.

 // Auxiliary variables
 // HomogTransf pose;

  Vec trans(3);
  Vec transC = Vec("1300 300 200",3); // Safe location in the center of the robot workspace.
  Quaternion quat;
  Quaternion quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand facing down
  Quaternion quatR = Quaternion("0.0 0.0 -1.0 0.0"); // Hand rotated
  Vec transPick= Vec("1300 300 52",3); //for hex object 1300 300 56.25; // for rect 1300 300 77
  Vec transLift= Vec("1300 300 150",3);  // Lift the hand upto this point while the object is drooping in the hand.
  Vec transPlace= Vec("1300 300 150",3);  // Place Point after droop action
  Vec transPush_safe= Vec("1500 256 162",3);  // before and after push z=162.5 is center (center ~240 for hex and for rect = ~235)
  Vec transPush_end= Vec("1600 256 162",3); //1628 object touches for hex z=161.75 mid
  //Vec transPush_end= Vec("1586 257 163",3); //1628 object touches for rect
  
  // " ' " USING SET JOINTS " ' "
  // Do the first command in joint coordinates
  
  robot.SetSpeed(TCPs, ORIs);
  robot.SetZone(ZONE);
    
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  
  
  //Activate the gripper
  //Be ready to close the fingers
  usleep(3000000);

  robot.SetCartesian(transPick[0], transPick[1], transPick[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  
  //Close the gripper
  usleep(5000000);
  
    
  robot.SetCartesian(transLift[0], transLift[1], transLift[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  
  robot.SetCartesian(transLift[0], transLift[1], transLift[2], quatR[0], quatR[1], quatR[2], quatR[3]);
  
  robot.SetCartesian(transPush_safe[0], transPush_safe[1], transPush_safe[2], quatR[0], quatR[1], quatR[2], quatR[3]);
  
    
    //*************************PUSHING STARTS**************
   
  //Start recording the force on the F/T sensor

  robot.SetSpeed(TCPss, ORIss);
  usleep(3000000);
  
  robot.SetCartesian(transPush_end[0], transPush_end[1], transPush_end[2], quatR[0], quatR[1], quatR[2], quatR[3]);
  
  usleep(3000000);
  
    robot.SetCartesian(transPush_safe[0], transPush_safe[1], transPush_safe[2], quatR[0], quatR[1], quatR[2], quatR[3]);

   //Start recording the force on the F/T sensor
  //the max value is important to us
  
  //*************************PUSHING ENDS**************
  
  
  robot.SetSpeed(TCPs, ORIs);
  
  robot.SetCartesian(transPlace[0], transPlace[1], transPlace[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  
  //Open the gripper
  usleep(5000000);

  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);

  ROS_INFO("Prehensile linear pushing Completed!");

  return 0;
}
