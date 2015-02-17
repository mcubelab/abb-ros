#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <matVec/matVec.h>
//#include <stdio.h>
//#include <unistd.h>

// Run the follwing command to start Robotiq node
//rosrun robotiq_c_model_control CModelTcpNode.py 192.168.1.86

// Speed of the robot
#define TCPs 80.0   // Slow speed mm / s
#define ORIs 35.0   // degrees / s

#define TCPss 20.0   // super Slow speed mm / s
#define ORIss 10.0   // degrees / s

// Zone of the robot
#define ZONE 0 //0 means every commanded position is a fine position. (No fly-by)


int main(int argc, char** argv)
{
  ros::init(argc, argv, "amazon");
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

  Vec transPickDuck= Vec("1060 250 165",3); //for hex object 1300 300 56.25; // for rect 1300 300 77

  Vec transLiftDuck= Vec("1060 250 350",3);  // Lift the hand upto this point while the object is drooping in the hand.

  Vec transPlaceDuck= Vec("1560 300 150",3);  // Place Point after droop action
  Vec transPlaceDuck2= Vec("1060 250 150",3);  // Place Point after droop action

Vec transPickEraser= Vec("1060 243 70",3); //for hex object 1300 300 56.25; // for rect 1300 300 77

  Vec transLiftEraser= Vec("1060 243 350",3);  // Lift the hand upto this point while the object is drooping in the hand.

  Vec transPlaceEraser= Vec("1460 250 71",3);  // Place Point after droop action


Vec transPickToy= Vec("880 250 105",3); //for hex object 1300 300 56.25; // for rect 1300 300 77

  Vec transLiftToy= Vec("880 250 250",3);  // Lift the hand upto this point while the object is drooping in the hand.

  Vec transPlaceToy= Vec("1550 250 120",3);  // Place Point after droop action


  
  // " ' " USING SET JOINTS " ' "
  // Do the first command in joint coordinates
  
  robot.SetSpeed(TCPs, ORIs);
  robot.SetZone(ZONE);
    
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
usleep(3000000);
  //robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  
//+++++++++++++DUCK Out+++++++++++++++++++

  robot.SetCartesian(transPickDuck[0], transPickDuck[1], transPickDuck[2]+200, quatC[0], quatC[1], quatC[2], quatC[3]);

  robot.SetCartesian(transPickDuck[0], transPickDuck[1], transPickDuck[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  
  //Close the gripper
  usleep(3000000);
  
    
  robot.SetCartesian(transLiftDuck[0], transLiftDuck[1], transLiftDuck[2], quatC[0], quatC[1], quatC[2], quatC[3]);

robot.SetCartesian(transPlaceDuck[0], transPlaceDuck[1], transPlaceDuck[2], quatC[0], quatC[1], quatC[2], quatC[3]);

  //Open the gripper
  usleep(3000000);

robot.SetCartesian(transPlaceDuck[0], transPlaceDuck[1], transPlaceDuck[2]+150, quatC[0], quatC[1], quatC[2], quatC[3]);

  
  //+++++++++++++Eraser Out+++++++++++++++++++

 robot.SetCartesian(transPickEraser[0], transPickEraser[1], transPickEraser[2]+250, quatC[0], quatC[1], quatC[2], quatC[3]);

  robot.SetCartesian(transPickEraser[0], transPickEraser[1], transPickEraser[2]+100, quatR[0], quatR[1], quatR[2], quatR[3]);

  robot.SetCartesian(transPickEraser[0], transPickEraser[1], transPickEraser[2], quatR[0], quatR[1], quatR[2], quatR[3]);
  
  //Close the gripper
  usleep(3000000);
  
  robot.SetCartesian(transLiftEraser[0], transLiftEraser[1], transLiftEraser[2], quatR[0], quatR[1], quatR[2], quatR[3]);

    
  robot.SetCartesian(transPlaceEraser[0], transPlaceEraser[1], transPlaceEraser[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  
  //Open the gripper
  usleep(3000000);

  robot.SetCartesian(transPlaceEraser[0], transPlaceEraser[1], transPlaceEraser[2]+200, quatC[0], quatC[1], quatC[2], quatC[3]);

robot.SetCartesian(transPlaceDuck[0], transPlaceDuck[1], transPlaceDuck[2]+100, quatC[0], quatC[1], quatC[2], quatC[3]);

  robot.SetCartesian(transPlaceDuck[0], transPlaceDuck[1], transPlaceDuck[2]-30, quatC[0], quatC[1], quatC[2], quatC[3]);

  //Close the gripper
  usleep(3000000);

  robot.SetCartesian(transPlaceDuck[0], transPlaceDuck[1], transPlaceDuck[2]+150, quatC[0], quatC[1], quatC[2], quatC[3]);

robot.SetCartesian(transPlaceDuck2[0], transPlaceDuck2[1], transPlaceDuck2[2]+250, quatC[0], quatC[1], quatC[2], quatC[3]);

robot.SetCartesian(transPlaceDuck2[0], transPlaceDuck2[1], transPlaceDuck2[2], quatC[0], quatC[1], quatC[2], quatC[3]);

  //Open the gripper
  usleep(3000000);

robot.SetCartesian(transPlaceDuck2[0], transPlaceDuck2[1], transPlaceDuck2[2]+250, quatC[0], quatC[1], quatC[2], quatC[3]);

robot.SetCartesian(transPickToy[0], transPickToy[1], transPickToy[2]+150, quatC[0], quatC[1], quatC[2], quatC[3]);

robot.SetCartesian(transPickToy[0], transPickToy[1], transPickToy[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  
      //Close the gripper
  usleep(3000000);

robot.SetCartesian(transLiftToy[0], transLiftToy[1], transLiftToy[2], quatC[0], quatC[1], quatC[2], quatC[3]);

robot.SetCartesian(transLiftToy[0]+300, transLiftToy[1], transLiftToy[2]+30, quatC[0], quatC[1], quatC[2], quatC[3]);

robot.SetCartesian(transPlaceToy[0], transPlaceToy[1], transPlaceToy[2]+200, quatC[0], quatC[1], quatC[2], quatC[3]);

robot.SetCartesian(transPlaceToy[0], transPlaceToy[1], transPlaceToy[2], quatC[0], quatC[1], quatC[2], quatC[3]);

//Open the gripper
  usleep(3000000);

robot.SetCartesian(transPlaceToy[0], transPlaceToy[1], transPlaceToy[2]+100, quatC[0], quatC[1], quatC[2], quatC[3]);



  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);

  ROS_INFO("Prehensile linear pushing Completed!");

  return 0;
}
