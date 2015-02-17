#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <regrasp_comm/regrasp_comm.h>
#include <matVec/matVec.h>

// Speed of the robot
#define TCP 45
#define ORI 15

// Speed and Force of the hand
#define HAND_SPD 0.8
#define HAND_FORCE 0.8

// Opening and closing finger
#define HAND_OPEN 20.0
#define HAND_RELEASE 74.0
#define HAND_CLOSE 110.0

// Zone of the robot
#define ZONE 0 //0 means every commanded position is a fine position. (No fly-by)

const double angle = 90.0*PI/180.0; // The rotation angle

ros::NodeHandle *nodePtr;
RobotComm robot;
HandComm hand;
RegraspComm regrasp;

// Globle variables:
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

Vec frame = Vec("100 350 75",3); // Frame position
Vec cam = Vec("550 280 270",3);   // Camera position
Vec trans(3);                     // A temporary vector
Vec transC = Vec("600 285 250",3);// Safe location in the center of the robot workspace.
Vec table = Vec("600 285 0",3);

Vec x = Vec("1.0 0.0 0.0", 3);
Vec y = Vec("0.0 1.0 0.0", 3);
Vec z = Vec("0.0 0.0 1.0", 3);

// Functions:
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

void initialize()
{  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  ROS_INFO("Initializeded");
}
void testhand()
{
  hand.SetAngle(HAND_OPEN);
  hand.WaitRest(1.25);
  hand.SetAngle(HAND_CLOSE);
  hand.WaitRest(1.25);
  ROS_INFO("Test finished");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Regrasp_Newhand");
  ros::NodeHandle node;
  nodePtr = &node;

  robot.subscribe(nodePtr);
  hand.subscribe(nodePtr);
  regrasp.subscribe(nodePtr);

  hand.Calibrate();

  hand.SetSpeed(HAND_SPD);
  hand.SetForce(HAND_FORCE);

   
  // Make sure robot communication is blocking
  if (!robot.SetComm(BLOCKING))
    return -1;
  
  // Set default speed limits
  if (!robot.SetSpeed(TCP, ORI))
    return -1;
  
  // Set the default "zone" of our robot (amount of interpolation we allow)
  if (!robot.SetZone(ZONE))
    return -1;
  
  ROS_INFO("Beginning test...");

  // All cartesian moves are with respect to the proximal left corner of the table.
  // (where the default work object is located)
  // X-axis along the edge that goes left to right (facing the robot)
  // Y-axis along the edge that goes front to back
  // Z-axis pointing up.


int i=0;
while(true)
{
  i+=1;
  ROS_INFO("Action 1 in loop %d",i);
  regrasp.StandtoLie(0, 0, 80, -1.45);  // From -2.0 to 2.0
  regrasp.LietoStand(0, 0, 110, 0);    // From -0.5 to 0.5
  regrasp.Rotate(-1.5);

  ROS_INFO("Action 2 in loop %d",i);
  regrasp.StandtoLie(0, 0, 80, -0.7);
  regrasp.LietoStand(0, 0, 110, -0.25);    // From -0.5 to 0.5
  regrasp.Rotate(1.5);

  ROS_INFO("Action 3 in loop %d",i);
  regrasp.StandtoLie(0, 0, 80, -1.2); // From -2.0 to 2.0
  regrasp.LietoStand(0, 0, 100,0);   // From -0.5 to 0.5
  regrasp.Rotate(0);

  ROS_INFO("Action 4 in loop %d",i);
  regrasp.StandtoLie(0, 0, 80, 1.2); // From -2.0 to 2.0
  regrasp.LietoStand(0, 0, 110,0);   // From -0.5 to 0.5
  regrasp.Rotate(-1.5);
}

  ROS_INFO("Demo Completed!");

  return 0;
}
