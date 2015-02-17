#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <matVec/matVec.h>

// Speed of the robot
#define TCPs 100.0   // Slow speed mm / s
#define ORIs 35.0   // degrees / s

#define HAND_FORCE 0.9
#define HAND_SPEED 0.85
#define HAND_CLOSE_ANGLE 50.0  // Max closing angle
#define HAND_CLOSE_ANGLE_partial 20.0 
#define HAND_OPEN_ANGLE 10.0   // Max opening angle

// Zone of the robot
#define ZONE 0 //0 means every commanded position is a fine position. (No fly-by)

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FlipInAir");
  ros::NodeHandle node;

  RobotComm robot(&node);
  HandComm hand(&node);


  ROS_INFO("Set robot_node configuration");

  //Make sure robot communication is blocking
  if (!robot.SetComm(BLOCKING))
    return -1;
  
  // Set default speed limits
  if (!robot.SetSpeed(TCPs, ORIs))
    return -1;
  
  // Set the default "zone" of our robot (amount of interpolation we allow)
  if (!robot.SetZone(ZONE))
    return -1;
  
  ROS_INFO("Regrasp FlipInAir started...");

  // All cartesian moves are with respect to the proximal left corner of the table.
  // (where the default work object is located)
  // X-axis along the edge that goes left to right (facing the robot)
  // Y-axis along the edge that goes front to back
  // Z-axis pointing up.

  // Auxiliary variables
 
  double motorAngle;
  double fingerAngle[3];
  
  Vec trans(3);
  Vec transC = Vec("600 300 200",3); // Safe location in the center of the robot workspace.
  Quaternion quat;
  Quaternion quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand facing down
  Vec transO = Vec("600 200 100",3);  //Location of Hand before picking the object (Object location + 100 mm in Z direction)
  Vec transPick= Vec("600 200 84",3);  // Pick Point when mat is placed on the table..
  //Vec transPick= Vec("600 200 75",3);  // Pick Point when mat is not placed on the table..
  
  Quaternion quatO = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand grasping the object from top
  

  // " ' " USING SET JOINTS " ' "
  // Do the first command in joint coordinates
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  hand.Calibrate();
  hand.WaitRest(1.0);

  hand.SetSpeed(HAND_SPEED);

  robot.SetCartesian(transO[0], transO[1], transO[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  
  //Open the hand
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(0.250);
  ROS_INFO("Hand Opened");
  robot.SetCartesian(transO[0], transO[1], transO[2]-10, quatO[0], quatO[1], quatO[2], quatO[3]);
  
  //Close the hand
    
  robot.SetCartesian(transPick[0], transPick[1], transPick[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(0.250);
  ROS_INFO("Hand Closed");
    
  //*************************ACTUAL REGRASP STARTS**************
  
  //robot.SetJoints(0,60,20,0,-65,17);
  //robot.SetJoints(0,60.25,20,0,-60,18);
  //robot.SetJoints(0,54.45,20,0,-52.5,27);
 //robot.SetJoints(0,54.45,20,0,-52.5,30);
 //robot.SetJoints(0,54.45,20,0,-52.5,31);
   
   robot.SetJoints(0,20,20,0,-40,31);
   
/*
 For,
robot.SetJoints(0,54.45,20,0,-52.5,31);
simplehands@giant:~$ rosservice call /robot_GetCartesian 
x: 612.35
y: 188.3
z: 130.4
q0: 0.487
qx: 0.72
qy: 0.41
qz: -0.275
ret: 1
*/
  //Vec transDropFlip= Vec("612 188 130",3);
  Vec transDropFlip= Vec("612 188 120.5",3);  // Object is dropped from this location, the location is chosensuch that object flips and land on surface stable..
  Quaternion quatDropFlip = Quaternion("0.487 0.720 0.410 -0.275");
  robot.SetCartesian(transDropFlip[0], transDropFlip[1], transDropFlip[2], quatDropFlip[0], quatDropFlip[1], quatDropFlip[2], quatDropFlip[3]);
  usleep(3000000);
  
  //Open the hand slightly
  hand.GetAngles(motorAngle, fingerAngle);
  printf("finger angles= %lf	%lf	%lf \n",fingerAngle[0], fingerAngle[1], fingerAngle[2]);
  printf("motor angle= %lf \n",motorAngle);
  double minfingerAngle=fingerAngle[0];
  if (fingerAngle[1]<minfingerAngle)
  {
    minfingerAngle=fingerAngle[1];
  }
  if (fingerAngle[2]<minfingerAngle)
  {
    minfingerAngle=fingerAngle[2];
  }
  printf("min finger angle= %lf \n",minfingerAngle);
  printf("drop angle= %lf \n",minfingerAngle-20);
  int sleeptime= 415000; //570000;
  printf("sleep time = %d \n", sleeptime);
 
  //Open the hand
  hand.SetAngle(minfingerAngle-20);
  
  usleep(sleeptime);
  
  hand.SetAngle(HAND_CLOSE_ANGLE-17);
  hand.WaitRest(0.250);
  ROS_INFO("Hand Closed");
  
   //*************************ACTUAL REGRASP ENDS**************
  
  robot.SetJoints(0,0,0,0,90,0);
  
  
  ROS_INFO("Regrasp FlipInAir Completed!");

  return 0;
}
