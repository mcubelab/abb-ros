//Bounceless catching, Minimize the relative velocity between the hand and object at the time of catch 

#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <matVec/matVec.h>

// Speed of the robot
#define TCPs 100.0   // Slow speed mm / s
#define ORIs 35.0   // degrees / s

#define TCPf 350.0   // FAST SPEED mm / s
#define ORIf 50.0   // degrees / s

//#define N_ITERATIONS 150
//#define N_STEPS 50

#define HAND_FORCE 0.9
#define HAND_SPEED 0.9
#define HAND_CLOSE_ANGLE 70.0  // Max closing angle
#define HAND_CLOSE_ANGLE_partial 10.0 
#define HAND_OPEN_ANGLE_drop 60.0 
//#define HAND_OPEN_ANGLE_drop 30.0   // opening angle sufficient enough to drop the object
					//(variable angle???)
#define HAND_OPEN_ANGLE 10.0   // Max opening angle

// Zone of the robot
#define ZONE 0 //0 means every commanded position is a fine position. (No fly-by)
#define ZONE_d 4 // d for drop: zone when robot drops the object and moves down fast.

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FlipInAir");
  //ros::init(argc, argv, "regrasp1");
  ros::NodeHandle node;

  RobotComm robot(&node);
  HandComm hand(&node);


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
  
  ROS_INFO("Regrasp FlipInAirLoop started...");

  // All cartesian moves are with respect to the proximal left corner of the table.
  // (where the default work object is located)
  // X-axis along the edge that goes left to right (facing the robot)
  // Y-axis along the edge that goes front to back
  // Z-axis pointing up.

  // Auxiliary variables
 // HomogTransf pose;
  double motorAngle;
  double fingerAngle[3];
  //double motorAngleD;
  //double fingerAngleD[3];
  Vec trans(3);
  Vec transC = Vec("600 300 200",3); // Safe location in the center of the robot workspace.
  Quaternion quat;
  Quaternion quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand facing down
  Vec transO = Vec("600 200 100",3);  //Location of Hand before picking the object (Object location + 100 mm in Z direction)
  Vec transPick= Vec("600 200 85",3);  // Pick Point when mat is placed on the table..
  //Vec transPick= Vec("600 200 75",3);  // Pick Point when mat is not placed on the table..
  
  Quaternion quatO = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand grasping the object from top
  //Quaternion quatO = Quaternion("0.7071 0.7071 0.0 0.0"); // Hand grasping the object from back
  
  /*
  Quaternion quatOl1= ("0.7071 0.0 0.0 0.7071");
  Quaternion quatOl2= ("0.7071 0.7071 0.0 0.0");
  Quaternion quatO = quatOl1^quatOl2; // Hand grasping the object from left side
  */

  // " ' " USING SET JOINTS " ' "
  // Do the first command in joint coordinates
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  
  hand.Calibrate();
  hand.WaitRest(1.0);
  
for (int i =0; i<50; i++)
{
  hand.SetSpeed(HAND_SPEED);
  
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
    
  //Open the hand
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(0.250);
  //ROS_INFO("Hand Opened");
  robot.SetCartesian(transO[0], transO[1], transO[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  //Close the hand
    
  hand.SetAngle(HAND_CLOSE_ANGLE_partial);
  hand.WaitRest(0.250);
   //ROS_INFO("Hand Closed partial");
   
  //hand.SetAngle(HAND_CLOSE_ANGLE);
  
   robot.SetCartesian(transPick[0], transPick[1], transPick[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(0.250);
 // ROS_INFO("Hand Closed");
    
 // robot.SetJoints(0.0, 0.0, 0.0, 0.0, -60.0, 18.0);
  //robot.SetJoints(-1.17,60,0,0.1,90,-170.04);
  // -1.17 53 8 0.1 93 -157.04
  //front 0 50 20 0 -50 -100
  //robot.SetJoints(0,60,20,0,-65,17);
  //robot.SetJoints(0,60.25,20,0,-60,18);
  //robot.SetJoints(0,54.45,20,0,-52.5,27);
 //robot.SetJoints(0,54.45,20,0,-52.5,30);
 //robot.SetJoints(0,54.45,20,0,-52.5,31);
   
   robot.SetJoints(0,20,20,0,-40,31);
   //robot.SetCartesian(transPick[0], transPick[1], transPick[2]+200, quatO[0], quatO[1], quatO[2], quatO[3]);
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
  Vec transDropFlip= Vec("612 188 132.5",3);  // Object is dropped from this location, the location is such taht object flips and land on surface stable..
  Quaternion quatDropFlip = Quaternion("0.487 0.720 0.410 -0.275");
  robot.SetCartesian(transDropFlip[0], transDropFlip[1], transDropFlip[2], quatDropFlip[0], quatDropFlip[1], quatDropFlip[2], quatDropFlip[3]);
  usleep(3000000);
  
  //Open the hand slightly
  hand.GetAngles(motorAngle, fingerAngle);
  //printf("finger angles= %lf	%lf	%lf \n",fingerAngle[0], fingerAngle[1], fingerAngle[2]);
  //printf("motor angle= %lf \n",motorAngle);
  double minfingerAngle=fingerAngle[0];
  if (fingerAngle[1]<minfingerAngle)
  {
    minfingerAngle=fingerAngle[1];
  }
  if (fingerAngle[2]<minfingerAngle)
  {
    minfingerAngle=fingerAngle[2];
  }
  //printf("min finger angle= %lf \n",minfingerAngle);
 // printf("drop angle= %lf \n",minfingerAngle-20);
  int sleeptime= 570000; //15710*1000/minfingerAngle;   // 91750 to 91800, perfornamnce is not constant
  //printf("sleep time = %d \n", sleeptime);
  
  //1-Open hand just enough to drop the object
  hand.SetAngle(minfingerAngle-20);
  //hand.SetAngle(HAND_OPEN_ANGLE_drop);
  
  usleep(sleeptime);
  
  hand.SetAngle(HAND_CLOSE_ANGLE-20);
  hand.WaitRest(0.250);
  //ROS_INFO("Hand Closed");
  
  //robot.SetJoints(0,25,20,0,-55,18);
  robot.SetJoints(0,0,0,0,90,0);
  //usleep(5000000);
  ROS_INFO("Regrasp #: %d", i);
  }
  /*
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  
  //Open the hand
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(0.250);
  ROS_INFO("Hand Opened");
  
  //robot.SetJoints(-1.17,75,-50,0.1,65,-1.04);
   //Close the hand
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(0.250);
  ROS_INFO("Hand Closed");
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 65.0, 0.0);
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
 */
  
  ROS_INFO("Hurry!! 25 Flip in Air done!");

  return 0;
}
