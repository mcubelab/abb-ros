#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <matVec/matVec.h>

// Speed of the robot
#define TCPs 100.0   // Slow speed mm / s
#define ORIs 30.0   // degrees / s

#define HAND_FORCE 0.90
#define HAND_SPEED 0.95
#define HAND_CLOSE_ANGLE 55.0 //60.0  // Max closing angle
#define HAND_CLOSE_ANGLE_partial 20.0 
//#define HAND_OPEN_ANGLE_drop 60.0 
//#define HAND_OPEN_ANGLE_drop 30.0   // opening angle sufficient enough to drop the object
					//(variable angle???)
#define HAND_OPEN_ANGLE 10.0   // Max opening angle

// Zone of the robot
#define ZONE 0 //0 means every commanded position is a fine position. (No fly-by)

int main(int argc, char** argv)
{
  //ros::init(argc, argv, "regrasp");
  ros::init(argc, argv, "FlipInHand");
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
  
  ROS_INFO("Regrasp FlipInHand started...");

  // All cartesian moves are with respect to the proximal left corner of the table.
  // (where the default work object is located)
  // X-axis along the edge that goes left to right (facing the robot)
  // Y-axis along the edge that goes front to back
  // Z-axis pointing up.

  // Auxiliary variables
 // HomogTransf pose;
  double motorAngle;
  double fingerAngle[3];
  double motorAngleD;
  double fingerAngleD[3];
  Vec trans(3);
  Vec transC = Vec("600 300 200",3); // Safe location in the center of the robot workspace.
  Quaternion quat;
  Quaternion quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand facing down
  Vec transO = Vec("600 200 100",3);  //Location of Hand before picking the object (Object location + 100 mm in Z direction)
  Vec transPick= Vec("600 200 79",3);  // Pick Point when mat is placed on the table..
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
  
  robot.SetSpeed(TCPs, ORIs);
  robot.SetZone(ZONE);
  
  hand.SetSpeed(HAND_SPEED);
  hand.SetForce(HAND_FORCE);
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  
  //hand.SetEncoder(15000);
  //usleep(2000000);
  hand.Calibrate();
  hand.WaitRest(1.0);

 for (int i=0; i<50; i++)
 {
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  hand.SetSpeed(HAND_SPEED);
  hand.SetForce(HAND_FORCE);

  robot.SetCartesian(transO[0], transO[1], transO[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  
  //Open the hand
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(0.250);
  //ROS_INFO("Hand Opened");
  robot.SetCartesian(transO[0], transO[1], transO[2]-10, quatO[0], quatO[1], quatO[2], quatO[3]);
  //Close the hand
  
  hand.SetAngle(HAND_CLOSE_ANGLE_partial);
  hand.WaitRest(0.250);
  // ROS_INFO("Hand Closed partial");
  //hand.SetAngle(HAND_CLOSE_ANGLE);
  
  //robot.SetCartesian(transO[0], transO[1], transO[2]-28, quatO[0], quatO[1], quatO[2], quatO[3]);
  robot.SetCartesian(transPick[0], transPick[1], transPick[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(0.250);
 // ROS_INFO("Hand Closed");
  
  //robot.SetJoints(-1.37,31.99,16.84,0.1,-72,30);
  //robot.SetJoints(-1.37,31.99,16.84,0.1,-90,-90);
  //robot.SetJoints(-1.37,31.99,16.84,0.1,-68,-90);  // for P-3 with new friction tip fingers
  robot.SetJoints(-1.37,31.99,16.84,0.1,-75.5,-88);    // for P-3.5
  usleep(2000000);
  
  //Open the hand slightly
  hand.GetAngles(motorAngle, fingerAngle);
 // printf("finger angles= %lf	%lf	%lf \n",fingerAngle[0], fingerAngle[1], fingerAngle[2]);
 // printf("motor angle= %lf \n",motorAngle);
  double minfingerAngle=fingerAngle[0];
  if (fingerAngle[1]<minfingerAngle)
  {
    minfingerAngle=fingerAngle[1];
  }
  if (fingerAngle[2]<minfingerAngle)
  {
    minfingerAngle=fingerAngle[2];
  }
 // printf("min finger angle= %lf \n",minfingerAngle);
 // printf("drop angle= %lf \n",minfingerAngle-20);
  int sleeptime= 648000; //645000; //645000; //730000; //2000000; //Variable time would be ideal   
  //printf("sleep time = %d \n", sleeptime);
  
  //1-Open hand just enough to drop the object
  hand.SetAngle(minfingerAngle-20);
  //hand.SetAngle(HAND_OPEN_ANGLE_drop);
  
  usleep(sleeptime);
  
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(0.250);
  //ROS_INFO("Hand Closed");
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(0.250);
  printf("\n Regrasp Run #: %d\n", i);
  usleep(3000000);
  }
  
  //robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  //robot.SetCartesian(transO[0], transO[1], transO[2]-5, quatO[0], quatO[1], quatO[2], quatO[3]);
  //hand.SetAngle(HAND_OPEN_ANGLE);
  //hand.WaitRest(0.250);
  //ROS_INFO("object placed");
  //robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);

  ROS_INFO("Regrasp FlipInHand Completed!");

  return 0;
}
