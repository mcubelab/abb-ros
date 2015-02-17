#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <matVec/matVec.h>
#include <vision_comm/vision_comm.h>

// Speed of the robot
#define TCPs 80.0   // Slow speed mm / s
#define ORIs 25.0   // degrees / s

#define TCPf 350.0   // FAST SPEED mm / s
#define ORIf 50.0   // degrees / s

#define TCPss 40.0   // SuperSlow speed mm / s
#define ORIss 20.0   // degrees / s

#define HAND_FORCE 0.99
#define HAND_SPEED 0.90
#define HAND_SPEED_F2E 0.65

#define HAND_CLOSE_ANGLE 70.0  // Max closing angle changed from 60
#define HAND_CLOSE_ANGLE_partial 20.0 
#define HAND_OPEN_ANGLE 20.0   // Max opening angle
#define HAND_OPEN_ANGLE_vibrate 43.0

// Zone of the robot
#define ZONE 0 //0 means every commanded position is a fine position. (No fly-by)

int main(int argc, char** argv)
{

  //ros::init(argc, argv, "regrasp");
  ros::init(argc, argv, "GroundRoll");
  ros::NodeHandle node;

  RobotComm robot(&node);
  HandComm hand(&node);
  VisionComm vision(&node);
    
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
  
  ROS_INFO("Regrasp GroundRoll started...");

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
  int Tsleep_F2E= 1085; //1078; //486000;
    
  Vec trans(3);
  Vec transC = Vec("600 300 200",3); // Safe location in the center of the robot workspace.
  Quaternion quat;
  Quaternion quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand facing down
  Vec transO = Vec("600 200 100",3);  //Location of Hand before picking the object (Object location + 100 mm in Z direction)
  Vec transPick=Vec("400 150 80",3);	//Pick Point
  Vec transCamera=Vec("250 280 170",3); 
  
  Quaternion quatO = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand grasping the object from top
  //Quaternion quatO = Quaternion("0.7071 0.7071 0.0 0.0"); // Hand grasping the object from back
  
  
  //Quaternion quatOl1= ("0.7071 0.0 0.0 0.7071");
  //Quaternion quatOl2= ("0.7071 0.7071 0.0 0.0");
  //Quaternion quatO = quatOl1^quatOl2; // Hand grasping the object from left side
  

  // " ' " USING SET JOINTS " ' "
  // Do the first command in joint coordinates
  
  robot.SetSpeed(TCPs, ORIs);
  robot.SetZone(ZONE);
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  hand.Calibrate();
  hand.WaitRest(1.0);

  hand.SetSpeed(HAND_SPEED);
  hand.SetForce(HAND_FORCE);
  
  robot.SetCartesian(transO[0], transO[1], transO[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  
  for (int i=0; i<50; i++)
  {
    hand.SetAngle(HAND_OPEN_ANGLE);
    hand.WaitRest(1);
    ROS_INFO("Hand Opened");
    //robot.SetCartesian(transO[0], transO[1], transO[2]-10, quatO[0], quatO[1], quatO[2], quatO[3]);

    //Close the hand
    
    hand.SetAngle(HAND_CLOSE_ANGLE_partial);
    hand.WaitRest(1);
    // ROS_INFO("Hand Closed partial");
    //hand.SetAngle(HAND_CLOSE_ANGLE);
    
    usleep(3000000);
    //robot.SetCartesian(transO[0], transO[1], transO[2]-28, quatO[0], quatO[1], quatO[2], quatO[3]);
    robot.SetCartesian(transPick[0], transPick[1], transPick[2]+50, quatO[0], quatO[1], quatO[2], quatO[3]);
    robot.SetCartesian(transPick[0], transPick[1], transPick[2], quatO[0], quatO[1], quatO[2], quatO[3]);
    
    hand.SetAngle(HAND_CLOSE_ANGLE);
    hand.WaitRest(1);
    //ROS_INFO("Hand Closed");
    
    robot.SetCartesian(transPick[0], transPick[1], transPick[2]+50, quatO[0], quatO[1], quatO[2], quatO[3]);
    
    robot.SetCartesian(transCamera[0], transCamera[1], transCamera[2], quatO[0], quatO[1], quatO[2], quatO[3]);
    usleep(2000000);
    string filename;
    vision.CaptureImage(0, filename);
    
    robot.SetCartesian(transPick[0], transPick[1], transPick[2], quatO[0], quatO[1], quatO[2], quatO[3]);
    
    hand.GetAngles(motorAngle, fingerAngle);
    //printf("Before Roll finger angles= %lf	%lf	%lf \n",fingerAngle[0], fingerAngle[1], fingerAngle[2]);
    //printf("motor angle= %lf \n",motorAngle);
    double RollminfingerAngle=fingerAngle[0];
    if (fingerAngle[1]<RollminfingerAngle)
    {
      RollminfingerAngle=fingerAngle[1];
    }
    if (fingerAngle[2]<RollminfingerAngle)
    {
      RollminfingerAngle=fingerAngle[2];
    }
    
   // printf("RollMin finger angle= %lf \n",RollminfingerAngle);
    //printf("drop angle= %lf \n",minfingerAngle-10);
    double RollReleaseAngle= RollminfingerAngle-2;
  //  printf("Roll release angle= %lf \n",RollReleaseAngle);

    hand.SetAngle(RollReleaseAngle);
    hand.WaitRest(1); 
    usleep(2000000);
    
    robot.SetSpeed(TCPss, ORIss);
    robot.SetCartesian(transPick[0]-58, transPick[1], transPick[2], quatO[0], quatO[1], quatO[2], quatO[3]);  // X-movement 109.95 for full rotation = circumfernce of cyl dia 35 mm
    hand.SetAngle(HAND_CLOSE_ANGLE);
    hand.WaitRest(1);
    usleep(1000000);
    
    robot.SetSpeed(TCPs, ORIs);
    robot.SetCartesian(transPick[0]-58, transPick[1], transPick[2]+50, quatO[0], quatO[1], quatO[2], quatO[3]);
    robot.SetCartesian(transCamera[0], transCamera[1], transCamera[2], quatO[0], quatO[1], quatO[2], quatO[3]);
    usleep(2000000);
    vision.CaptureImage(0, filename);
    
    robot.SetCartesian(transPick[0], transPick[1], transPick[2]+150, quatO[0], quatO[1], quatO[2], quatO[3]);
      
    ROS_INFO("Regrasp Run #: %d", i);
  }
  
  //robot.SetCartesian(transPick[0], transPick[1], transPick[2]+200, quatO[0], quatO[1], quatO[2], quatO[3]);
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  ROS_INFO("GroundRoll Completed!");



  return 0;
}
