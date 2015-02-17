// ROLL ANGLE CHANGED, double RollAngle=17; earlier 17.5 
//CHANGES NEEDED IN regrasp_node.cpp   ADD move robot command

#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <matVec/matVec.h>

// Speed of the robot
#define TCPs 80.0   // Slow speed mm / s
#define ORIs 25.0   // degrees / s

#define TCPf 350.0   // FAST SPEED mm / s
#define ORIf 50.0   // degrees / s

#define TCPss 40.0   // SuperSlow speed mm / s
#define ORIss 20.0   // degrees / s

#define HAND_FORCE 0.99
#define HAND_SPEED 0.90
#define HAND_SPEED_F2E 0.9

#define HAND_CLOSE_ANGLE 60.0  // Max closing angle changed from 70
#define HAND_CLOSE_ANGLE_partial 20.0 
#define HAND_OPEN_ANGLE 20.0   // Max opening angle
#define HAND_OPEN_ANGLE_vibrate 43.0

// Zone of the robot
#define ZONE 0 //0 means every commanded position is a fine position. (No fly-by)

int main(int argc, char** argv)
{
  //ros::init(argc, argv, "regrasp");
  ros::init(argc, argv, "RollInHand");
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
  
  ROS_INFO("Regrasp RollInHand started...");

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
  int Tsleep_F2E= 10000; //1085; //1078; //486000;
  double RollAngle=17; //10; 
    
  Vec trans(3);
  Vec transC = Vec("600 300 200",3); // Safe location in the center of the robot workspace.
  Quaternion quat;
  Quaternion quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand facing down
  Vec transO = Vec("600 200 100",3);  //Location of Hand before picking the object (Object location + 100 mm in Z direction)
  Vec transPick=Vec("600 200 73",3)+Vec("0 0 6.5",3);	//Pick Point
  //Vec transSafe=Vec("700 325 810",3);	//Safe Point
  //Vec transPushStop=Vec("801 325 810",3);	//Push Stop Point
  
  Vec transTouchPt = Vec("935.25 235 790",3); //the point at which the cylinder will touch the wall/external fixture
  //(If we put a coordinate frame there, the orientation of that coordinate frame is same as 

  Vec transPushStop = transTouchPt - Vec("70 0 0",3);	//Push Stop Point
  
  Vec transSafe = transPushStop - Vec("100 0 0",3);	//Safe Point
    
  Quaternion quatO = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand grasping the object from top
  
  Quaternion quatPush = Quaternion("1 0 0 0"); // Hand pushing an object 
  Quaternion quatPushIntm = Quaternion("0.7071 0 0 -0.7071");
  Quaternion quatPushInv = Quaternion("0 0 0 1"); // Hand pushing the object in opposite direction
  
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
  
  //Open the hand
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(1);
  ROS_INFO("Hand Opened");
  robot.SetCartesian(transO[0], transO[1], transO[2]-10, quatO[0], quatO[1], quatO[2], quatO[3]);

  //Close the hand
  
  hand.SetAngle(HAND_CLOSE_ANGLE_partial);
  hand.WaitRest(1);
   ROS_INFO("Hand Closed partial");
  //hand.SetAngle(HAND_CLOSE_ANGLE);
  
  robot.SetCartesian(transPick[0], transPick[1], transPick[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(1);
  ROS_INFO("Hand Closed");
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  usleep(1000000);

  //******************BOUNCELESS CATCH TO GO FROM F2E*********************\\

  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0); // Inverting the position of the hand
  

  robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);   //this is the topmost position
  usleep(2000000);
  
  hand.SetSpeed(HAND_SPEED_F2E);
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
  
  printf("sleep time = %d \n", Tsleep_F2E);
  
  
  hand.SetAngle(minfingerAngle-20);
 
  usleep(Tsleep_F2E);
  
  robot.SetSpeed(TCPf, ORIf); //Might not be necessary as the motion velocity is limited my conditions in RAPID code, same for ZONE vlaue 
  
  //Catch the object == Move the robot down == RAPID Code
  robot.SpecialCommand(1,1082.4,1002.96,3.64,15.98,9.63);
  // (Arguments are == 1 (special command number=Nikhil's regrasp code), Z at Acc switch, Z at catch, Acc1 till acc switch, Acc2 till catch, Acc3 till stop) 
  
  hand.GetAngles(motorAngleD, fingerAngleD);
  ROS_INFO("Object dropped");
  printf("new finger angles= %lf	%lf  	 %lf \n",fingerAngleD[0], fingerAngleD[1], fingerAngleD[2]);
  printf("new motor angle= %lf \n",motorAngleD);
  
  //Close the hand 
  hand.SetSpeed(HAND_SPEED);
  hand.SetAngle(HAND_CLOSE_ANGLE);
  printf("Started regrasping");
  
  robot.SetSpeed(TCPs, ORIs);
  robot.SetZone(ZONE);
  robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0);
  //robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);

  //******************VIBRATION*********************\\

  hand.SetAngle(HAND_OPEN_ANGLE_vibrate);
  hand.WaitRest(1);
  ROS_INFO("Object ready for vibration");
  usleep(2000000);
  
   //CALL SPECIAL COMMAND FOR VIBRATION
    
  robot.SpecialCommand(4,0.5,0,0,0,0);
      
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(1);
  ROS_INFO("Hand Closed after Vibration");

  //*************************PUSH Translate (USING EXTERNAL FIXTURE) starts*****************\\
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0);  //To avoid singularity
  
  hand.GetAngles(motorAngle, fingerAngle);
  printf("finger angles= %lf	%lf	%lf \n",fingerAngle[0], fingerAngle[1], fingerAngle[2]);
  printf("motor angle= %lf \n",motorAngle);
  double NewminfingerAngle=fingerAngle[0];
  if (fingerAngle[1]<NewminfingerAngle)
  {
    NewminfingerAngle=fingerAngle[1];
  }
  if (fingerAngle[2]<NewminfingerAngle)
  {
    NewminfingerAngle=fingerAngle[2];
  }
  printf("NewMin finger angle= %lf \n",NewminfingerAngle);
  printf("drop angle= %lf \n",NewminfingerAngle);
    
  robot.SetCartesian(transSafe[0],transSafe[1],transSafe[2], quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  ROS_INFO("1 Completed!");
  
  hand.SetAngle(NewminfingerAngle);
  usleep(500000);
    
  robot.SetCartesian(transPushStop[0], transPushStop[1], transPushStop[2], quatPush[0], quatPush[1], quatPush[2], quatPush[3]); 
  ROS_INFO("2 Completed!");

  hand.SetAngle(HAND_CLOSE_ANGLE);
  usleep(500000);
  
  robot.SetCartesian(transSafe[0], transSafe[1], transSafe[2], quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transSafe[0], transSafe[1], transSafe[2], quatPushIntm[0], quatPushIntm[1], quatPushIntm[2], quatPushIntm[3]);
  robot.SetCartesian(transSafe[0], transSafe[1], transSafe[2], quatPushInv[0], quatPushInv[1], quatPushInv[2], quatPushInv[3]);
  
  ROS_INFO("3 Completed!");
  
  hand.SetAngle(NewminfingerAngle);
  usleep(500000);
  
  robot.SetCartesian(transPushStop[0], transPushStop[1], transPushStop[2],  quatPushInv[0], quatPushInv[1], quatPushInv[2], quatPushInv[3]); 
  ROS_INFO("4 Completed!");
  
  hand.SetAngle(HAND_CLOSE_ANGLE);
  usleep(1000000);
  
  robot.SetCartesian(transSafe[0], transSafe[1], transSafe[2], quatPushInv[0], quatPushInv[1], quatPushInv[2], quatPushInv[3]);
    //*************************PUSH Translate Ends****************\\
 
 
  //******************Roll in Hand Starts********************* 
  
  robot.SetJoints(0,0,0,0,10,-90);
    
  hand.GetAngles(motorAngle, fingerAngle);
  printf("Before Roll finger angles= %lf	%lf	%lf \n",fingerAngle[0], fingerAngle[1], fingerAngle[2]);
  printf("motor angle= %lf \n",motorAngle);
  double RollmaxfingerAngle=fingerAngle[0];
  if (fingerAngle[1]>RollmaxfingerAngle)
  {
    RollmaxfingerAngle=fingerAngle[1];
  }
  if (fingerAngle[2]>RollmaxfingerAngle)
  {
    RollmaxfingerAngle=fingerAngle[2];
  }
  printf("RollMax finger angle= %lf \n",RollmaxfingerAngle);
  double RollReleaseAngle= RollmaxfingerAngle-RollAngle;
  printf("Roll release angle= %lf \n",RollReleaseAngle);
  
  hand.SetAngle(RollReleaseAngle);
  hand.WaitRest(1); 
  usleep(2000000);
  
  double HAND_CLOSEAfterRegrasp_ANGLE=RollReleaseAngle+10;
  hand.SetAngle(HAND_CLOSEAfterRegrasp_ANGLE);
  usleep(1000000);
  //******************Roll in Hand Ends********************* 
  
  robot.SetSpeed(TCPs, ORIs);
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  robot.SetCartesian(transPick[0], transPick[1], transPick[2]+100, quatO[0], quatO[1], quatO[2], quatO[3]);
  
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(1);
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  ROS_INFO("RollInHand Completed!");

  return 0;
}
