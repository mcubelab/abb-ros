//run simplehands@giant:~/Documents/hands/code/launch$ ./run_vision_script for using cameras

#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <matVec/matVec.h>

// Speed of the robot
#define TCPs 80.0   // Slow speed mm / s
#define ORIs 25.0   // degrees / s

#define TCPss 40.0   // Slow speed mm / s
#define ORIss 20.0

#define TCPf 350.0   // FAST SPEED mm / s
#define ORIf 50.0   // degrees / s

#define HAND_FORCE 0.99
#define HAND_SPEED 0.90
#define HAND_SPEED_F2E 0.65

#define HAND_CLOSE_ANGLE 70.0  // Max closing angle changed from 60
#define HAND_CLOSE_ANGLE_partial 20.0 
#define HAND_OPEN_ANGLE 20.0   // Max opening angle
#define HAND_OPEN_ANGLE_vibrate 43.0
#define HAND_OPEN_ANGLE_RollGrab 37

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
  
  ROS_INFO("Regrasp Vert_Cyl started...");

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
  int Tsleep_F2E= 180; //1085; //1078; //486000;
  double MatThickness= 6.5; //Thickness of the rubber mat on workstation =6.5mm
  //double RollAngle=13; 
    
  Vec trans(3);
  Vec transC = Vec("600 300 200",3); // Safe location in the center of the robot workspace.
  Quaternion quat;
  Quaternion quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand facing down
  Vec transO = Vec("600 200 100",3);  //Location of Hand before picking the object (Object location + 100 mm in Z direction)
  Vec transPick=Vec("600 200 73",3)+Vec("0 0 6.5",3);	//Pick Point
  //printf("Pickx = %f Picky= %f pickz=%f\n",transPick[0], transPick[1], transPick[2]);
  Vec transSafe=Vec("700 325 810",3);	//Safe Point
  Vec transPushStop=Vec("801 325 810",3);	//Push Stop Point
  //Vec transLowPick=transPushStop+Vec("0 37 0",3);	//Pick Point after Push Stop motion
  Vec transPlace=transPick+Vec("0 0 -1",3);	//Place Point
  Vec transPick2=transPick+Vec("0 34 0",3);	//2nd Pick Point
  Vec transPlace2=Vec("600 50 193",3);	//Place Point
  Vec transPick3a=transPlace2+Vec("0 150 -40",3);	//3rd Pick Point
  Vec transPick3b=transPick3a+Vec("0 -81 -75.5",3);
  
  Quaternion quatO = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand grasping the object from top
  Quaternion quatPick3a= Quaternion("0.7071 0.7071 0.0 0.0"); // Hand grasping the object from back
  Quaternion quatPick3b = Quaternion("0.5 0.5 0.5 -0.5");
  /*
  Quaternion quatOl1= ("0.7071 0.0 0.0 0.7071");
  Quaternion quatOl2= ("0.7071 0.7071 0.0 0.0");
  Quaternion quatO = quatOl1^quatOl2; // Hand grasping the object from left side
  */
  
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
  //ROS_INFO("Hand Opened");
  robot.SetCartesian(transO[0], transO[1], transO[2]-10, quatO[0], quatO[1], quatO[2], quatO[3]);

  //Close the hand
  
  hand.SetAngle(HAND_CLOSE_ANGLE_partial);
  hand.WaitRest(1);
   //ROS_INFO("Hand Closed partial");
  //hand.SetAngle(HAND_CLOSE_ANGLE);
  
  //robot.SetCartesian(transO[0], transO[1], transO[2]-28, quatO[0], quatO[1], quatO[2], quatO[3]);
  robot.SetCartesian(transPick[0], transPick[1], transPick[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(1);
  //ROS_INFO("Hand Closed");
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  usleep(1000000);

  //******************BOUNCELESS CATCH TO GO FROM F2E*********************\\

//for (int i=0; i<50; i++)
//{
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0); // Inverting the position of the hand
  
  //robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0); // Rotating the object (may not be Necessary)
  robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);   //this is the topmost position
  usleep(2000000);
  
  hand.SetSpeed(HAND_SPEED_F2E);
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
  //printf("drop angle= %lf \n",minfingerAngle-10);
  //printf("drop angle= %lf \n",minfingerAngle-10);
  
 // printf("sleep time = %d \n", Tsleep_F2E);
  
  //1-Open hand just enough to drop the object
  
  hand.SetAngle(minfingerAngle-10);
  //hand.SetAngle(minfingerAngle-10);
  //hand.SetAngle(HAND_OPEN_ANGLE_drop);
  
  usleep(Tsleep_F2E);
  
  robot.SetSpeed(TCPf, ORIf); //Might not be necessary as the motion velocity is limited my conditions in RAPID code, same for ZONE vlaue 
  
  //Catch the object == Move the robot down == RAPID Code
  robot.SpecialCommand(1,1082.4,1002.96,3.64,15.98,9.63);
  // (Arguments are == 1 (special command number=Nikhil's regrasp cose), Z at Acc switch, Z at catch, Acc1 till acc switch, Acc2 till catch, Acc3 till stop) 
  
  hand.GetAngles(motorAngleD, fingerAngleD);
  //ROS_INFO("Object dropped");
  //printf("new finger angles= %lf	%lf  	 %lf \n",fingerAngleD[0], fingerAngleD[1], fingerAngleD[2]);
 // printf("new motor angle= %lf \n",motorAngleD);
  
  //Close the hand 
  hand.SetSpeed(HAND_SPEED);
  hand.SetAngle(HAND_CLOSE_ANGLE);
  //printf("Started regrasping");
  
  robot.SetSpeed(TCPs, ORIs);
  robot.SetZone(ZONE);
  robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0);
  //robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);

  //******************VIBRATION*********************\\

  /*
  hand.SetAngle(HAND_OPEN_ANGLE_vibrate);
  hand.WaitRest(1);
  ROS_INFO("Object ready for vibration");
  usleep(2000000);
  
   //CALL SPECIAL COMMAND FOR VIBRATION
      //ros::Duration(3.0).sleep();
      
  robot.SpecialCommand(4,0.5,0,0,0,0);
  */    
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(1);
  //ROS_INFO("Hand Closed after Vibration");

  //*************************PUSH Translate*****************\\
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0);  //To avoid singularity
  
  hand.GetAngles(motorAngle, fingerAngle);
  //printf("finger angles= %lf	%lf	%lf \n",fingerAngle[0], fingerAngle[1], fingerAngle[2]);
  //printf("motor angle= %lf \n",motorAngle);
  double NewminfingerAngle=fingerAngle[0];
  if (fingerAngle[1]<NewminfingerAngle)
  {
    NewminfingerAngle=fingerAngle[1];
  }
  if (fingerAngle[2]<NewminfingerAngle)
  {
    NewminfingerAngle=fingerAngle[2];
  }
  //printf("NewMin finger angle= %lf \n",NewminfingerAngle);
  //printf("drop angle= %lf \n",minfingerAngle-10);
  //printf("drop angle= %lf \n",NewminfingerAngle-0.05);
    
  robot.SetCartesian(transSafe[0],transSafe[1],transSafe[2], quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  //ROS_INFO("1 Completed!");
  
  hand.SetAngle(NewminfingerAngle-0.05);
  usleep(500000);
    
  robot.SetCartesian(transPushStop[0], transPushStop[1], transPushStop[2], quatPush[0], quatPush[1], quatPush[2], quatPush[3]); 
  //ROS_INFO("2 Completed!");

  hand.SetAngle(HAND_CLOSE_ANGLE);
  usleep(500000);
  
  robot.SetCartesian(transSafe[0], transSafe[1], transSafe[2], quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transSafe[0], transSafe[1], transSafe[2], quatPushIntm[0], quatPushIntm[1], quatPushIntm[2], quatPushIntm[3]);
  robot.SetCartesian(transSafe[0], transSafe[1], transSafe[2], quatPushInv[0], quatPushInv[1], quatPushInv[2], quatPushInv[3]);
  
  //ROS_INFO("3 Completed!");
  
  hand.SetAngle(NewminfingerAngle-.05);
  usleep(500000);
  
  robot.SetCartesian(transPushStop[0], transPushStop[1], transPushStop[2],  quatPushInv[0], quatPushInv[1], quatPushInv[2], quatPushInv[3]); 
  //ROS_INFO("4 Completed!");
  
  hand.SetAngle(HAND_CLOSE_ANGLE);
  usleep(1000000);
  
  robot.SetCartesian(transSafe[0], transSafe[1], transSafe[2], quatPushInv[0], quatPushInv[1], quatPushInv[2], quatPushInv[3]);
  
  robot.SetJoints(0,0,0,0,10,-90);
  
  /*  
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
  //printf("drop angle= %lf \n",minfingerAngle-10);
  double RollReleaseAngle= RollmaxfingerAngle-RollAngle;
  */
  //printf("HAND_OPEN_ANGLE_RollGrab= %lf \n",HAND_OPEN_ANGLE_RollGrab);
  
  hand.SetAngle(HAND_OPEN_ANGLE_RollGrab);
  hand.WaitRest(1); 
  usleep(2000000);
  
  double HAND_CLOSEAfterRegrasp_ANGLE=HAND_OPEN_ANGLE_RollGrab+15;
  hand.SetAngle(HAND_CLOSEAfterRegrasp_ANGLE);
  usleep(1000000);
  
  for (int i=0; i<50; i++)
{
  robot.SetSpeed(TCPs, ORIs);
  //robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0);
  robot.SetJoints(0.0, 30.0, 30.0, 0.0, -45.0, 0.0);
  robot.SetJoints(0.0, 30.0, 30.0, 0.0, 35.0, 0.0);
  
  robot.SetCartesian(transPlace[0], transPlace[1], transPlace[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(1);
  
  robot.SetCartesian(transPick2[0], transPick2[1], transPick2[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(1);
  
  robot.SetSpeed(TCPss, ORIss);
  robot.SetCartesian(transPick2[0], transPick2[1], transPick2[2]+150, quatO[0], quatO[1], quatO[2], quatO[3]);
  
  robot.SetSpeed(TCPs, ORIs);
  robot.SetCartesian(transPlace2[0], transPlace2[1], transPlace2[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  usleep(4000000);
  
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(1);
  
  robot.SetCartesian(transPlace2[0], transPlace2[1], transPlace2[2]+150, quatO[0], quatO[1], quatO[2], quatO[3]);
  usleep(2000000);
  
  robot.SetJoints(0.0, 40.0, 30.0, 0.0, -90.0, 0.0);
  
  robot.SetSpeed(TCPs, ORIs);
  //robot.SetCartesian(transPick3a[0], transPick3a[1], transPick3a[2], quatPick3a[0], quatPick3a[1], quatPick3a[2], quatPick3a[3]);
  robot.SetCartesian(transPick3b[0], transPick3b[1], transPick3b[2], quatPick3b[0], quatPick3b[1], quatPick3b[2], quatPick3b[3]);
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(1);
  ROS_INFO("Regrasp #: %d", i);
  }
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0);
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  
  ROS_INFO("Hurry 50 VertCyls done!");

  return 0;
}
