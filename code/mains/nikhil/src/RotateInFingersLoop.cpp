//THE PINTS ON TOP EDGE OF THE FACE OF THE FRAME HAVE COORDINATES (900, Y(var), 112), vary value of Y TO GET THE EDGE
//ROBOT MOVES ARE DEFINED WRT THE POINT transFrame (900, 230, 112), CHANGE THIS VALUE IF FRAME IS MOVED
//CURRENTLY IT WONT TAKE INTO ACCOUNT CHANGES IN THE ORIENTATION OF THE FRAME, THE FACE HAS TO BE PARALLEL TO Y-Z PLANE.
//regrasp_node changed , check it once.

#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <matVec/matVec.h>

// Speed of the robot
#define TCPs 100.0   // Slow speed mm / s
#define ORIs 30.0   // degrees / s

#define TCPf 350.0   // FAST SPEED mm / s
#define ORIf 50.0   // degrees / s

#define TCPss 40.0   // SuperSlow speed mm / s
#define ORIss 15.0   // degrees / s

#define HAND_FORCE 0.99
#define HAND_SPEED 0.98
//#define HAND_SPEED_F2E 0.65

#define HAND_CLOSE_ANGLE 70  // Max closing angle changed from 70
#define HAND_CLOSE_ANGLE_partial 20.0 
#define HAND_OPEN_ANGLE 10.0   // Max opening angle
//#define HAND_OPEN_ANGLE_rotate 43.0

// Zone of the robot
#define ZONE 0 //0 means every commanded position is a fine position. (No fly-by)
#define ZONE_Rotate 1 // 1 means 0.3mm zone value. Used here for smooth rotation of the object in fingers..

int main(int argc, char** argv)
{
  //ros::init(argc, argv, "regrasp");
  ros::init(argc, argv, "RotateInFingers");
  ros::NodeHandle node;

  RobotComm robot(&node);
  HandComm hand(&node);

  ROS_INFO("Set robot_node configuration");

  // robot.SetWorkObject(WORK_Z, WORK_Y, WORK_Z, WORK_Q0, WORK_QX, WORK_QY, WORK_QZ))   // Not necessary
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
  
  ROS_INFO("Regrasp RotateInFingers started...");

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
  //int Tsleep_F2E= 1085; //1078; //486000;
  //double RollAngle=10; 
    
  Vec trans(3);
  Vec transC = Vec("600 300 200",3); // Safe location in the center of the robot workspace.
  Quaternion quat;
  Quaternion quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand facing down
  Vec transO = Vec("600 200 100",3);  //Location of Hand before picking the object (Object location + 100 mm in Z direction)
  Vec transPick=Vec("600 200 79",3);	//Pick Point when mat is placed on the table top
  Vec transSafe1=Vec("830 230 182.5",3);	//Safe Point for face1
  Vec transPlace=Vec("595 199 79",3);	//Place Point is chosen such that objects lands on the Pick Pt location
  Vec transSafe2=Vec("830 230 250",3);	//Safe Point for face2
  
  
  //******************TOUCH POINT (not exaclty :) )*********************
  Vec transFrame=Vec("900 230 112",3);
  //********************************************************************
  
  //Vec transPushStop1=Vec("870 230 175",3);	//Push Stop Point for face1
  //Vec transSafe2=Vec("800 230 185",3);	//Safe Point for face2
  //Vec transPushStop2=Vec("870 230 185",3);	//Push Stop Point for face2
  
  //Vec transLowPick=transPushStop+Vec("0 37 0",3);	//Pick Point after Push Stop motion
  //Vec transRollPlace=transPick+Vec("50 0 -1",3);	//Roll Place Point
  
  Quaternion quatO = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand grasping the object from top
  Quaternion quatPush = Quaternion("0.0 0.7071 0.7071 0.0");
  
  //Quaternion quatO = Quaternion("0.7071 0.7071 0.0 0.0"); // Hand grasping the object from back
  
  /*
  Quaternion quatOl1= ("0.7071 0.0 0.0 0.7071");
  Quaternion quatOl2= ("0.7071 0.7071 0.0 0.0");
  Quaternion quatO = quatOl1^quatOl2; // Hand grasping the object from left side
  */
  
  
  //Quaternion quatPush = Quaternion("1 0 0 0"); // Hand pushing an object 
  //Quaternion quatPushIntm = Quaternion("0.7071 0 0 -0.7071");
  //Quaternion quatPushInv = Quaternion("0 0 0 1"); // Hand pushing the object in opposite direction
  
  // " ' " USING SET JOINTS " ' "
  // Do the first command in joint coordinates
 
  
  robot.SetSpeed(TCPs, ORIs);
  robot.SetZone(ZONE);
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  hand.Calibrate();  //Hand is not Calibateed in the code as there are only 2 fingers, the Calibration will respond with error..
  hand.WaitRest(1.0);

  hand.SetSpeed(HAND_SPEED);
  hand.SetForce(HAND_FORCE);
  
  
  robot.SetCartesian(transO[0], transO[1], transO[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  
  //Open the hand
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(1);
  ROS_INFO("Hand Opened");
  //robot.SetCartesian(transO[0], transO[1], transO[2], quatO[0], quatO[1], quatO[2], quatO[3]);

  //Close the hand
 
  hand.SetAngle(HAND_CLOSE_ANGLE_partial);
  hand.WaitRest(1);
   ROS_INFO("Hand Closed partial");
  //hand.SetAngle(HAND_CLOSE_ANGLE);
  
  //robot.SetCartesian(transO[0], transO[1], transO[2]-28, quatO[0], quatO[1], quatO[2], quatO[3]);
  robot.SetCartesian(transPick[0], transPick[1], transPick[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(1);
  ROS_INFO("Hand Closed");
  
  //*************************PUSH Rotate*****************\\
  
  /*
  robot.SetCartesian(transSafe1[0], transSafe1[1], transSafe1[2], quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  
  hand.GetAngles(motorAngle, fingerAngle);
  printf("finger angles= %lf	%lf	%lf \n",fingerAngle[0], fingerAngle[1], fingerAngle[2]);
  printf("motor angle= %lf \n",motorAngle);
  double maxfingerAngle=fingerAngle[0];
  if (fingerAngle[1]>maxfingerAngle)
  {
    maxfingerAngle=fingerAngle[1];
  }
  if (fingerAngle[2]>maxfingerAngle)
  {
    maxfingerAngle=fingerAngle[2];
  }
  printf("Max finger angle= %lf \n",maxfingerAngle);
  //printf("drop angle= %lf \n",minfingerAngle-10);
  printf("Loose angle= %lf \n",maxfingerAngle-0.5);
    
  hand.SetAngle(maxfingerAngle-0.5);
  usleep(500000);
    
  robot.SetSpeed(TCPss, ORIss);  
  robot.SetCartesian(transPushStop1[0], transPushStop1[1], transPushStop1[2], quatPush[0], quatPush[1], quatPush[2], quatPush[3]); 
  //ROS_INFO("1 Completed!");

  hand.SetAngle(HAND_CLOSE_ANGLE);
  usleep(500000);
  
  robot.SetSpeed(TCPs, ORIs);
  robot.SetCartesian(transSafe2[0],transSafe2[1],transSafe2[2], quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  //ROS_INFO("2 Completed!");
  
  hand.GetAngles(motorAngle, fingerAngle);
  printf("New finger angles= %lf	%lf	%lf \n",fingerAngle[0], fingerAngle[1], fingerAngle[2]);
  printf("New motor angle= %lf \n",motorAngle);
  double NewmaxfingerAngle=fingerAngle[0];
  if (fingerAngle[1]>NewmaxfingerAngle)
  {
    NewmaxfingerAngle=fingerAngle[1];
  }
  if (fingerAngle[2]>NewmaxfingerAngle)
  {
    NewmaxfingerAngle=fingerAngle[2];
  }
  printf("New Max finger angle= %lf \n",NewmaxfingerAngle);
  //printf("drop angle= %lf \n",minfingerAngle-10);
  printf("New Loose angle= %lf \n",NewmaxfingerAngle-0.5);
  
  hand.SetAngle(NewmaxfingerAngle-0.5);
  usleep(500000);
  
  robot.SetSpeed(TCPss, ORIss);
  robot.SetCartesian(transPushStop2[0], transPushStop2[1], transPushStop2[2], quatPush[0], quatPush[1], quatPush[2], quatPush[3]); 
  //ROS_INFO("1 Completed!");

  hand.SetAngle(HAND_CLOSE_ANGLE);
  usleep(500000); 
  robot.SetCartesian(transPushStop2[0], transPushStop2[1], transPushStop2[2]+50, quatPush[0], quatPush[1], quatPush[2], quatPush[3]);  
  */
  
  //*************************PUSH Rotate*****************\\
  
  for (int i=0; i<25; i++)
  {
  robot.SetCartesian(transSafe1[0], transSafe1[1], transSafe1[2], quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  usleep(500000);
  
  robot.SetSpeed(TCPss, ORIss);
  robot.SetZone(ZONE_Rotate);
  
  Vec transPtA1=transFrame+Vec("-50 0 65",3);
  Vec transPtA2=transFrame+Vec("-45 0 70",3);
  Vec transPtA3=transFrame+Vec("-40 0 75",3);
  Vec transPtA4=transFrame+Vec("-35 0 80",3);
  Vec transPtA5=transFrame+Vec("-30 0 83",3);
  Vec transPtA6=transFrame+Vec("-25 0 85",3);
  Vec transPtA7=transFrame+Vec("-20 0 86",3);
  Vec transPtA8=transFrame+Vec("-15 0 87",3);
  Vec transPtA9=transFrame+Vec("-10 0 86",3);
  Vec transPtA10=transFrame+Vec("-5 0 86",3);
  Vec transPtA11=transFrame+Vec("00 0 86",3);
  Vec transPtA12=transFrame+Vec("05 0 86",3);
  Vec transPtA13=transFrame+Vec("10 0 86",3);
  Vec transPtA14=transFrame+Vec("10 0 186",3);
  
  robot.SetCartesian(transPtA1[0],transPtA1[1],transPtA1[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtA2[0],transPtA2[1],transPtA2[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtA3[0],transPtA3[1],transPtA3[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtA4[0],transPtA4[1],transPtA4[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtA5[0],transPtA5[1],transPtA5[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtA6[0],transPtA6[1],transPtA6[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtA7[0],transPtA7[1],transPtA7[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtA8[0],transPtA8[1],transPtA8[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtA9[0],transPtA9[1],transPtA9[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtA10[0],transPtA10[1],transPtA10[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtA11[0],transPtA11[1],transPtA11[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtA12[0],transPtA12[1],transPtA12[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtA13[0],transPtA13[1],transPtA13[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtA14[0],transPtA14[1],transPtA14[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  
  hand.SetAngle(HAND_CLOSE_ANGLE);
  usleep(1000000);
  robot.SetCartesian(transPtA1[0],transPtA1[1],transPtA1[2]+30,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  //robot.SetCartesian(850,230,197,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  usleep(500000);
  
  Vec transPtB1=transFrame+Vec("-40 0 84",3);
  Vec transPtB2=transFrame+Vec("-35 0 81",3);
  Vec transPtB3=transFrame+Vec("-15 0 80",3);
  Vec transPtB4=transFrame+Vec("-5 0 79",3);
  Vec transPtB5=transFrame+Vec("0 0 78",3);
  Vec transPtB6=transFrame+Vec("5 0 76",3);
  Vec transPtB7=transFrame+Vec("10 0 74",3);
  Vec transPtB8=transFrame+Vec("20 0 73",3);
  Vec transPtB9=transFrame+Vec("25 0 72",3);
  Vec transPtB10=transFrame+Vec("25 0 172",3);
  
  robot.SetCartesian(transPtB1[0],transPtB1[1],transPtB1[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtB2[0],transPtB2[1],transPtB2[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtB3[0],transPtB3[1],transPtB3[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtB4[0],transPtB4[1],transPtB4[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtB5[0],transPtB5[1],transPtB5[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtB6[0],transPtB6[1],transPtB6[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtB7[0],transPtB7[1],transPtB7[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtB8[0],transPtB8[1],transPtB8[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtB9[0],transPtB9[1],transPtB9[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(transPtB10[0],transPtB10[1],transPtB10[2],quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  
  /*
  robot.SetCartesian(850,230,177,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(855,230,182,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(860,230,187,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(865,230,192,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(870,230,195,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(875,230,197,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(880,230,198,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(885,230,199,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(890,230,198,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(895,230,198,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(900,230,198,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(905,230,198,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(910,230,198,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(910,230,298,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  hand.SetAngle(HAND_CLOSE_ANGLE);
  //hand.WaitRest(1);
  usleep(1000000);
  robot.SetCartesian(850,230,197,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  usleep(500000);
  
  robot.SetCartesian(860,230,196,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(865,230,193,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(885,230,192,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(895,230,191,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(900,230,190,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(905,230,188,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(910,230,186,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(920,230,185,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(925,230,184,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  robot.SetCartesian(925,230,284,quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  */
  
  hand.SetAngle(HAND_CLOSE_ANGLE);
  usleep(1000000);
  robot.SetCartesian(transSafe2[0], transSafe2[1], transSafe2[2], quatPush[0], quatPush[1], quatPush[2], quatPush[3]);
  //hand.WaitRest(1);
  printf("Number of Runs= %d \n",i);
  }
  
  robot.SetSpeed(TCPs, ORIs);
  robot.SetZone(ZONE);
  robot.SetCartesian(transPlace[0], transPlace[1], transPlace[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(1);
  
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  ROS_INFO("RotateInFingers Completed!");

  return 0;
}
