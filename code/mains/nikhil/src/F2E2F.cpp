//  Finger-tip grasp to Enveloping grasp to Fingertip grasp

#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <matVec/matVec.h>

#define ZONE 0
#define ZONE3 3
#define ZONE2 2

// Speed of the robot
#define TCPs 80.0   // Slow speed mm / s
#define ORIs 25.0   // degrees / s

#define TCPf 400.0   // FAST SPEED mm / s
#define ORIf 70.0   // degrees / s

#define HAND_FORCE 0.99
#define HAND_SPEED 0.9
#define HAND_SPEED_E2F 0.99
#define HAND_SPEED_F2E 0.80
#define HAND_SPEED_F2E1 0.75
#define HAND_SPEED_Vibrate 0.75

#define HAND_CLOSE_ANGLE 70.0  // Max closing angle
#define HAND_CLOSE_ANGLE_partial 15.0 //Just enough to grasp the object chnaged from 32
#define HAND_OPEN_ANGLE_E2F 9.0
#define HAND_OPEN_ANGLE 10.0   // Max opening angle
#define HAND_CLOSE_ANGLE_After 75.0
#define HAND_OPEN_ANGLE_Vibrate2 35.0 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "F2E2F");
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
  
  ROS_INFO("Beginning Regrasp Action : E2F");

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
  int TsleepE2F=6000; //5650; //159000;  //*******************Tsleep******************
  int TsleepF2E=200000; 
  
  Vec trans(3);
  Vec transC = Vec("600 300 200",3); // Safe location in the center of the robot workspace.
 
  Quaternion quat;
  Quaternion quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand facing down
 
  Vec transO = Vec("600 200 100",3);  //Location of Hand before picking the object (Object location + 100 mm in Z direction)
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
  
  hand.SetAngle(HAND_CLOSE_ANGLE_partial);
  hand.WaitRest(0.250);
   ROS_INFO("Hand Closed partial");
  //hand.SetAngle(HAND_CLOSE_ANGLE);
  
  robot.SetCartesian(transO[0], transO[1], transO[2]-18, quatO[0], quatO[1], quatO[2], quatO[3]);
  
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(0.250);
  ROS_INFO("Hand Closed");
  
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
    
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0); // Inverting the position of the hand
  
  //double xT, yT, zT, q0T, qxT, qyT, qzT;
  //robot.GetCartesian(xT, yT, zT, q0T, qxT, qyT, qzT);
  //printf ("xT=%lf\n yT=%lf\n zT=%lf\n q0T=%lf\n qxT=%lf\n qyT=%lf\n qzT=%lf\n", xT, yT, zT, q0T, qxT, qyT, qzT);
  
  //robot.SetSpeed(TCPf, ORIf);
  //robot.SetZone(ZONESeq);
  
  robot.SetCartesian(600, 700, 800, 0.7071, 0.0, 0.0, -0.7071);
  
  for (int i=0; i<5; i++)
  {
  
  robot.SetSpeed(TCPf, ORIf);
  robot.SetZone(ZONE2);
  /*if (i==0)
  {
  robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);
  }
  */
  
  robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);
  //usleep(5000000);
  
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
  //int sleeptime= 1000; //330150; //15710*1000/minfingerAngle;   // 91750 to 91800, perfornamnce is not constant
  printf("sleep time = %d \n", TsleepF2E);
  
  //1-Open hand just enough to drop the object
  if (i==0)
  {
  hand.SetSpeed(HAND_SPEED_F2E1);
  }
  else
  {
  hand.SetSpeed(HAND_SPEED_F2E);
  }
  
  hand.SetAngle(minfingerAngle-20);
  //hand.SetAngle(HAND_OPEN_ANGLE_drop);
  
  usleep(TsleepF2E);
  
  //robot.SetSpeed(TCPf, ORIf); //Might not be necessary as the motion velocity is limited my conditions in RAPID code, same for ZONE vlaue 
  
  //Catch the object == Move the robot down == RAPID Code
  robot.SpecialCommand(1,1082.4,1002.96,3.64,15.98,9.63);
  // (Arguments are == 1 (special command number=Nikhil's regrasp cose), Z at Acc switch, Z at catch, Acc1 till acc switch, Acc2 till catch, Acc3 till stop) 
  
  hand.GetAngles(motorAngleD, fingerAngleD);
  ROS_INFO("Object dropped");
  printf("new finger angles= %lf	%lf  	 %lf \n",fingerAngleD[0], fingerAngleD[1], fingerAngleD[2]);
  printf("new motor angle= %lf \n",motorAngleD);
  /*
  //Close the hand 
  hand.SetAngle(HAND_CLOSE_ANGLE);
  printf("Started regrasping");
  hand.WaitRest(0.250);
  */
  
  hand.SetSpeed(HAND_SPEED);
  hand.SetForce(HAND_FORCE); 
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(1);
  /* 
  for (int n =0; n<1; n++)
  {
   hand.SetAngle(HAND_CLOSE_ANGLE_partial);
   hand.WaitRest(1);
   usleep(500000);
   hand.SetAngle(HAND_CLOSE_ANGLE);
   hand.WaitRest(1);
   usleep(500000);
  }
  
  hand.SetAngle(HAND_OPEN_ANGLE_Vibrate2);
 // hand.WaitRest(1);
 // ROS_INFO("Object ready for vibration");
  //usleep(1000000);
  
   //CALL SPECIAL COMMAND FOR VIBRATION
      //ros::Duration(3.0).sleep();
      
  //robot.SpecialCommand(4,0.5,0,0,0,0);
  
  hand.SetSpeed(HAND_SPEED);
  hand.SetForce(HAND_FORCE);    
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(1);
  ROS_INFO("Hand Closed after Vibration");
  */
  
  //Open the hand just enough to grab the object quickly during fast move
  hand.SetAngle(HAND_OPEN_ANGLE_E2F);
  hand.WaitRest(0.250);
  ROS_INFO("Hand Partially Opened");
  
  robot.SetCartesian(600, 700, 800, 0.7071, 0.0, 0.0, -0.7071);   
  //usleep(2000000);
  
   //Close the hand 
   
  hand.SetSpeed(HAND_SPEED_E2F); 
  hand.SetAngle(HAND_CLOSE_ANGLE);
  printf("Started regrasping");
  //usleep(180000);
  //usleep(179999); //Old rubber coated fingers
  //usleep(153000);                    //Works for 140000 to 150000, Need to optimize
  usleep(TsleepE2F);    
  
  //E2F==Env2Fing == Move the robot down == RAPID Code
  
  robot.SpecialCommand(3,1080.0,900.0,16.71,18.71,18.71); //better position control for catching
  
  //robot.SpecialCommand(3,1080.0,850.0,17.71,18.71,18.71);   //Maximum throw
  
  // (Arguments are == 3 (special command number=Nikhil's E2F code), Z at topmost position in fast move, Z at catch/before stop,...
  // ...Acc1 till top pos, Acc2 till catch/point before stop, Acc3 after catch till lowermost pos) 
  
  robot.SetSpeed(TCPf, ORIf);
  robot.SetZone(ZONE2);
  //robot.SetCartesian(600, 700, 800, 0.7071, 0.0, 0.0, -0.7071);
  //robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);   //this is the topmost position
  }
  
  robot.SetSpeed(TCPs, ORIs);
  robot.SetZone(ZONE);
  
  robot.SetCartesian(600, 700, 800, 0.7071, 0.0, 0.0, -0.7071);
  robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);
  
  hand.SetSpeed(HAND_SPEED);
  hand.SetAngle(HAND_CLOSE_ANGLE_After);
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0);
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  robot.SetCartesian(transO[0], transO[1], transO[2]-5, quatO[0], quatO[1], quatO[2], quatO[3]);
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(0.250);
  ROS_INFO("object placed");
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);

  ROS_INFO("Regrasp E2F Completed!");

  return 0;
}
