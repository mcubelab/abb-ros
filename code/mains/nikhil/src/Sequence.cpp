#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <matVec/matVec.h>
#include <regrasp_comm/regrasp_comm.h>

//#include <util_comm/util_comm.h>
//#include <objRec_comm/objRec_comm.h>

//#include <ctime>
//#include <unistd.h>

// Speed of the robot
#define TCPs 120.0   // Slow speed mm / s
#define ORIs 30.0   // degrees / s

#define TCPf 350.0   // FAST SPEED mm / s
#define ORIf 50.0   // degrees / s

//Speed of the hand in different regrasps
#define HAND_FORCE 0.99
#define HAND_FORCE_F2E 0.99
#define HAND_FORCE_Vibrate 0.90
#define HAND_FORCE_E2F 0.99
#define HAND_FORCE_FIH 0.90
#define HAND_FORCE_Topple 0.99
#define HAND_FORCE_Press .99

//Force of the hand in different regrasps
#define HAND_SPEED 0.9
#define HAND_SPEED_F2E 0.99 //0.9
#define HAND_SPEED_Vibrate 0.75
#define HAND_SPEED_E2F 0.99
#define HAND_SPEED_FIH 0.95
#define HAND_SPEED_Topple 0.52
#define HAND_SPEED_Press 0.99
#define HAND_SPEED_Squeeze 0.99

//angles required for different regrasps
#define HAND_CLOSE_ANGLE 70.0  // Max closing angle changed from 60
#define HAND_CLOSE_ANGLE_partial 20.0 
#define HAND_OPEN_ANGLE 10.0   // Max opening angle
#define HAND_OPEN_ANGLE_Vibrate1 40.0 
#define HAND_OPEN_ANGLE_Vibrate2 35.0 //55.0 // Angle during vibration changed from 60
#define HAND_CLOSE_ANGLE_BeforeE2F 60.0
#define HAND_OPEN_ANGLE_E2F 9 //27.0  //30   // Angle before grabbing the object perfectly
#define HAND_CLOSE_ANGLE_AfterE2F 70.0
#define HAND_CLOSE_ANGLE_FIH 60.0 
#define HAND_CLOSE_ANGLE_AfterFIH 75.0 
#define HAND_OPEN_ANGLE_Topple 20 //28.0 //30.0 //Just enough to grasp the object chnaged from 32
#define HAND_CLOSE_ANGLE_AfterTopple 75
#define HAND_OPEN_ANGLE_Press 30.0 
#define HAND_CLOSE_ANGLE_Press 70.0

// Zone of the robot
#define ZONE 0 //0 means every commanded position is a fine position. (No fly-by)
#define ZONE3 3

int main(int argc, char** argv)
{
  //ros::init(argc, argv, "regrasp");
  ros::init(argc, argv, "Seqence");
  ros::NodeHandle node;

  RobotComm robot(&node);
  HandComm hand(&node);
  RegraspComm regrasp(&node);


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
  
  ROS_INFO("Regrasp Sequence started...");

  // All cartesian moves are with respect to the proximal left corner of the table.
  // (where the default work object is located)
  // X-axis along the edge that goes left to right (facing the robot)
  // Y-axis along the edge that goes front to back
  // Z-axis pointing up.

  // Auxiliary variables

  double motorAngle;
  double fingerAngle[3];
  double motorAngleD;
  double fingerAngleD[3];
  double minfingerAngle;
  
  Vec trans(3);
  Vec transC = Vec("600 300 200",3); // Safe location in the center of the robot workspace.
  Quaternion quat;
  Quaternion quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand facing down
  Vec transO = Vec("600 200 100",3);  //Location of Hand before picking the object (Object location + 100 mm in Z direction)
  Vec transPick=Vec("600 200 83",3);	//Pick Point
  Vec transPlaceF=Vec("605 195 85",3);	//Final Place Point
  
  //Sleep times required for different regrasps
  int Tsleep_F2E= 1630; //1568; //1560; //1558; //1402000; //500000; //486000;
  int Tsleep_E2F= 18200; //10000; //6095; //5991; //6000; //5150; //5509; //try 5510; working well till 5540 //5590; //239000; //164000;
  int Tsleep_Flip= 645000; //730480; //730550;//730500; //2000000; 
  int Tsleep_Topple=84750; //82995;//82992; //82990; //139000; //138870; //148000; //4950; 
  int Tsleep_Press=550000;
  int TsleepSqueeze=550000; 
  
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
   ROS_INFO("Hand Closed partially");
  //hand.SetAngle(HAND_CLOSE_ANGLE);
  
  //robot.SetCartesian(transO[0], transO[1], transO[2]-28, quatO[0], quatO[1], quatO[2], quatO[3]);
  robot.SetCartesian(transPick[0], transPick[1], transPick[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(1);
  ROS_INFO("Hand Closed");

  //******************BOUNCELESS CATCH TO GO FROM F2E*********************\\

  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0); // Inverting the position of the hand
  
  /*
  //robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0); // Rotating the object (may not be Necessary)
  //robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);   //this is the topmost position
  //usleep(2000000);
  
  hand.SetSpeed(HAND_SPEED_F2E);
  hand.SetForce(HAND_FORCE_F2E);
  
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
  //printf("drop angle= %lf \n",minfingerAngle-10);
  printf("drop angle= %lf \n",minfingerAngle-20);
  
  printf("sleep time = %d \n", Tsleep_F2E);
  
  //1-Open hand just enough to drop the object
  
  hand.SetAngle(minfingerAngle-20);
  hand.WaitRest(0.250);
   //hand.SetAngle(minfingerAngle-10);
  //hand.SetAngle(HAND_OPEN_ANGLE_drop);
  
  //usleep(Tsleep_F2E);
  
  //robot.SetSpeed(TCPf, ORIf); //Might not be necessary as the motion velocity is limited my conditions in RAPID code, same for ZONE vlaue 
  
  //Catch the object == Move the robot down == RAPID Code
  //robot.SpecialCommand(1,1082.4,1002.96,3.64,15.98,9.63);
  // (Arguments are -> 1 (special command number for F2E RAPID code), Z at Acc switch, Z at catch, Acc1 till acc switch, Acc2 till catch, Acc3 till stop) 
  
  hand.GetAngles(motorAngleD, fingerAngleD);
  ROS_INFO("Object dropped");
  printf("new finger angles= %lf	%lf  	 %lf \n",fingerAngleD[0], fingerAngleD[1], fingerAngleD[2]);
  printf("new motor angle= %lf \n",motorAngleD);
  
  //Close the hand
  hand.SetSpeed(HAND_SPEED);
  hand.SetForce(HAND_FORCE); 
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(1);
  //printf("Started regrasping");
  
   /*
  robot.SetSpeed(TCPs, ORIs);
  robot.SetZone(ZONE);
  robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0);
  //robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  */
  
  //******************VIBRATION*********************\\

/*  hand.SetSpeed(HAND_SPEED_Press);
  hand.SetForce(HAND_FORCE_Press); 
  
  for (int n =0; n<1; n++)
  {
   hand.SetAngle(HAND_CLOSE_ANGLE_partial);
   hand.WaitRest(1);
   usleep(500000);
   hand.SetAngle(HAND_CLOSE_ANGLE_OpCl);
   hand.WaitRest(1);
   usleep(500000);
  }
  
  hand.SetSpeed(HAND_SPEED_Vibrate);
  hand.SetForce(HAND_FORCE_Vibrate);
  
  hand.SetAngle(HAND_OPEN_ANGLE_Vibrate1);
  hand.WaitRest(1);
  ROS_INFO("Object ready for vibration");
  usleep(1000000);
  
   //CALL SPECIAL COMMAND FOR VIBRATION
      //ros::Duration(3.0).sleep();
      
  robot.SpecialCommand(4,0.5,0,0,0,0);
  
  hand.SetAngle(HAND_CLOSE_ANGLE_OpCl);
  hand.WaitRest(1);
  
  hand.SetSpeed(HAND_SPEED_Press);
  hand.SetForce(HAND_FORCE_Press); 
  
  for (int n =0; n<1; n++)
  {
   hand.SetAngle(HAND_CLOSE_ANGLE_partial);
   hand.WaitRest(1);
   usleep(500000);
   hand.SetAngle(HAND_CLOSE_ANGLE_OpCl);
   hand.WaitRest(1);
   usleep(500000);
  }
  
  hand.SetAngle(HAND_OPEN_ANGLE_Vibrate2);
  hand.WaitRest(1);
  ROS_INFO("Object ready for vibration");
  usleep(1000000);
  
   //CALL SPECIAL COMMAND FOR VIBRATION
      //ros::Duration(3.0).sleep();
      
  robot.SpecialCommand(4,0.5,0,0,0,0);
  
  hand.SetSpeed(HAND_SPEED);
  hand.SetForce(HAND_FORCE);    
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(1);
  ROS_INFO("Hand Closed after Vibration");


 for (int n =0; n<1; n++)
  {
   hand.SetAngle(HAND_OPEN_ANGLE_Press);
   hand.WaitRest(1);
   usleep(500000);
   hand.SetAngle(HAND_CLOSE_ANGLE_Press);
   hand.WaitRest(1);
   usleep(500000);
  }
  
  hand.SetAngle(HAND_OPEN_ANGLE_Vibrate2);
  hand.WaitRest(1);
  ROS_INFO("Object ready for vibration");
  usleep(500000);
  
   //CALL SPECIAL COMMAND FOR VIBRATION
    
  robot.SpecialCommand(4,0.5,0,0,0,0);
  
  hand.SetSpeed(HAND_SPEED);
  hand.SetForce(HAND_FORCE);    
  hand.SetAngle(HAND_CLOSE_ANGLE_BeforeE2F);
  hand.WaitRest(1);
  ROS_INFO("Hand Closed after Vibration");
 
  hand.SetSpeed(HAND_SPEED_Press);
  hand.SetForce(HAND_FORCE_Press); 
  hand.SetAngle(HAND_CLOSE_ANGLE_Press);
  //usleep(Tsleep_Press);
  hand.WaitRest(1);
  ROS_INFO("Hand Closed");
    
  //robot.SetSpeed(TCPf, ORIf);
  robot.SetCartesian(600, 700, 800, 0.7071, 0.0, 0.0, -0.7071);
  */
  
  robot.SetCartesian(600, 700, 950, 0.7071, 0.0, 0.0, -0.7071);
  
  for (int i=0; i<1; i++)
  {
  
  hand.GetAngles(motorAngle, fingerAngle);
  printf("finger angles= %lf	%lf	%lf \n",fingerAngle[0], fingerAngle[1], fingerAngle[2]);
  printf("motor angle= %lf \n",motorAngle);
  minfingerAngle=fingerAngle[0];
  if (fingerAngle[1]<minfingerAngle)
  {
    minfingerAngle=fingerAngle[1];
  }
  if (fingerAngle[2]<minfingerAngle)
  {
    minfingerAngle=fingerAngle[2];
  }
  printf("min finger angle= %lf \n",minfingerAngle);
  printf("drop angle= %lf \n",minfingerAngle-10);
  //int sleeptime= 1000; //330150; //15710*1000/minfingerAngle;   // 91750 to 91800, perfornamnce is not constant
  //printf("sleep time = %d \n", TsleepF2E);
  
  //1-Open hand just enough to drop the object
 
  hand.SetSpeed(HAND_SPEED_F2E);
   
  hand.SetAngle(minfingerAngle-10);
  hand.WaitRest(0.250);
  //hand.WaitRest(0.250);
  
  //hand.SetAngle(HAND_OPEN_ANGLE_drop);
  
  //usleep(TsleepF2E);
  
  //robot.SetSpeed(TCPf, ORIf); //Might not be necessary as the motion velocity is limited my conditions in RAPID code, same for ZONE vlaue 
  
  //Catch the object == Move the robot down == RAPID Code
  //robot.SpecialCommand(1,1082.4,1002.96,3.64,15.98,9.63);
  // (Arguments are == 1 (special command number=Nikhil's regrasp cose), Z at Acc switch, Z at catch, Acc1 till acc switch, Acc2 till catch, Acc3 till stop) 
  
  //robot.SetSpeed(TCPf, ORIf);
  //robot.SetZone(ZONE3);
  //robot.SetCartesian(600, 700, 850, 0.7071, 0.0, 0.0, -0.7071);
  
 // hand.GetAngles(motorAngleD, fingerAngleD);
  //ROS_INFO("Object dropped");
  //printf("new finger angles= %lf	%lf  	 %lf \n",fingerAngleD[0], fingerAngleD[1], fingerAngleD[2]);
  //printf("new motor angle= %lf \n",motorAngleD);
  /*
  //Close the hand 
  hand.SetAngle(HAND_CLOSE_ANGLE);
  printf("Started regrasping");
  hand.WaitRest(0.250);
  */
  
  hand.SetSpeed(HAND_SPEED_Squeeze);
  hand.SetForce(HAND_FORCE); 
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(0.250);
  //usleep(TsleepSqueeze);
  ROS_INFO("Hand Closed");
  
  robot.SetCartesian(600, 700, 800, 0.7071, 0.0, 0.0, -0.7071);
  
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
  
  //robot.SetCartesian(600, 700, 800, 0.7071, 0.0, 0.0, -0.7071);   
  //usleep(2000000);
  
   //Close the hand 
   
  hand.SetSpeed(HAND_SPEED_E2F); 
  hand.SetAngle(HAND_CLOSE_ANGLE);
  printf("Started regrasping");
  usleep(Tsleep_E2F);    
  
  //E2F==Env2Fing == Move the robot down == RAPID Code
  
  robot.SpecialCommand(3,1080.0,950.0,16.71,18.71,18.71); //better position control for catching
  usleep(750000); 
  //robot.SpecialCommand(3,1080.0,850.0,17.71,18.71,18.71);   //Maximum throw
  
  // (Arguments are == 3 (special command number=Nikhil's E2F code), Z at topmost position in fast move, Z at catch/before stop,...
  // ...Acc1 till top pos, Acc2 till catch/point before stop, Acc3 after catch till lowermost pos) 
  
  //robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);   //this is the topmost position
  }
  
  /*
  //*********************************E2F*****************\\
    
  hand.SetSpeed(HAND_SPEED_E2F);
  hand.SetForce(HAND_FORCE_E2F);
  
  hand.SetAngle(HAND_OPEN_ANGLE_E2F);
  hand.WaitRest(0.250);
  ROS_INFO("Hand Partially Opened");
  
  //robot.SetCartesian(600, 700, 800, 0.7071, 0.0, 0.0, -0.7071);   
  //usleep(2000000);
  
   //Close the hand 
   
  hand.SetSpeed(HAND_SPEED_E2F);
  hand.SetForce(HAND_FORCE_E2F);
  hand.SetAngle(HAND_CLOSE_ANGLE);
  printf("Started regrasping");
  //usleep(180000);
  //usleep(179999); //Old rubber coated fingers
  //usleep(145000);                    //Works for 140000 to 150000, Need to optimize
  usleep(Tsleep_E2F);
  
  //E2F==Env2Fing == Move the robot down == RAPID Code
  
  robot.SpecialCommand(3,1080.0,950.0,16.71,18.71,18.71); //better position control for catching
  //robot.SpecialCommand(3,1080.0,850.0,17.25,18.71,18.71); //better position control for catching
  
  //robot.SpecialCommand(3,1080.0,850.0,17.71,18.71,18.71);   //Maximum throw
  
  // (Arguments are == 3 (special command number=Nikhil's E2F code), Z at topmost position in fast move, Z at catch/before stop,...
  // ...Acc1 till top pos, Acc2 till catch/point before stop, Acc3 after catch till lowermost pos) 
  
  
  robot.SetSpeed(TCPs, ORIs);
  robot.SetZone(ZONE);
  
  //robot.SetCartesian(600, 700, 800, 0.7071, 0.0, 0.0, -0.7071);
  robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);
 */
  
  hand.SetSpeed(HAND_SPEED);
  hand.SetForce(HAND_FORCE);
  hand.SetAngle(HAND_CLOSE_ANGLE_AfterE2F);
  hand.WaitRest(1);
  
  robot.SetSpeed(TCPs, ORIs);
  robot.SetZone(ZONE);
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0);


  //******************FlipInHand****************************\\
  
  hand.SetSpeed(HAND_SPEED_FIH);
  hand.SetForce(HAND_FORCE_FIH);

  //robot.SetJoints(-1.37,31.99,16.84,0.1,-68,-90);
  robot.SetJoints(-1.37,31.99,16.84,0.1,-75.5,-88);
  hand.SetAngle(HAND_CLOSE_ANGLE_FIH);
  usleep(2000000);
  
  //Open the hand slightly
  hand.GetAngles(motorAngle, fingerAngle);
  printf("finger angles= %lf	%lf	%lf \n",fingerAngle[0], fingerAngle[1], fingerAngle[2]);
  printf("motor angle= %lf \n",motorAngle);
  
  minfingerAngle=fingerAngle[0];
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
    
  printf("sleep time = %d \n", Tsleep_Flip);
  
  //1-Open hand just enough to drop the object
  hand.SetAngle(minfingerAngle-20);
  //hand.SetAngle(HAND_OPEN_ANGLE_drop);
  
  usleep(Tsleep_Flip);
  
  hand.SetAngle(HAND_CLOSE_ANGLE_FIH);
  hand.WaitRest(1);
  ROS_INFO("Hand Closed");
  
  hand.SetSpeed(HAND_SPEED);
  hand.SetForce(HAND_FORCE);
  hand.SetAngle(HAND_CLOSE_ANGLE_AfterFIH);
  hand.WaitRest(1);
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  
  //******************StandtoLie*************************\\

  regrasp.StandtoLie(-7,-30,102,-0.52); 
  
  //run regrasp node for stand to lie............. simplehands@giant:~$rosrun regrasp_node regrasp_node

  //************************* BOUNCELESS CATCH F2E *****************\\

  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0); // Inverting the position of the hand
  
  robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);   //this is the topmost position
  usleep(2000000);
  
  hand.SetSpeed(HAND_SPEED_F2E);
  hand.SetForce(HAND_FORCE_F2E);
  
  //Open the hand slightly
  hand.GetAngles(motorAngle, fingerAngle);
  printf("finger angles= %lf	%lf	%lf \n",fingerAngle[0], fingerAngle[1], fingerAngle[2]);
  printf("motor angle= %lf \n",motorAngle);
  minfingerAngle=fingerAngle[0];
  if (fingerAngle[1]<minfingerAngle)
  {
    minfingerAngle=fingerAngle[1];
  }
  if (fingerAngle[2]<minfingerAngle)
  {
    minfingerAngle=fingerAngle[2];
  }
  printf("min finger angle= %lf \n",minfingerAngle);
  printf("drop angle= %lf \n",minfingerAngle-10);
  printf("Tsleep_F2E = %d \n", Tsleep_F2E);
  
  //1-Open hand just enough to drop the object
  hand.SetAngle(minfingerAngle-10);
  //hand.SetAngle(HAND_OPEN_ANGLE_drop);
  
  usleep(Tsleep_F2E);
  
  robot.SetSpeed(TCPf, ORIf); //Might not be necessary as the motion velocity is limited my conditions in RAPID code, same for ZONE vlaue 
  
  //Catch the object == Move the robot down == RAPID Code
  robot.SpecialCommand(1,1082.4,1002.96,3.64,15.98,9.63);
  // (Arguments are == 1 (special command number=Nikhil's regrasp cose), Z at Acc switch, Z at catch, Acc1 till acc switch, Acc2 till catch, Acc3 till stop) 
  
  hand.GetAngles(motorAngleD, fingerAngleD);
  ROS_INFO("Object dropped");
  printf("new finger angles= %lf	%lf  	 %lf \n",fingerAngleD[0], fingerAngleD[1], fingerAngleD[2]);
  printf("new motor angle= %lf \n",motorAngleD);
  
  hand.SetSpeed(HAND_SPEED);
  hand.SetForce(HAND_FORCE); 
  //Close the hand 
  hand.SetAngle(HAND_CLOSE_ANGLE);
  printf("Started regrasping");
  
  /*robot.SetSpeed(TCPs, ORIs);
  robot.SetZone(ZONE);
  robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);
  */
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0);
  //robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);

  //******************VIBRATION*********************\\
  
  /*
  hand.SetSpeed(HAND_SPEED_Press);
  hand.SetForce(HAND_FORCE_Press); 
  
  for (int n =0; n<1; n++)
  {
   hand.SetAngle(HAND_CLOSE_ANGLE_partial);
   hand.WaitRest(1);
   usleep(500000);
   hand.SetAngle(HAND_CLOSE_ANGLE_OpCl);
   hand.WaitRest(1);
   usleep(500000);
  }
  
  hand.SetSpeed(HAND_SPEED_Vibrate);
  hand.SetForce(HAND_FORCE_Vibrate);
  
  hand.SetAngle(HAND_OPEN_ANGLE_Vibrate1);
  hand.WaitRest(1);
  ROS_INFO("Object ready for vibration");
  usleep(1000000);
  
   //CALL SPECIAL COMMAND FOR VIBRATION
      //ros::Duration(3.0).sleep();
      
  robot.SpecialCommand(4,0.5,0,0,0,0);
  hand.SetAngle(HAND_CLOSE_ANGLE_OpCl);
  hand.WaitRest(1);
  */ 
  hand.SetSpeed(HAND_SPEED_Press);
  hand.SetForce(HAND_FORCE_Press); 
  
  for (int n =0; n<1; n++)
  {
   hand.SetAngle(HAND_OPEN_ANGLE_Press);
   hand.WaitRest(1);
   usleep(500000);
   hand.SetAngle(HAND_CLOSE_ANGLE_Press);
   hand.WaitRest(1);
   usleep(500000);
  }
  
  hand.SetAngle(HAND_OPEN_ANGLE_Vibrate2);
  hand.WaitRest(1);
  ROS_INFO("Object ready for vibration");
  usleep(500000);
  
   //CALL SPECIAL COMMAND FOR VIBRATION
      //ros::Duration(3.0).sleep();
      
  robot.SpecialCommand(4,0.5,0,0,0,0);
  
  hand.SetSpeed(HAND_SPEED);
  hand.SetForce(HAND_FORCE);    
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(1);
  ROS_INFO("Hand Closed after Vibration");
  
  //*********************************Topple**********************\\

  //robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);   //this is the topmost position
  //usleep(2000000);
    
  //Open the hand just enough to grab the object quickly during fast move
  
   
  robot.SetCartesian(600, 700, 800, 0.7071, 0.0, 0.0, -0.7071);   
  usleep(2000000);
 
  hand.SetAngle(HAND_OPEN_ANGLE_Topple);
  hand.WaitRest(1);
  ROS_INFO("Hand Partially Opened");
   
  hand.SetSpeed(HAND_SPEED_Topple);
  hand.SetForce(HAND_FORCE_Topple);
  
   //Close the hand 
  hand.SetAngle(HAND_CLOSE_ANGLE);
  printf("Started regrasping");
  //usleep(4950);
  usleep(Tsleep_Topple);
  
  //Adapted from E2F==Env2Fing == Move the robot down == RAPID Code
    
  robot.SpecialCommand(3,1080.0,850.0,17.71,18.71,18.71);   //Maximum throw
  
  // (Arguments are == 3 (special command number=Nikhil's E2F code), Z at topmost position in fast move, Z at catch/before stop,...
  // ...Acc1 till top pos, Acc2 till catch/point before stop, Acc3 after catch till lowermost pos) 
    
  robot.SetSpeed(TCPs, ORIs);
  robot.SetZone(ZONE);
  
  robot.SetCartesian(600, 700, 800, 0.7071, 0.0, 0.0, -0.7071);
  robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);
  
  hand.SetSpeed(HAND_SPEED);
  hand.SetForce(HAND_FORCE);     
  hand.SetAngle(HAND_CLOSE_ANGLE_AfterTopple);
  hand.WaitRest(1);
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0);
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);

  
  //******************StandtoLie*************************\\

  regrasp.StandtoLie(-5,-30,103,-0.52); //TRY TO RUN IT BEFORE CHECKING SEQUENCE CODE****************** We need to run regrasp node as well

  //*******************DONE!!!********************\\
  
  robot.SetSpeed(TCPs, ORIs);
  robot.SetZone(ZONE);

  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  robot.SetCartesian(transO[0], transO[1], transO[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  robot.SetCartesian(transPlaceF[0], transPlaceF[1], transPlaceF[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  
  hand.SetSpeed(HAND_SPEED);
  hand.SetForce(HAND_FORCE);
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(1);
  ROS_INFO("object placed");
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  
  ROS_INFO("Regrasp Sequence Completed!");

  return 0;
}
