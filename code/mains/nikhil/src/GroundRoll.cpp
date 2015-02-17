//ROLL RELEASE ANGLE = MIN FINGER ANGLE  (I THINK THIS IS NOT CHANGED)
//DIRECTION OF ROLLING IS REVERESE AS TWO FINGERS PUSHING THE OBJECT IS BETTER THAN ONLY ONE FINGER PUSHING IT.
//CHANGED ROLLONGROUND in regrasp_node.cpp

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
  Vec transPick=Vec("600 200 73",3)+Vec("0 0 6.5",3);	//Pick Point
  Vec transSafe=Vec("600 200 200",3)+Vec("0 0 6.5",3);	//Safe Point
  Vec transPushStop=Vec("600 200 70",3)+Vec("0 0 6.5",3);	//Push Stop Point
  Vec transLowPick=transPushStop+Vec("0 37 0",3);	//Pick Point after Push Stop motion
  Vec transRollPlace=transPick+Vec("150 0 -1",3);	//Roll Place Point
  
  Quaternion quatO = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand grasping the object from top


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
    
  robot.SetCartesian(transPick[0], transPick[1], transPick[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(1);
  ROS_INFO("Hand Closed");
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  usleep(1000000);

  //******************BOUNCELESS CATCH TO GO FROM F2E*********************\\

  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0); // Inverting the position of the hand
  
  //robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0); // Rotating the object (may not be Necessary)
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
  //printf("drop angle= %lf \n",minfingerAngle-10);
  printf("drop angle= %lf \n",minfingerAngle-10);
  
  printf("sleep time = %d \n", Tsleep_F2E);
  
  //1-Open hand just enough to drop the object
  
  hand.SetAngle(minfingerAngle-10);
  usleep(Tsleep_F2E);
  
  robot.SetSpeed(TCPf, ORIf); //Might not be necessary as the motion velocity is limited my conditions in RAPID code, same for ZONE vlaue 
  
  robot.SpecialCommand(1,1082.4,1002.96,3.64,15.98,9.63);
  // (Arguments are == 1 (special command number=Nikhil's regrasp cose), Z at Acc switch, Z at catch, Acc1 till acc switch, Acc2 till catch, Acc3 till stop) 
  
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

  //*************************PUSH Translate Starts*****************\\
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -20.0, 0.0);  //To avoid singularity
  
  //**************Computations****************\\
  
  RotMat Xrot;
  Vec rotXAxis = Vec("1.0 0.0 0.0", 3);
  double rotXAngle = -90;      //Angle to touch the fixture
  Xrot.setAxisAngle(rotXAxis, rotXAngle*PI/180.0);
  HomogTransf TRotX=HomogTransf(Xrot, Vec("0.0 0.0 0.0",3));
  
  RotMat Yrot;
  Vec rotYAxis = Vec("0.0 1.0 0.0", 3);
  double rotYAngle = 180;      //Angle to touch the fixture
  Yrot.setAxisAngle(rotYAxis, rotYAngle*PI/180.0);
  HomogTransf TRotY=HomogTransf(Yrot, Vec("0.0 0.0 0.0",3));
  
  HomogTransf TPos=HomogTransf(Quaternion("1.0 0.0 0.0 0.0").getRotMat(), transSafe); 
  HomogTransf Cdown=HomogTransf(Quaternion("0.0 0.7071 0.7071 0.0").getRotMat(), transSafe);
  
  hand.GetAngles(motorAngle, fingerAngle);
  printf("finger angles= %lf	%lf	%lf \n",fingerAngle[0], fingerAngle[1], fingerAngle[2]);
  printf("motor angle= %lf \n",motorAngle);
  double MaxfingerAngle=fingerAngle[0];
  if (fingerAngle[1]>MaxfingerAngle)
  {
    MaxfingerAngle=fingerAngle[1];
  }
  if (fingerAngle[2]>MaxfingerAngle)
  {
    MaxfingerAngle=fingerAngle[2];
  }
  printf("Max finger angle= %lf \n",MaxfingerAngle);
  //printf("drop angle= %lf \n",minfingerAngle-10);
  printf("drop angle= %lf \n",MaxfingerAngle-1);
  
  //*********************************************\\
  
  HomogTransf TouchPos=HomogTransf(((TPos*TRotX*TPos.inv())*Cdown).getRotation(), transSafe);
  robot.SetCartesian(TouchPos);
  ROS_INFO("1 Completed!");
  
  hand.SetAngle(MaxfingerAngle-.25);
  usleep(500000);
  
  HomogTransf PushStPos=HomogTransf(((TPos*TRotX*TPos.inv())*Cdown).getRotation(), transPushStop);
  robot.SetCartesian(PushStPos);
  ROS_INFO("2 Completed!");

  hand.SetAngle(HAND_CLOSE_ANGLE);
  usleep(500000);
  
  robot.SetCartesian(TouchPos);
  ROS_INFO("3 Completed!");
  
  HomogTransf TouchInvPos=HomogTransf(((TPos*TRotY*TRotX*TPos.inv())*Cdown).getRotation(), transSafe);
  robot.SetCartesian(TouchInvPos);
  ROS_INFO("4 Completed!");
  
  hand.SetAngle(MaxfingerAngle-.25);
  usleep(500000);
  
  HomogTransf PushStInvPos=HomogTransf(((TPos*TRotY*TRotX*TPos.inv())*Cdown).getRotation(), transPushStop);
  robot.SetCartesian(PushStInvPos);
  ROS_INFO("5 Completed!");
  
  hand.SetAngle(HAND_OPEN_ANGLE+10);
  usleep(1000000);
  
  HomogTransf LowPickPos=HomogTransf(((TPos*TRotY*TRotX*TPos.inv())*Cdown).getRotation(), transLowPick);
  robot.SetCartesian(LowPickPos);
  ROS_INFO("6 Completed!");
  usleep(1000000);
  
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(1);  
  //*************************PUSH Translate Ends*****************\\
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
   
   //**********************Roll on Ground STARTS*************
   
  robot.SetCartesian(transRollPlace[0], transRollPlace[1], transRollPlace[2], quatO[0], quatO[1], quatO[2], quatO[3]);
    
  hand.GetAngles(motorAngle, fingerAngle);
  printf("Before Roll finger angles= %lf	%lf	%lf \n",fingerAngle[0], fingerAngle[1], fingerAngle[2]);
  printf("motor angle= %lf \n",motorAngle);
  double RollminfingerAngle=fingerAngle[0];
  if (fingerAngle[1]<RollminfingerAngle)
  {
    RollminfingerAngle=fingerAngle[1];
  }
  if (fingerAngle[2]<RollminfingerAngle)
  {
    RollminfingerAngle=fingerAngle[2];
  }
  printf("RollMin finger angle= %lf \n",RollminfingerAngle);
  double RollReleaseAngle= RollminfingerAngle;
  printf("Roll release angle= %lf \n",RollReleaseAngle);
  
  hand.SetAngle(RollReleaseAngle);
  hand.WaitRest(1); 
  usleep(2000000);
  
  robot.SetSpeed(TCPss, ORIss);
  robot.SetCartesian(transRollPlace[0]-57, transRollPlace[1], transRollPlace[2], quatO[0], quatO[1], quatO[2], quatO[3]);  // X-movement 109.95 for full rotation = circumfernce of cyl dia 35 mm
  
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(1);
  usleep(1000000);
   //**********************Roll on Ground ends************* 
    
  robot.SetSpeed(TCPs, ORIs);
  robot.SetCartesian(transPick[0], transPick[1], transPick[2]+200, quatO[0], quatO[1], quatO[2], quatO[3]);
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  ROS_INFO("GroundRoll Completed!");

  return 0;
}
