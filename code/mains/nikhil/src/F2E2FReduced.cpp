//  Fingertip grasp to Enveloping grasp to Fingertip grasp
//ANGLE CHANGED, TIME CHANGED

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

#define TCPf 300.0   // FAST SPEED mm / s
#define ORIf 60.0   // degrees / s

#define HAND_FORCE 0.99
#define HAND_SPEED 0.9
#define HAND_SPEED_Squeeze 0.99
#define HAND_SPEED_E2F 1.0 //0.9 // 1.0 is new optimized speed value (optimization doen using GP on 12-29-2013 //Earlier value= 0.9 //0.99
#define HAND_SPEED_F2E 0.99
#define HAND_SPEED_F2E1 0.99
#define HAND_SPEED_Vibrate 0.75

#define HAND_CLOSE_ANGLE 60.0  // Max closing angle
#define HAND_CLOSE_ANGLE_partial 15.0 //Just enough to grasp the object 
#define HAND_OPEN_ANGLE_E2F 0.0 //4.0 // 0.0 is new optimized value //4.0 //optimized angle was 6, but not working on 12/26/2013 //9.0 
#define HAND_OPEN_ANGLE 10.0   // Max opening angle
#define HAND_CLOSE_ANGLE_After 70.0
#define HAND_OPEN_ANGLE_Vibrate2 35.0

int main(int argc, char** argv)
{
  ros::init(argc, argv, "F2E2FReduced");
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
  
  ROS_INFO("Beginning Regrasp Action : F2E2FReduced");

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
  int TsleepE2F= 26000; //8700; //8500; //25000 isnew oprimized value
  //Earlier values...1500; //45500; //11500=optimized time, but The height of the object from the palm is quite high, I dont hitng its very stable, so time changed to 45500 which grasps the object at the center of the thickness of the object; //*******************Tsleep******************
  int TsleepSqueeze=800000; //550000; 
  
  Vec trans(3);
  Vec transC = Vec("600 300 200",3); // Safe location in the center of the robot workspace.
 
  Quaternion quat;
  Quaternion quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand facing down
 
  Vec transO = Vec("600 200 100",3);  //Location of Hand before picking the object 
  Quaternion quatO = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand grasping the object from top
   
  Vec transPick= Vec("600 200 82",3);  //Pick Position
   
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
  //ROS_INFO("Hand Opened");
  
  //Close the hand
  robot.SetCartesian(transPick[0], transPick[1], transPick[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(0.250);
  //ROS_INFO("Hand Closed");
  
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
    
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0); // Inverting the position of the hand
  
  robot.SetCartesian(600, 700, 950, 0.7071, 0.0, 0.0, -0.7071);
   
  for (int i=0; i<50; i++)
  {
  
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
  
  //1-Open hand just enough to drop the object
  if (i==0)
  {
  hand.SetSpeed(HAND_SPEED_F2E1);
  }
  else
  {
  hand.SetSpeed(HAND_SPEED_F2E);
  }
  
  hand.SetAngle(minfingerAngle-10);
  hand.WaitRest(0.250);
  //hand.WaitRest(0.250);
  
  robot.SetSpeed(TCPf, ORIf);
  robot.SetZone(ZONE3);
    
  hand.GetAngles(motorAngleD, fingerAngleD);
  //ROS_INFO("Object dropped");
  //printf("new finger angles= %lf	%lf  	 %lf \n",fingerAngleD[0], fingerAngleD[1], fingerAngleD[2]);
  //printf("new motor angle= %lf \n",motorAngleD);
  
  hand.SetSpeed(HAND_SPEED_Squeeze);
  hand.SetForce(HAND_FORCE); 
  hand.SetAngle(HAND_CLOSE_ANGLE);
  //hand.WaitRest(0.250);
  usleep(TsleepSqueeze);
    
  robot.SetCartesian(600, 700, 800, 0.7071, 0.0, 0.0, -0.7071);
    
  //Open the hand just enough to grab the object quickly during fast move
  hand.SetAngle(HAND_OPEN_ANGLE_E2F);
  hand.WaitRest(0.250);
  //ROS_INFO("Hand Partially Opened");
  
  //Close the hand 
  hand.SetSpeed(HAND_SPEED_E2F); 
  hand.SetAngle(HAND_CLOSE_ANGLE);
  usleep(TsleepE2F);    
  
  //E2F==Env2Fing == RAPID Code
  
  robot.SpecialCommand(3,1080.0,950.0,16.71,18.71,18.71); //better position control for catching
  usleep(750000); 
  //robot.SpecialCommand(3,1080.0,850.0,17.71,18.71,18.71);   //Maximum throw
   
  printf("\n\n Regrasp Run #: %d\n\n", i);
  }
  
  robot.SetSpeed(TCPs, ORIs);
  robot.SetZone(ZONE);
  
  robot.SetCartesian(600, 700, 800, 0.7071, 0.0, 0.0, -0.7071);
  robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);
  
  hand.SetSpeed(HAND_SPEED);
  hand.SetAngle(HAND_CLOSE_ANGLE_After);
  hand.WaitRest(0.250);
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0);
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  robot.SetCartesian(transO[0], transO[1], transO[2]-5, quatO[0], quatO[1], quatO[2], quatO[3]);
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(0.250);
  //ROS_INFO("object placed");
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);

  //ROS_INFO("Regrasp F2E2FReduced Completed!");

  return 0;
}
