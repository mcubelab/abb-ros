//  Enveloping grasp to Fingertip grasp

#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <matVec/matVec.h>
#include <regrasp_comm/regrasp_comm.h>

#define ZONE 0

// Speed of the robot
#define TCPs 80.0   // Slow speed mm / s
#define ORIs 25.0   // degrees / s

#define TCPf 350.0   // FAST SPEED mm / s
#define ORIf 50.0   // degrees / s

#define HAND_FORCE 0.99
#define HAND_SPEED_Topple 0.52 //.65
#define HAND_SPEED 0.95
#define HAND_SPEED_Press 0.99

#define HAND_CLOSE_ANGLE 70.0  // Max closing angle
#define HAND_CLOSE_ANGLE_After 75.0  // Max closing angle
#define HAND_OPEN_ANGLE_Topple 20.0 //15.0 
#define HAND_OPEN_ANGLE 20.0   // Max opening angle
#define HAND_OPEN_ANGLE_Press 30.0 
#define HAND_CLOSE_ANGLE_Press 60.0
#define HAND_OPEN_ANGLE_Vibrate 35.0

int main(int argc, char** argv)
{
  //ros::init(argc, argv, "E2F");
  ros::init(argc, argv, "toppleLoop");
  ros::NodeHandle node;

  RobotComm robot(&node);
  HandComm hand(&node);
  RegraspComm regrasp(&node);

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
  
  ROS_INFO("Beginning Regrasp Action : Topple");

  // All cartesian moves are with respect to the proximal left corner of the table.
  // (where the default work object is located)
  // X-axis along the edge that goes left to right (facing the robot)
  // Y-axis along the edge that goes front to back
  // Z-axis pointing up.

  // Auxiliary variables
 // HomogTransf pose;

  double motorAngle;
  double fingerAngle[3];
  int Tsleep_Topple=82999;
  
  //double motorAngleD;
  //double fingerAngleD[3];

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
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  //robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  
  hand.SetSpeed(HAND_SPEED);
  hand.SetForce(HAND_FORCE);
  
  //hand.SetEncoder(15000);
  //usleep(2000000);
  
  hand.Calibrate();
  hand.WaitRest(1.0);

  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  
  //Open the hand
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(0.250);
  ROS_INFO("Hand Opened");
  
  usleep(2000000); //Keep an object in the hand

  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(0.250);
  ROS_INFO("Hand Closed");
  
  usleep(500000);
  
  for(int i=0; i<50; i++)
  {  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0); // Inverting the position of the hand
  
   
  hand.SetSpeed(HAND_SPEED);
  hand.SetForce(HAND_FORCE);  
  
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(0.250);
  //ROS_INFO("Hand Opened");
  
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(0.250);
 // ROS_INFO("Hand Closed");
  
  hand.SetSpeed(HAND_SPEED_Press);
  //hand.SetForce(HAND_FORCE_Press); 
  
  for (int n =0; n<1; n++)
  {
   hand.SetAngle(HAND_OPEN_ANGLE_Press);
   hand.WaitRest(1);
   usleep(500000);
   hand.SetAngle(HAND_CLOSE_ANGLE_Press);
   hand.WaitRest(1);
   usleep(500000);
  }
  
  hand.SetAngle(HAND_OPEN_ANGLE_Vibrate);
  hand.WaitRest(1);
 // ROS_INFO("Object ready for vibration");
  usleep(500000);
  
   //CALL SPECIAL COMMAND FOR VIBRATION
      //ros::Duration(3.0).sleep();
      
  robot.SpecialCommand(4,0.5,0,0,0,0);
  
  hand.SetSpeed(HAND_SPEED);
  hand.SetForce(HAND_FORCE);    
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(1);
  //ROS_INFO("Hand Closed after Vibration");
  //double xT, yT, zT, q0T, qxT, qyT, qzT;
  //robot.GetCartesian(xT, yT, zT, q0T, qxT, qyT, qzT);
  //printf ("xT=%lf\n yT=%lf\n zT=%lf\n q0T=%lf\n qxT=%lf\n qyT=%lf\n qzT=%lf\n", xT, yT, zT, q0T, qxT, qyT, qzT);
  
  //robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);   //this is the topmost position
 // usleep(2000000);
    
  //Open the hand just enough to grab the object quickly during fast move
  
 
  
  robot.SetCartesian(600, 700, 800, 0.7071, 0.0, 0.0, -0.7071);   
  usleep(2000000);
  
  hand.SetAngle(HAND_OPEN_ANGLE_Topple);
  hand.WaitRest(0.250);
  //ROS_INFO("Hand Partially Opened");
  
   //Close the hand 
  hand.SetSpeed(HAND_SPEED_Topple);
  hand.SetAngle(HAND_CLOSE_ANGLE);
  //printf("Started regrasping");
  //usleep(4950);
  //usleep(172000);
  //usleep(67100);
  //usleep(138870);
  //usleep(83000);
  usleep(Tsleep_Topple);
  //E2F==Env2Fing == Move the robot down == RAPID Code
  //robot.SpecialCommand(3,1080.0,850.0,14.71,18.71,18.71); //better position control for catching
  
  robot.SpecialCommand(3,1080.0,900.0,17.71,18.71,18.71);   //Maximum throw
  
  // (Arguments are == 3 (special command number=Nikhil's E2F code), Z at topmost position in fast move, Z at catch/before stop,...
  // ...Acc1 till top pos, Acc2 till catch/point before stop, Acc3 after catch till lowermost pos) 
  
  
  robot.SetSpeed(TCPs, ORIs);
  robot.SetZone(ZONE);
  
  //robot.SetCartesian(600, 700, 800, 0.7071, 0.0, 0.0, -0.7071);
  //robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);
  hand.SetSpeed(HAND_SPEED);
  hand.SetAngle(HAND_CLOSE_ANGLE_After);
  hand.WaitRest(0.250);
  
  //robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0);
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  
  //******************StandtoLie*************************\\

  regrasp.StandtoLie(-5,-35,102,-0.52); 
  printf("\n\n Regrasp Run #: %d\n\n", i);
  }
  
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  robot.SetCartesian(transO[0], transO[1], transO[2]-15, quatO[0], quatO[1], quatO[2], quatO[3]);
  
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(0.250);
  ROS_INFO("Hurray 50 Runs Done!!");
  
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);

  //ROS_INFO("Regrasp topple Completed!");

  return 0;
}
