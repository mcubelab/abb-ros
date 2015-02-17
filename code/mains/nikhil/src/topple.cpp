#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <vision_comm/vision_comm.h>
#include <matVec/matVec.h>

#define ZONE 0

// Speed of the robot
#define TCPs 150.0   // Slow speed mm / s
#define ORIs 35.0   // degrees / s

#define TCPf 350.0   // FAST SPEED mm / s
#define ORIf 50.0   // degrees / s

#define HAND_FORCE 0.99
#define HAND_SPEED 0.52 //.65
#define HAND_SPEED_F 0.95

#define HAND_CLOSE_ANGLE 70.0  // Max closing angle
#define HAND_CLOSE_ANGLE_After 70.0  // Max closing angle
#define HAND_OPEN_ANGLE_topple 16 //17 //20.0 cahnge after switching to 6V
#define HAND_OPEN_ANGLE 20.0   // Max opening angle
#define HAND_OPEN_ANGLE_Vibrate 35
int main(int argc, char** argv)
{
  //ros::init(argc, argv, "E2F");
  ros::init(argc, argv, "topple");
  ros::NodeHandle node;

  RobotComm robot(&node);
  HandComm hand(&node);
  VisionComm vision(&node);


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
  int Tsleep_Topple=35000; //44700; 44675; //10050 (for angle = 17); //44775 (for angle = 16); //49200; (43400 to 44700 works for  angle =16, find the best value in that range)
  //(try 43320 to 44000 for fingertip catching)
  Vec trans(3);
  Vec transC = Vec("600 300 200",3); // Safe location in the center of the robot workspace.
 
  Quaternion quat;
  Quaternion quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand facing down
 
  Vec transO = Vec("600 200 100",3);  //Location of Hand before picking the object (Object location + 100 mm in Z direction)
  Quaternion quatO = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand grasping the object from top
  
  // " ' " USING SET JOINTS " ' "
  // Do the first command in joint coordinates
   robot.SetSpeed(TCPs, ORIs);
  robot.SetZone(ZONE);
  //robot.SetJoints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);

  
  hand.SetSpeed(HAND_SPEED_F);
  hand.SetForce(HAND_FORCE);
  
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
  
  usleep(1000000);
    
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0); // Inverting the position of the hand
  
    
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
   
  robot.SetCartesian(600, 700, 800, 0.7071, 0.0, 0.0, -0.7071);   //this is the topmost position
  usleep(2000000);
   
  hand.SetAngle(HAND_OPEN_ANGLE_topple);
  hand.WaitRest(0.250);
  ROS_INFO("Hand Partially Opened");
  
   //Close the hand 
  hand.SetSpeed(HAND_SPEED);
  hand.SetAngle(HAND_CLOSE_ANGLE);
  printf("Started regrasping");
  
  usleep(Tsleep_Topple);
    
  robot.SpecialCommand(3,1080.0,850.0,17.71,18.71,18.71);   //Maximum throw
  
  // (Arguments are == 3 (special command number), Z1, Z2, Acc1 till Z1, Acc2 till Z2, Acc3 after Z2) 
  
  robot.SetSpeed(TCPs, ORIs);
  robot.SetZone(ZONE);
  
  robot.SetCartesian(600, 700, 800, 0.7071, 0.0, 0.0, -0.7071);
  robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);
  hand.SetSpeed(HAND_SPEED_F);
  hand.SetAngle(HAND_CLOSE_ANGLE_After);
  hand.WaitRest(0.250);
  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0);
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  robot.SetCartesian(transC[0], transC[1], transC[2]-80, quatC[0], quatC[1], quatC[2], quatC[3]);
    
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(0.250);
  ROS_INFO("object placed");
  
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);

  ROS_INFO("Regrasp topple Completed!");
  
  string filename;
  vision.CaptureImage(1, filename);
  
  

  return 0;
}
