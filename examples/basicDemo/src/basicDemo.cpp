#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <matVec/matVec.h>

// Speed of the robot
#define TCP 60.0   // mm / s
#define ORI 20.0   // degrees / s

// Zone of the robot
#define ZONE 0 //0 means every commanded position is a fine position. (No fly-by)


int main(int argc, char** argv)
{

  std::string robotid = "";
  if(argc == 2)
    robotid = argv[1];
    
  ros::init(argc, argv,  std::string("basicDemo") + robotid);
  ros::NodeHandle node;
  
  RobotComm robot(&node, robotid);

  ROS_INFO("Set robot_node configuration");

  // robot.SetWorkObject(WORK_X, WORK_Y, WORK_Z, WORK_Q0, WORK_QX, WORK_QY, WORK_QZ))   // Not necessary
  // robot.SetTool(TOOL_X, TOOL_Y, TOOL_Z, TOOL_Q0, TOOL_QX, TOOL_QY, TOOL_QZ))   // Not necessary

  // Make sure robot communication is blocking
  if (!robot.SetComm(BLOCKING))
    return -1;
  
  // Set default speed limits
  if (!robot.SetSpeed(TCP, ORI))
    return -1;
  
  // Set the default "zone" of our robot (amount of interpolation we allow)
  if (!robot.SetZone(ZONE))
    return -1;
    
  ROS_INFO("Beginning demo...");

  // All cartesian moves are with respect to the proximal left corner of the table.
  // (where the default work object is located)
  // X-axis along the edge that goes left to right (facing the robot)
  // Y-axis along the edge that goes front to back
  // Z-axis pointing up.

  // Auxiliary variables
  HomogTransf pose;
  Vec trans(3);
  //Vec transC = Vec("600 300 200",3); // Safe location in the center of the robot workspace.
  Vec transC = Vec("350 0 250",3); // Safe location in the center of the robot workspace. mcube
  Quaternion quat;
  Quaternion quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand facing down
  

  // Do the first command in joint coordinates
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);

  // Move to the center of the robot workspace
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
    

  // Move in a square
  robot.SetCartesian(transC[0] + 50.0, transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  robot.SetCartesian(transC[0] + 50.0, transC[1] + 50.0, transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  robot.SetCartesian(transC[0], transC[1] + 50.0, transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  	
    
  // Rotate orientation of TCP
  if (!robot.SetZone(1))
    return -1;
  //robot.SetZone(4);    
  Vec x = Vec("1.0 0.0 0.0", 3);
  Vec y = Vec("0.0 1.0 0.0", 3);
  RotMat rotC = quatC.getRotMat();
  double angle = 10.0*PI/180.0;
  for (int i=0; i<40; i++)
    {
      double kx = sin(2*PI*i/40.0);      
      double ky = cos(2*PI*i/40.0);
      Vec axis = x*kx + y*ky;
      RotMat rot;
      rot.setAxisAngle(axis, angle);          
      pose = HomogTransf(rot*rotC,transC);
      trans = pose.getTranslation();
      quat = pose.getRotation().getQuaternion(); 
      robot.SetCartesian(trans[0], trans[1], trans[2], quat[0], quat[1], quat[2], quat[3]);
      //ros::Duration(0.1).sleep();
    }

  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  
  ROS_INFO("Demo Completed!");

  return 0;
}
