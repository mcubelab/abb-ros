//
// Name: Bowei Tang
//
// File Name: regrasp_node.cpp
//
// Last Modified: 3/06/2013
//
// This file implements the hand node, which controls P3, a simple hand 
// designed in the Manipulation Lab. P3 is controlled by an arduino. This
// program interacts with the arduino over a USB serial port. It records
// and publishes finger, motor, and palm sensor data. It allows the user
// to change the speed and current limit of the motor and then command it
// to move to a certain pose. Since the motor encoder is relative, it also
// implements a calibration routine so we can get consistent motor poses 
// across runs. Please look at the arduino code on P3 for more details with
// communication

#include "regrasp_node.h"

const double angle = 90.0*PI/180.0; // The rotation angle

// Globle variables:
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

Vec frame = Vec("100 350 75",3); // Frame position
Vec cam = Vec("550 280 270",3);   // Camera position
Vec trans(3);                     // A temporary vector
Vec transC = Vec("600 285 250",3);// Safe location in the center of the robot workspace.
Vec table = Vec("600 285 0",3);

Vec x = Vec("1.0 0.0 0.0", 3);
Vec y = Vec("0.0 1.0 0.0", 3);
Vec z = Vec("0.0 0.0 1.0", 3);

//////////////////////////////////////////////////////////////////////////
// Constructor. Saves the node, and configures some status variables
RegraspController::RegraspController(ros::NodeHandle *n) 
{
  node = n;
  robot.subscribe(n);
  hand.subscribe(n);
}

//////////////////////////////////////////////////////////////////////////
// This function sets up to advertise all services offered by this node
void RegraspController::advertiseServices()
{
  handle_regrasp_Ping = 
    node->advertiseService("regrasp_Ping", &RegraspController::regrasp_Ping, this);
  handle_regrasp_Initialize = 
    node->advertiseService("regrasp_Initialize", &RegraspController::regrasp_Initialize, this);
  handle_regrasp_Rotate = 
    node->advertiseService("regrasp_Rotate", &RegraspController::regrasp_Rotate, this);
  handle_regrasp_FreeDrop = 
    node->advertiseService("regrasp_FreeDrop", &RegraspController::regrasp_FreeDrop, this);
  handle_regrasp_FramePick = 
    node->advertiseService("regrasp_FramePick", &RegraspController::regrasp_FramePick, this);
  handle_regrasp_FrameDrop = 
    node->advertiseService("regrasp_FrameDrop", &RegraspController::regrasp_FrameDrop, this);
  handle_regrasp_StandtoLie = 
    node->advertiseService("regrasp_StandtoLie", &RegraspController::regrasp_StandtoLie, this);
  handle_regrasp_LietoStand = 
    node->advertiseService("regrasp_LietoStand", &RegraspController::regrasp_LietoStand, this);
  handle_regrasp_ThrowCatch = 
    node->advertiseService("regrasp_ThrowCatch", &RegraspController::regrasp_ThrowCatch, this);
}

//////////////////////////////////////////////////////////////////////////
// Check if the regrasp node is working

//// Could I use connectedArduino??

bool RegraspController::regrasp_Ping(regrasp_comm::regrasp_Ping::Request& req, 
    regrasp_comm::regrasp_Ping::Response& res)
{
  // Return true if we are connected to the arduino
  return true;
}


//////////////////////////////////////////////////////////////////////////
// Service Initializeded
bool RegraspController::regrasp_Initialize(regrasp_comm::regrasp_Initialize::Request& req, 
    regrasp_comm::regrasp_Initialize::Response& res)
{
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  ROS_INFO("Initializeded");
  res.ret = 1;
  return true;
}


//////////////////////////////////////////////////////////////////////////
// Service to Rotate the objects on the table.
bool RegraspController::regrasp_Rotate(regrasp_comm::regrasp_Rotate::Request& req, 
    regrasp_comm::regrasp_Rotate::Response& res)
{
  HomogTransf pose;
  Quaternion quatC, quatP;
  RotMat rotC, rotZ;

  Vec transCorrect = table + Vec("0 0 75",3);
  Vec transCorrectU = table + Vec("0 0 200",3);
  quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand faces down
  rotC = quatC.getRotMat(); 

  rotZ.setAxisAngle(z, req.rotateangle*angle);

  pose = HomogTransf(rotZ*rotC,transCorrect);
  quatP = pose.getRotation().getQuaternion();

// Action

if (req.rotateangle != 0)
{
  robot.SetSpeed(STCP, SORI);
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]); // Initial position
  robot.SetCartesian(transCorrectU[0], transCorrectU[1], transCorrectU[2], quatC[0], quatC[1], quatC[2], quatC[3]);  // Safty position
  robot.SetCartesian(transCorrect[0], transCorrect[1], transCorrect[2], quatP[0], quatP[1], quatP[2], quatP[3]);     // Place down and rotate
  hand.SetAngle(HAND_OPEN);
  hand.WaitRest(0.25);
  robot.SetCartesian(transCorrectU[0], transCorrectU[1], transCorrectU[2], quatP[0], quatP[1], quatP[2], quatP[3]);
  robot.SetCartesian(transCorrectU[0], transCorrectU[1], transCorrectU[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  robot.SetCartesian(transCorrect[0], transCorrect[1], transCorrect[2], quatC[0], quatC[1], quatC[2], quatC[3]);  
  hand.SetAngle(HAND_CLOSE_LIE);
  hand.WaitRest(0.25);
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]); // Initial position
}
else 
{
  robot.SetSpeed(STCP, SORI);
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]); // Initial position
  robot.SetCartesian(transCorrectU[0], transCorrectU[1], transCorrectU[2], quatC[0], quatC[1], quatC[2], quatC[3]);  // Safty position
  robot.SetCartesian(transCorrect[0], transCorrect[1], transCorrect[2], quatP[0], quatP[1], quatP[2], quatP[3]);     // Place down and rotate
  hand.SetAngle(HAND_OPEN);
  hand.WaitRest(0.25);
  hand.SetAngle(HAND_CLOSE_LIE);
  hand.WaitRest(0.25);
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]); // Initial position
}

  ROS_INFO("Rotate finished");
  res.ret = 1;
  return true;
}


//////////////////////////////////////////////////////////////////////////
// Service to put the objects on the table with another side of it.
bool RegraspController::regrasp_FreeDrop(regrasp_comm::regrasp_FreeDrop::Request& req, 
    regrasp_comm::regrasp_FreeDrop::Response& res)
{

  ROS_INFO("FreeDrop action finished");
  res.ret = 1;
  return true;
}


//////////////////////////////////////////////////////////////////////////
// Service to put the objects on the frame then pick it in the vertical direction
bool RegraspController::regrasp_FramePick(regrasp_comm::regrasp_FramePick::Request& req, 
    regrasp_comm::regrasp_FramePick::Response& res)
{
  
  ROS_INFO("FramePick action finished");
  res.ret = 1;
  return true;
}


//////////////////////////////////////////////////////////////////////////
// Service to use the frame to help drop the objects and change a vertical grasping direction
bool RegraspController::regrasp_FrameDrop(regrasp_comm::regrasp_FrameDrop::Request& req, 
    regrasp_comm::regrasp_FrameDrop::Response& res)
{
  HomogTransf pose;
  Quaternion quat, quatC, quatX, quatZ;
  RotMat rotDown, rotC, rotX, rotZ;

  quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand faces down

  rotC = quatC.getRotMat();

  Vec transF = frame + Vec("36 0 25",3);       // Hand get the position that needs to be calculated to make sure the angle is right

// Action
  robot.SetCartesian(transF[0], transF[1], transF[2], quatC[0], quatC[1], quatC[2], quatC[3]); // Hand get back and ready to Rotate down.
  hand.SetAngle(HAND_RELEASE);                 // When the hand releases, the stuff will Rotate around the touching point, but will not drop down
  hand.WaitRest(1.5);
  hand.SetAngle(HAND_CLOSE);
  hand.WaitRest(5.25);

  ROS_INFO("FrameDrop action finished");
  res.ret = 1;
  return true;
}

bool RegraspController::regrasp_LietoStand(regrasp_comm::regrasp_LietoStand::Request& req, 
    regrasp_comm::regrasp_LietoStand::Response& res)
{
  ROS_INFO("LietoStand action start");
  HomogTransf pose;
  Quaternion quat, quatC, quatCommon, quatMore, quatL;
  RotMat rotC, rotZ, rotZr, rotZl, rotX;

  Vec transU = frame + Vec("0 0 300",3); // Hand on top
  Vec transPut = frame + Vec("-15 5 0",3); // The lower position of the hand to put the block

  std::cout << transPut << std::endl;

  transPut[0] = transPut[0] + req.x;
  transPut[1] = transPut[1] + req.y;
  transPut[2] = transPut[2] + req.z;
  
  std::cout << transPut << std::endl;
  Vec transPutP = frame + Vec("0 0 100",3); // The higher position of the hand to put the block
  Vec transL = frame + Vec("150 0 45",3); // Hand on left, ready to pick
  Vec transPick = frame + Vec("30 3 32",3); // Picking position

  quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand faces down
  rotC = quatC.getRotMat(); 

  rotZ.setAxisAngle(z, angle);
  rotX.setAxisAngle(x, -angle);
  rotZr.setAxisAngle(z, req.rotateangle*angle);  //rotate 45 degree more
  
  
  pose = HomogTransf(rotC,transU);
  quatCommon = pose.getRotation().getQuaternion();   //commonlly rotate 90 degree

  pose = HomogTransf(rotC*rotZr,transU);
  quatMore = pose.getRotation().getQuaternion();   //rotate 45 degree more
    
  pose = HomogTransf(rotC*rotZ*rotZ*rotX,transL);   
  quatL = pose.getRotation().getQuaternion();

// HOLD
  hand.SetAngle(HAND_CLOSE_LIE);
  hand.WaitRest(1.25);

// Action:
  robot.SetSpeed(FTCP, FORI);
  robot.SetCartesian(transU[0], transU[1], transU[2], quatCommon[0], quatCommon[1], quatCommon[2], quatCommon[3]); // Hand get to top and ready to put.
  robot.SetSpeed(STCP, SORI);
  robot.SetCartesian(transPut[0], transPut[1], transPut[2], quatMore[0], quatMore[1], quatMore[2], quatMore[3]); // Put down.
  hand.SetAngle(HAND_OPEN);                 // When the hand releases, the stuff will rotate around the touching point, but will not drop down
  hand.WaitRest(1.5);

  robot.SetCartesian(transU[0], transU[1], transU[2], quatCommon[0], quatCommon[1], quatCommon[2], quatCommon[3]); // Hand get back
  robot.SetSpeed(FTCP, FORI);
  robot.SetJoints(-29.0, 33.0, 28.0, 20.0, 40.0, -147.0); // An safty position
  robot.SetCartesian(transL[0], transL[1], transL[2], quatL[0], quatL[1], quatL[2], quatL[3]); // Hand get to left and ready to pick
  robot.SetSpeed(STCP, SORI);
  robot.SetCartesian(transPick[0], transPick[1], transPick[2], quatL[0], quatL[1], quatL[2], quatL[3]); // Pick
  hand.SetAngle(HAND_CLOSE_STAND);
  hand.WaitRest(1.25);
  robot.SetCartesian(transU[0], transU[1], transU[2], quatL[0], quatL[1], quatL[2], quatL[3]); // For security
  robot.SetSpeed(FTCP, FORI);
  robot.SetJoints(-29.0, 33.0, 28.0, 20.0, 40.0, -147.0);
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]); // Hand get back

  res.ret = 1;
  return true;
}


bool RegraspController::regrasp_StandtoLie(regrasp_comm::regrasp_StandtoLie::Request& req, 
    regrasp_comm::regrasp_StandtoLie::Response& res)
{
  ROS_INFO("StandtoLie action start");
  HomogTransf pose;
  Quaternion quat, quatC, quatCommon, quatMore, quatMoreTwo, quatL, quatLPick;
  RotMat rotC, rotZ, rotHalf, rotNormal, rotX, rotZPick;

  Vec transU = frame + Vec("0 0 300",3); // Hand on top
  Vec transPut = frame + Vec("-15 -5 0",3); // The position of the hand to put the block
  
  transPut[0] = transPut[0] + req.x;
  transPut[1] = transPut[1] + req.y - 60*sin(req.rotateangle*angle);
  transPut[2] = transPut[2] + req.z - 80*(1-cos(req.rotateangle*angle));
  Vec transL = frame + Vec("150 0 45",3); // Hand on left, ready to pick
  Vec transPick = frame + Vec("30 3 30",3); //Picking Position after flipinhand.....Vec("25 3 40",3); // Picking position
  //Vec transPick = frame + Vec("34 3 30",3);  // Picking position for place and pick x=37 earlier>>39>>34


  quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand faces down
  rotC = quatC.getRotMat(); 

  rotZ.setAxisAngle(z, angle);
  rotX.setAxisAngle(x, -angle);
  rotHalf.setAxisAngle(y, req.rotateangle*angle/2);
  rotNormal.setAxisAngle(y, req.rotateangle*angle);  //rotate 45 degree more
  rotZPick.setAxisAngle(z, -135*PI/180);
  
  pose = HomogTransf(rotC,transU);
  quatCommon = pose.getRotation().getQuaternion();   //commenlly rotate 90 degree

  pose = HomogTransf(rotC*rotNormal,transU);
  quatMore = pose.getRotation().getQuaternion();
    
  pose = HomogTransf(rotC*rotZ*rotZ*rotX,transL);
  quatL = pose.getRotation().getQuaternion();
  
  pose = HomogTransf(rotC*rotZ*rotZ*rotX*rotZPick,transL);  //Added by nikhil as sequense of regrasps needed specific orientaion
  quatLPick = pose.getRotation().getQuaternion();

// HOLD
  hand.SetAngle(HAND_CLOSE_STAND);
  hand.WaitRest(1.25);

// Action:
  robot.SetSpeed(FTCP, FORI);
  robot.SetCartesian(transU[0], transU[1], transU[2], quatCommon[0], quatCommon[1], quatCommon[2], quatCommon[3]); // Hand get to top and ready to put.
  robot.SetSpeed(STCP, SORI);
  robot.SetCartesian(transPut[0], transPut[1], transPut[2], quatMore[0], quatMore[1], quatMore[2], quatMore[3]); // Put down first step.
  hand.SetAngle(HAND_RELEASE);                 // When the hand releases, the stuff will rotate around the touching point, but will not drop down
  hand.WaitRest(1.0);
  hand.SetAngle(HAND_OPEN);
  robot.SetCartesian(transU[0], transU[1], transU[2], quatCommon[0], quatCommon[1], quatCommon[2], quatCommon[3]); // Hand get back
  robot.SetSpeed(FTCP, FORI);
  robot.SetJoints(-29.0, 33.0, 28.0, 20.0, 40.0, -147.0); // A safty position
  hand.SetAngle(HAND_OPEN_BeforePick);
  hand.WaitRest(1.0);
  robot.SetCartesian(transL[0], transL[1], transL[2], quatL[0], quatL[1], quatL[2], quatL[3]); // Hand get to left and ready to pick
  robot.SetCartesian(transL[0], transL[1], transL[2], quatLPick[0], quatLPick[1], quatLPick[2], quatLPick[3]);
  
  robot.SetSpeed(STCP, SORI);
  //robot.SetCartesian(transPick[0], transPick[1], transPick[2], quatL[0], quatL[1], quatL[2], quatL[3]); // Pick
  robot.SetCartesian(transPick[0], transPick[1], transPick[2], quatLPick[0], quatLPick[1], quatLPick[2], quatLPick[3]); // Added by Nikhil 
  hand.SetAngle(HAND_CLOSE_LIE);
  ros::Duration(1.5).sleep();
  robot.SetSpeed(8,10);
  robot.SetCartesian(transPick[0], transPick[1], transPick[2]+10.0, quatLPick[0], quatLPick[1], quatLPick[2], quatLPick[3]); // Pick
  robot.SetSpeed(STCP, SORI);
  hand.WaitRest(0.25);
  robot.SetCartesian(transU[0], transU[1], transU[2], quatLPick[0], quatLPick[1], quatLPick[2], quatLPick[3]); // Added by Nikhil
  robot.SetCartesian(transU[0], transU[1], transU[2], quatL[0], quatL[1], quatL[2], quatL[3]); // for Security
  
  robot.SetSpeed(FTCP, FORI);
  robot.SetJoints(-29.0, 33.0, 28.0, 20.0, 40.0, -147.0);
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]); // Hand get back

  res.ret = 1;
  return true;
}


bool RegraspController::regrasp_ThrowCatch(regrasp_comm::regrasp_ThrowCatch::Request& req, 
    regrasp_comm::regrasp_ThrowCatch::Response& res)
{
  res.ret = 1;
  return true;
}



//////////////////////////////////////////////////////////////////////////
// Main loop. Initializes the hand node, forks a process to read in sensorquatMoreOne
// data, advertises services and messages, and waits for shutdown
int main(int argc, char** argv)
{
  ros::init(argc, argv, "regrasp_controller");
  ros::NodeHandle node;
  RegraspController Regrasp(&node);

  //Advertise ROS services
  ROS_INFO("REGRASP_CONTROLLER: Advertising ROS services...");
  Regrasp.advertiseServices();

  //Main ROS loop
  ROS_INFO("REGRASP_CONTROLLER: Running node /regrasp_controller...");
  //Multithreaded spinner so that callbacks can be handled on separate threads.
  ros::spin();
  ROS_INFO("REGRASP_CONTROLLER: Shutting down node /regrasp_controller...");

  
  ROS_INFO("REGRASP_CONTROLLER: Done.");
  return 0;
}




