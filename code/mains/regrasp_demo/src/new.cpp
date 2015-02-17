#include <ros/ros.h>
#include <ros/package.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <matVec/matVec.h>
#include <regrasp_comm/regrasp_comm.h>
#include <util_comm/util_comm.h>
#include <objRec_comm/objRec_comm.h>
#include <handRec_comm/handRec_comm.h>
#include <matlab_comm/matlab_comm.h>
#include <tableVision_comm/tableVision_comm.h>

#include <ctime>
#include <unistd.h>

// Speed of the robot
#define TCPs 100.0   // Slow speed mm / s
#define ORIs 35.0   // degrees / s

#define HAND_FORCE 0.90
#define HAND_SPEED 0.90

#define HAND_CLOSE_ANGLE_AfterRegrasp 100.0  
#define HAND_CLOSE_ANGLE 80.0  // Max closing angle
#define HAND_OPEN_ANGLE 20.0   // Max opening angle

// Zone of the robot
#define ZONE 0 //0 means every commanded position is a fine position. (No fly-by)

static const Vec TRANS_PICK("600 200 73",3);
static const Vec TRANS_PLACE("600 200 73",3);

static const char BASE_FOLDER[] = "/home/simplehands/Desktop/new_logs/regrasp_logs";

Vec trans(3);
Vec transC = Vec("600 300 200",3); // Safe location in the center of the robot workspace.
//Quaternion quat;
Quaternion quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand facing down
Quaternion quatDrop = Quaternion("0.5 -0.5 -0.5 0.5");
Vec transO = Vec("600 200 80",3);  //Location of Hand before picking the object 
Quaternion quatO = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand grasping the object from top

Vec transOrient = Vec("1075 710 320", 3);
Quaternion quatOrient = Quaternion("0.5398 -0.8417 0.01009 0.0");
// Vec transPickOrient = Vec("1079.8 701.7 266.3",3);
// Quaternion quatPickOrient = Quaternion("0.2694 -0.4369 -0.7361 -0.4412");
Vec transPlatform = Vec("900 385 380", 3); 
Quaternion quatPlatform = Quaternion("0.0 0.707 0.707 0.0");
Vec transNewBlock = Vec("200 450 90", 3);
Quaternion quatNewBlock = Quaternion("0.0 0.707 0.707 0.0");

double motorAngle;
double fingerAngle[3];
int Envalue[3];
int motEn;
int Envalue2[3];
int motEn2;

bool orient(RobotComm robot, HandComm hand);
bool flipInHand(RobotComm robot, HandComm hand);
bool getPose(RobotComm robot, HandComm hand, HandRecComm handRec, HomogTransf objPose);
bool placeOnPlatform(RobotComm robot, HandComm hand);
bool getNewBlock(RobotComm robot, HandComm hand);
bool findBlockOnTable(RobotComm robot, HandComm hand, TableVisionComm tableVision, 
		      Vec blockVec, Quaternion blockQuat);
HomogTransf lookAt(Vec pose, Vec eye, Vec up);
bool getRandomGrasp(RobotComm robot, double approach[7], double pick[7], double angle);

static const int TOTAL_BLOCKS = 1;
int numResetBlocks = TOTAL_BLOCKS;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "regrasp_flipinhand");
  ros::NodeHandle node;

  RobotComm robot(&node);
  HandComm hand(&node);
  RegraspComm regrasp(&node);
  HandRecComm handRec(&node);
  TableVisionComm tableVision(&node);
  MatlabComm matlab;

  matlab.subscribe(&node);
  std::string pkg_path = ros::package::getPath("regrasp_demo");
  pkg_path += "/matlab_scripts";
  matlab.addPath(pkg_path.c_str());

  handRec.SetObject(RecObj::BIG_TRIANGLE);
  handRec.SetPrefOrient(Quaternion("1.0 0.0 0.0 0.0"));

  hand.SetForce(HAND_FORCE);
  hand.SetSpeed(HAND_SPEED);

  //Make sure robot communication is blocking
  if (!robot.SetComm(BLOCKING))
    return -1;

  // Set default speed limits
  if (!robot.SetSpeed(TCPs, ORIs))
    return -1;

  // Set the default "zone" of our robot (amount of interpolation we allow)
  if (!robot.SetZone(ZONE))
    return -1;

  ROS_INFO("Initialize");
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);  
  hand.SetAngle(5);
  hand.WaitRest(0.250);

  ROS_INFO("Picking up the block");
  robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
  robot.SetCartesian(transO[0], transO[1], transO[2], quatO[0], quatO[1], quatO[2], quatO[3]);
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(0.250);
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0); 

  // flipInHand(robot, hand);
  // robot.SetCartesian(transC[0], transC[1], transC[2], quatDrop[0], quatDrop[1], quatDrop[2], quatDrop[3]);
  // hand.SetAngle(0);
  // hand.WaitRest(0.250);

  // return 0;

  ROS_INFO("Precisely orienting the block.");
  orient(robot, hand);

  ROS_INFO("Placing on platform");
  placeOnPlatform(robot, hand);

  srand((unsigned)time(NULL)); 
  int regrasp_num=0;

  while (regrasp_num<=10)
    //while (regrasp <= 1000)
  {
    regrasp_num+=1;
    //ROS_INFO("Regrasp Flip in Hand %d",regrasp_num);
 
    /******************* Pick off platform ************************/
    // Pick object off platform with a randomized grasp

    //Open the hand
    hand.SetAngle(0);
    hand.WaitRest(0.250);

    bool rand_suc = false;
    double approach [7];
    double pick [7];
    double angle;
    do 
      {
	rand_suc = getRandomGrasp(robot, approach, pick, angle);
      } while (!rand_suc);

    // execute grasp
    robot.SetCartesian(approach[0], approach[1], approach[2], 
		       approach[3], approach[4], approach[5], approach[6]);
    robot.SetCartesian(pick[0], pick[1], pick[2], pick[3], pick[4], pick[5], pick[6]);
    hand.SetAngle(angle);
    hand.WaitRest(0.250);


    // robot.SetJoints(0.0,0.0,0.0,0.0,90.0,0.0);
    /*********************************************************/

    /********************* Show to Camera ********************/

    // Close the hand, and wait until we're done
    hand.SetAngle(HAND_CLOSE_ANGLE);
    //  hand.SetAngle(30.0);
    hand.WaitRest(0.25);

    HomogTransf init_objPose;
    bool success = getPose(robot, hand, handRec, init_objPose);
    /*********************************************************/    

    /***************** Execute Regrasp ***********************/
    // Execute regrasp
    flipInHand(robot, hand);
    /*********************************************************/
   
    /***************** Show to Camera ************************/
    // Show to Camera
    HomogTransf final_objPose;
    bool final_success = getPose(robot, hand, handRec, final_objPose);
    /*********************************************************/
    robot.SetCartesian(transC[0], transC[1], transC[2], quatDrop[0], quatDrop[1], quatDrop[2], quatDrop[3]);
  hand.SetAngle(0);
  hand.WaitRest(0.250);


    /***************** Reset *********************************/
    if (final_success) // block still in the hand
      {
        ROS_INFO("Still holding the block. Setting the block down to reset.");
	robot.SetCartesian(transC[0], transC[1], transC[2], 
			   quatDrop[0], quatDrop[1], quatDrop[2], quatDrop[3]);
  	hand.SetAngle(HAND_OPEN_ANGLE);
  	hand.WaitRest(0.250);
  
	// move arm out of the way
	robot.SetJoints(30.0,0.0,0.0,0.0,90.0,0.0);
      }

    // look for block on the table
    Vec blockVec(3);
    Quaternion blockQuat;
    bool suc = findBlockOnTable(robot, hand, tableVision, blockVec, blockQuat);
    
    if (!suc) // no block
      {
  	getNewBlock(robot, hand);
      }
    else // block seen on the table
      {	
  	//Open the hand
  	hand.SetAngle(HAND_OPEN_ANGLE);
  	hand.WaitRest(0.250);
  	ROS_INFO("Hand Opened");
	
  	robot.SetCartesian(blockVec[0], blockVec[1], blockVec[2]+100, 
  			   blockQuat[0], blockQuat[1], blockQuat[2], blockQuat[3]); 
  	robot.SetCartesian(blockVec[0], blockVec[1], blockVec[2], 
  			   blockQuat[0], blockQuat[1], blockQuat[2], blockQuat[3]); 
	
  	hand.SetAngle(HAND_CLOSE_ANGLE);
  	hand.WaitRest(0.250);
  	ROS_INFO("Hand Closed");
	
      }

    // Place object on orienter
    orient(robot, hand);
    // Place on platform
    placeOnPlatform(robot, hand);
    
   }

  return 0;
}


bool getPose(RobotComm robot, HandComm hand, HandRecComm handRec, HomogTransf objPose)
{
  cout << "Getting object pose from camera" << endl;

  hand.GetAngles(motorAngle, fingerAngle);
  double avg_angle = (fingerAngle[0]+fingerAngle[1]+fingerAngle[2])/3;
  if (avg_angle < 35)
    {
      int obj = -1;
      bool ret = handRec.GetPose(obj, objPose);
    }
  else
    {
      ROS_INFO("No block in the hand. Skipping vision.");
      return false;
    }

  return true;	  




      // do 
      // 	{
      // 	  count++;

      // 	  int obj = -1;
      // 	  ret = handRec.GetPose(obj, objPose);

      // 	  hand.GetAngles(motorAngle, fingerAngle);
      // 	  avg_angle = (fingerAngle[0]+fingerAngle[1]+fingerAngle[2])/3;

      // 	  ret = getPose(robot, hand, util, objPose);
      // 	}
      // while ((ret == 0) && (count< 1)); // vision failed but block in hand

}

bool flipInHand(RobotComm robot, HandComm hand)
{
  cout << "Executing flip in hand" << endl;

  robot.SetJoints(-20,55,20,0,-110,-90);
  
  usleep(1700000);
  
  //Open the hand slightly
  hand.GetAngles(motorAngle, fingerAngle);
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
  int sleeptime = 1500000; 
      
  //1-Open hand just enough to drop the object
  hand.SetAngle(minfingerAngle-25);
  usleep(sleeptime);
  
  hand.SetAngle(HAND_CLOSE_ANGLE_AfterRegrasp);
  hand.WaitRest(0.250);
  ROS_INFO("Hand Closed");
  
  robot.SetJoints(0,0,0,0,90,0);
  
  return true;
}

bool orient(RobotComm robot, HandComm hand)
{
  cout << "Place pick object on orienter" << endl;
  robot.SetJoints(0.0,0.0,0.0,0.0,90.0,0.0);

  // Place on the orienter  
  robot.SetCartesian(transOrient[0], transOrient[1], transOrient[2], 
                     quatOrient[0], quatOrient[1], quatOrient[2], 
                     quatOrient[3]);
  hand.SetAngle(0);
  hand.WaitRest(0.250);
  ROS_INFO("Object on Orienter");
  robot.SetCartesian(transOrient[0], transOrient[1]-15, transOrient[2], 
                     quatOrient[0], quatOrient[1], quatOrient[2], 
                     quatOrient[3]);
  
  // Pick up off the orienter
  robot.SetCartesian(transOrient[0], transOrient[1]-25, transOrient[2]-55, 
                     quatOrient[0], quatOrient[1], quatOrient[2], 
                     quatOrient[3]);
  hand.SetAngle(15);
  hand.WaitRest(0.250);
  robot.SetCartesian(transOrient[0], transOrient[1]-5, transOrient[2]-55, 
                     quatOrient[0], quatOrient[1], quatOrient[2], 
                     quatOrient[3]);
  hand.SetAngle(60);
  hand.WaitRest(0.250);

  robot.SetCartesian(transOrient[0], transOrient[1]-25, transOrient[2]-35, 
                     quatOrient[0], quatOrient[1], quatOrient[2], 
                     quatOrient[3]);
  robot.SetCartesian(transOrient[0], transOrient[1]-25, transOrient[2]-35, 
                     quatOrient[0], quatOrient[1], quatOrient[2], 
                     quatOrient[3]);
  robot.SetJoints(0.0,0.0,0.0,0.0,90.0,0.0);
  
  return true;
}
    
bool placeOnPlatform(RobotComm robot, HandComm hand)
{
  cout << "Placing the object on the platform" << endl;
  // Place object on platform (assuming you are already holding block)
  robot.SetCartesian(transPlatform[0], transPlatform[1], 
		     transPlatform[2]+100, quatPlatform[0], 
		     quatPlatform[1], quatPlatform[2], quatPlatform[3]);
  robot.SetCartesian(transPlatform[0], transPlatform[1], transPlatform[2], 
		     quatPlatform[0], quatPlatform[1], quatPlatform[2], 
		     quatPlatform[3]);
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(0.250);
  
  // retract
  robot.SetCartesian(transPlatform[0], transPlatform[1], 
		     transPlatform[2]+150, quatPlatform[0], 
		     quatPlatform[1], quatPlatform[2], quatPlatform[3]);
  //robot.SetJoints(0.0,0.0,0.0,0.0,90.0,0.0);
  
  return true;
}

bool getNewBlock(RobotComm robot, HandComm hand)
{
  cout << "Getting new block" << endl;

  double blockHeight = 0.03; // TODO
  if (numResetBlocks > 0)
    {
      numResetBlocks--;
      hand.SetAngle(HAND_OPEN_ANGLE);
      hand.WaitRest(0.250);

      // transNewBlock[2] is the height to pick up the lowest block
      double z = transNewBlock[2] + numResetBlocks*blockHeight;
      
      robot.SetCartesian(transNewBlock[0], transNewBlock[1], z+100, 
			 quatNewBlock[0], quatNewBlock[1], quatNewBlock[2], quatNewBlock[3]);
      robot.SetCartesian(transNewBlock[0], transNewBlock[1], z, 
			 quatNewBlock[0], quatNewBlock[1], quatNewBlock[2], quatNewBlock[3]);
      
      hand.SetAngle(HAND_CLOSE_ANGLE);
      hand.WaitRest(0.250);
      robot.SetJoints(0.0,0.0,0.0,0.0,90.0,0.0);
    }
  else
    {
      ROS_INFO("NO MORE RESET BLOCKS. WAITING TO BE REFILLED. TAP FINGER WHEN READY.");

      double mot_ang;
      Vec ini_fing_angs(3);
      Vec cur_fing_angs(3);
      ros::Rate loop_rate(20);
      while(ros::ok())
	{
	  hand.GetAngles(mot_ang, cur_fing_angs);
	  if ((ini_fing_angs - cur_fing_angs).norm() > 2.0)
	    break;
	  loop_rate.sleep();
	}
      numResetBlocks = TOTAL_BLOCKS;
    }
   
  return true;
}

bool findBlockOnTable(RobotComm robot, HandComm hand, TableVisionComm tableVision, 
		      Vec blockVec, Quaternion blockQuat)
{
  cout << "Looking for block on the table" << endl;

  int count = 0;
  bool success = false;

  do 
    {
      success = tableVision.GetPose(RecObj::BIG_TRIANGLE, blockVec, blockQuat);
    }
  while ((!success) && (count<50));
  
  return success;
}


HomogTransf lookAt(Vec pose, Vec eye, Vec up)
{

  Vec z = eye - pose;
  z.normalize();

  Vec y = up;
  
  Vec x(3); 
  x[0] =  y[1]*z[2] - y[2]*z[1];
  x[1] = -y[0]*z[2] + y[2]*z[0];
  x[2] =  y[0]*z[1] - y[1]*z[0];

  y[0] =  z[1]*x[2] - z[2]*x[1];
  y[1] = -z[0]*x[2] + z[2]*x[0];
  y[2] =  z[0]*x[1] - z[1]*x[0];
   
  x.normalize();
  y.normalize();

  HomogTransf M;
  M[0][0] = x[0];  
  M[0][1] = x[1];  
  M[0][2] = x[2];  
  M[0][3] = -x[0]*eye[0] + -x[1]*eye[1] + -x[2]*eye[2];
  M[1][0] = y[0];  
  M[1][1] = y[1];  
  M[1][2] = y[2];  
  M[1][3] = -y[0]*eye[0] + -y[1]*eye[1] + -y[2]*eye[2];
  M[2][0] = z[0];  
  M[2][1] = z[1];  
  M[2][2] = z[2];  
  M[2][3] = -z[0]*eye[0] + -z[1]*eye[1] + -z[2]*eye[2];
  M[3][0] = 0.0;   
  M[3][1] = 0.0;   
  M[3][2] = 0.0;   
  M[3][3] = 1.0;

  return M;

}

bool getRandomGrasp(RobotComm robot, double approach[7], double pick[7], double angle)
{
    // neg x, pos y, both in z
    int minApproachX = 15;
    int minApproachY = 15;
    int minApproachZ = 35;
    int maxApproachX = 75;
    int maxApproachY = 75;
    int maxApproachZ = 100;
    Vec transApproach = Vec(transPlatform);
    ROS_INFO("orig transApproach: [%f %f %f]", transApproach[0], transApproach[1], transApproach[2]);
    
    // we want to approach from the positive y, neg x, and pos z
    transApproach[0] = transApproach[0] + ((double)(rand() % (maxApproachX)) + minApproachX);
    transApproach[1] = transApproach[1] + ((double)(rand() % (maxApproachY)) + minApproachY);
    transApproach[2] = transApproach[2] + ((double)(rand() % (maxApproachZ)) + minApproachZ);

    //ROS_INFO("transApproach: [%f %f %f]", transApproach[0], transApproach[1], transApproach[2]);
    
    // randomize dist from center of object
    int maxOffsetX = 5;
    int maxOffsetY = 5;
    int maxOffsetZ = 5;
    Vec transPick = Vec(transPlatform);
    transPick[0] = ((double)(rand() % (maxOffsetX*2)) - maxOffsetX) + transPick[0];
    transPick[1] = ((double)(rand() % (maxOffsetY*2)) - maxOffsetY) + transPick[1];
    transPick[2] = ((double)(rand() % (maxOffsetZ*2)) - maxOffsetZ) + transPick[2];


    Vec pose = transPick;
    Vec eye = transApproach;
    Vec up = Vec("1 0 0", 3); // "up" direction
    HomogTransf h = lookAt(eye, pose, up);
    Quaternion quatPick = h.getQuaternion();
    quatPick.normalize();
    Vec t = h.getTranslation();

    // randomize finger angle
    int minCloseOffset = 25;
    int maxCloseOffset = 80-minCloseOffset;
    angle = 80;//(rand() % maxCloseOffset)+minCloseOffset;
    
    approach[0] = transApproach[0];
    approach[1] = transApproach[1];
    approach[2] = transApproach[2];
    approach[3] = quatPick[0]; 
    approach[4] = quatPick[1];
    approach[5] = quatPick[2]; 
    approach[6] = quatPick[3];

    pick[0] = transPick[0];
    pick[1] = transPick[1];
    pick[2] = transPick[2];
    pick[3] = quatPick[0];
    pick[4] = quatPick[1];
    pick[5] = quatPick[2];
    pick[6] = quatPick[3];

    ROS_INFO("close angle: %f", angle);
    ROS_INFO("approach vec: [%f %f %f]", transApproach[0], transApproach[1], transApproach[2]);
    ROS_INFO("pick pose: [%f %f %f]", transPick[0], transPick[1], transPick[2]);
    ROS_INFO("grasp quat: [%f %f %f %f]", quatPick[0], quatPick[1], quatPick[2], quatPick[3]);

    // Check that poses are safe
    HomogTransf approach_pose = HomogTransf(approach);
    HomogTransf pick_pose = HomogTransf(pick);
    double app_joints[6];
    double pick_joints[6];
    bool app_suc = robot.GetIK(approach_pose, app_joints);
    bool pick_suc = robot.GetIK(pick_pose, pick_joints);

    if (app_suc && pick_suc)
      {
	return true;
      }
    else 
      {
	return false;
      }

}
