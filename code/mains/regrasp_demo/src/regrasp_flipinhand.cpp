//In Hand flipping of an Object
//Last Updated on 3/12/2013 12.55 AM by Nikhil 

#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <matVec/matVec.h>
#include <regrasp_comm/regrasp_comm.h>
#include <util_comm/util_comm.h>
#include <objRec_comm/objRec_comm.h>

#include <ctime>
#include <unistd.h>

// Speed of the robot
#define TCPs 100.0   // Slow speed mm / s
#define ORIs 35.0   // degrees / s

#define TCPf 350.0   // FAST SPEED mm / s
#define ORIf 50.0   // degrees / s

//#define N_ITERATIONS 150
//#define N_STEPS 50

#define HAND_FORCE 0.90
#define HAND_SPEED 0.90
#define HAND_CLOSE_ANGLE 80.0  // Max closing angle
#define HAND_CLOSE_ANGLE_AfterRegrasp 100.0  // Hand close angle after regrasp..changed from 90 to 95 as Stand to Lie uses 100 as close angle
#define HAND_CLOSE_ANGLE_partial 40.0 
#define HAND_OPEN_ANGLE_drop 60.0 // opening angle sufficient enough to drop the object

#define HAND_OPEN_ANGLE 20.0   // Max opening angle

// Zone of the robot
#define ZONE 0 //0 means every commanded position is a fine position. (No fly-by)
//#define ZONE_d 4 // d for drop: zone when robot drops the object and moves down fast.


// Maximum and minimum pertubation errors
#define MAX_ERX 20
#define MIN_ERX -10
#define MAX_ERY 15
#define MIN_ERY -20
#define MAX_ERA 20
#define MIN_ERA -20
#define MAX_ERZ 3
#define MIN_ERZ -3

static const Vec TRANS_PICK("600 200 73",3);
static const Vec TRANS_PLACE("600 200 75",3);

static const HomogTransf initialGuess(Quaternion("-0.0009 0.9916 -0.1294 -0.0055").getRotMat(), Vec("0.4812291  0.3341413  0.420",3));
static const HomogTransf successGuess(Quaternion("-0.4291 0.0757 0.7083 0.5553").getRotMat(), Vec("0.5165313  0.2740930  0.3970514",3));
static const HomogTransf failGuess(Quaternion("-0.0009 0.9916 -0.1294 -0.0055").getRotMat(), Vec("0.4812291  0.3341413  0.4637221",3));

//quaternion for failure position with respect to hand
static const Quaternion QUAT_FAIL("-0.406977, 0.00945516, 0.0159353, 0.913228");

static const char BASE_FOLDER[] = "/home/simplehands/Desktop/regrasp_logs";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "regrasp_flipinhand");
  ros::NodeHandle node;

  RobotComm robot(&node);
  HandComm hand(&node);
  RegraspComm regrasp(&node);
  UtilComm util(&node);
  ObjRecComm objRec(&node);
  
  /*
  // Quick vision test code
  int obj3;
  HomogTransf vision3;
  std::string filenames3[NUM_OBJREC_VIEWS];
  filenames3[0] = "/home/simplehands/Desktop/objRec/pointsFolder/points__2013_03_13__01_43_28.pcd";
  filenames3[1] = "/home/simplehands/Desktop/objRec/pointsFolder/points__2013_03_13__01_43_32.pcd";
  filenames3[2] = "/home/simplehands/Desktop/objRec/pointsFolder/points__2013_03_13__01_43_35.pcd";
  filenames3[3] = "/home/simplehands/Desktop/objRec/pointsFolder/points__2013_03_13__01_43_39.pcd";
  objRec.SetGuess(initialGuess);
  util.getObjMultiFiles(obj3, vision3, filenames3);
  
  return 0;
  */
  
  

  hand.SetForce(HAND_FORCE);
  hand.SetSpeed(HAND_SPEED);

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

  ROS_INFO("Regrasp FlipInHand started...");
  ROS_INFO("Initial Joint Move");
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);

  // All cartesian moves are with respect to the proximal left corner of the table.
  // (where the default work object is located)
  // X-axis along the edge that goes left to right (facing the robot)
  // Y-axis along the edge that goes front to back
  // Z-axis pointing up.

  // Auxiliary variables
  // HomogTransf pose;
  double motorAngle;
  double fingerAngle[3];
  int Envalue[3];
  int motEn;
  int Envalue2[3];
  int motEn2;
  int SorF=0;	//Success=1 , Failure=0
  
  bool error;

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

  int obj1, obj2;
  std::string filenames1[NUM_OBJREC_VIEWS];
  std::string filenames2[NUM_OBJREC_VIEWS];

  HomogTransf placeHT;
  HomogTransf vision1; //before regrasp
  Vec transvision1;
  Quaternion quatvision1;

  HomogTransf vision2; //after regrasp
  Vec transvision2;
  Quaternion quatvision2;

  
  Quaternion quatPlace;
  Quaternion quatFailPlace;
  RotMat rotC, rotZ;
  // Vec tabletop = Vec("600 200 0",3); //center
  Vec z = Vec("0 0 1",3);
  //int dataPt= 5;
  float Erx=0.0, Ery=0.0, ErA=0.0, ErzPick=0.0;
  //int MaxErx, MaxEry, MaxErA, MaxErzPick;
  //MaxErx = 400; //range=400, actual = -200 to +200  (divided by 10 mm)
  //MaxEry= 400;  //range=400, actual = -200 to +200  (divided by 10 mm)
  //MaxErA= 400; //range=400, actual = -200 to +200   (divided by 10 degrees)
  //MaxErzPick=60; //range=60, actual = -30 to +30 (divided by 10 mm)
  //bool firsttime=true;
    
  Vec transPlace =  TRANS_PLACE;
   float ZeroErA=120.0;
   float TotErA=120.0;
   
   
  time_t rawtime;
  struct tm * timeinfo;
  char buffer [1024];
  char filename [1024];

  time ( &rawtime );
  timeinfo = localtime ( &rawtime );

  strftime (buffer,1024,"flip_%Y_%m_%d_%H_%M_%S.txt",timeinfo);
  sprintf(filename, "%s/%s", BASE_FOLDER, buffer);

  FILE *fp;
  fp=fopen(filename, "a");
  fprintf(fp,"CurrentTime,EnM,En1,En2,En3,EnM2,En12,En22,En32,Erx,Ery,ErzPick,ErA,vision1trans0,vision1trans1,vision1trans2,vision1quat0,vision1quat1,vision1quat2,vision1quat3,vision2trans0,vision2trans1,vision2trans2,vision2quat0,vision2quat1,vision2quat2,vision2quat3,SorF\n");
  fclose(fp);
  
  srand((unsigned)time(NULL)); 
  int regrasp_num=0;

  while (true)
  {
    regrasp_num+=1;
    ROS_INFO("Regrasp Flip in Hand %d",regrasp_num);
    
    //ErzPick = ((rand()%MaxErzPick)-(MaxErzPick/2))/10.0;
    ErzPick = ((double)rand() / RAND_MAX) * (MAX_ERZ - MIN_ERZ) + MIN_ERZ;

    //Vec transPick("600 200 73",3);
    Vec transPick = TRANS_PICK;
    transPick[2]=transPick[2]+ErzPick;

      // " ' " USING SET JOINTS " ' "
      // Do the first command in joint coordinates
      
      robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
      hand.Calibrate();
      hand.WaitRest(1.0);

      //hand.SetSpeed(HAND_SPEED);

      robot.SetCartesian(transO[0], transO[1], transO[2], quatO[0], quatO[1], quatO[2], quatO[3]);

      //Open the hand
      hand.SetAngle(HAND_OPEN_ANGLE);
      hand.WaitRest(0.250);
      ROS_INFO("Hand Opened");
      robot.SetCartesian(transPick[0], transPick[1], transPick[2]+10, quatO[0], quatO[1], quatO[2], quatO[3]);
      //Close the hand

      hand.SetAngle(HAND_CLOSE_ANGLE_partial);
      hand.WaitRest(0.250);
      ROS_INFO("Hand Closed partial");
      //hand.SetAngle(HAND_CLOSE_ANGLE);

      robot.SetCartesian(transPick[0], transPick[1], transPick[2], quatO[0], quatO[1], quatO[2], quatO[3]);

      hand.SetAngle(HAND_CLOSE_ANGLE);
      hand.WaitRest(0.250);
      ROS_INFO("Hand Closed");



      //Show it to the Camera |O|* Homogeneous Tf = vision1

      objRec.SetGuess(initialGuess);
      if (!util.getObjMulti(obj1, vision1, filenames1))
      {
        if (obj1 < 0)
        {
          ROS_WARN("vision 1:Failed to detect initial pose of object. I THINK I MIGHT HAVE MIGHT HAVE DROPPED IT??? Exiting program just to be safe");
          robot.SetSpeed(TCPs, ORIs);
          robot.SetJoints(0,0,0,0,90,0);
          break;
        }
        else
        {
          ROS_WARN("vision 1:Failed to detect initial pose of object, but I know I'm holding an object. Continuing anyways...");
        }
      }

      transvision1 = vision1.getTranslation();
      quatvision1 = vision1.getRotation().getQuaternion();

      robot.SetSpeed(TCPs, ORIs);
      robot.SetJoints(0,0,0,0,90,0);
      hand.WaitRest(0.250);
      hand.GetEncoders(motEn, Envalue); //Get Encoder values
      float EnSum=Envalue[0]+Envalue[1]+Envalue[2];
      printf("Ensum/3=%f\n", EnSum/3.0);
      if ((EnSum)/3.0 >2250)
      {
        ROS_WARN("BASED ON ENCODER VALUES, IT SEEMS I MIGHT HAVE DROPPED THE OBJECT???Exiting program just to be safe");
        robot.SetSpeed(TCPs, ORIs);
        robot.SetJoints(0,0,0,0,90,0);
        break;
      }
      
      //robot.SetJoints(-1.37,31.99,16.84,0.1,-87,-90);
      robot.SetJoints(-1.37,31.99,16.84,0.1,-70,-90);
      usleep(1700000);

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
      printf("drop angle= %lf \n",minfingerAngle-20);
      int sleeptime= 2500000; //15710*1000/minfingerAngle;   // 91750 to 91800, perfornamnce is not constant
      printf("sleep time = %d \n", sleeptime);

      //1-Open hand just enough to drop the object
      //hand.SetAngle(minfingerAngle-25);
      hand.SetAngle(minfingerAngle-25);
      //hand.SetAngle(HAND_OPEN_ANGLE_drop);
      //hand.WaitRest(0.250);
      usleep(sleeptime);

      hand.SetAngle(HAND_CLOSE_ANGLE_AfterRegrasp);
      hand.WaitRest(0.250);
      ROS_INFO("Hand Closed");

      robot.SetJoints(0,0,0,0,90,0);

      //Show it to the Camera |O|* Homogeneous Tf = vision2
      error = false;
      objRec.SetGuess(successGuess);
      if (!util.getObjMulti(obj2, vision2, filenames2))
      {
        ROS_INFO("vision 2:Couldn't find object in expected pose. Checking if it's at a failure pose...");
        objRec.SetGuess(failGuess);
        if (!util.getObjMultiFiles(obj2, vision2, filenames2))
        {
          ROS_WARN("vision 2:Couldn't find object in expected failure pose. Saving position and exiting...");
          error = true;
        }
      }

      transvision2 = vision2.getTranslation();
      quatvision2 = vision2.getRotation().getQuaternion();

      robot.SetSpeed(TCPs, ORIs);
      robot.SetJoints(0,0,0,0,90,0);
      hand.WaitRest(0.250);
      hand.GetEncoders(motEn2, Envalue2); //Get Encoder values
	    float En2Sum=Envalue2[0]+Envalue2[1]+Envalue2[2];
      printf("En2sum/3=%f\n", En2Sum/3.0);
      if ((En2Sum)/3.0 >2250)
      {
        ROS_WARN("BASED ON ENCODER VALUES2, IT SEEMS I MIGHT HAVE DROPPED THE OBJECT??? Saving position and exiting...");
        error = true;
      }
	  
	  // Determine if the regrasp was successful
    double dist = quatvision2.dist(QUAT_FAIL);
    ROS_INFO("dist = %f", dist);
	  SorF = (dist>=0.5);
	  
	  // SAVE DATA TO FILE HERE
	  
    FILE *fp;
    fp=fopen(filename, "a");
    if (fp == NULL)
    {
      ROS_WARN("Couldn't Open File!!");
      break;       
    }

struct timeval  tv;
gettimeofday(&tv, NULL);

double time_in_mill = (tv.tv_sec) * 1000.0 + (tv.tv_usec) / 1000.0 ;

    // write to the file 

    fprintf(fp,"%lf,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d", time_in_mill,motEn,Envalue[0],Envalue[1],Envalue[2],motEn2,Envalue2[0],Envalue2[1],Envalue2[2],Erx,Ery,ErzPick,ErA,transvision1[0],transvision1[1],transvision1[2],quatvision1[0],quatvision1[1],quatvision1[2],quatvision1[3],transvision2[0],transvision2[1],transvision2[2],quatvision2[0],quatvision2[1],quatvision2[2],quatvision2[3],SorF);

    for (int j=0; j < NUM_OBJREC_VIEWS; j++)
    {
      fprintf(fp, ",%s", filenames1[j].c_str());
    }

    for (int j=0; j < NUM_OBJREC_VIEWS; j++)
    {
      fprintf(fp, ",%s", filenames2[j].c_str());
    }

    fprintf(fp, "\n");

    // close the file 
    fclose(fp);

    // If we had an error from vision2, quit now
    if (error)
    {
      break;
    }
	  
	  
	  
	  
	  

    //Erx = ((rand()%MaxErx)-(MaxErx/2))/10.0;
    //Ery = ((rand()%MaxEry)-(MaxEry/2))/10.0;
    //ErA = ((rand()%MaxErA)-(MaxErA/2))/10.0;
    
    Erx = ((double)rand() / RAND_MAX) * (MAX_ERX - MIN_ERX) + MIN_ERX;
    Ery = ((double)rand() / RAND_MAX) * (MAX_ERY - MIN_ERY) + MIN_ERY;
    ErA = ((double)rand() / RAND_MAX) * (MAX_ERA - MIN_ERA) + MIN_ERA;


    //transPlace =  Vec("600 200 75",3);
    transPlace = TRANS_PLACE;
    transPlace[0]=transPlace[0]+Erx;
    transPlace[1]=transPlace[1]+Ery;
    transPlace[2]=transPlace[2];

    quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand facing down
    rotC = quatC.getRotMat();
    ZeroErA=120.0;
    TotErA=(ZeroErA+ErA)*PI/180;

    rotZ.setAxisAngle(z, TotErA); 

    placeHT = HomogTransf(rotC*rotZ,transPlace);
    quatPlace = placeHT.getRotation().getQuaternion();
    rotZ.setAxisAngle(z, TotErA - 120.0 / 180.0 * PI);
    placeHT = HomogTransf(rotC*rotZ,transPlace);
    quatFailPlace = placeHT.getRotation().getQuaternion();


    printf("Erx=%f\t Ery=%f\t ErA=%f\t ErzPick=%f\n",Erx,Ery,ErA,ErzPick);
    printf("transPick x y z = %lf\t %lf\t %lf\n", transPick[0], transPick[1], transPick[2]);
    printf("transPlace0=%lf\t transPlace1=%lf\t transPlace2=%lf\t quatPlace0=%lf\t quatPlace1=%lf\t quatPlace2=%lf\t quatPlace3=%lf\n",transPlace[0], transPlace[1], transPlace[2], quatPlace[0], quatPlace[1], quatPlace[2], quatPlace[3]);
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  //START StandtoLie
	  if (SorF==1)
	  {
	    regrasp.StandtoLie(-0.5,0,-30,105);
      robot.SetCartesian(transPlace[0], transPlace[1], transPlace[2], quatPlace[0], quatPlace[1], quatPlace[2], quatPlace[3]);
	  }
	  else
	  {
	    robot.SetCartesian(transPlace[0], transPlace[1], transPlace[2], quatFailPlace[0], quatFailPlace[1], quatFailPlace[2], quatFailPlace[3]);
	  }

      //robot.SetCartesian(transO[0], transO[1], transO[2], quatO[0], quatO[1], quatO[2], quatO[3]);
      //Open the hand
      hand.SetAngle(HAND_OPEN_ANGLE);
      hand.WaitRest(0.250);
      ROS_INFO("Hand Opened");
      ROS_INFO("Regrasp Flip in Hand Completed!");
    }
    
  /*
     robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
     robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
     robot.SetCartesian(transO[0], transO[1], transO[2], quatO[0], quatO[1], quatO[2], quatO[3]);
     hand.SetAngle(HAND_OPEN_ANGLE);
     hand.WaitRest(0.250);
     ROS_INFO("object placed");
     usleep(20000000);
     robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]);
   */  

  return 0;
}
