#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <regrasp_comm/regrasp_comm.h>
#include <matVec/matVec.h>
#include <util_comm/util_comm.h>
#include <objRec_comm/objRec_comm.h>

// Speed of the robot
#define TCP 45
#define ORI 15

#define TCPs 100.0   // Slow speed mm / s
#define ORIs 35.0   // degrees / s

#define TCPf 350.0   // FAST SPEED mm / s
#define ORIf 50.0   // degrees / s

// Speed and Force of the hand
#define HAND_SPD 0.8
#define HAND_FORCE 0.8

// Opening and closing finger
#define HAND_OPEN 20.0
#define HAND_RELEASE 74.0
#define HAND_CLOSE 90.0

// Zone of the robot
#define ZONE 0 //0 means every commanded position is a fine position. (No fly-by)


#define MAX_ERX 10
#define MIN_ERX -10
#define MAX_ERY 5
#define MIN_ERY -5
#define MAX_ERA 0.22
#define MIN_ERA -0.22


const double angle = 90.0*PI/180.0; // The rotation angle //Nowhere used in the code

ros::NodeHandle *nodePtr;
RobotComm robot;
HandComm hand;
RegraspComm regrasp;
UtilComm util;

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

Vec transInit = Vec("600 200 75",3);  //Location of Hand for picking the object
Quaternion quatInit = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand grasping the object from top



static const HomogTransf initialGuess(Quaternion("0.4305 0.0923 0.7011 -0.5601").getRotMat(), Vec("0.500863 0.3054066 0.4562078",3));
static const HomogTransf successGuess(Quaternion("-0.0009 0.9916 -0.1294 -0.0055").getRotMat(), Vec("0.4812291  0.3341413  0.420",3));

static const char BASE_FOLDER[] = "/home/simplehands/Desktop/regrasp_logs";

// Functions:
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

void initialize()
{  
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  ROS_INFO("Initialized");
}
void testhand()
{
  hand.SetAngle(HAND_OPEN);
  hand.WaitRest(1.25);
  hand.SetAngle(HAND_CLOSE);
  hand.WaitRest(1.25);
  ROS_INFO("Test finished");
}


int main(int argc, char** argv)
{
  double randx, randy, randangle;
  int i=0;
  ros::init(argc, argv, "Regrasp_Newhand");
  ros::NodeHandle node;
  nodePtr = &node;

  robot.subscribe(nodePtr);
  hand.subscribe(nodePtr);
  regrasp.subscribe(nodePtr);
  UtilComm util(&node);
  ObjRecComm objRec(&node);
/*

  // Quick vision test code
  int obj3;
  HomogTransf vision3;
  std::string filenames3[NUM_OBJREC_VIEWS];
  objRec.SetGuess(initialGuess);
  util.getObjMulti(obj3, vision3, filenames3);
  filenames3[0] = "/home/simplehands/Desktop/objRec/pointsFolder/points__2013_03_13__01_43_28.pcd";
  filenames3[1] = "/home/simplehands/Desktop/objRec/pointsFolder/points__2013_03_13__01_43_32.pcd";
  filenames3[2] = "/home/simplehands/Desktop/objRec/pointsFolder/points__2013_03_13__01_43_35.pcd";
  filenames3[3] = "/home/simplehands/Desktop/objRec/pointsFolder/points__2013_03_13__01_43_39.pcd";
  objRec.SetGuess(initialGuess);
  util.getObjMultiFiles(obj3, vision3, filenames3);

  cout << vision3.getRotation().getQuaternion() << endl;
  cout << vision3.getTranslation() << endl;
  
  return 0;

  */



  // Make sure robot communication is blocking
  if (!robot.SetComm(BLOCKING))
    return -1;

  // Set default speed limits
  if (!robot.SetSpeed(TCPs, ORIs))
    return -1;

  // Set the default "zone" of our robot (amount of interpolation we allow)
  if (!robot.SetZone(ZONE))
    return -1;

  ROS_INFO("Beginning test...");
  ROS_INFO("Regrasp PlaceandPick started...");
  ROS_INFO("Initial Joint Move");
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  hand.Calibrate();

  // All cartesian moves are with respect to the proximal left corner of the table.
  // (where the default work object is located)
  // X-axis along the edge that goes left to right (facing the robot)
  // Y-axis along the edge that goes front to back
  // Z-axis pointing up.


  // HomogTransf objPose;
  // int objNum;
  int Envalue[3];
  int motEn;
  int Envalue2[3];
  int motEn2;
  int SorF=0;    //Success=1 , Failure=0
  bool error;
  int obj1, obj2;
  std::string filenames1[NUM_OBJREC_VIEWS];
  std::string filenames2[NUM_OBJREC_VIEWS];

  HomogTransf placeHT;
  HomogTransf vision1; //before regrasp Stand to Lie
  Vec transvision1;
  Quaternion quatvision1;

  HomogTransf vision2; //after regrasp Stand to  Lie
  Vec transvision2;
  Quaternion quatvision2;

  time_t rawtime;
  struct tm * timeinfo;
  char buffer [1024];
  char filename [1024];

  time ( &rawtime );
  timeinfo = localtime ( &rawtime );

  strftime (buffer,1024,"place_%Y_%m_%d_%H_%M_%S.txt",timeinfo);
  sprintf(filename, "%s/%s", BASE_FOLDER, buffer);

  FILE *fp;
  fp=fopen(filename, "a");
  fprintf(fp,"CurrentTime,EnM,En1,En2,En3,EnM2,En12,En22,En32,Erx,Ery,ErAngle,vision1trans0,vision1trans1,vision1trans2,vision1quat0,vision1quat1,vision1quat2,vision1quat3,vision2trans0,vision2trans1,vision2trans2,vision2quat0,vision2quat1,vision2quat2,vision2quat3,SorF\n"); 
  fclose(fp);
  srand (time(NULL)); // Set a random seed

  //Pick the object (only done once)
  robot.SetCartesian(transC[0], transC[1], transC[2], quatInit[0], quatInit[1], quatInit[2], quatInit[3]);
  robot.SetCartesian(transInit[0], transInit[1], transInit[2], quatInit[0], quatInit[1], quatInit[2], quatInit[3]);
  hand.SetAngle(HAND_CLOSE);
  hand.WaitRest(1.25);
  ROS_INFO("Object Picked for 1st run");
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);
  
  //START REGRASP LOOP
  while(true)
  {
    hand.SetSpeed(HAND_SPD);
    hand.SetForce(HAND_FORCE);

    i+=1;
    ROS_INFO("Regrasp action %d",i);

    randx = ((double)rand() / RAND_MAX) * (MAX_ERX - MIN_ERX) + MIN_ERX;
    randy = ((double)rand() / RAND_MAX) * (MAX_ERY - MIN_ERY) + MIN_ERY;
    randangle = ((double)rand() / RAND_MAX) * (MAX_ERA - MIN_ERA) + MIN_ERA;

    randx = 0;
    randy = 0;
    randangle = 0;

    /*
    randx = rand() % 200 - 100;
    randy = rand() % 200 - 100;  
    randangle = rand() % 200 - 100;
     */

    ROS_INFO("X: %lf mm; Y: %lf mm; Angle: %lf degree",randx, randy, randangle*90.0);
    regrasp.LietoStand(randx, randy, 80, randangle);  // The random x and y is from -20mm to 20mm, the random rotation angle is from -45 to 45 degree.

    // util.GetObjFromViews(objNum, objPose);
    //Show it to the Camera |O|* Homogeneous Tf = vision1

    /*

    Vec zt(3);
    zt[0] = -randx / 1000.0;
    zt[1] = 0;
    zt[2] = -randy / 1000.0;

    RotMat rz;
    rz.rotZ(PI/2*randangle);

    HomogTransf xyErr;
    xyErr.setTranslation(zt);

    HomogTransf toObj(Quaternion("0.6533 0.6533 -0.2706 0.2706").getRotMat(), Vec("-0.003450895 0.004250628 0.003792212",3));
    HomogTransf thErr(rz, Vec("0 0 0",3));

    objRec.SetGuess(initialGuess*xyErr*toObj*thErr);

    */
    Vec z1(3);
    z1[0] = -sqrt(2)/2*randy - sqrt(2)/2*randx;
    z1[1] = sqrt(2)/2*randy - sqrt(2)/2*randx;
    z1[2] = 0;

    z1 /= 1000.0;

    HomogTransf T1;
    T1.setTranslation(z1);

    HomogTransf T2;
    T2.setTranslation(Vec("0.029 0.029 0",3));

    RotMat rz;
    rz.rotZ(PI/2*randangle);

    HomogTransf R;
    R.setRotation(rz);

    objRec.SetGuess(initialGuess*T1*T2*R*T2.inv());
    //objRec.SetGuess(initialGuess);
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
        ROS_WARN("BASED ON ENCODER VALUES, IT SEEMS I MIGHT HAVE DROPPED THE OBJECT??? Exiting program just to be safe");
        robot.SetSpeed(TCPs, ORIs);
        robot.SetJoints(0,0,0,0,90,0);
        break;
      }

    // Save the date
    regrasp.StandtoLie(0, 0, 96, 0);    // From -0.5 to 0.5 Nikhil: Changed z to 96 from 90 as hand might get damaged if object is vetrical in palm 
    // util.GetObjFromViews(objNum, objPose);
    //Show it to the Camera |O|* Homogeneous Tf = vision2
    error = false;
    
    objRec.SetGuess(successGuess);
    if (!util.getObjMulti(obj2, vision2, filenames2))
    {
      if (obj2 < 0)
      {
        ROS_WARN("vision 2:Failed to detect pose of object. I THINK I MIGHT HAVE MIGHT HAVE DROPPED IT??? Saving position and exiting...");
        error = true;
      }
      else
      {
        ROS_WARN("vision 2:Failed to detect pose of object, but I know I'm holding an object. Continuing anyways...");
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
    SorF = (error);

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

    fprintf(fp,"%lf,%d,%d,%d,%d,%d,%d,%d,%d,%lf,%lf,%lf,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d", time_in_mill,motEn,Envalue[0],Envalue[1],Envalue[2],motEn2,Envalue2[0],Envalue2[1],Envalue2[2],randx,randy,randangle,transvision1[0],transvision1[1],transvision1[2],quatvision1[0],quatvision1[1],quatvision1[2],quatvision1[3],transvision2[0],transvision2[1],transvision2[2],quatvision2[0],quatvision2[1],quatvision2[2],quatvision2[3],SorF);

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
    // Save the date

    //regrasp.Rotate(0);
  }

  ROS_INFO("Demo Completed!");

  return 0;
}
