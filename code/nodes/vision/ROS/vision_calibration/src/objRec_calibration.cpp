//
// Name: Robbie Paolini
//
// File Name: objRec_calibration.cpp
//
// Last Modifed: 1/28/2013
//
///////////////////////////////////////////////
// Object Recognition Calibration Suite
//
// Step 0: Grasp an object
// 
// Calibration 1: Move object around with robot, and record both robot and object pose. Use matlab to compute transform from robot to object and transform from robot to kinect. 
//
// Evaluation: Move to many different random positions and angles. Record everything. See what our error looks like when reconstructing the object pose
//
//
//

#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <matVec/matVec.h>
#include <robot_comm/robot_comm.h>
#include <objRec_comm/objRec_comm.h>
#include <handRec_comm/handRec_comm.h>
#include <hand_comm/hand_comm.h>
#include <matlab_comm/matlab_comm.h>
#include <util_comm/util_comm.h>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <unistd.h>
#include <sys/stat.h>

// If this is defined, we will not move the fingers throughout the program
//#define SKIP_GRASPING


#define CAL_HOME_J1 -6.34
#define CAL_HOME_J2 2.23
#define CAL_HOME_J3 11.74
#define CAL_HOME_J4 0.0
#define CAL_HOME_J5 76.03
#define CAL_HOME_J6 -6.34

#define INIT_J1 7.12
#define INIT_J2 41.39
#define INIT_J3 15.62
#define INIT_J4 -62.50
#define INIT_J5 63.35
#define INIT_J6 52.82

#define INIT_X 540.0
#define INIT_Y 300.0
#define INIT_Z 200.0
#define INIT_Q0 0.3162
#define INIT_QX -0.6325
#define INIT_QY -0.6325
#define INIT_QZ 0.3162

#define CAL_TCP 80// 40 //80
#define CAL_ORI 30// 15 //30

#define TRANS_DX 200.0
#define TRANS_DY 200.0




#define GRASP_SPEED 0.8
#define GRASP_FORCE 0.9

#define SENSE_ANG 10.0
#define CLOSE_ANG 100.0


static const HomogTransf origObjRecPoses[NUM_OBJREC_VIEWS] =
{
  HomogTransf(Quaternion("0.0 -0.2588 0.9659 0.0").getRotMat(), Vec("500.0 300.0 460.0", 3)),
  HomogTransf(Quaternion("0.0 0.7071 0.7071 0.0").getRotMat(), Vec("500.0 300.0 460.0", 3)),
  HomogTransf(Quaternion("0.0 0.9659 -0.2588 0.0").getRotMat(), Vec("500.0 300.0 460.0", 3)),
  HomogTransf(Quaternion("0.5 -0.5 -0.5 0.5").getRotMat(), Vec("530.0 300.0 460.0", 3))
};

static const HomogTransf boxOffset(
    Quaternion("1.0 0.0 0.0 0.0").getRotMat(),Vec("0.0 0.0 85.0",3));
//static const double widths[3] = {0.10, 0.15, 0.04};
static const double widths[3] = {0.10, 0.15, 0.04};



static const std::string cameraName("sideKinect");
static const Quaternion camera_quat("-0.4955 0.5103 -0.4993 0.4947");
static const Vec camera_vec("-325.8 303.0 428.3",3);
static const Vec camera_view("-325.8 303.0 428.3",3);
static const HomogTransf PATTERN_START_POSE = HomogTransf(Quaternion("0.7071 0 -0.7071 0").getRotMat(), Vec("550, 300, 430", 3));
// x, y, z
static const double TRANS_LIM[3][2] = 
{
  {-200.0, 100.0},
  {-100.0, 100.0},
  {-100.0, 100.0}
};
static const double Z_ROT_LIM[2] = {-PI/4-PI/2, PI/4-PI/2};



/*
static const std::string cameraName("upperRightKinect");
static const Quaternion camera_quat("0.2546 0.6532 -0.6577 -0.2758");
static const Vec camera_vec("1380.8 367.5 698.2",3);
static const Vec camera_view("1380.8 367.5 500",3);
static const HomogTransf PATTERN_START_POSE = HomogTransf(Quaternion("0 0.7071 0 0.7071").getRotMat(), Vec("550, 350, 200", 3));
// x, y, z
static const double TRANS_LIM[3][2] = 
{
  {-100.0, 100.0},
  {-100.0, 100.0},
  {-100.0, 0.0}
};
static const double Z_ROT_LIM[2] = {-PI/10+PI/2, PI/10+PI/2};
*/

/*
static const std::string cameraName("upperLeftKinect");
static const Quaternion camera_quat("0.2572 0.6567 0.6613 0.2555");
static const Vec camera_vec("-158.9 395.7 701.9",3);
static const Vec camera_view("-158.9 395.7 500",3);
static const HomogTransf PATTERN_START_POSE = HomogTransf(Quaternion("0.7071 0 -0.7071 0").getRotMat(), Vec("450, 350, 200", 3));
// x, y, z
static const double TRANS_LIM[3][2] = 
{
  {-100.0, 100.0},
  {-100.0, 100.0},
  {-100.0, 0.0}
};
static const double Z_ROT_LIM[2] = {-PI/10-PI/2, PI/10-PI/2};
*/


/*
static const std::string cameraName("upperCenterKinect");
static const Quaternion camera_quat("-0.0138 0.0107 0.9651 0.2613");
static const Vec camera_vec("601.7 -179.4 901.2",3);
static const Vec camera_view("601.7 -179.4 500",3);
static const HomogTransf PATTERN_START_POSE = HomogTransf(Quaternion("0.5 0.5 -0.5 0.5").getRotMat(), Vec("600, 350, 200", 3));
// x, y, z
static const double TRANS_LIM[3][2] = 
{
  {-250.0, 250.0},
  {-100.0, 100.0},
  {-100.0, 0.0}
};
static const double Z_ROT_LIM[2] = {-PI/3, PI/3};
*/

/*
static const std::string cameraName("lowerCenterKinect");
static const Quaternion camera_quat("0.8660 -0.500 0 0");
static const Vec camera_vec("600 -280 -120",3);
static const Vec camera_view("600 -280 -50",3);
static const HomogTransf PATTERN_START_POSE = HomogTransf(Quaternion("0.5 0.5 0.5 -0.5").getRotMat(), Vec("600, 300, 200", 3));
// x, y, z
static const double TRANS_LIM[3][2] = 
{
  {-200.0, 200.0},
  {-100.0, 100.0},
  {-100.0, -100.0}
};
static const double Z_ROT_LIM[2] = {-PI/15, PI/15};
*/


//static const Vec camera_vec("-325.8 303.0 428.3",3);
//static const Vec camera_view("-325.8 303.0 428.3",3);

//static const HomogTransf PATTERN_START_POSE = HomogTransf(Quaternion("0.5 -0.5 -0.5 0.5").getRotMat(), Vec("530.0 300.0 430.0", 3));

/*
// x, y, z
static const double TRANS_LIM[3][2] = 
{
  {-100.0, 100.0},
  {-100.0, 100.0},
  {-100.0, 100.0}
};
*/
// x, y, z
static const double ANG_LIM[3][2] = 
{
  {-PI/15,PI/15},//{-PI/3, PI/3},
  {-PI/15,PI/15},//{-PI/3, PI/6},
  {-PI/15,PI/15},//{-PI, PI}
};

/*
// x, y, z
static const double TRANS_LIM[3][2] = 
{
  {-100.0, 150.0},
  {-50.0, 100.0},
  {-200.0, 200.0}
};

// x, y, z
static const double ANG_LIM[3][2] = 
{
  {-PI/15,PI/15},//{-PI/3, PI/3},
  {-PI/15,PI/15},//{-PI/3, PI/6},
  {-PI/15,PI/15},//{-PI, PI}
};
*/

// x, y, z
static const double TRANS_CHG[3][2] = 
{
  {-100.0, 100.0},
  {-100.0, 100.0},
  {-100.0, 100.0}
};


static const double ANG_CHG[3][2] = 
{
  {-PI/15,PI/15},
  {-PI/15,PI/15},
  {-PI/15,PI/15},
};



// x, y, z
static const double EVAL_TRANS[3][2] = 
{
  {-20.0, 20.0},
  {-20.0, 20.0},
  {-20.0, 20.0}
};


static const double EVAL_ANG[3][2] = 
{
  {-PI/20,PI/20},
  {-PI/20,PI/20},
  {-PI/20,PI/20},
};


#define NUM_TRANSLATIONS 100
#define NUM_ROTATIONS 4

#define NUM_EVALS 200

/*
static const HomogTransf START_POSE[NUM_ROTATIONS] =
{
  HomogTransf(Quaternion("0.3162 -0.6325 -0.6325 0.3162").getRotMat(), Vec("500 300 430",3)),
};
*/

static const HomogTransf START_POSE[NUM_ROTATIONS] =
{
  HomogTransf(Quaternion("0.0 -0.2588 0.9659 0.0").getRotMat(), Vec("500.0 300.0 460.0", 3)),
  HomogTransf(Quaternion("0.0 0.7071 0.7071 0.0").getRotMat(), Vec("500.0 300.0 460.0", 3)),
  HomogTransf(Quaternion("0.0 0.9659 -0.2588 0.0").getRotMat(), Vec("500.0 300.0 460.0", 3)),
  HomogTransf(Quaternion("0.5 -0.5 -0.5 0.5").getRotMat(), Vec("530.0 300.0 460.0", 3))
};



static const HomogTransf objGuess(Quaternion("0.0029 -0.9261 0.3773 0.0048").getRotMat(), Vec("0.4062 38.2020 31.7654", 3));

static const char BASE_FOLDER[] = "/home/simplehands/Desktop/objRec/calibration";

using namespace std;


ofstream results;

void writeResult(HomogTransf robotPose, HomogTransf objPose)
{
  Vec r_t = robotPose.getTranslation();
  Vec o_t = objPose.getTranslation();

  Quaternion r_q = robotPose.getRotation().getQuaternion();
  Quaternion o_q = objPose.getRotation().getQuaternion();

  results << r_t[0] << ", " << r_t[1] << ", " << r_t[2] << ", " << r_q[0] << ", " << r_q[1] << ", " << r_q[2] << ", " << r_q[3];
  results << ", ";
  results << o_t[0] << ", " << o_t[1] << ", " << o_t[2] << ", " << o_q[0] << ", " << o_q[1] << ", " << o_q[2] << ", " << o_q[3];
  results << endl;
}


int main(int argc, char** argv)
{
  // Declare variables
  ros::init(argc, argv, "objRec_calibration");
  ros::NodeHandle node;

  srand(time(NULL));

  HandComm hand(&node);
  RobotComm robot(&node);  
  MatlabComm matlab(&node);
  UtilComm util(&node);
  ObjRecComm objRec(&node);
  HandRecComm handRec(&node);

  ///std::string picFileName;
  //double aux;
  //int nMarkers;
  //double confidence;
  //double x, y, theta, alpha, dist;


  int test;
  cout << "Object Recognition Calibration Suite" << endl;
  cout << "------------------------" << endl;
  cout << "1: Rough Calibration" << endl;
  cout << "2: Parameter Evaluation" << endl;
  cout << "3: Testing" << endl;
  cout << "4: Evaluation" << endl;
  cout << "5: Fine Calibration" << endl;
  cout << "6: Pattern Calibration" << endl;
  cout << "------------------------" << endl;
  cout << "What would you like to do? ";
  cin >> test;


  cout << endl << "------------------------" << endl;
  cout << "Moving to home position and calibrating hand..." << endl;


  // Set up hand and robot
  robot.SetSpeed(CAL_TCP, CAL_ORI);
  robot.SetZone(ZONE_FINE);
  robot.SetJoints(CAL_HOME_J1, CAL_HOME_J2, CAL_HOME_J3, 
      CAL_HOME_J4, CAL_HOME_J5, CAL_HOME_J6);

#ifndef SKIP_GRASPING
  
  double mot_start, fing_start[NUM_FINGERS];
  double mot_ang, fing_angs[NUM_FINGERS];

  double chg;

  if (test != 6)
  {
    hand.Calibrate();


    hand.SetSpeed(GRASP_SPEED);
    hand.SetForce(GRASP_FORCE);
    hand.SetAngle(SENSE_ANG);
    hand.WaitRest(0.5);

    hand.GetAngles(mot_start, fing_start);
    cout << "Please give an object to the robot." << endl;

    do
    {
      ros::Duration(0.1).sleep();
      hand.GetAngles(mot_ang, fing_angs);
      chg = 0.0;
      for (int i=0; i < NUM_FINGERS; i++)
      {
        chg += (fing_start[i]-fing_angs[i]) * (fing_start[i]-fing_angs[i]);
      }
      chg = sqrt(chg);
    }while (chg < 2.0);

    hand.SetAngle(CLOSE_ANG);
    hand.WaitRest(0.25);
    ros::Duration(0.5).sleep();
  }

#endif

  /*
  robot.SetJoints(INIT_J1, INIT_J2, INIT_J3,
      INIT_J4, INIT_J5, INIT_J6);
  robot.SetCartesian(INIT_X, INIT_Y, INIT_Z, 
      INIT_Q0, INIT_QX, INIT_QY, INIT_QZ);
*/

  switch (test)
  {
    // Evaluate accuracy of vision system by purturbing the hand slightly 
    //  from its original vision position and see how the estimated pose 
    //  changes.
    case 4:
      {
        mkdir(BASE_FOLDER, 0755);

        time_t rawtime;
        struct tm * timeinfo;
        char buffer [1024];
        char filename [1024];
        char filename2 [1024];
        std::string pcdfile;
        std::vector<std::string> filenames;

        time ( &rawtime );
        timeinfo = localtime ( &rawtime );

        strftime (buffer,1024,"eval_%Y_%m_%d_%H_%M_%S.txt",timeinfo);
        sprintf(filename, "%s/%s", BASE_FOLDER, buffer);
        ROS_INFO("%s", filename);

        strftime (buffer,1024,"eval_res_%Y_%m_%d_%H_%M_%S.txt",timeinfo);
        sprintf(filename2, "%s/%s", BASE_FOLDER, buffer);
        ROS_INFO("%s", filename2);


        // First, let's get the default recognition poses that handRec_node
        // uses
        XmlRpc::XmlRpcValue my_list;
        XmlRpc::XmlRpcValue inner_list;
        double pose[7];
        std::vector<HomogTransf> origHandRecPoses;

        node.getParam("/handRec/defaultRecPoses", my_list);
        ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        origHandRecPoses.resize(my_list.size());
        for (int32_t i = 0; i < my_list.size(); ++i) 
        {
          inner_list = my_list[i];
          ROS_ASSERT(inner_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
          for (int32_t j = 0; j < inner_list.size(); ++j)
          {
            ROS_ASSERT(inner_list[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            pose[j] = static_cast<double>(inner_list[j]);
          }
          origHandRecPoses[i] = HomogTransf(pose);
        }

        int numHandRecViews = origHandRecPoses.size();

        filenames.resize(numHandRecViews);

        // We will look for a big triangle, and use this for evaluation
        handRec.SetObject(RecObj::BIG_TRIANGLE);
        handRec.ClearGuess();

        HomogTransf iniPose =  origHandRecPoses[0];
        HomogTransf objPose;
        int obj;

        robot.SetCartesianJ(iniPose);

        handRec.GetPoseSpec(obj, objPose, origHandRecPoses, filenames);

        // Remember the orientation of the object, so we can make sure our
        // other objects are also returned with that orientation
        handRec.SetPrefOrient(objPose.getQuaternion());

        results.open(filename, ios::out | ios::app);
        writeResult(iniPose, objPose);
        results.close();

        results.open(filename2, ios::out | ios::app);

        for (int i=0; i < numHandRecViews; i++)
        {
          Vec o_t = origHandRecPoses[i].getTranslation();
          Quaternion o_q = origHandRecPoses[i].getQuaternion();

          results << o_t[0] << ", " << o_t[1] << ", " << o_t[2] << ", " << o_q[0] << ", " << o_q[1] << ", " << o_q[2] << ", " << o_q[3] << ", ";
        }

        for (int i=0; i < numHandRecViews; i++)
        {
          if (i == numHandRecViews - 1)
            results << filenames[i] << endl;
          else
            results << filenames[i] << ", ";
        }
        results.close();

        cout << "Moving the robot to random positions and recording vision output" << endl;

        // Move the robot to different positions, save where vision thinks the marker is, and where it actually should be
        for (int i=0; i < NUM_EVALS; i++)
        {
          double dt[3];
          double da[3];
          double mag = 0;
          // Randomly generate a pertubation
          for (int j=0; j < 3; j++)
          {
            dt[j] = ((double)rand() / RAND_MAX) * (EVAL_TRANS[j][1] - EVAL_TRANS[j][0]) + EVAL_TRANS[j][0];
            da[j] = ((double)rand() / RAND_MAX) * (EVAL_ANG[j][1] - EVAL_ANG[j][0]) + EVAL_ANG[j][0];
            mag += da[j]*da[j];
          }

          ROS_INFO("dt: (%2.2f, %2.2f, %2.2f), da: (%2.2f, %2.2f, %2.2f)", dt[0], dt[1], dt[2], da[0], da[1], da[2]);

          double ang = sqrt(mag);

          for (int j=0; j < 3; j++)
          {
            da[j] /= ang;
          }

          HomogTransf robotPose;
          RotMat r;
          r.setAxisAngle(Vec(da,3), ang);
          robotPose.setTranslation(Vec(dt,3) + iniPose.getTranslation());
          robotPose.setRotation(iniPose.getRotation() * r);

          HomogTransf diff = iniPose.inv() * robotPose;

          std::vector<HomogTransf> randPoses(numHandRecViews);
          for (int k=0; k < numHandRecViews; k++)
          {
            randPoses[k] = origObjRecPoses[k] * diff;
          }

          handRec.GetPoseSpec(obj, objPose, randPoses, filenames);
          cout << "OBJECT POSE: " << objPose << endl;

          results.open(filename, ios::out | ios::app);
          writeResult(robotPose, objPose);
          results.close();


          results.open(filename2, ios::out | ios::app);

          for (int j=0; j < numHandRecViews; j++)
          {
            Vec o_t = randPoses[j].getTranslation();
            Quaternion o_q = randPoses[j].getQuaternion();

            results << o_t[0] << ", " << o_t[1] << ", " << o_t[2] << ", " << o_q[0] << ", " << o_q[1] << ", " << o_q[2] << ", " << o_q[3] << ", ";
          }

          for (int j=0; j < numHandRecViews; j++)
          {
            if (j == numHandRecViews - 1)
              results << filenames[j] << endl;
            else
              results << filenames[j] << ", ";
          }
          results.close();

        }
        cout << endl << "Done." << endl;

        break;
      }

    case 5:
      {
        mkdir(BASE_FOLDER, 0755);

        time_t rawtime;
        struct tm * timeinfo;
        char buffer [1024];
        char filename [1024];
        char filename1 [1024];
        char filename2 [1024];
        std::string pcdfile;

        time ( &rawtime );
        timeinfo = localtime ( &rawtime );

        strftime (buffer,1024,"fine_cal_%Y_%m_%d_%H_%M_%S.txt",timeinfo);
        sprintf(filename, "%s/%s", BASE_FOLDER, buffer);
        sprintf(filename1, "%s/vals_%s", BASE_FOLDER, buffer);
        sprintf(filename2, "%s/files_%s", BASE_FOLDER, buffer);


        ROS_INFO("%s", filename);


        for (int k=0; k < NUM_ROTATIONS; k++)
        {
          HomogTransf iniPose = START_POSE[k];
          HomogTransf objPose;
          int obj;

          robot.SetCartesian(iniPose);
          ros::Duration(0.5).sleep();
          
          /*

          objRec.SavePoints(pcdfile);
          results.open (filename1, ios::out | ios::app);

          Vec r_t = iniPose.getTranslation();
          Quaternion r_q = iniPose.getRotation().getQuaternion();
          results << r_t[0] << ", " << r_t[1] << ", " << r_t[2] << ", " << r_q[0] << ", " << r_q[1] << ", " << r_q[2] << ", " << r_q[3] << endl;
          
          results.close();

          results.open (filename2, ios::out | ios::app);
          results << pcdfile << endl;
          results.close();

          */
          
          HomogTransf guess = iniPose * objGuess;
          Vec t = guess.getTranslation();
          t /= 1000.0;
          guess.setTranslation(t);
          objRec.SetGuess(guess);
          util.getObjectFromCamera(obj, objPose);

          results.open(filename, ios::out | ios::app);
          writeResult(iniPose, objPose);
          results.close();

          /*
          cout << "Do you accept? (y/n): ";
          char ans;
          do
          {
            cin >> ans;
          }while (ans != 'y' && ans != 'n');
          if (ans == 'y')
          {
            writeResult(iniPose, objPose);
            cout << "." << flush;
          }
          */

          cout << "Moving the robot to random positions and recording vision output" << endl;

          // Move the robot to different positions, save where vision thinks the marker is, and where it actually should be
          for (int i=0; i < NUM_TRANSLATIONS; i++)
          {
            double dt[3];
            double da[3];
            double mag = 0;
            // Randomly generate a pertubation
            for (int j=0; j < 3; j++)
            {
              dt[j] = ((double)rand() / RAND_MAX) * (TRANS_CHG[j][1] - TRANS_CHG[j][0]) + TRANS_CHG[j][0];
              da[j] = ((double)rand() / RAND_MAX) * (ANG_CHG[j][1] - ANG_CHG[j][0]) + ANG_CHG[j][0];
              mag += da[j]*da[j];
            }

            ROS_INFO("dt: (%2.2f, %2.2f, %2.2f), da: (%2.2f, %2.2f, %2.2f)", dt[0], dt[1], dt[2], da[0], da[1], da[2]);


            double ang = sqrt(mag);

            for (int j=0; j < 3; j++)
            {
              da[j] /= ang;
            }

            HomogTransf robotPose;
            RotMat r;
            r.setAxisAngle(Vec(da,3), ang);
            robotPose.setTranslation(Vec(dt,3) + iniPose.getTranslation());
            robotPose.setRotation(iniPose.getRotation() * r);

            // Move the robot there
            robot.SetCartesian(robotPose);

            // Take a picture and see where the vision system thinks the robot is
            ros::Duration(0.5).sleep();

            /*
            objRec.SavePoints(pcdfile);
            results.open (filename1, ios::out | ios::app);

            Vec r_t = robotPose.getTranslation();
            Quaternion r_q = robotPose.getRotation().getQuaternion();
            results << r_t[0] << ", " << r_t[1] << ", " << r_t[2] << ", " << r_q[0] << ", " << r_q[1] << ", " << r_q[2] << ", " << r_q[3] << endl;
          
            results.close();

            results.open (filename2, ios::out | ios::app);
            results << pcdfile << endl;
            results.close();
             */

            HomogTransf guess = robotPose * objGuess;
            Vec t = guess.getTranslation();
            t /= 1000.0;
            guess.setTranslation(t);
            objRec.SetGuess(guess);
            util.getObjectFromCamera(obj, objPose);

            results.open(filename, ios::out | ios::app);
            writeResult(robotPose, objPose);
            results.close();

            /*
            cout << "Do you accept? (y/n/q): ";
            do
            {
              cin >> ans;
            }while (ans != 'y' && ans != 'n' && ans != 'q');
            if (ans == 'q')
            {
              break;
            }
            else if (ans == 'y')
            {
              writeResult(robotPose, objPose);
              cout << "." << flush;
            }
            */
          }
        }

        /*
           cout << endl << "Computing optimal pixel to mm ratio." << endl;


        // Now compute the optimal constant to multiply all the vision x and y values by that will get it to most closely match our robot x and y values
        matlab.sendMat("robot_x", Mat(robot_x, NUM_TRANSLATION, 1));
        matlab.sendMat("robot_y", Mat(robot_y, NUM_TRANSLATION, 1));
        matlab.sendMat("vision_x", Mat(vision_x, NUM_TRANSLATION, 1));
        matlab.sendMat("vision_y          HomogTransf guess = iniPose * objGuess;
          Vec t = guess.getTranslation();
          t /= 1000.0;
          guess.setTranslation(t);
          objRec.SetGuess(guess);", Mat(vision_y, NUM_TRANSLATION, 1));

        matlab.sendCommand("cx = pinv([vision_x, ones(size(vision_x))])*robot_x;");
        matlab.sendCommand("cy = pinv([vision_y, ones(size(vision_y))])*robot_y;");
        matlab.sendCommand("cx = cx(1);");
        matlab.sendCommand("cy = cy(1);");

        double cx = matlab.getValue("cx");
        double cy = matlab.getValue("cy");

        cout << "---------------------------" << endl;
        cout << "Results: cx = " << cx << ", cy = " << cy << endl;
        cout << "---------------------------" << endl;


         */

        cout << endl << "Done." << endl;

        break;
      }





    case 3:
      {
        mkdir(BASE_FOLDER, 0755);

        time_t rawtime;
        struct tm * timeinfo;
        char buffer [1024];
        char filename [1024];

        time ( &rawtime );
        timeinfo = localtime ( &rawtime );

        strftime (buffer,1024,"param_opt_%Y_%m_%d_%H_%M_%S.txt",timeinfo);
        sprintf(filename, "%s/%s", BASE_FOLDER, buffer);


        while(true)
        {
          HomogTransf objPose;
          int obj;
          std::string filenames[NUM_OBJREC_VIEWS];

          /*
          HomogTransf robotPose(Quaternion("0.0 -0.2588 0.9659 0.0").getRotMat(), Vec("500.0 300.0 460.0", 3));


          HomogTransf guess = robotPose * objGuess;
          Vec t = guess.getTranslation();
          t /= 1000.0;
          guess.setTranslation(t);
          objRec.SetGuess(guess);
          */

          util.getObjMulti(obj, objPose, filenames);


          cout << "Do you accept? (y/n/q): ";
          char ans;
          do
          {
            cin >> ans;
          }while (ans != 'y' && ans != 'n' && ans!= 'q');
          if (ans == 'y')
          {
            results.open(filename, ios::out | ios::app);

            Vec o_t = objPose.getTranslation();

            Quaternion o_q = objPose.getRotation().getQuaternion();

            results << o_t[0] << ", " << o_t[1] << ", " << o_t[2] << ", " << o_q[0] << ", " << o_q[1] << ", " << o_q[2] << ", " << o_q[3];

            for (int i=0; i < NUM_OBJREC_VIEWS; i++)
            {
              results << ", " << filenames[i];
            }

            results << endl;

            results.close();
          }
          else if (ans == 'q')
          {
            break;
          }


            robot.SetJoints(CAL_HOME_J1, CAL_HOME_J2, CAL_HOME_J3, 
      CAL_HOME_J4, CAL_HOME_J5, CAL_HOME_J6);

#ifndef SKIP_GRASPING

            hand.SetSpeed(GRASP_SPEED);
            hand.SetForce(GRASP_FORCE);
            hand.SetAngle(SENSE_ANG);
            hand.WaitRest(0.5);

            hand.GetAngles(mot_start, fing_start);
            cout << "Please give an object to the robot." << endl;

            do
            {
              ros::Duration(0.1).sleep();
              hand.GetAngles(mot_ang, fing_angs);
              chg = 0.0;
              for (int i=0; i < NUM_FINGERS; i++)
              {
                chg += (fing_start[i]-fing_angs[i]) * (fing_start[i]-fing_angs[i]);
              }
              chg = sqrt(chg);
            }while (chg < 2.0);

            hand.SetAngle(CLOSE_ANG);
            hand.WaitRest(0.25);
            ros::Duration(0.5).sleep();
#endif
        }
        break;

        /*

           HomogTransf iniPose;
           HomogTransf objPose;
           int obj;

           robot.GetCartesian(iniPose);
           util.getObjectFromCamera(obj, objPose);

        // Move the robot to different positions, save where vision thinks the marker is, and where it actually should be
        for (int i=0; i < NUM_TRANSLATION; i++)
        {
        double dt[3];
        double da[3];
        double mag = 0;
        // Randomly generate a pertubation
        for (int j=0; j < 3; j++)
        {
        dt[j] = ((double)rand() / RAND_MAX) * (TRANS_LIM[j][1] - TRANS_LIM[j][0]) + TRANS_LIM[j][0];
        da[j] = ((double)rand() / RAND_MAX) * (ANG_LIM[j][1] - ANG_LIM[j][0]) + ANG_LIM[j][0];
        mag += da[j]*da[j];
        }

        ROS_INFO("dt: (%2.2f, %2.2f, %2.2f), da: (%2.2f, %2.2f, %2.2f)", dt[0], dt[1], dt[2], da[0], da[1], da[2]);

        double ang = sqrt(mag);

        for (int j=0; j < 3; j++)
        {
        da[j] /= ang;
        }

        HomogTransf robotPose;
        RotMat r;
        r.setAxisAngle(Vec(da,3), ang);
        robotPose.setTranslation(Vec(dt,3) + iniPose.getTranslation());
        robotPose.setRotation(iniPose.getRotation() * r);

        // Move the robot there
        robot.SetCartesian(robotPose);

        // Take a picture and see where the vision system thinks the robot is
        ros::Duration(0.5).sleep();
        util.getObjectFromCamera(obj, objPose);
        }

        cout << endl << "Done." << endl;

        break;

         */


      }

    case 6:
      {

        // For this calibration, we'll set the tool frame to be the
        // flange of the robot
        robot.SetTool(0,0,0,1,0,0,0);

        mkdir(BASE_FOLDER, 0755);

        time_t rawtime;
        struct tm * timeinfo;
        char buffer [1024];
        char filename [1024];
        std::string pcd_file;

        time ( &rawtime );
        timeinfo = localtime ( &rawtime );

        strftime (buffer,1024,"pattern_cal_%Y_%m_%d_%H_%M_%S.txt",timeinfo);
        sprintf(filename, "%s/%s", BASE_FOLDER, buffer);

        ROS_INFO("%s", filename);

        HomogTransf iniPose =  PATTERN_START_POSE;
        double last_ik[NUM_JOINTS];
        robot.SetCartesianJ(iniPose);
        robot.GetIK(iniPose, last_ik);
        ros::Duration(0.5).sleep();

        objRec.SetCamera(camera_vec/1000.0, camera_quat, cameraName);

        for (int i = 0; i < 2000; i++)
        {
          double dt[3];
          double da;
          for (int j=0; j < 3; j++)
          {
            dt[j] = ((double)rand() / RAND_MAX) * (TRANS_LIM[j][1] - TRANS_LIM[j][0]) + TRANS_LIM[j][0];
          }

          da = ((double)rand() / RAND_MAX) * (Z_ROT_LIM[1] - Z_ROT_LIM[0]) + Z_ROT_LIM[0];

          Vec t(dt, 3);

          t += iniPose.getTranslation();

          Vec z_axis = camera_view - t;

          // Make change a unit vector. This is the direction our tool will
          // point.
          z_axis.normalize();

          // upperRightKinect, upperLeftKinect
          // Initial pose of tool x-axis, pointing in the global +y
          // direction
          Vec x_axis("0 1 0", 3);

          // Compute the new y axis
          Vec new_y = z_axis ^ x_axis;
          new_y.normalize();

          // Finally, compute our x axis
          x_axis = new_y ^ z_axis;

          // Create our rotation matrix with these new axes
          RotMat r(x_axis, new_y, z_axis);
          RotMat rz;
          rz.rotZ(da);

          /*
          
          Vec y_axis("-1 0 0", 3);  // upperCenterKinect
          //Vec y_axis("1 0 0", 3);     // lowerCenterKinect

          Vec x_axis = y_axis ^ z_axis;
          x_axis.normalize();

          // Finally, compute our x axis
          y_axis = z_axis ^ x_axis;

          // Create our rotation matrix with these new axes
          RotMat r(x_axis, y_axis, z_axis);
          RotMat rz;
          rz.rotZ(da);
          
          */


          HomogTransf robotPose(r * rz, t);

          /*

          double dt[3];
          double da[3];
          double mag = 0;
          // Randomly generate a pertubation
          for (int j=0; j < 3; j++)
          {
            dt[j] = ((double)rand() / RAND_MAX) * (TRANS_LIM[j][1] - TRANS_LIM[j][0]) + TRANS_LIM[j][0];
            da[j] = ((double)rand() / RAND_MAX) * (ANG_LIM[j][1] - ANG_LIM[j][0]) + ANG_LIM[j][0];
            mag += da[j]*da[j];
          }

          ROS_INFO("dt: (%2.2f, %2.2f, %2.2f), da: (%2.2f, %2.2f, %2.2f)", dt[0], dt[1], dt[2], da[0], da[1], da[2]);

          double ang = sqrt(mag);

          for (int j=0; j < 3; j++)
          {
            da[j] /= ang;
          }

          HomogTransf robotPose;
          RotMat r;
          r.setAxisAngle(Vec(da,3), ang);
          robotPose.setTranslation(Vec(dt,3) + iniPose.getTranslation());
          robotPose.setRotation(iniPose.getRotation() * r);

          */


          // Make sure that this is a reachable position
          double ik_joints[NUM_JOINTS];
          if (!robot.GetIK(robotPose, ik_joints))
          {
            i--;
            ROS_INFO("skipping: dt = [%f, %f, %f], da = %f", dt[0], dt[1], dt[2], da*180/PI);
            continue;
          }
          
          double joint_mag = 0;
          for (int j=0; j < NUM_JOINTS; ++j)
          {
            printf("%f\t\t%f\n", last_ik[j], ik_joints[j]);
            joint_mag += (last_ik[j] - ik_joints[j])*(last_ik[j] - ik_joints[j]);
          }

          ROS_INFO("JM: %f", joint_mag);

          if (joint_mag >= 20000)
          {
            i--;
            ROS_INFO("SKIPPING: dt = [%f, %f, %f], da = %f", dt[0], dt[1], dt[2], da*180/PI);
            continue;

          }

          if (ik_joints[4] > 114.5)
          {
            i--;
            ROS_INFO("SCIPPING: dt = [%f, %f, %f], da = %f", dt[0], dt[1], dt[2], da*180/PI);
            continue;
          }

          memcpy(last_ik, ik_joints, NUM_JOINTS * sizeof(double));


          ROS_INFO("#%d dt = [%f, %f, %f], da = %f", i+1, dt[0], dt[1], dt[2], da*180/PI);

          // Move the robot there
          robot.SetCartesianJ(robotPose);

          // Take a picture and see where the vision system thinks the robot is
          ros::Duration(0.5).sleep();

          // Set up the bounds to filter this point
          HomogTransf boxBounds = robotPose * boxOffset;
          Vec trans = boxBounds.getTranslation();
          trans /= 1000.0;
          boxBounds.setTranslation(trans);

          objRec.SetBounds(widths, boxBounds);
          objRec.SavePoints(pcd_file, true, false, cameraName);

          results.open(filename, ios::out | ios::app);

          Vec r_t = robotPose.getTranslation() / 1000.0;  // Convert to mm
          Quaternion r_q = robotPose.getRotation().getQuaternion();

          results << r_t[0] << ", " << r_t[1] << ", " << r_t[2] << ", " << r_q[0] << ", " << r_q[1] << ", " << r_q[2] << ", " << r_q[3];
          results << ", " << pcd_file << endl;

          results.close();
        }

        robot.SetDefaults();

        break;
      }




    case 1:
    default:
      {
        objRec.ClearGuess();
        mkdir(BASE_FOLDER, 0755);

        time_t rawtime;
        struct tm * timeinfo;
        char buffer [1024];
        char filename [1024];

        time ( &rawtime );
        timeinfo = localtime ( &rawtime );

        strftime (buffer,1024,"results_%Y_%m_%d_%H_%M_%S.txt",timeinfo);
        sprintf(filename, "%s/%s", BASE_FOLDER, buffer);

        ROS_INFO("%s", filename);

        results.open (filename);

        ROS_INFO("is_open: %d", (int)results.is_open());

        HomogTransf iniPose;
        HomogTransf objPose;
        int obj;

        robot.GetCartesian(iniPose);
        util.getObjectFromCamera(obj, objPose);

        cout << "Do you accept? (y/n): ";
        char ans;
        do
        {
          cin >> ans;
        }while (ans != 'y' && ans != 'n');
        if (ans == 'y')
        {
          writeResult(iniPose, objPose);
          cout << "." << flush;
        }

        cout << "Moving the robot to random positions and recording vision output" << endl;

        // Move the robot to different positions, save where vision thinks the marker is, and where it actually should be
        while (true)
        {
          double dt[3];
          double da[3];
          double mag = 0;
          // Randomly generate a pertubation
          for (int j=0; j < 3; j++)
          {
            dt[j] = ((double)rand() / RAND_MAX) * (TRANS_LIM[j][1] - TRANS_LIM[j][0]) + TRANS_LIM[j][0];
            da[j] = ((double)rand() / RAND_MAX) * (ANG_LIM[j][1] - ANG_LIM[j][0]) + ANG_LIM[j][0];
            mag += da[j]*da[j];
          }

          ROS_INFO("dt: (%2.2f, %2.2f, %2.2f), da: (%2.2f, %2.2f, %2.2f)", dt[0], dt[1], dt[2], da[0], da[1], da[2]);


          double ang = sqrt(mag);

          for (int j=0; j < 3; j++)
          {
            da[j] /= ang;
          }

          /*
             HomogTransf robotPose;
             HomogTransf poseChg;

             RotMat r;
             r.setAxisAngle(Vec(da,3), ang);
             poseChg.setTranslation(Vec(dt,3));
             poseChg.setRotation(r);

             robotPose = iniPose * poseChg;

             HomogTransf endPose = robotPose * HomogTransf(Quaternion("1.0 0.0 0.0 0.0").getRotMat(), Vec("0.0 0.0 100.0",3));
             if (endPose.getTranslation()[2] < 0.0)
             {
             cout << "robot pose of: " << robotPose << " is too dangerous. Trying again." << endl;
             i--;
             continue;
             }

             cout << "initial pose: " << iniPose << endl;
             cout << "pose change: " << poseChg << endl;
             cout << "Moving to: " << robotPose << endl;

           */

          HomogTransf robotPose;
          RotMat r;
          r.setAxisAngle(Vec(da,3), ang);
          robotPose.setTranslation(Vec(dt,3) + iniPose.getTranslation());
          robotPose.setRotation(iniPose.getRotation() * r);

          // Move the robot there
          robot.SetCartesian(robotPose);

          // Take a picture and see where the vision system thinks the robot is
          ros::Duration(0.5).sleep();
          util.getObjectFromCamera(obj, objPose);

          cout << "Do you accept? (y/n/q): ";
          do
          {
            cin >> ans;
          }while (ans != 'y' && ans != 'n' && ans != 'q');
          if (ans == 'q')
          {
            break;
          }
          else if (ans == 'y')
          {
            writeResult(robotPose, objPose);
            cout << "." << flush;
          }
        }

        /*
           cout << endl << "Computing optimal pixel to mm ratio." << endl;


        // Now compute the optimal constant to multiply all the vision x and y values by that will get it to most closely match our robot x and y values
        matlab.sendMat("robot_x", Mat(robot_x, NUM_TRANSLATION, 1));
        matlab.sendMat("robot_y", Mat(robot_y, NUM_TRANSLATION, 1));
        matlab.sendMat("vision_x", Mat(vision_x, NUM_TRANSLATION, 1));
        matlab.sendMat("vision_y", Mat(vision_y, NUM_TRANSLATION, 1));

        matlab.sendCommand("cx = pinv([vision_x, ones(size(vision_x))])*robot_x;");
        matlab.sendCommand("cy = pinv([vision_y, ones(size(vision_y))])*robot_y;");
        matlab.sendCommand("cx = cx(1);");
        matlab.sendCommand("cy = cy(1);");

        double cx = matlab.getValue("cx");
        double cy = matlab.getValue("cy");

        cout << "---------------------------" << endl;
        cout << "Results: cx = " << cx << ", cy = " << cy << endl;
        cout << "---------------------------" << endl;


         */
        results.close();

        cout << endl << "Done." << endl;

        break;
      }

    case 2:
      {
        mkdir(BASE_FOLDER, 0755);

        time_t rawtime;
        struct tm * timeinfo;
        char buffer [1024];
        char filename [1024];

        time ( &rawtime );
        timeinfo = localtime ( &rawtime );

        strftime (buffer,1024,"params_%Y_%m_%d_%H_%M_%S.txt",timeinfo);
        sprintf(filename, "%s/%s", BASE_FOLDER, buffer);

        HomogTransf objPose;
        int obj;


        int num_params[5] = {7,7,1,6,1};
        double normal_radius[] = {0.0075, 0.01, 0.0125, 0.015, 0.0175, 0.02, 0.0225};
        double feature_radius[] = {0.0075, 0.01, 0.0125, 0.015, 0.0175, 0.02, 0.0225};
        int num_samples[] = {10};
        double min_sample_dist[] = {0.002, 0.004, 0.006, 0.008, 0.01, 0.012};
        int k_correspondences[] = {10};



        for (int i=0; i < num_params[0]; i++)
        {
          cout << "***** " << i << endl;
          for (int j=0; j < num_params[1]; j++)
          {
            cout << "** " << j << endl;
            for (int k=0; k < num_params[2]; k++)
            {
              for (int l=0; l < num_params[3]; l++)
              {
                cout << l << endl;
                for (int m = 0; m < num_params[4]; m++)
                {
                  objRec.SetParams(normal_radius[i], feature_radius[j], num_samples[k], min_sample_dist[l], k_correspondences[m]);
                  for (int n=0; n < 10; n++)
                  {
                    util.getObjectFromCamera(obj, objPose);

                    Vec o_t = objPose.getTranslation();
                    Quaternion o_q = objPose.getRotation().getQuaternion();

                    results.open (filename, ios::out | ios::app);
                    results << normal_radius[i] << ", " << feature_radius[j] << ", " << num_samples[k] << ", " << min_sample_dist[l] << ", " << k_correspondences[m] << ", " << o_t[0] << ", " << o_t[1] << ", " << o_t[2] << ", " << o_q[0] << ", " << o_q[1] << ", " << o_q[2] << ", " << o_q[3] << endl;
                    results.close();
                  }
                }
              }
            }
          }
        }


        cout << endl << "Done." << endl;

        break;
      }
      /*
      ////////////////////////////////////////////////////////
      // HAND CENTER CALIBRATION
      ////////////////////////////////////////////////////////
      case 2:
      {
      cout << "Rotating the robot to random positions and recording vision output" << endl;

      double vision_x[NUM_ROTATION];
      double vision_y[NUM_ROTATION];

      double dtheta;

      // Move the robot to different positions, save where vision thinks the marker is, and where it actually should be
      for (int i=0; i < NUM_ROTATION; i++)
      {
      // Randomly generate a rotation, but make sure we have an evenly 
      //  distributed amount of rotations through each quadrant
      if ( i < NUM_ROTATION/4)
      dtheta = ((double)rand() / RAND_MAX) * PI/2;
      else if (i < NUM_ROTATION/2)
      dtheta = ((double)rand() / RAND_MAX) * PI/2 + PI/2;
      else if (i < 3*NUM_ROTATION/4)
      dtheta = ((double)rand() / RAND_MAX) * PI/2 + PI;
      else
      dtheta = ((double)rand() / RAND_MAX) * PI/2 + 3*PI/2;

      Quaternion curQuat;
      curQuat[0] = CAMERA_Q0;
      curQuat[1] = CAMERA_QX;
      curQuat[2] = CAMERA_QY;
      curQuat[3] = CAMERA_QZ;

      RotMat dRot;
      dRot.rotX(dtheta);

      Quaternion newQuat = dRot.getQuaternion() ^ curQuat;

      // Rotate the hand
      robot.SetCartesian(
      CAMERA_X, 
      CAMERA_Y, 
      CAMERA_Z, 
      newQuat[0], newQuat[1], newQuat[2], newQuat[3]);

      // Take a picture and see where the vision system thinks the robot is
      ros::Duration(0.25).sleep();
      vision.CaptureImage(GRASP_DETECT, picFileName);
      vision.GetInfo(picFileName, nMarkers, confidence, aux, x, y, theta, alpha, dist);

      // Save the center of the marker each time
      vision_x[i] = x;
      vision_y[i] = y;

      cout << "." << flush;2,
      }

      cout << endl << "Computing optimal center of hand." << endl;


      // Now compute the optimal constant to multiply all the vision x and y values by that will get it to most closely match our robot x and y values
      matlab.sendMat("x", Mat(vision_x, NUM_ROTATION, 1));
      matlab.sendMat("y", Mat(vision_y, NUM_ROTATION, 1));

      matlab.sendCommand("c = pinv([x,y,ones(size(x))])*(-x.^2+y.^2);");
      matlab.sendCommand("cx = -0.5*c(1)");
      matlab.sendCommand("cy = -0.5*c(2)");

      double cx = matlab.getValue("cx");
      double cy = matlab.getValue("cy");

      cout << "---------------------------" << endl;
      cout << "Results: cx = " << cx << ", cy = " << cy << endl;
      cout << "---------------------------" << endl;

      break;
  }
    case 3:
  {
    cout << "Moving the robot to random positions and recording vision output" << endl;


    double init_x = x;
    double init_y = y;
    double init_th = alpha;


    double robot_x[NUM_MOVES];
    double robot_y[NUM_MOVES];
    double robot_th[NUM_MOVES];
    double vision_x[NUM_MOVES];
    double vision_y[NUM_MOVES];
    double vision_th[NUM_MOVES];

    // Move the robot to different positions, save where vision thinks the marker is, and where it actually should be
    for (int i=0; i < NUM_MOVES; i++)
    {
      // Randomly generate a pertubation
      double dx = ((double)rand() / RAND_MAX) * TRANS_DX - TRANS_DX / 2.0;
      double dy = ((double)rand() / RAND_MAX) * TRANS_DY - TRANS_DY / 2.0;

      double dtheta;

      // Randomly generate a rotation, but make sure we have an evenly 
      //  distributed amount of rotations through each quadrant
      if ( i < NUM_MOVES/4)
        dtheta = ((double)rand() / RAND_MAX) * PI/2;
      else if (i < NUM_MOVES/2)
        dtheta = ((double)rand() / RAND_MAX) * PI/2 + PI/2;
      else if (i < 3*NUM_MOVES/4)
        dtheta = ((double)rand() / RAND_MAX) * PI/2 + PI;
      else
        dtheta = ((double)rand() / RAND_MAX) * PI/2 + 3*PI/2;

      Quaternion curQuat;
      curQuat[0] = CAMERA_Q0;
      curQuat[1] = CAMERA_QX;
      curQuat[2] = CAMERA_QY;
      curQuat[3] = CAMERA_QZ;

      RotMat dRot;
      dRot.rotX(dtheta);

      Quaternion newQuat = dRot.getQuaternion() ^ curQuat;


      // Move the robot there
      robot.SetCartesian(
          CAMERA_X + dx, 
          CAMERA_Y, 
          CAMERA_Z + dy, 
          newQuat[0], newQuat[1], newQuat[2], newQuat[3]);

      // Take a picture and see where the vision system thinks the robot is
      ros::Duration(0.5).sleep();
      vision.CaptureImage(GRASP_DETECT, picFileName);
      vision.GetInfo(picFileName, nMarkers, confidence, aux, x, y, theta, alpha, dist);

      // Save both the robot and vision planar coordinates
      robot_x[i] = dx;
      robot_y[i] = dy;
      robot_th[i] = dtheta;
      vision_x[i] = x - init_x;
      vision_y[i] = y - init_y;
      double ang = alpha - init_th;
      if (ang > PI)
        ang -= 2*PI;
      else if (ang < -PI)
        ang += 2*PI;
      vision_th[i] = ang;

      cout << "." << flush;
    }

    cout << endl << "Computing error bounds" << endl;


    // Now compute the optimal constant to multiply all the vision x and y values by that will get it to most closely match our robot x and y values
    matlab.sendMat("robot_x", Mat(robot_x, NUM_MOVES, 1));
    matlab.sendMat("robot_y", Mat(robot_y, NUM_MOVES, 1));
    matlab.sendMat("robot_th", Mat(robot_th, NUM_MOVES, 1));
    matlab.sendMat("vision_x", Mat(vision_x, NUM_MOVES, 1));
    matlab.sendMat("vision_y", Mat(vision_y, NUM_MOVES, 1));
    matlab.sendMat("vision_th", Mat(vision_th, NUM_MOVES, 1));

    matlab.sendCommand("cx = pinv([vision_x, ones(size(vision_x))])*robot_x;");
    matlab.sendCommand("cy = pinv([vision_y, ones(size(vision_y))])*robot_y;");
    matlab.sendCommand("cth = pinv([vision_th, ones(size(vision_th))])*robot_th;");

    matlab.sendCommand("sx = sqrt(sum(([vision_x, ones(size(vision_x))]*cx - robot_x).^2/length(vision_x)));");
    matlab.sendCommand("sy = sqrt(sum(([vision_y, ones(size(vision_y))]*cy - robot_y).^2/length(vision_y)));");
    matlab.sendCommand("sth = sqrt(sum(([vision_th, ones(size(vision_th))]*cth - robot_th).^2/length(vision_th)));");

    double sx = matlab.getValue("sx");
    double sy = matlab.getValue("sy");
    double sth = matlab.getValue("sth");

    cout << "-------------------------------------" << endl;
    cout << "Results: sx = " << sx << ", sy = " << sy << ", sth = " << sth << endl;
    cout << "-------------------------------------" << endl;

    break;
  }
    default:
  ROS_INFO("Sorry. Invalid Selection. Goodbye!");
  return 1;

  */
  }


  // Go back to our home position, and we're done
  robot.SetJoints(CAL_HOME_J1, CAL_HOME_J2, CAL_HOME_J3, 
      CAL_HOME_J4, CAL_HOME_J5, CAL_HOME_J6);


  return 0;
}
