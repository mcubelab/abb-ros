//
// Name: Robbie Paolini
//
// File Name: vision_calibration.cpp
//
// Last Modifed: 7/30/2012
//
///////////////////////////////////////////////
// Vision Calibration Suite
//
// Step 0: Grasp a marker
// 
// Calibration 1: Move marker around linearly to find pixel to mm ratio
//
// Calibration 2: Rotate object around hand to find center of hand in the camera frame
//
// Evaluation: Move to many different random positions and angles. Record everything. See how we stack up in x, y, theta reconstruction of rectangle.
//
//
//



#include <ros/ros.h>
#include <matVec/matVec.h>
#include <robot_comm/robot_comm.h>
#include <vision_comm/vision_comm.h>
#include <util_comm/util_comm.h>
#include <grasp_comm/grasp_comm.h>
#include <matlab_comm/matlab_comm.h>
#include <iostream>
#include <cstdlib>
#include <ctime>

#define CAMERA_X 55.0
#define CAMERA_Y 465.0
#define CAMERA_Z 660.0
#define CAMERA_Q0 0.7071
#define CAMERA_QX 0.7071
#define CAMERA_QY 0.0
#define CAMERA_QZ 0.0

#define CAL_TCP 50
#define CAL_ORI 50

#define TRANS_DX 40.0
#define TRANS_DY 40.0


#define NUM_TRANSLATION 100
#define NUM_ROTATION 200
#define NUM_MOVES 100


int main(int argc, char** argv)
{
  // Declare variables
  ros::init(argc, argv, "vision_calibration");
  ros::NodeHandle node;

  srand(time(NULL));

  UtilComm util(&node);  
  RobotComm robot(&node);  
  GraspComm grasper(&node);
  MatlabComm matlab(&node);
  VisionComm vision(&node);

  GM_InputParams grasperInParams;
  GM_OutputParams grasperOutParams;
  grasperInParams.type = GM_TYPE_USER;

  std::string picFileName;
  double aux;
  int nMarkers;
  double confidence;
  double x, y, theta, alpha, dist;


  int test;
  cout << "Vision Calibration Suite" << endl;
  cout << "------------------------" << endl;
  cout << "1: Pixel to mm ratio calibration" << endl;
  cout << "2: Center of palm calibration" << endl;
  cout << "3: Accuracy Evaluation" << endl;
  cout << "------------------------" << endl;
  cout << "What would you like to do? ";
  cin >> test;


  cout << endl << "------------------------" << endl;
  cout << "Moving to home position, calibrating hand and vision system..." << endl;

  // Set up hand and robot
  util.go_home();
  util.calibrate_hand();
  util.calibrate_vision();

  cout << "Please give a marker to the robot." << endl;

  grasperOutParams = grasper.grasp(grasperInParams);

  cout << "Preparing system..." << endl;

  util.go_safe_vision();

  // Wait a bit to make sure the table is no longer shaking
  ros::Duration(0.5).sleep(); //0.25 seconds

  vision.CaptureImage(GRASP_DETECT, picFileName);
  vision.GetInfo(picFileName, nMarkers, confidence, aux, x, y, theta, alpha, dist);

  if (nMarkers != 1)
  {
    ROS_ERROR("I cannot calibrate vision if I'm not holding 1 marker!! Goodbye!");
    return 1;
  }

  robot.SetSpeed(CAL_TCP, CAL_ORI);
  robot.SetZone(ZONE_FINE);
  
  robot.SetCartesian(CAMERA_X, CAMERA_Y, CAMERA_Z, 
      CAMERA_Q0, CAMERA_QX, CAMERA_QY, CAMERA_QZ);


  switch (test)
  {
    ////////////////////////////////////////////////////////
    // PIXEL TO MM CALIBRATION
    ////////////////////////////////////////////////////////
    case 1:
      {
        cout << "Translating the robot to random positions and recording vision output" << endl;

        double init_x = x;
        double init_y = y;

        double robot_x[NUM_TRANSLATION];
        double robot_y[NUM_TRANSLATION];
        double vision_x[NUM_TRANSLATION];
        double vision_y[NUM_TRANSLATION];

        // Move the robot to different positions, save where vision thinks the marker is, and where it actually should be
        for (int i=0; i < NUM_TRANSLATION; i++)
        {
          // Randomly generate a pertubation
          double dx = ((double)rand() / RAND_MAX) * TRANS_DX - TRANS_DX / 2.0;
          double dy = ((double)rand() / RAND_MAX) * TRANS_DY - TRANS_DY / 2.0;

          // Move the robot there
          robot.SetCartesian(
              CAMERA_X + dx, 
              CAMERA_Y, 
              CAMERA_Z + dy, 
              CAMERA_Q0, CAMERA_QX, CAMERA_QY, CAMERA_QZ);

          // Take a picture and see where the vision system thinks the robot is
          ros::Duration(0.5).sleep();
          vision.CaptureImage(GRASP_DETECT, picFileName);
          vision.GetInfo(picFileName, nMarkers, confidence, aux, x, y, theta, alpha, dist);

          // Save both the robot and vision planar coordinates
          robot_x[i] = dx;
          robot_y[i] = dy;
          vision_x[i] = x - init_x;
          vision_y[i] = y - init_y;
          
          cout << i+1 << "/" << NUM_TRANSLATION << endl;
          //cout << "." << flush;
        }
        
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

        break;
      }

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
            dtheta = ((double)rand() / RAND_MAX) * PI/2 - 3*PI/4;
          else if (i < NUM_ROTATION/2)
            dtheta = ((double)rand() / RAND_MAX) * PI/2 - PI/4;
          else if (i < 3*NUM_ROTATION/4)
            dtheta = ((double)rand() / RAND_MAX) * PI/2 + PI/4;
          else
            dtheta = ((double)rand() / RAND_MAX) * PI/2 + 3*PI/4;

          Quaternion curQuat;
          curQuat[0] = CAMERA_Q0;
          curQuat[1] = CAMERA_QX;
          curQuat[2] = CAMERA_QY;
          curQuat[3] = CAMERA_QZ;


          Quaternion rotQuat;
          rotQuat[0] = cos(dtheta/2.0);
          rotQuat[1] = 0.0;
          rotQuat[2] = 0.0;
          rotQuat[3] = sin(dtheta/2.0);

          Quaternion newQuat = curQuat ^ rotQuat;

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
          
          cout << "." << flush;
        }
        
        cout << endl << "Computing optimal center of hand." << endl;


        // Now compute the optimal constant to multiply all the vision x and y values by that will get it to most closely match our robot x and y values
        matlab.sendMat("x", Mat(vision_x, NUM_ROTATION, 1));
        matlab.sendMat("y", Mat(vision_y, NUM_ROTATION, 1));

        matlab.sendCommand("c = pinv([x,y,ones(size(x))])*(-x.^2-y.^2);");
        matlab.sendCommand("cx = -0.5*c(1);");
        matlab.sendCommand("cy = -0.5*c(2);");

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


        //double init_x = x;
        //double init_y = y;
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
          if ( i < NUM_ROTATION/4)
            dtheta = ((double)rand() / RAND_MAX) * PI/2 - 3*PI/4;
          else if (i < NUM_ROTATION/2)
            dtheta = ((double)rand() / RAND_MAX) * PI/2 - PI/4;
          else if (i < 3*NUM_ROTATION/4)
            dtheta = ((double)rand() / RAND_MAX) * PI/2 + PI/4;
          else
            dtheta = ((double)rand() / RAND_MAX) * PI/2 + 3*PI/4;

          Quaternion curQuat;
          curQuat[0] = CAMERA_Q0;
          curQuat[1] = CAMERA_QX;
          curQuat[2] = CAMERA_QY;
          curQuat[3] = CAMERA_QZ;


          Quaternion rotQuat;
          rotQuat[0] = cos(dtheta/2.0);
          rotQuat[1] = 0.0;
          rotQuat[2] = 0.0;
          rotQuat[3] = sin(dtheta/2.0);

          Quaternion newQuat = curQuat ^ rotQuat;

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

          vision_x[i] = x;
          vision_y[i] = y;

          // For the angle of the marker, subtract our initial angle, so that we are at least close to the actual angle.
          double ang = alpha - init_th;
          
          cout << "Initial vision angle: " << ang << endl;
          
          // Now, make sure we're between -PI and PI
          if (ang > PI)
            ang -= 2*PI;
          else if (ang < -PI)
            ang += 2*PI;

          cout << "Constrained angle: " << ang << endl;

          // We now expect the angle found by vision to be very close to the angle from the robot. Since the marker has 180 degree symmetry, if our values are not close, we are probably off by PI.
          if (fabs(ang - dtheta) > PI/2)
          {
            if (ang > 0)
              ang -= PI;
            else
              ang += PI;
          }

          cout << "Changed angle: " << ang << ", dtheta = " << dtheta << endl;

          vision_th[i] = ang;

          //cout << "." << flush;
        }

        cout << endl << "Computing error bounds" << endl;


        // Now compute the optimal constant to multiply all the vision x and y values by that will get it to most closely match our robot x and y values
        matlab.sendMat("robot_x", Mat(robot_x, NUM_MOVES, 1));
        matlab.sendMat("robot_y", Mat(robot_y, NUM_MOVES, 1));
        matlab.sendMat("robot_th", Mat(robot_th, NUM_MOVES, 1));
        matlab.sendMat("vision_x", Mat(vision_x, NUM_MOVES, 1));
        matlab.sendMat("vision_y", Mat(vision_y, NUM_MOVES, 1));
        matlab.sendMat("vision_th", Mat(vision_th, NUM_MOVES, 1));

        // For the x and y values of the hand, we need to transform them 
        // back to the original configuration of the marker and hand. 
        // This will allow us to compare all of the predicted x's 
        // and y's by the vision system in the same frame.
        matlab.sendCommand("dx = cos(robot_th).*(vision_x - robot_x) + sin(robot_th).*(vision_y - robot_y)");
        matlab.sendCommand("dy = -sin(robot_th).*(vision_x - robot_x) + cos(robot_th).*(vision_y - robot_y)");
        matlab.sendCommand("sx = std(dx);");
        matlab.sendCommand("sy = std(dy);");
        matlab.sendCommand("sth = std(vision_th - dtheta);");

        /*
        matlab.sendCommand("cx = pinv([vision_x, ones(size(vision_x))])*robot_x;");
        matlab.sendCommand("cy = pinv([vision_y, ones(size(vision_y))])*robot_y;");
        matlab.sendCommand("cth = pinv([vision_th, ones(size(vision_th))])*robot_th;");

        matlab.sendCommand("sx = sqrt(sum(([vision_x, ones(size(vision_x))]*cx - robot_x).^2/length(vision_x)));");
        matlab.sendCommand("sy = sqrt(sum(([vision_y, ones(size(vision_y))]*cy - robot_y).^2/length(vision_y)));");
        matlab.sendCommand("sth = sqrt(sum(([vision_th, ones(size(vision_th))]*cth - robot_th).^2/length(vision_th)));");

        */

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
  }







  util.go_home();
  util.random_drop();

  return 0;
}
