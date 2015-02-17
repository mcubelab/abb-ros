/* Nick Stanley
   Carnegie Mellon University
   Simple Hands Manipulation Lab
   June 2012

Description: This program is designed for the Simple Hands robotic arm and the checkerboard tool attached to it. The robot arm moves around and the Kinect (hanging above it) looks below for the checkerboard corners. When it gets a full list of the corners, it converts them into 3D points, and then writes them to a file, along with the points taken from the robot's internal sensors.
*/
#include <ros/ros.h>
#include <robot_comm/robot_comm.h>

#include <matVec/matVec.h>
#include "calibration_corner_detection.h"

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// Standard C++ includes
#include <iostream>
#include <string>
#include <fstream>
#include <math.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

// Definitions for hardware positions of the robot.

#define WORK_X 808.5 //500.0
#define WORK_Y -612.86 //0.0
#define WORK_Z 0.59 //250.0
#define WORK_Q0 0.7084 //0.0
#define WORK_QX 0.0003882 //0.0
#define WORK_QY -0.0003882 //1.0
#define WORK_QZ 0.7058 //0.0

#define TOOL_X 0.0
#define TOOL_Y 0.0
#define TOOL_Z 0.0
#define TOOL_Q0 1.0
#define TOOL_QX 0.0
#define TOOL_QY 0.0
#define TOOL_QZ 0.0

#define TCP 50.0
#define ORI 20.0
#define ZONE 0

using namespace message_filters;

bool points_initialized = false;
bool image_initialized = false;

// A list of the destinations in x,y,z.
const int numPositions = 6;
const int numJoints = 6;

float jointPositions [numPositions][numJoints] = 
	{
	{-8.0, 44.8, 23.9, 1.2, 21.1, -20.7}, // first position 
	{16.6, 48.3, 20.3, 0.9, 21.0, 4.2}, // second position
	{23.3, 39.6, 22.7, 0.6, 27.3, 15.1}, // etc.
	{-23.4, 44.0, 21.7, 0.8, 24.1, -32.2},
	{-7.7, 34.1, 29.4, 0.6, 26.3, -13.2},
	{22.1, 46.4, 21.7, 0.2, 21.4, 17.4}
	};

// Facing downward, every time.
Quaternion calib_rot("0.001 0.727 0.686 0.0");

// All of the positions of the points relative to the tool frame.
Vec translations[12] = 
{
  Vec("-90 -45 28.575", 3),
  Vec("-90 -15 28.575", 3),
  Vec("-90 15 28.575", 3),
  Vec("-90 45 28.575", 3),
  Vec("-120 -45 28.575", 3),
  Vec("-120 -15 28.575", 3),
  Vec("-120 15 28.575", 3),
  Vec("-120 45 28.575", 3),
  Vec("-150 -45 28.575", 3),
  Vec("-150 -15 28.575", 3),
  Vec("-150 15 28.575", 3),
  Vec("-150 45 28.575", 3)
};

// The current pose of the robot.
Vec currentPosition(3);
Vec currentOrientation(4);

// The current image, cloud.
cv_bridge::CvImagePtr callback_image;
pcl::PointCloud<pcl::PointXYZ>::Ptr callback_cloud (new pcl::PointCloud<pcl::PointXYZ>);

// Making sure the robot doesn't get ahead of himself.
bool locked = false;
bool newPointCloud = false;

void callback(const sensor_msgs::ImageConstPtr& image_msg, 
    const sensor_msgs::PointCloud2ConstPtr & point_cloud){

  if (!locked && !newPointCloud){
    points_initialized = true;
    image_initialized = true;
    locked = true;
    // If we're safe, we set the current image and cloud to the global variable.
    ROS_INFO("Loading new data.");
    callback_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BAYER_GRBG8); 

    writeImageToFile(&(callback_image->image));

    pcl::fromROSMsg (*point_cloud, *callback_cloud);
    ROS_INFO("New data loaded.");
    newPointCloud = true;
    locked = false;
  }

}

int main(int argc, char** argv){
  ros::init(argc, argv, "calibration_node");
  ros::NodeHandle node; 

  // Initialize node, get everything working.

  RobotComm robot(&node);

  // Make sure our robot communication is blocking
  if (!robot.SetComm(BLOCKING))
  {
    ROS_ERROR("Unable to set robot to BLOCKING");
    return -1;
  }
  // Make sure that our robot's work object is set up
  else if (!robot.SetWorkObject(WORK_X, WORK_Y, WORK_Z, 
		      WORK_Q0, WORK_QX, WORK_QY, WORK_QZ))
  {
    ROS_ERROR("Unable to set robot work object");
    return -1;
  }
  // Make sure our tool frame is correctly set up
  else if (!robot.SetTool(TOOL_X, TOOL_Y, TOOL_Z, 
		      TOOL_Q0, TOOL_QX, TOOL_QY, TOOL_QZ))
  {
    ROS_ERROR("Unable to set robot tool frame");
    return -1;
  }
  // Set our default speed limits
  else if (!robot.SetSpeed(TCP, ORI))
  {
    ROS_ERROR("Unable to set speed");
    return -1;
  }
  // Set the default "zone" of our robot (amount of interpolation we allow)
  else if (!robot.SetZone(ZONE))
  {
    ROS_ERROR("Unable to set zone");
    return -1;
  }

  ROS_INFO("Settings complete!");

  ROS_INFO("Initializing subscribers...");

  message_filters::Subscriber<sensor_msgs::Image> imageSub (node, "/camera/rgb/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pointSub (node, "/camera/depth/points", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2>
    MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imageSub, pointSub);

  sync.registerCallback(boost::bind(&callback, _1, _2));

  // Now getting callbacks!
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Subscribers initialized.");

	ros::Rate stop_check(30);
	ros::Rate wait_length(2);

  while (!points_initialized || !image_initialized)
		stop_check.sleep();  

  ROS_INFO("Moving to initial position.");

  // Go to the initial position.
  robot.SetJoints(-21.8, 9.3, 39.8, 0.0, 40.89, -21.8);

  // Open the file and start writing the data.
  ofstream dataFile;
  string dataPath = "/home/simplehands/Documents/hands/code/nodes/calibration/data.txt";
  dataFile.open(dataPath.c_str());

  ros::Rate waitTime(0.5);

  // Go through all the positions.
  int index;
  for (index = 0; index < numPositions; index++){

    Vec calib_pos(numJoints);
		for (int i = 0; i < numJoints; i++)
		{
    	calib_pos[i] = jointPositions[index][i];
		}

    // Go to the indexth position.
    ROS_INFO("Moving...");
    robot.SetJoints(calib_pos[0], calib_pos[1], calib_pos[2], calib_pos[3], calib_pos[4], calib_pos[5]);

    // Wait a half second.
		
		waitTime.sleep(); // calls it once to finish remaining cycle time
    waitTime.sleep(); // then calls it again for a full cycle

    newPointCloud = false; // ensures that it is stopped when it gets the newest point cloud

    // Prepare everything for corner detection.
    
    // These are defined in calibration_corner_detection.h
    int numCorners = BOARD_HEIGHT * BOARD_WIDTH;

    // We will use these to get the points from the Kinect.
    CvPoint2D32f * corners        = new CvPoint2D32f  [numCorners];
    pcl::PointXYZ * corner_points = new pcl::PointXYZ [numCorners];

		// Wait a half second before continuing.

		wait_length.sleep();
		wait_length.sleep();

    // If we are safe, start processing.
    while (locked || !newPointCloud)
		{
		stop_check.sleep();
		stop_check.sleep();
		}

      ROS_INFO("Processing images...");
      locked = true;

      IplImage preimage (callback_image->image); 
      IplImage * image = &preimage;

      // Detects the corners, returns 1 if it works, 0 else.
      // Repeats as necessary.
      int errorNo;
      errorNo = getCorners(corners, image);
      while (errorNo != -1){
        ROS_INFO("Corners unable to be found. Found only %d.", errorNo);
				wiggle(&robot, 1);
        locked = false; 
        newPointCloud = false;
				stop_check.sleep();
				stop_check.sleep();
        while (!newPointCloud)
				{
				stop_check.sleep(); 
				stop_check.sleep();
				}
        locked = true;
        newPointCloud = true;
        errorNo = getCorners(corners, image);
      }

      ROS_INFO("We have our corners!");

      // Get Robot points here. Start some math.

      robot.GetCartesian(currentPosition[0], currentPosition[1], currentPosition[2],
        currentOrientation[0], currentOrientation[1], currentOrientation[2], currentOrientation[3]);

      ROS_INFO("Our pose is <%f, %f, %f, %f, %f, %f, %f>.", 
          currentPosition[0], currentPosition[1], currentPosition[2], 
          currentOrientation[0], currentOrientation[1], 
          currentOrientation[2], currentOrientation[3]);

      Vec transWR(4);

      int i;
      // Create the matrices necessary for work.
      // (WR is "World To Robot")
      for (i = 0; i < 3; i++)
        transWR[i] = currentPosition[i];
      // Homogeneous coordinates.
      transWR[3] = 1;
      Quaternion quatWR;
      for (i = 0; i < 4; i++)
        quatWR[i] = currentOrientation[i];

      // We know the position of the point relative to the robot,
      // so we're getting it relative to the world.
      HomogTransf TWR (quatWR.getRotMat(), transWR); // TODO: Might just need a 3 vector

      ROS_INFO("Matrices generated.");

      Vec pointInWorld(4);
      for (i = 0; i < numCorners; i++){
        corner_points[i] = callback_cloud->at( (int) (corners[i].x + 0.5), (int) (corners[i].y + 0.5) );
        dataFile << corner_points[i].x << ", ";
        dataFile << corner_points[i].y << ", ";
        dataFile << corner_points[i].z << ", ";
        pointInWorld = TWR * translations[i];
        dataFile << pointInWorld[0] << ", ";
        dataFile << pointInWorld[1] << ", ";
        dataFile << pointInWorld[2] << endl;
      }

      ROS_INFO("Unlocking MutEx expressions.");
      locked = false;
      newPointCloud = false;
    
  }

  // Save to file.
  dataFile.close();
  dataFile.clear();

}

