#include <ros/ros.h>

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
#include "pc_utils.h"
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

int n_boards;
const int board_dt = 4;
int board_w;
int board_h;

// The width is in columns, not bytes, as we are looking for 
// the position of the pixel and not of the data contained within it.
int imgWidth;
int imgHeight;

// The number of corners on the board.
const int n_corners = 42;
// An array containing the locations of each of the corners. 
// For the ith corner, the pixel location is at (2 * i, 2 * i + 1). 
int cornerLocs [n_corners * 2];
// The number of corners that the algorithm detects.
int corner_count;
// States if the corners have been recorded before, 
// and if the corner data has been read.
bool hasRecorded = false;
bool hasCornerData = false;

// The minimum and maximum values of the x and y values for the corners.. 
int maxX;
int minX;
int maxY;
int minY;

FILE* fd;

int blah = 0;

namespace enc = sensor_msgs::image_encodings;
using namespace message_filters;

void image_callback(const sensor_msgs::ImageConstPtr & msg){

  // Gets the incoming image width and height.
  imgWidth = msg->width;
  imgHeight = msg->height;

  if (blah == 10){
    //ROS_INFO("got 10");
    return;
  }
  // fd = fopen("/home/simplehands/Desktop/container/test.txt", "w");

  // ROS_INFO("opened");

  cv_bridge::CvImagePtr cim = cv_bridge::toCvCopy(msg, enc::BAYER_GRBG8);

  board_w = 7; // Board width in squares
	board_h = 6; // Board height 
	n_boards = 2; // Number of boards
	int board_n = board_w * board_h;
	CvSize board_sz = cvSize( board_w, board_h );
  
  CvCapture* capture = cvCreateCameraCapture(1);
	//assert( capture );

	cvNamedWindow( "Calibration" );
	// Allocate Sotrage
	CvMat* image_points		= cvCreateMat( n_boards*board_n, 2, CV_32FC1 );
	CvMat* object_points		= cvCreateMat( n_boards*board_n, 3, CV_32FC1 );
	CvMat* point_counts			= cvCreateMat( n_boards, 1, CV_32SC1 );
	CvMat* intrinsic_matrix		= cvCreateMat( 3, 3, CV_32FC1 );
	CvMat* distortion_coeffs	= cvCreateMat( 5, 1, CV_32FC1 );

	CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];
	int successes = 0;
	int step, frame = 0;

	//IplImage *image = cvQueryFrame( capture );
  IplImage preimage((cim->image));
  IplImage* image = &preimage;
	IplImage *gray_image = cvCreateImage( cvGetSize(image), 8, 1 );

  

	// Capture Corner views loop until we've got n_boards
	// succesful captures (all corners on the board are found)
  int ttt = 0;
	//while( successes < n_boards ){
  while(ttt < 1){
    ttt ++;
    // ROS_INFO("ttt %d blah %d\n", ttt, blah);
		// Skp every board_dt frames to allow user to move chessboard
		if( frame++ % board_dt == 0 ){
      //ROS_INFO("enter if 1\n");

			// Find chessboard corners:
      while(image == NULL){
       // ROS_INFO("NNN\n");
      }
			int found = cvFindChessboardCorners( image, board_sz, corners,
				&corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );

      // If we have corners, store them in the global variable for point cloud comparison.
      if (corner_count > 0){
        ROS_INFO("Corner Count = %d.", corner_count);
        hasCornerData = true;

        // Setting up for getting min and max...
        minX = imgWidth;
        minY = imgHeight;
        maxX = maxY = 0;

        for (int i = 0; i < corner_count; i++){
          // Record the corner locations.
          cornerLocs[2 * i    ] = (int) corners[i].x;
          cornerLocs[2 * i + 1] = (int) corners[i].y;

          // Get min and max for x and y.
          // See initialization at the top of this file for details.
          if (cornerLocs[2 * i] < minX){
            minX = cornerLocs[2 * i];
          }
          if (cornerLocs[2 * i + 1] < minY){
            minY = cornerLocs[2 * i + 1];
          }
          if (cornerLocs[2 * i] > maxX){
            maxX = cornerLocs[2 * i];
          }
          if (cornerLocs[2 * i + 1] > maxY){
            maxY = cornerLocs[2 * i + 1];
          }

        }
      }
      else
        ROS_INFO("Not recording, no corners detected.");

      //ROS_INFO("found %d\n\n\n", found);
      //assert(!found);
			// Get subpixel accuracy on those corners

			//cvCvtColor( image, gray_image, CV_BGR2GRAY );
			//cvCvtColor( image, gray_image, CV_RGB2GRAY);

      //ROS_INFO("during 0\n");

      cvFindCornerSubPix( gray_image, corners, corner_count, cvSize( 11, 11 ), 
				cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

      //ROS_INFO("during 1\n");

			// Draw it
			cvDrawChessboardCorners( image, board_sz, corners, corner_count, found );
			//ROS_INFO("during 2\n");

      cvShowImage( "Calibration", image );

      //ROS_INFO("blah 1\n");


			// If we got a good board, add it to our data
			if( corner_count == board_n ){
            //ROS_INFO("enter if 2\n");

				step = successes*board_n;
        //fd = fopen("cali.txt", "w");
				for( int i=step, j=0; j < board_n; ++i, ++j ){
					CV_MAT_ELEM( *image_points, float, i, 0 ) = corners[j].x;
					CV_MAT_ELEM( *image_points, float, i, 1 ) = corners[j].y;
					CV_MAT_ELEM( *object_points, float, i, 0 ) = j/board_w;
					CV_MAT_ELEM( *object_points, float, i, 1 ) = j%board_w;
					CV_MAT_ELEM( *object_points, float, i, 2 ) = 0.0f;
          // ROS_INFO("coner.x %f, corner.y %f", corners[j].x, corners[j].y);
          char c[50];
          sprintf(c, "%f %f", corners[j].x, corners[j].y);
          // ROS_INFO("%s", c);
          //fputs(c, fd);
          // fprintf(fd, "%s", c);
				}
        ROS_INFO("\n");
        blah++;
        //fclose(fd);
				CV_MAT_ELEM( *point_counts, int, successes, 0 ) = board_n;
				successes++;
			}
          //ROS_INFO("end if 1\n");

		} 

    //ROS_INFO("blah 2\n");

		// Handle pause/unpause and ESC
		int c = cvWaitKey( 15 );
		if( c == 'p' ){
      
      //ROS_INFO("blah in if\n");

			c = 0;
			while( c != 'p' && c != 27 ){
				c = cvWaitKey( 250 );
			}
		}
		if( c == 27 ){
      // fclose(fd);
			return;
    }
      //return 0;
		image = cvQueryFrame( capture ); // Get next image
	} // End collection while loop

  // fclose(fd);
  return;
}

void points_callback(const sensor_msgs::PointCloud2ConstPtr & msg){

  // These are the files where we'll be recording pixel and point data.
  ofstream cornerDataFile;
  ofstream BGDataFile;

  // These are the strings containing the location of the files.
  string cornerDataString = "/home/simplehands/Documents/kinectData4.txt";
  string BGDataString = "/home/simplehands/Documents/kinectData3.txt";

  // We only start recording if we haven't recorded before (we want the first data set)  
  // and if we actually have the data.
  if (hasCornerData && !hasRecorded){
    cornerDataFile.open (cornerDataString.c_str());
  
    // Convert to PCL Format
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    fromROSMsg (*msg, *cloud);

    // For each corner, write the x, y, and z location of the corresponding
    // point into the data file. We use corner_count because it might detect
    // fewer than n_corners, which would result in accessing unused memory.
    int n = 0;
    int size = corner_count;
    ROS_INFO("Starting write of main point cloud.");
    for (n = 0; n < size; n++){
      /*ROS_INFO("Writing point [%d, %d], and its value is <%f, %f, %f>.",
          cornerLocs[2 * n], cornerLocs[2 * n + 1], 
          cloud->at(cornerLocs[2 * n], cornerLocs[2 * n + 1]).x,
          cloud->at(cornerLocs[2 * n], cornerLocs[2 * n + 1]).y,
          cloud->at(cornerLocs[2 * n], cornerLocs[2 * n + 1]).z);*/
      cornerDataFile << cloud->at(cornerLocs[2 * n], cornerLocs[2 * n + 1]).x;
      cornerDataFile << ", ";
      cornerDataFile << cloud->at(cornerLocs[2 * n], cornerLocs[2 * n + 1]).y;
      cornerDataFile << ", ";
      cornerDataFile << cloud->at(cornerLocs[2 * n], cornerLocs[2 * n + 1]).z;
      cornerDataFile << endl;
    }
    ROS_INFO("File closing...");
    cornerDataFile.close();
    cornerDataFile.clear();
    ROS_INFO("File closed; stream cleared.");

    // This is the size of the border around the corners
    // that we record for the "surrounding area" point
    // cloud, in pixels.
    int threshold = 10;

    BGDataFile.open(BGDataString.c_str());

    ROS_INFO("MaxX = %d; MinX = %d; MaxY = %d; MinY = %d.", maxX, minX, maxY, minY);
    ROS_INFO("Starting write of \"background\" point cloud.");

    // Simply writing each point's x, y, and z  to the data file in the area
    // defined by the corners and the border size.
    for (int i = minX - threshold; i <= maxX + threshold; i++){
      for (int j = minY - threshold; j <= maxY + threshold; j++){
        /* ROS_INFO("Point at [%d, %d] is <%f, %f, %f>.", 
          i, j, 
          cloud->at(i, j).x, 
          cloud->at(i, j).y, 
          cloud->at(i, j).z); */
        BGDataFile << cloud->at(i, j).x << ", ";
        BGDataFile << cloud->at(i, j).y << ", ";
        BGDataFile << cloud->at(i, j).z << endl;
      }
    }

    ROS_INFO("File closing...");
    BGDataFile.close();
    BGDataFile.clear();
    ROS_INFO("File closed; stream cleared.");

    hasRecorded = true;

  }
}

void synthesized_callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::PointCloud2ConstPtr & point_cloud){
  image_callback(image_msg);
  points_callback(point_cloud);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "cv");

  ros::NodeHandle n;
  message_filters::Subscriber<sensor_msgs::Image> imageSub (n, "/camera/rgb/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pointSub (n, "/camera/depth/points", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy; 
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imageSub, pointSub);

  sync.registerCallback(boost::bind(&synthesized_callback, _1, _2));

  ros::spin();
  return 0;
}
