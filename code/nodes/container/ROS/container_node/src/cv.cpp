#include <ros/ros.h>
#include <std_msgs/String.h>


#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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

int n_boards = 0;
const int board_dt = 4;
int board_w;
int board_h;

FILE* fd;

int blah = 0;

namespace enc = sensor_msgs::image_encodings;

void callback(const sensor_msgs::ImageConstPtr & msg){
  if (blah == 10){
    //ROS_INFO("got 10");
    return;
  }
  fd = fopen("/home/simplehands/Desktop/container/test.txt", "w");

  ROS_INFO("opened");

  cv_bridge::CvImagePtr cim = cv_bridge::toCvCopy(msg, enc::BAYER_GRBG8);

  board_w = 4; // Board width in squares
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
	int corner_count;
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
    ROS_INFO("ttt %d blah %d\n", ttt, blah);
		// Skp every board_dt frames to allow user to move chessboard
		if( frame++ % board_dt == 0 ){
      //ROS_INFO("enter if 1\n");

			// Find chessboard corners:
      while(image == NULL){
        ROS_INFO("NNN\n");
      }
			int found = cvFindChessboardCorners( image, board_sz, corners,
				&corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );

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
          ROS_INFO("coner.x %f, corner.y %f", corners[j].x, corners[j].y);
          char c[50];
          sprintf(c, "%f %f", corners[j].x, corners[j].y);
          ROS_INFO("%s", c);
          //fputs(c, fd);
          fprintf(fd, "%s", c);
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
      fclose(fd);
			return;
    }
      //return 0;
		image = cvQueryFrame( capture ); // Get next image
	} // End collection while loop

  fclose(fd);
  return;
}

void callback2(const sensor_msgs::ImageConstPtr & msg){
/*  while(true)
    ROS_INFO("b\n");
  */
  return;
}
int main(int argc, char **argv){
  ros::init(argc, argv, "cv");

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, callback);
  image_transport::Subscriber sub2 = it.subscribe("/camera/depth/image_raw", 1, callback2);

  ros::spin();
  return 1;
}
