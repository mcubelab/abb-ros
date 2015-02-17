/* Nick Stanley
   Carnegie Mellon University
   Simple Hands Manipulation Lab
   June 2012
   */


#ifndef calibration_corner_detection
#define calibration_corner_detection

#include <time.h>
#include <robot_comm/robot_comm.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define MAX_BUFFER 256

#define BOARD_WIDTH 4
#define BOARD_HEIGHT 3
#define BOARD_SIZE ( BOARD_WIDTH * BOARD_HEIGHT ) 

/* Takes the image and returns the pixel locations of the corners. 

  parameters: CvPoint2D32f * addressOfCorners, the address of the array of 2D vectors that will be filled with the corner locations.
              IplImage * addressOfImage, the address of the image to be processed.
  returns: -1 if found != 0 and if the number of corners is equal to the number found. 
           The number of corners found, otherwise.

  Note that found != 0 "if all of the corners have been found and they have been placed in a certain order (row by row, left to right in every row), otherwise, if the function fails to find all the corners or reorder them, it returns 0." For more documentation on cvFindChessBoardCorners(), see http://opencv.willowgarage.com/documentation/camera_calibration_and_3d_reconstruction.html .
*/

using namespace std;

inline int getCorners(CvPoint2D32f * addressOfCorners, IplImage * addressOfImage){

  int corner_count; // Stores the number of corners seen by the algorithm.
  CvSize board_sz = cvSize ( BOARD_WIDTH, BOARD_HEIGHT );
  int found = cvFindChessboardCorners ( addressOfImage, board_sz, addressOfCorners, &corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

  if (found == 0){
    return corner_count;
  }
  else 
    return -1;
}

inline bool writeImageToFile(cv::Mat * savedImage)
{
	ROS_INFO("Writing image.");

	char newFilename [MAX_BUFFER];
	time_t timer;
	timer = time(NULL);
	tm * now;
	now = localtime(&timer);
	sprintf(newFilename, 
	"/home/simplehands/Documents/hands/code/nodes/calibration/saved_images/%d:%02d:%02d__%02d:%02d:%02d.png",
		now->tm_year+ 1900,
		now->tm_mon + 1,
		now->tm_mday,
		now->tm_hour,
		now->tm_min,
		now->tm_sec);
	ROS_INFO("Determined filename.");

	cv::imwrite(newFilename, *savedImage);
	ROS_INFO("Image written.");

	return true;
}

inline bool wiggle(RobotComm * robot, float amplitude)
{
	double joints [6]; 
  robot->GetJoints(joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);

  srand ( clock() ); 
  float movement [6];
	float magnitude = 0;
	
  for (int i = 0; i < 6; i++)
	{
		movement[i] = rand() % 199 - 99; // Between -99 and 99
		magnitude += movement[i] * movement[i];
	}
	magnitude = sqrt(magnitude);

	for (int i = 0; i < 6; i++){
		movement[i] = movement[i] * amplitude / magnitude; 
		joints[i] += movement[i];
	}

	robot->SetJoints(joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
  return true;
} 

#endif
