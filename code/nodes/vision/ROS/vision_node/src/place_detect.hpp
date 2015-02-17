/**
 * @file place_detect.hpp
 *
 * Header file for using image processing to detect 
 *  if a marker has been placed successfully
 * @author Robbie Paolini
 */

#ifndef PLACE_DETECT_H
#define PLACE_DETECT_H

// Standard C++ includes
#include <iostream>
#include <string>
#include <fstream>
#include <math.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <ctype.h>
#include <highgui.h>

#include <sys/stat.h>

#include <ros/ros.h>
#include <vision_comm/vision_comm.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
//#include <cv_bridge/CvBridge.h>

//! The cropped height and width of the image to process
#define PIMG_WIDTH 137//210//220//300
#define PIMG_HEIGHT 343//660//670//710

// The top corner of the image to process
#define PIMG_X   264//525//500//400
#define PIMG_Y   74//70//190//160


//! The cropped height and width of the image to process
#define DIMG_WIDTH 425
#define DIMG_HEIGHT 165

// The top corner of the image to process
#define DIMG_X   150
#define DIMG_Y   125

//! The cropped height and width of the image to process
#define IIMG_WIDTH 52
#define IIMG_HEIGHT 100

// The top corner of the image to process
#define IIMG_X   242
#define IIMG_Y   185

#define P_H_CHG 2
#define P_S_CHG 2
#define P_V_CHG 2

#define P_NUM_COLORS 5

#define P_CLOSE_KERNEL_SIZE 5
#define P_OPEN_KERNEL_SIZE 5

#define NUM_TASKS 3

static const CvSize pImgSize[NUM_TASKS] =
{
  cvSize(PIMG_WIDTH, PIMG_HEIGHT),
  cvSize(DIMG_WIDTH, DIMG_HEIGHT),
  cvSize(IIMG_WIDTH, IIMG_HEIGHT)
};

static const CvRect roiRect[NUM_TASKS] = 
{
  cvRect(PIMG_X, PIMG_Y, PIMG_WIDTH, PIMG_HEIGHT),
  cvRect(DIMG_X, DIMG_Y, DIMG_WIDTH, DIMG_HEIGHT),
  cvRect(IIMG_X, IIMG_Y, IIMG_WIDTH, IIMG_HEIGHT)
};

// Holds the HSV ranges for each color in the following format:
// (Hlow, Slow, Vlow, Hhigh, Shigh, Vhigh). Note that Hue can
// wrap around. These ranges can be found by running getColors.cpp
static const unsigned char pColorRanges[P_NUM_COLORS][6] = {
  {27, 151, 98, 51, 243, 255},    // Yellow
  {251, 174, 128, 21, 255, 255},  // Orange
  {217, 150, 81, 36, 219, 255},   // Pink
  {78, 122, 42, 108, 245, 217},   // Blue
  {58, 144, 35, 77, 234, 161}     // Green
};
/*
static const unsigned char pColorRanges[P_NUM_COLORS][6] = {
  {20, 103, 163, 45, 228, 255},   // Yellow
  {3, 155, 165, 20, 255, 255},    // Orange
  {166, 105, 151, 59, 214, 255},  // Pink
  {81, 131, 101, 106, 247, 255},  // Blue
  {60, 147, 72, 78, 241, 255},    // Green
};
*/
/*
static const unsigned char pColorRanges[P_NUM_COLORS][6] = {
  {30, 152, 114, 50, 240, 255},   // Yellow
  {252, 173, 102, 21, 255, 255},  // Orange
  {228, 137, 82, 29, 227, 255},   // Pink
  {79, 123, 25, 107, 243, 221},   // Blue
  {59, 148, 28, 77, 235, 177}     // Green
};
*/
static const int placeDetectThresh[NUM_TASKS][P_NUM_COLORS] = {
  // Placing Detection
  {
  12000,  // Yellow
  12000,  // Orange
  12000,  // Pink
  12000,  // Blue
  12000   // Green
  },
  //Dropping Detection
  {
  2000,  // Yellow
  2000,  // Orange
  2000,  // Pink
  2000,  // Blue
  2000   // Green
  },
  //Insertion Detection
  {
  100,  // Yellow
  100,  // Orange
  100,  // Pink
  100,  // Blue
  100   // Green
  }
};

class PlaceDetect
{
  public:
    PlaceDetect();
    ~PlaceDetect();

    bool detectMarker(char * filename);
    bool detectDrop(char * filename);
    bool detectInsert(char * filename);
    void setCalibrationImage(char * filename);

  private:

    bool detectSuccess(char * filename, int j);

    /* Save an image with no marker to compare */
    IplImage * calibrationImg[NUM_TASKS];

    IplImage* imgHsv[NUM_TASKS];       // Holds down sampled version of iH

    IplImage *diffImg[NUM_TASKS];         // Holds difference between bg and current image
    IplImage *imgHsvS[NUM_TASKS];      // Holds signed version of down sampled image

    IplImage *matchH[NUM_TASKS];       // Holds 255 or 0 for something satisfying hue
    IplImage *matchS[NUM_TASKS];       // Holds 255 or 0 for something satisfying sat
    IplImage *matchV[NUM_TASKS];       // Holds 255 or 0 for something satisfying value

    IplImage *diffH[NUM_TASKS];        // Holds difference in hue
    IplImage *diffHH[NUM_TASKS];       // Holds wrap around difference in hue
    IplImage *diffS[NUM_TASKS];        // Holds difference in saturation
    IplImage *diffV[NUM_TASKS];        // Holds difference in value

    IplImage *alive[NUM_TASKS];        // Holds pixels still viable after bg subtract

    IplImage *rgbImg[NUM_TASKS];

    IplImage *Hvals[NUM_TASKS];        // Holds hue values from down sampled image
    IplImage *Svals[NUM_TASKS];        // Holds saturation values from down sampled image
    IplImage *Vvals[NUM_TASKS];        // Holds value values from down sampled image

    // Holds 255 or 0 for each pixel based on whether that color exists
    IplImage* colors[NUM_TASKS][P_NUM_COLORS];
};


#endif // PLACE_DETECT_H
