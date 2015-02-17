/**
 * @file processor.hpp
 *
 * Header file for marker recognition processing
 * @author Alex Zirbel
 * @author Robbie Paolini
 */

#ifndef PROCESSOR_H
#define PROCESSOR_H

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
//#include <opencv/cv.h>
//#include <opencv/ml.h>
//#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
//#include <opencv2/ml/ml.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <cv_bridge/CvBridge.h>

/////////////////
// Image Sizes
/////////////////

//! The expected height and width of images to be processed
#define IMG_WIDTH 1280
#define IMG_HEIGHT 960

// Down sampled image size
#define DS_WIDTH 640
#define DS_HEIGHT 480

// Conversion between downsampled pixels and mm
#define MM_TO_PIX   2.41//2.4               // Multiply mm by this to get pixels
#define PIX_TO_MM   (1.0 / MM_TO_PIX) // Multiply pixels by this to get mm

////////////////////////////////
// Hand and Marker Information
////////////////////////////////

// Pixel location of the center of the hand (Down Sampled). 
//  used to get x and y displacement.
#define HAND_CENTER_X 312.89
#define HAND_CENTER_Y 242.26

// Height and width of marker in down-sampled pixels
#define MARKER_WIDTH 45//43
#define MARKER_HEIGHT 325//313

// The actual length of the marker in mm
#define MARKER_LENGTH_MM 131

// Number of different marker colors we're looking for
#define NUM_COLORS 5


/////////////////////////////////////////////
// Image Subtraction and Color Thresholding
/////////////////////////////////////////////

// When doing background subtraction, we eliminate all pixels that are within
//  these values from the calibration image
#define CAL_H_CHG 2
#define CAL_S_CHG 2
#define CAL_V_CHG 2

// Opening and closing kernel sizes when we are doing a 
//  series of erode and dilate operations
#define CLOSE_KERNEL_SIZE 5
#define OPEN_KERNEL_SIZE 5



// How far off a second recognized marker can be until we 
//  count it as a different marker when doing our 2D gaussian fit
#define TH_MATCH  0.1   // (rad)
#define W_MATCH   ((MARKER_WIDTH/2.0)*(MARKER_WIDTH/2.0))  // (down-sampled pix)
#define H_MATCH   ((MARKER_HEIGHT/2.0)*(MARKER_HEIGHT/2.0))  // (down-sampled pix)
#define POS_MATCH ((MARKER_WIDTH/2)*(MARKER_WIDTH/2))


///////////////////
// Template Files
///////////////////

// Header for all of the template files
#define TEMP_FILE_HEADER "/home/simplehands/Desktop/images/templates/t"

// Number of rotated template images. Note that since the marker is 
// rectangular, we only need 180 degrees of rotation. Hence, our 
// pose accuracy will be: 180.0/NUM_TEMPLATES degrees
#define NUM_TEMPLATES   720

// Conversion between radians and template index and vice versa
#define RAD_TO_TEMP_NUM (NUM_TEMPLATES / PI)
#define TEMP_NUM_TO_RAD (PI / NUM_TEMPLATES)

//////////////////////
// Template Matching
//////////////////////

// Region of interest dimensions to use when template matching marker
#define ROI_WIDTH 333//321
#define ROI_HEIGHT 333//321

// This is the number of templates away from the guessed orientation we'll 
//  look at in either direction. Thus, our guess must always be within 
//  TEMP_NUM_TO_RAD * TEMPLATE_WINDOW for us to find the true orientation
#define TEMPLATE_WINDOW   4

// Cartesian neighborhood radius around marker to check (pixels)
#define W_WINDOW 10
#define H_WINDOW 50

// distance in mm to move the ROI by each time 
//  (this in effect will be our marker position resolution)
#define ROI_STEP 1.0

// We calculte certainty by dividing the number of correlated pixels
//  by this value. The more pixels we match, the more certain we are
//  of our computed marker position
#define CORR_SCALE (MARKER_WIDTH * MARKER_HEIGHT)

// Need pi for various angle calculations
#define PI 3.14159265

using namespace std;
using namespace cv;

// If the pixel count for a given color is below this value, then
//  there is no marker of this color in the image
static const int min_mark_pix[NUM_COLORS] = {
  3000, // Yellow
  3000, // Orange
  3000, // Pink
  3000, // Blue
  3000  // Green
};

// If the number of pixels we count for a given color is above this value, 
//  then there is more than one marker of this color in the image
static const int max_mark_pix[NUM_COLORS] = {
  14000,  // Yellow
  14000,  // Orange
  15000,  // Pink
  14000,  // Blue
  14000   // Green
};


// Holds the HSV ranges for each color in the following format:
// (Hlow, Slow, Vlow, Hhigh, Shigh, Vhigh). Note that Hue can
// wrap around. These ranges can be found by running getColors.cpp
static const unsigned char colorRanges[NUM_COLORS][6] = {
  {30, 163, 82, 50, 238, 255},  // Yellow
  {1, 188, 68, 14, 255, 255},   // Orange
  {238, 153, 57, 20, 222, 255}, // Pink
  {76, 119, 23, 108, 244, 183}, // Blue
  {58, 148, 20, 76, 233, 151}   // Green
};
/*static const unsigned char colorRanges[NUM_COLORS][6] = {
  {30, 152, 114, 50, 240, 255},   // Yellow
  {252, 173, 102, 21, 255, 255},  // Orange
  {228, 137, 82, 29, 227, 255},   // Pink
  {79, 123, 25, 107, 243, 221},   // Blue
  {59, 148, 28, 77, 235, 177}     // Green
};*/

// Names of each color based on index (for User Interface)
static const string intToColor[NUM_COLORS] =
{
  "Yellow",
  "Orange",
  "Pink",
  "Blue",
  "Green"
};

// RGB values for each of our marker colors (for User Interface)
static const CvScalar intToScalarColor[NUM_COLORS] =
{
  cvScalar(100,255,255),
  cvScalar(0,100,235),
  cvScalar(100,100,250),
  cvScalar(255,100,0),
  cvScalar(100,200,0)
};

// Size of our original image
static const CvSize bigImgSize = cvSize(IMG_WIDTH, IMG_HEIGHT);
// Size of our downsampled image
static const CvSize dsImgSize = cvSize(DS_WIDTH, DS_HEIGHT);
// Size of our region of interest
static const CvSize roiSize = cvSize(ROI_WIDTH, ROI_HEIGHT);

// Data to pass back to vision_node for communication
struct OutputData
{
  int numMarkers;
  double certainty[MAX_MARKERS];
  double posx[MAX_MARKERS];
  double posy[MAX_MARKERS];
  double theta[MAX_MARKERS];
  double alpha[MAX_MARKERS];
  double r[MAX_MARKERS];
};

// Class definition
class Processor
{
  public:
    Processor();
    ~Processor();
    void setVerbose(bool isVerbose);
    OutputData* processMarkers(char * filename);
    void setCalibrationImage(char * filename);
    bool isVerbose();

  private:
    CvMat* get_non_zeros(const IplImage* img);
    void getColorImgs(IplImage *imgHsv, IplImage* colors[NUM_COLORS]);
    void getEstimate(IplImage *colors[NUM_COLORS], int &num_markers, 
        int &idx, double &theta, double &x, double &y);
    void refineEstimate(IplImage *img, double theta, double x, double y, 
        OutputData *output);
    void drawMarkerBox(IplImage *img, CvScalar color, 
        double theta, double x, double y);
    string loadDebugNames(char *filename);

    // Displays status messages as it goes if verbose is true
    bool verbose;
    string curDebugName;

    // We will keep track of most IplImages we use so that we don't
    // waste time freeing and reallocating memory each time.

    IplImage* calImgS;      // Holds most recent picture of hand with no marker

    IplImage* iH;           // Holds HSV of initial cropped picture from camera
    IplImage* imgHsv;       // Holds down sampled version of iH

    // Holds 255 or 0 for each pixel based on whether that color exists
    IplImage* colors[NUM_COLORS];

    IplImage *diff;         // Holds difference between bg and current image
    IplImage *imgHsvS;      // Holds signed version of down sampled image

    IplImage *matchH;       // Holds 255 or 0 for something satisfying hue
    IplImage *matchS;       // Holds 255 or 0 for something satisfying sat
    IplImage *matchV;       // Holds 255 or 0 for something satisfying value

    IplImage *diffH;        // Holds difference in hue
    IplImage *diffHH;       // Holds wrap around difference in hue
    IplImage *diffS;        // Holds difference in saturation
    IplImage *diffV;        // Holds difference in value

    IplImage *alive;        // Holds pixels still viable after bg subtract

    IplImage *Hvals;        // Holds hue values from down sampled image
    IplImage *Svals;        // Holds saturation values from down sampled image
    IplImage *Vvals;        // Holds value values from down sampled image

    // Array of template images of markers at different angles
    IplImage *templateImgs[NUM_TEMPLATES];

    IplImage *matchRes;     // Result of convolution between ROI and template
};


#endif
