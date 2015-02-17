//
// Name: Robbie Paolini
//
// File Name: getColors.cpp
//
// Last Modified: 8/5/2011
//
// This program takes in a calibration image and spits out hsv values for all 
//  of the pixels within a certain area for each marker. This is useful for 
//  giving us statistical information about the color of each marker. 
// The best way to run this program is to first take a picture with all 
//  of the markers in the image either all horizontal or all vertical. Then,
//  in GIMP or some other program, determine the pixel values for rectangles
//  that cover each marker. 
// Then, you can run the program passing it the calibration image and 
//  number of standard deviations away to set color ranges for,
//  and you're done!

#include <ros/ros.h>
#include <vision_comm/CaptureImage.h>
#include <vision_comm/GetInfo.h>
#include <vision_comm/CalibrateVision.h>
#include <cstdlib>
#include <std_msgs/String.h>
#include <string.h>
#include <vector>
#include <dirent.h>
#include <sys/types.h>
#include <iostream>
#include <complex>
#include <cmath>

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;

#define PI 3.14159265

#define NUM_COLORS 5

// Number of standard deviations to use when calculating ranges
#define NUM_STDEV 2.0

// Color order from left to right in the calibration image
const string colorOrder[NUM_COLORS] =
{
    "Yellow",
    "Green",
    "Pink",
    "Blue",
    "Orange"
};

// (x1, y1, w, h) of bounding rectangles for each color
const int rectCoords[NUM_COLORS][4] =
{
    {382, 72, 83, 648},
    {476, 71, 75, 650},
    {569, 72, 71, 652},
    {657, 69, 75, 655},
    {752, 70, 72, 667}
};

// Rotate an image by a given angle
void imgRotate(IplImage *img, IplImage *newImg, float angle)
{     
    CvPoint2D32f center;  
    CvMat *translate = cvCreateMat(2, 3, CV_32FC1);  
    cvSetZero(translate);  
    center.x = img->width/2;  
    center.y = img->height/2;  
    cv2DRotationMatrix(center, angle, 1.0, translate);  
    cvWarpAffine(img, newImg, translate, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(0));  
    cvReleaseMat(&translate);  
}

int main(int argc, char **argv)
{
    /* The model image is the picture with 5 different-colored markers */
    char *model_path;
    
    // This keeps track of how many standard deviations to use for our range
    double num_stdev;

    // Read in the arguments from the user
    if(argc == 2)
    {
        model_path = argv[1];
        num_stdev = NUM_STDEV;
    }
    else if (argc == 3)
    {
        char* pEnd;
        model_path = argv[1];
        num_stdev = strtod(argv[2], &pEnd);
    }
    else
    {
        cout << "usage: get_colors <model image> [num stdev]" << endl;
        return 1;
    }

    // Get the image, and convert it to hsv
    IplImage *modelImg;
    IplImage *hsvImg;
    if(!(modelImg = cvLoadImage(model_path)))
    {
        cout << "Problem opening image: " << model_path << endl;
        return 1;
    }
    hsvImg = cvCreateImage(cvGetSize(modelImg), IPL_DEPTH_8U, 3);
    cvCvtColor(modelImg, hsvImg, CV_BGR2HSV);

    CvScalar mean;
    CvScalar stdev;

    int ranges[5][6];

    // First, compute the ranges for S and V
    for (int i=0; i<NUM_COLORS; i++)
    {
        cvSetImageROI(hsvImg, cvRect(rectCoords[i][0], rectCoords[i][1], rectCoords[i][2], rectCoords[i][3]));
        cvAvgSdv(hsvImg, &mean, &stdev);
        cvResetImageROI(hsvImg);

        // Compute ranges s and v
        for (int j=1; j < 3; j++)
        {
            int min = mean.val[j] - num_stdev * stdev.val[j];
            int max = mean.val[j] + num_stdev * stdev.val[j];
            if (min < 0)
                min = 0;
            if (max > 255)
                max = 255;

            ranges[i][j] = min;
            ranges[i][j+3] = max;
        }
    }

    // Now, compute the ranges for Hue. Note that we need to compute 
    //  wrap-around averages and standard deviations
    for (int k=0; k<NUM_COLORS; k++)
    {
        complex<double> expsum(0.0, 0.0);
        int n = 0;
        // Each (i,j) is a pixel in the image.
        for(int i = rectCoords[k][0]; i < rectCoords[k][0] + rectCoords[k][2]; i++)
        {
            for (int j = rectCoords[k][1]; j < rectCoords[k][1] + rectCoords[k][3]; j++)
            {
                /* Get Hue for this pixel */
                uchar HVal = ((uchar*)(hsvImg->imageData + j*hsvImg->widthStep))
                    [i*hsvImg->nChannels + 0];

                // Convert our hue between 0 and 2*pi, and put in on the 
                //  unit circle in the complex plane
                complex<double> curAng(0.0, ((int)HVal / 255.0) * 2 * PI);

                expsum += exp(curAng);
                n++;
            }
        }
        // Now, our mean is the average angle on that unit circle
        double hmean = arg(expsum) / (2*PI) * 255;

        // Our standard deviation uses the distance from center as a metric
        // The closer to the edge of the circle, the less spread our data is
        double r = abs(expsum) / n;
        double hstd = sqrt(-2*log(r)) / (2*PI) * 255;

        // Now, compute our ranges, and wrap around if necessary
        int maxh = (int)round(hmean + num_stdev * hstd);
        int minh = (int)round(hmean - num_stdev * hstd);

        if (maxh > 255)
            maxh -= 256;
        if (minh < 0)
            minh += 256;
        ranges[k][0] = minh;
        ranges[k][3] = maxh;

        // Print out the ranges for this color (hmin, smin, vmin, hmax, smax, vmax)
        cout << "    {" 
            << ranges[k][0] << ", " << ranges[k][1] << ", " 
            << ranges[k][2] << ", " << ranges[k][3] << ", " 
            << ranges[k][4] << ", " << ranges[k][5] << "} \t// " << colorOrder[k].c_str() << endl;
    }

    return 0;
}


