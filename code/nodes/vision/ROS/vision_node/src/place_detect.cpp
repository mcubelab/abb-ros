// 
// Name: Robbie Paolini
//
// Program Name: place_detect.cpp
//
// Last Modified: 8/5/2011
//
// This program detects whether or not a marker has been placed in a 
// given location. This occurs by simply doing a background subtraction,
// and seeing if there's enough of a difference.

#include "place_detect.hpp"

using namespace std;
using namespace cv;

/*
 * Return true or false as to whether a marker is placed in a given location.
 *
 * Simply do a background subtraction, and find the magnitude of the difference.
 * If it exceeds a certain value, then a marker must be there. 
 */
bool PlaceDetect::detectMarker(char *filename)
{
  return detectSuccess(filename, 0);
}

bool PlaceDetect::detectDrop(char *filename)
{
  return detectSuccess(filename, 1);
}

bool PlaceDetect::detectInsert(char *filename)
{
  return detectSuccess(filename, 2);
}

bool PlaceDetect::detectSuccess(char * filename, int j)
{
  IplImage *bigImg = NULL;

  // Load the passed image into memory
  if(!(bigImg = cvLoadImage(filename)))
  {
    ROS_WARN("Problem Opening Image: %s", filename);
    return false;
  }

  // Crop the current image, and convert it to HSV
  cvSetImageROI(bigImg, roiRect[j]);
  cvCopy(bigImg, rgbImg[j]);
  cvCvtColor(rgbImg[j], imgHsv[j], CV_BGR2HSV);
  cvConvertScale(imgHsv[j], imgHsvS[j]);

  cvResetImageROI(bigImg);
  cvReleaseImage(&bigImg);

  // Subtract the current image from the calibration image
  cvAbsDiff(imgHsvS[j], calibrationImg[j], diffImg[j]);

  // Split up the absolute difference into 3 channels
  cvSplit(diffImg[j], diffH[j], diffS[j], diffV[j], NULL);

  // Make sure we wrap around to get the lowest difference for hue
  cvAbsDiffS(diffH[j], diffHH[j], cvScalar(256));
  cvMin(diffHH[j], diffH[j], diffH[j]);

  // Now, store in match* those values that are not subtracted out by the background
  cvCmpS(diffH[j], P_H_CHG, matchH[j], CV_CMP_GT);
  cvCmpS(diffS[j], P_S_CHG, matchS[j], CV_CMP_GT);
  cvCmpS(diffV[j], P_V_CHG, matchV[j], CV_CMP_GT);

  // AND these together to get a single channel of pixels we should still care about
  cvAnd(matchS[j], matchV[j], matchS[j]);
  cvAnd(matchS[j], matchH[j], alive[j]);

  // Split our image into 3 channels so we can compare colors
  cvSplit(imgHsvS[j], Hvals[j], Svals[j], Vvals[j], NULL);

  // Now, let's do color thresholding for all of our colors
  for (int i=0; i < P_NUM_COLORS; i++)
  {
    // initially fill this color with the pixels that were not subtracted
    // out when we did the background subtraction
    cvCopy(alive[j], colors[j][i]);

    // First, threshold hue
    if (pColorRanges[i][0] < pColorRanges[i][3])
    {
      // Only keep pixels that are between our 2 ranges
      cvCmpS(Hvals[j], pColorRanges[i][0], matchH[j], CV_CMP_GE);
      cvAnd(colors[j][i], matchH[j], colors[j][i]);
      cvCmpS(Hvals[j], pColorRanges[i][3], matchH[j], CV_CMP_LE);
      cvAnd(colors[j][i], matchH[j], colors[j][i]);
    }
    else
    {
      // If the range wraps around, only keep pixels that
      // are above the first value or below the second value
      cvCmpS(Hvals[j], pColorRanges[i][0], matchH[j], CV_CMP_GE);
      cvCmpS(Hvals[j], pColorRanges[i][3], matchV[j], CV_CMP_LE);
      cvOr(matchH[j], matchV[j], matchH[j]);
      cvAnd(colors[j][i], matchH[j], colors[j][i]);
    }

    // Now, only keep pixels that are between S
    cvCmpS(Svals[j], pColorRanges[i][1], matchS[j], CV_CMP_GE);
    cvAnd(colors[j][i], matchS[j], colors[j][i]);
    cvCmpS(Svals[j], pColorRanges[i][4], matchS[j], CV_CMP_LE);
    cvAnd(colors[j][i], matchS[j], colors[j][i]);

    // Only keep pixels that are between V
    cvCmpS(Vvals[j], pColorRanges[i][2], matchV[j], CV_CMP_GE);
    cvAnd(colors[j][i], matchV[j], colors[j][i]);
    cvCmpS(Vvals[j], pColorRanges[i][5], matchV[j], CV_CMP_LE);
    cvAnd(colors[j][i], matchV[j], colors[j][i]);

    // Now, attempt to close some gaps in the image
    cvMorphologyEx(colors[j][i], colors[j][i], NULL, 
        cvCreateStructuringElementEx(P_CLOSE_KERNEL_SIZE, P_CLOSE_KERNEL_SIZE, 
        (P_CLOSE_KERNEL_SIZE)/2, (P_CLOSE_KERNEL_SIZE)/2, CV_SHAPE_RECT), 
        MORPH_CLOSE);

    // Finally, remove any points that are by themselves
    cvMorphologyEx(colors[j][i], colors[j][i], NULL, 
        cvCreateStructuringElementEx(P_OPEN_KERNEL_SIZE, P_OPEN_KERNEL_SIZE, 
          (P_OPEN_KERNEL_SIZE)/2, (P_OPEN_KERNEL_SIZE)/2, CV_SHAPE_RECT),
        MORPH_OPEN);

    ROS_INFO("Num Pixels: %d, Thresh: %d", 
        cvCountNonZero(colors[j][i]), placeDetectThresh[j][i]);
    if (cvCountNonZero(colors[j][i]) > placeDetectThresh[j][i])
      return true;
  }

  return false;
}


/**
 * Sets the processor to calibration image that we assume to be an empty
 * image.  This is used for background subtraction and for changing the image
 * color balance (or thresholds) so that the processor knows what colors
 * will look like markers in future images.
 */
void PlaceDetect::setCalibrationImage(char *calibrationFilename)
{
  IplImage *bigImg = NULL;

  // Load the calibration image into memory
  if(!(bigImg = cvLoadImage(calibrationFilename)))
  {
    cout << "Problem opening image: " << calibrationFilename << endl;
    return;
  }

  IplImage *cImgHsv;
  
  for (int i=0; i<NUM_TASKS; i++)
  {
    cImgHsv = cvCreateImage(pImgSize[i], IPL_DEPTH_8U, 3);

    // Crop the overall image and copy the relevant area 
    //  to our calibration image
    cvSetImageROI(bigImg, roiRect[i]);
    cvCopy(bigImg, rgbImg[i]);

    // Now, convert the color to HSV, and make it signed
    cvCvtColor(rgbImg[i], cImgHsv, CV_BGR2HSV);
    cvConvertScale(cImgHsv, calibrationImg[i]);
    cvReleaseImage(&cImgHsv);

    cvResetImageROI(bigImg);
  }

  // Release our temporarily allocated images
  cvReleaseImage(&bigImg);
}

/**
 * Constructor for the PlaceDetect class. Allocates space for calibration image
 */
PlaceDetect::PlaceDetect()
{
  for (int i=0; i<NUM_TASKS; i++)
  {
    // Allocate space for calibration image 
    calibrationImg[i] = cvCreateImage(pImgSize[i], IPL_DEPTH_16S, 3);

    imgHsv[i] = cvCreateImage(pImgSize[i], IPL_DEPTH_8U, 3);

    diffImg[i] = cvCreateImage(pImgSize[i], IPL_DEPTH_16S, 3);
    imgHsvS[i] = cvCreateImage(pImgSize[i], IPL_DEPTH_16S, 3);

    matchH[i] = cvCreateImage(pImgSize[i], IPL_DEPTH_8U, 1);
    matchS[i] = cvCreateImage(pImgSize[i], IPL_DEPTH_8U, 1);
    matchV[i] = cvCreateImage(pImgSize[i], IPL_DEPTH_8U, 1);

    diffH[i] = cvCreateImage(pImgSize[i], IPL_DEPTH_16S, 1);
    diffHH[i] = cvCreateImage(pImgSize[i], IPL_DEPTH_16S, 1);
    diffS[i] = cvCreateImage(pImgSize[i], IPL_DEPTH_16S, 1);
    diffV[i] = cvCreateImage(pImgSize[i], IPL_DEPTH_16S, 1);

    alive[i] = cvCreateImage(pImgSize[i], IPL_DEPTH_8U, 1);
    rgbImg[i] = cvCreateImage(pImgSize[i], IPL_DEPTH_8U, 3);

    Hvals[i] = cvCreateImage(pImgSize[i], IPL_DEPTH_16S, 1);
    Svals[i] = cvCreateImage(pImgSize[i], IPL_DEPTH_16S, 1);
    Vvals[i] = cvCreateImage(pImgSize[i], IPL_DEPTH_16S, 1);

    for (int j=0; j < P_NUM_COLORS; j++)
    {
      colors[i][j] = cvCreateImage(pImgSize[i], IPL_DEPTH_8U, 1);
    }
  }
}


PlaceDetect::~PlaceDetect()
{
  for (int i=0; i<NUM_TASKS; i++)
  {
    cvReleaseImage(&calibrationImg[i]);

    cvReleaseImage(&diffImg[i]);
    cvReleaseImage(&imgHsv[i]);
    cvReleaseImage(&imgHsvS[i]);

    cvReleaseImage(&matchH[i]);
    cvReleaseImage(&matchS[i]);
    cvReleaseImage(&matchV[i]);

    cvReleaseImage(&diffH[i]);
    cvReleaseImage(&diffHH[i]);
    cvReleaseImage(&diffS[i]);
    cvReleaseImage(&diffV[i]);

    cvReleaseImage(&alive[i]);
    cvReleaseImage(&rgbImg[i]);

    cvReleaseImage(&Hvals[i]);
    cvReleaseImage(&Svals[i]);
    cvReleaseImage(&Vvals[i]);

    for (int j=0; j < P_NUM_COLORS; j++)
    {
      cvReleaseImage(&(colors[i][j]));
    }
  }

}
