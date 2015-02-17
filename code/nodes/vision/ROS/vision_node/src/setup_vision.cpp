//
// Name: Robbie Paolini
//
// Program Name: setup_vision.cpp
//
// Last Modified: 8/10/2012
//
// This program creates the necessary folders for vision_node to function properly.
// It also creates an array of rotated marker templates in the folder images/templates
// Note that this program uses constants about marker size, region of interest, number
//  of template files, etc from the the processor include file

#include "processor.hpp"

// Rotates an image by a given angle
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

#define MAIN_DIRECTORY "/home/simplehands/Desktop/images"

int main(int argc, char **argv)
{
  char buf[256];

  // Create necessary folders
  mkdir(MAIN_DIRECTORY, 0777);
  sprintf(buf, "%s/0", MAIN_DIRECTORY);
  mkdir(buf, 0777);
  sprintf(buf, "%s/1", MAIN_DIRECTORY);
  mkdir(buf, 0777);
  sprintf(buf, "%s/cal", MAIN_DIRECTORY);
  mkdir(buf, 0777);
  sprintf(buf, "%s/templates", MAIN_DIRECTORY);
  mkdir(buf, 0777);

  // Now let's create our template images
  // Allocate space for the original rectangular image and rotated images
  IplImage* rectImg = cvCreateImage(cvSize(ROI_WIDTH, ROI_HEIGHT), 8, 1);
  IplImage* rotImg = cvCreateImage(cvSize(ROI_WIDTH, ROI_HEIGHT), 8, 1);

  // Zero our image, and create a white rectangle the size of our marker
  //  in the center of the image and parallel to the x-axis
  cvZero(rectImg);
  cvRectangle(rectImg, 
      cvPoint(ROI_WIDTH/2-MARKER_HEIGHT/2, ROI_HEIGHT/2-MARKER_WIDTH/2), 
      cvPoint(ROI_WIDTH/2+MARKER_HEIGHT/2-1, ROI_HEIGHT/2+MARKER_WIDTH/2-1), 
      cvScalar(255), CV_FILLED);

  // Now, rotate our rectangular image by progressively larger angles,
  //  and save each to a file
  for (int i=0; i < NUM_TEMPLATES; i++)
  {
    double ang = 180.0 / NUM_TEMPLATES * i;
    imgRotate(rectImg, rotImg, ang);

    sprintf(buf, "%s%d.png", TEMP_FILE_HEADER, i);
    cvSaveImage(buf, rotImg);
  }

  return 0;
}

