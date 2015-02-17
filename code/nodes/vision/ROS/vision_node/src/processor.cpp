/**
 * @file processor.cpp
 * @brief Processes a given image to determine the number of highlighters in an 
 *  image, and if one, the pose of that marker. Note that the pose of the marker
 *  is with respect to the tool frame of the hand, which is z towards camera, 
 *  x to the right, and y upwards.
 *
 * To recognize markers, the following pipeline is followed:
 * 1. Convert Image to HSV, Down-Sample
 * 2. Background Subtraction
 * 3. Color Ranges -> NUM_COLORS Binary Images
 * 4. Erode->Dilate, Dilate->Erode each image
 * 5. Count pixels to determine how many markers are in each color image
 * 6. If only 1 total marker, fit a 2 dimensional gaussian -> (x,y,cov)
 * 7. SVD covariance matrix to get the dominant eigenvector -> (theta)
 * 8. Match rotated templates to a window around our initial guess
 * 9. Best match is final marker pose (x,y,theta)
 *
 * Note that for this to work well, 2 other helper programs need to be run:
 * getColors.cpp: This takes in a calibration image and finds 
 *                  suitable HSV ranges for each marker color
 * makeTemplates.cpp: This makes an array of rotated marker templates 
 *                      which are used to match markers
 *
 * @author: Alex Zirbel
 * @author: Robbie Paolini
 */

#include "processor.hpp"

/**
 * Helper function to round a number to the nearest integer
 */
double round(double r) {
    return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}

/**
 * Processes a given image to determine the number of highlighter markers
 * and their orientations.
 *
 * @param filename The name of the image, ex. "images/image.png". We assume
 *      that this is a valid filename - no advanced error checking is done.
 */
OutputData* Processor::processMarkers(char *filename)
{
  OutputData *output = (OutputData*) malloc (sizeof(OutputData));
  IplImage *large_img;

  int idx;
  double theta;
  double x, y;

  if(verbose)
    ROS_INFO("Processing %s ...", filename);

  // Load the requested image into memory
  if (!(large_img = cvLoadImage(filename)))
  {
    ROS_WARN("Problem loading file.");
    output->numMarkers = -1;
    return output;
  }

  // Set up a debug directory for this file and return the image name
  // to append to when creating debug images.
  curDebugName = loadDebugNames(filename);
  
  /*

  // Create an hsv image to process
  cvCvtColor(large_img, iH, CV_BGR2HSV);

  // Down Sample our HSV Image
  cvResize(iH, imgHsv);

  */
  cvCvtColor(large_img, imgHsv, CV_BGR2HSV);


  // Find where each color exists in this image
  getColorImgs(imgHsv, colors);

  // Now, find the number of markers in the image, and if only 1,
  //  an estimate of its position and orientation
  getEstimate(colors, output->numMarkers, idx, theta, x, y);

  // If at this point, we're sure we don't have 1 marker, we're done.
  if (output->numMarkers != 1)
  {
    ROS_INFO("Num Markers = %d", output->numMarkers);
    return output;
  }

  // Otherwise, search around the neighborhood we've found to get the best 
  //  estimate we can on this marker
  refineEstimate(colors[idx], theta, x, y, output);

  // If in verbose mode, overlay rectangles representing the initial guess and 
  //  refined estimate of the marker position on the original image
  if (verbose)
  {
    // Get a nice RGB image in our down-sampled size
    cvResize(large_img, imgHsv);
    
    // Draw the initial guess as a black rectangle
    drawMarkerBox(imgHsv, cvScalar(0,0,0), theta, x, y);

    // Draw the final estimate in the color of the current marker
    drawMarkerBox(imgHsv, intToScalarColor[idx], output->theta[0], 
        output->posx[0]*MM_TO_PIX + HAND_CENTER_X, 
        HAND_CENTER_Y - output->posy[0]*MM_TO_PIX);

    // Now save this image
    string str = curDebugName;
    cvSaveImage(curDebugName.append("_result.png").c_str(), imgHsv);
  }

  // Print out the results of our estimation
  ROS_INFO("Marker Pose: (x, y, th) = (%f, %f, %f)", 
      output->posx[0], output->posy[0], output->theta[0]);
  ROS_INFO("Color: %s, Certainty: %f, Num Markers: %d", 
      intToColor[idx].c_str(), output->certainty[0], output->numMarkers);

  // We now know the marker position, so we're done.
  return output;
}


/**
 * This function takes in a down sampled image from the camera, 
 * and returns NUM_COLORS black and white images (value either 255 or 0), 
 * which correspond to whether a given color is present at each pixel loc. 
 *
 * This is accomplished with the following steps:
 * 1. If image H, S, and V values are close enough to the background image's
 *    H, S, and V at that pixel, then we do not allow color to exist there.
 * 2. Out of the pixels still remaining, if the H, S, and V values there 
 *    fall within a given color's range, then color can exist there.
 */
void Processor::getColorImgs(IplImage *imgHsv, IplImage* colors[NUM_COLORS])
{
  // First, convert our image to signed integers so 
  //  weird things don't happen when we subtract
  cvConvertScale(imgHsv, imgHsvS);

  // Now, subtract our image from the background image and store the result
  cvAbsDiff(imgHsvS, calImgS, diff); 

  // Split up the absolute difference into 3 channels
  cvSplit(diff, diffH, diffS, diffV, NULL);

  // Make sure we wrap around to get the lowest difference for hue
  cvAbsDiffS(diffH, diffHH, cvScalar(256));
  cvMin(diffHH, diffH, diffH);

  // Now, store in match* those values that are not subtracted out by the background
  cvCmpS(diffH, CAL_H_CHG, matchH, CV_CMP_GT);
  cvCmpS(diffS, CAL_S_CHG, matchS, CV_CMP_GT);
  cvCmpS(diffV, CAL_V_CHG, matchV, CV_CMP_GT);

  // AND these together to get a single channel of pixels we should still care about
  cvAnd(matchS, matchV, matchS);
  cvAnd(matchS, matchH, alive);

  if (verbose)
  {
    string str = curDebugName;
    cvSaveImage(str.append("_bgsub.png").c_str(), alive);
  }

  // Split our image into 3 channels so we can compare colors
  cvSplit(imgHsvS, Hvals, Svals, Vvals, NULL);

  // Now, let's do color thresholding for all of our colors
  for (int i=0; i < NUM_COLORS; i++)
  {
    // initially fill this color with the pixels that were not subtracted
    // out when we did the background subtraction
    cvCopy(alive, colors[i]);

    // First, threshold hue
    if (colorRanges[i][0] < colorRanges[i][3])
    {
      // Only keep pixels that are between our 2 ranges
      cvCmpS(Hvals, colorRanges[i][0], matchH, CV_CMP_GE);
      cvAnd(colors[i], matchH, colors[i]);
      cvCmpS(Hvals, colorRanges[i][3], matchH, CV_CMP_LE);
      cvAnd(colors[i], matchH, colors[i]);
    }
    else
    {
      // If the range wraps around, only keep pixels that
      // are above the first value or below the second value
      cvCmpS(Hvals, colorRanges[i][0], matchH, CV_CMP_GE);
      cvCmpS(Hvals, colorRanges[i][3], matchV, CV_CMP_LE);
      cvOr(matchH, matchV, matchH);
      cvAnd(colors[i], matchH, colors[i]);
    }

    // Now, only keep pixels that are between S
    cvCmpS(Svals, colorRanges[i][1], matchS, CV_CMP_GE);
    cvAnd(colors[i], matchS, colors[i]);
    cvCmpS(Svals, colorRanges[i][4], matchS, CV_CMP_LE);
    cvAnd(colors[i], matchS, colors[i]);

    // Only keep pixels that are between V
    cvCmpS(Vvals, colorRanges[i][2], matchV, CV_CMP_GE);
    cvAnd(colors[i], matchV, colors[i]);
    cvCmpS(Vvals, colorRanges[i][5], matchV, CV_CMP_LE);
    cvAnd(colors[i], matchV, colors[i]);

    // Now, attempt to close some gaps in the image
    cvMorphologyEx(colors[i], colors[i], NULL, 
        cvCreateStructuringElementEx(CLOSE_KERNEL_SIZE, CLOSE_KERNEL_SIZE, 
        (CLOSE_KERNEL_SIZE)/2, (CLOSE_KERNEL_SIZE)/2, CV_SHAPE_RECT), 
        MORPH_CLOSE);

    // Finally, remove any points that are by themselves
    cvMorphologyEx(colors[i], colors[i], NULL, 
        cvCreateStructuringElementEx(OPEN_KERNEL_SIZE, OPEN_KERNEL_SIZE, 
          (OPEN_KERNEL_SIZE)/2, (OPEN_KERNEL_SIZE)/2, CV_SHAPE_RECT),
        MORPH_OPEN);
  }

  // Save binary color images if in verbose mode
  if (verbose)
  {
    for (int i=0; i<NUM_COLORS; i++)
    {
      string str = curDebugName;
      str.append("color_");
      str += (char)(i+'0');
      cvSaveImage(str.append(".png").c_str(), colors[i]);
    }
  }
}


/**
 * Create an Nx2 matrix which holds the locations of all 
 *  non-zero pixels in the image
 */
CvMat* Processor::get_non_zeros(const IplImage* img)
{
  // Find the number of non-zero pixels in the image
  int num_pix = cvCountNonZero(img);

  if (verbose)
  {
    ROS_INFO("Num Pix: %d", num_pix);
  }

  // If none, we can't create a matrix, so return
  if (num_pix == 0)
    return NULL;

  // Otherwise, allocate a large enough matrix
  CvMat* mat = cvCreateMat(num_pix, 2, CV_32FC1);

  // Now, go through the entire image and add any non-zero pixels
  int idx = 0;
  for( int i = 0; i < img->height; i++ )
  {
    for( int j = 0; j < img->width; j++ )
    {
      if( CV_IMAGE_ELEM(img,uchar,i,j) )
      {
        cvmSet(mat, idx, 0, j);
        cvmSet(mat, idx, 1, i);
        idx++;
      }
    }
  }

  // Return the allocated matrix
  return mat;
}



/**
 * Return an estimate of the total number of markers in an image, and if one,
 *  return an estimate of the pose (x,y,theta) of the marker.
 * 
 * Given NUM_COLORS binary images, first determine how many markers exist 
 *  in each image. If there is 1 marker, fit a 2D gaussian to it, and recover
 *  the center and covariance. Using SVD, we can then get the dominant 
 *  eigenvector direction, which is a good estimate of the angle of our marker.
 *  If, after continuing to search for markers in the other colors, if we 
 *  encounter another marker, and its pose estimate is not close enough to our
 *  previously discovered marker (some colors are recognized as multiple colors 
 *  eg. pink is recognized as pink and orange), then also return with too many
 *  markers. If at the end of all colors, we still only have 1 marker, 
 *  return its pose.
 */
void Processor::getEstimate(IplImage *colors[NUM_COLORS], int &num_markers, 
    int &idx, double &theta, double &x, double &y)
{
  double cur_th, cur_x, cur_y;
  int num_pix = 0;

  // Initially, we have found no markers
  num_markers = 0;

  for (int i=0; i < NUM_COLORS; i++)
  {
    // Get a list of points where we detected color
    CvMat* mat = get_non_zeros(colors[i]);

    // If there were no points or too few points detected, 
    // there is no marker in this image, so go to the next one
    if (!mat || mat->rows < min_mark_pix[i])
      continue;

    // If there were too many points in this image, then there is
    //  more than one marker, so we're done.
    if (mat->rows > max_mark_pix[i])
    {
      num_markers = 2;
      return;
    }

    // Now, calculate the average and covariance matrix of the points
    //  (This fits a 2D gaussian to our points)
    CvMat *cv = cvCreateMat(2,2,CV_32FC1);
    CvMat *avg = cvCreateMat(1,2,CV_32FC1);
    cvCalcCovarMatrix((const CvArr **)(&mat), 1, cv, avg, 
        CV_COVAR_NORMAL | CV_COVAR_ROWS | CV_COVAR_SCALE);

    // Now, recover the eigenvectors of the covariance matrix
    CvMat* eigVal = cvCreateMat(2, 1, CV_32FC1);
    CvMat* eigVecs = cvCreateMat(2, 2, CV_32FC1);
    cvSVD(cv, eigVal, eigVecs);
    
    // Extract the center of our gaussian, which will be the 
    //  estimate of the center of the marker
    cur_x = cvmGet(avg, 0, 0);
    cur_y = cvmGet(avg, 0, 1);

    // Calculate the angle of the marker from the x-axis, and bound it
    // between 0 and PI. (Marker is symmetric)
    cur_th = atan2(-cvmGet(eigVecs, 1, 0), cvmGet(eigVecs, 0, 0));
    if (cur_th < 0)
      cur_th += PI;

    if (verbose)
    {
      ROS_INFO("Vec1: lambda = %f, (%f, %f)", cvmGet(eigVal, 0, 0), cvmGet(eigVecs, 0, 0), cvmGet(eigVecs, 1, 0));
      ROS_INFO("Vec2: lambda = %f, (%f, %f)", cvmGet(eigVal, 1, 0), cvmGet(eigVecs, 0, 1), cvmGet(eigVecs, 1, 1));

      double ratio = cvmGet(eigVal, 0, 0) / cvmGet(eigVal, 1, 0);
      ROS_INFO("Ratio: %f", ratio);
      ROS_INFO("Theta: %f deg", 180*cur_th/PI);
      ROS_INFO("Center: (%f, %f)", cur_x, cur_y);
    }

    // If we have already discovered a different color marker, 
    //  and the new marker is at approximately the same location, 
    //  then choose the one we saw more pixels of. 
    //  (Still only have 1 marker) Otherwise, we have 2 different 
    //  colored markers in the image, so return 
    if (num_markers == 1)
    {
      // Compute the difference in angle
      double dt = fabs(cur_th - theta);
      // Convert the new marker center to coordinates in the frame of the
      //  first marker. This gives us an expression for how far away the 
      //  marker is in the "marker width" direction, and the "marker height"
      //  direction
      double dw = (x-cur_x)*sin(cur_th)-(cur_y-y)*cos(cur_th);
      double dh = (x-cur_x)*cos(cur_th)+(cur_y-y)*sin(cur_th);

      // Now, we can check if this new marker is in the same place as the
      //  old one. Note that we have a much tighter tolerance on the width
      //  than the height, since it's eaiser for markers to line up 
      //  width wise rather than height wise
      if (min(fabs(dt - PI), dt) < TH_MATCH
        && dw*dw < W_MATCH && dh*dh < H_MATCH)
      {
        // If this marker is approximately at the same place, we will
        //  treat this as the same marker. If more pixels are recognized
        //  for this color, use its pose information as our estimate
        if (num_pix < mat->rows)
        {
          num_pix = mat->rows;
          theta = cur_th;
          x = cur_x;
          y = cur_y;
          idx = i;
        }
      }
      else
      {
        // If this marker was not in the same place, then we have more than
        //  1 marker in this image, so we're done.
        num_markers = 2;
        return;
      }
    }
    else
    {
      // If this is the first marker we've found, remember this, and save our
      //  pose and detected pixel information
      num_markers++;
      num_pix = mat->rows;
      theta = cur_th;
      x = cur_x;
      y = cur_y;
      idx = i;
    }
  }
}


/**
 * Given a binary image and estimated marker position, refine the estimate
 *  by matching an array of rotated templates at different 
 *  locations in a window around the estimate
 */
void Processor::refineEstimate(IplImage *img, double theta, double x, double y, 
        OutputData *output)
{
  double min_x, min_y, step_x, step_y;

  int max_corr = 0;
  int cur_corr;
  int bestX=-1, bestY=-1;
  double bestTh=0;

  // Find the index of the template corresponding to the given angle
  int centerTemp = round(theta*RAD_TO_TEMP_NUM);

  // Now, find the outer edges of the template indices we'd like to explore
  int minTemp = centerTemp - TEMPLATE_WINDOW;
  int maxTemp = centerTemp + TEMPLATE_WINDOW;
  
  // Next, find the minimum corner of our cartesian bounding box 
  //  around the marker center estimate
  step_x = cos(theta);
  step_y = sin(theta);

  // Simply do some vector arithmetic to find the corner
  min_x = x + (H_WINDOW * -step_x + W_WINDOW * -step_y);
  min_y = y + (H_WINDOW * step_y + W_WINDOW * -step_x);

  // Determine the number of steps we will take along each edge of the 
  //  rectangle. The higher this number is, the finer our resolution 
  //  of marker center locations
  int h_steps = ceil((2*H_WINDOW+1) * PIX_TO_MM / ROI_STEP);
  int w_steps = ceil((2*W_WINDOW+1) * PIX_TO_MM / ROI_STEP);

  bestTh = theta * RAD_TO_TEMP_NUM;
  bestX = x;
  bestY = y;

  // Now that we have set up our ranges, go through all of the possible marker
  //  centers and orientations to find the best one.
  for (int hh = 0; hh < h_steps; hh++)
  {
    for (int ww = 0; ww < w_steps; ww++)
    {
      // Compute the current distance traveled along each rectangle edge
      double hv = hh * ROI_STEP * MM_TO_PIX;
      double wv = ww * ROI_STEP * MM_TO_PIX;

      // Find that location, and choose the closest integer pixel. 
      // This will be the center of our region of interest.
      int i = round(min_x + wv * step_y + hv * step_x);
      int j = round(min_y + wv * step_x + hv * -step_y);

      // If our region of interest goes outside of the image, then don't worry
      //  about this position
      if (i - (ROI_WIDTH/2) < 0 || j - (ROI_HEIGHT/2) < 0 || 
          i + (ROI_WIDTH/2) >= DS_WIDTH || j + (ROI_HEIGHT/2) >= DS_HEIGHT)
      {
        continue;
      }

      // Move the region of interest window to the desired location
      cvSetImageROI(img, cvRect(i - (ROI_WIDTH/2), j - (ROI_HEIGHT/2), 
          ROI_WIDTH, ROI_HEIGHT));

      // Now, go through our angle window
      for (int h=minTemp; h <= maxTemp; h++)
      {
        // Make sure our index is in range
        int idx = h;
        if (idx < 0)
          idx += NUM_TEMPLATES;
        else if (idx >= NUM_TEMPLATES)
          idx -= NUM_TEMPLATES;


        // AND our current region of interest with the current template
        //  to find the pixels that match
        cvAnd(img, templateImgs[idx], matchRes);

        // Count how many pixels were the same between the 2 images
        cur_corr = cvCountNonZero(matchRes);

        // Keep track of the best match
        if (cur_corr > max_corr)
        {
          max_corr = cur_corr;
          bestX = i;
          bestY = j;
          bestTh = idx;
        }
      }
      
      // Now that we're done with this position, reset our overall image ROI
      cvResetImageROI(img);
    }
  }

  if (verbose)
    ROS_INFO("New Center (x,y) = (%d, %d)", bestX, bestY);

  // Compute the xy center of the marker in mm, 
  //  relative to the center of the hand. Note that since y
  //  pixels increase as they go down, we negate to get the
  //  correct direction
  double posx = (bestX - HAND_CENTER_X) * PIX_TO_MM;
  double posy = (HAND_CENTER_Y - bestY) * PIX_TO_MM;

  // Convert the best template index we found back into radians
  double th = (bestTh * TEMP_NUM_TO_RAD);

  // Make sure the angle we return is between -90 and 90 degrees
  if (th > PI/2)
    th -= PI;

  output->theta[0] = th;

  // This is the angle of the marker with respect to the horizontal.
  // Let's now find the angle of the line through the origin and 
  // perpendicular to the marker.

  // First, compute the point on the marker that is closest to the origin:
  double cx = sin(th)*(-posy*cos(th)+posx*sin(th));
  double cy = cos(th)*(posy*cos(th)-posx*sin(th));
  
  // Now, compute the angle of this line with respect to the horizontal
  output->alpha[0] = atan2(cy, cx);

  // Finally, compute the distance between the line running through the 
  //  center of the marker and the center of the hand
  output->r[0] = fabs(posy * cos(th) - posx * sin(th));

  // Certainty of the position of the marker depends on how good of a 
  //  match we found. The more pixels we match, the more certain we 
  //  are about the position of the marker
  output->certainty[0] = (double)max_corr / CORR_SCALE;

  // Save the center of the marker as well
  output->posx[0] = posx;
  output->posy[0] = posy;

}


/**
* Draws a rectangle representing a marker at a given center 
*  and orientation on top of a provided image
*/
void Processor::drawMarkerBox(IplImage *img, CvScalar color, 
    double theta, double x, double y)
{
  double st = sin(theta);
  double ct = cos(theta);

  // Get the 4 corners of the rectangle
  int x1 = x + ct * (MARKER_HEIGHT/2) + st * (MARKER_WIDTH/2);
  int y1 = y - st * (MARKER_HEIGHT/2) + ct * (MARKER_WIDTH/2);
  int x2 = x - ct * (MARKER_HEIGHT/2) + st * (MARKER_WIDTH/2);
  int y2 = y + st * (MARKER_HEIGHT/2) + ct * (MARKER_WIDTH/2);
  int x3 = x - ct * (MARKER_HEIGHT/2) - st * (MARKER_WIDTH/2);
  int y3 = y + st * (MARKER_HEIGHT/2) - ct * (MARKER_WIDTH/2);
  int x4 = x + ct * (MARKER_HEIGHT/2) - st * (MARKER_WIDTH/2);
  int y4 = y - st * (MARKER_HEIGHT/2) - ct * (MARKER_WIDTH/2);

  // Form our polyhedron and draw the rectangle
  CvPoint  curve1[]={{x1,y1}, {x2,y2}, {x3,y3}, {x4,y4}};
  CvPoint* curveArr[1]={curve1};
  int      nCurvePts[1]={4};
  int      nCurves=1;
  int      isCurveClosed=1;
  int      lineWidth=1;

  cvPolyLine(img,curveArr,nCurvePts,nCurves,isCurveClosed,color,lineWidth);
}


/**
 * @brief Sets up the debugging file extension.
 * More specifically, if the processor is being run in verbose mode, creates
 * a new /debug/ directory within which images will be saved. If this folder
 * already exists, it is not modified. If verbose mode is not enabled, this
 * function still sets a default debug directory just in case the debug
 * directory variable is mistakenly used later: it simply sets the file's
 * directory to the debug directory.
 * 
 * @param filename The full path and name of an image file.
 * @return The root name of a debug file, ie /<dir>/debug/imgName. Note that
 *      there is no final extension.
 */
string Processor::loadDebugNames(char *filename)
{
  // Get the name of the image to be processed
  string imgName(filename);

  // imageNamePartial is the name of the image to be processed without
  //  the final extension.
  string imgNamePartial = imgName.substr(0,imgName.find_last_of("."));

  // These are used for debugging so that we can create a "debug" directory
  //  within the processing directory. Ignore them otherwise.
  string debugDirectory;

  // We need a default debug directory no matter what, but we should only
  // create a separate "debug" folder if the user has specified that they
  // want one with the "verbose" option.
  if(verbose)
    debugDirectory =
      imgName.substr(0,imgName.find_last_of("/")) + "/debug/";
  else
    debugDirectory = imgName.substr(0, imgName.find_last_of("/")) + "/";

  if(verbose && mkdir(debugDirectory.c_str(), 0777))
    ROS_INFO("Debug directory already exists: merging.");
  else if(verbose)
    ROS_INFO("Creating new debug directory.");

  string debugNamePartial =
    debugDirectory + 
    imgNamePartial.substr(imgNamePartial.find_last_of("/")+1);

  if(verbose) ROS_INFO("Debug Name: %s", debugNamePartial.c_str());

  return debugNamePartial;
}


/**
 * Sets the processor to calibration image that we assume to be an empty
 * image.  This is used for background subtraction and for changing the image
 * color balance (or thresholds) so that the processor knows what colors
 * will look like markers in future images.
 */
void Processor::setCalibrationImage(char *calibrationFilename)
{
  IplImage *bigImg = NULL;

  if(verbose)
  {
    ROS_INFO("Set Calibration Image called.");
  }

  // Load the calibration image from a file into memory
  if(!(bigImg = cvLoadImage(calibrationFilename)))
  {
    ROS_WARN("Problem opening image: %s", calibrationFilename);
    return;
  }

  // Convert the image to HSV
  IplImage* cImgHsv = cvCreateImage(cvGetSize(bigImg),IPL_DEPTH_8U,3);
  cvCvtColor(bigImg, cImgHsv, CV_BGR2HSV);

  // Now, downsample the image and convert the scale to a signed integer
  //IplImage* calibrationImgHsv = cvCreateImage(dsImgSize, IPL_DEPTH_8U, 3); 
  //cvResize(cImgHsv, calibrationImgHsv);
  //cvConvertScale(calibrationImgHsv, calImgS); 
  cvConvertScale(cImgHsv, calImgS);
}


/**
 * Sets the processor to either print out debugging and status message, or
 * to keep quiet about what it is doing.
 */
void Processor::setVerbose(bool isVerbose)
{
  verbose = isVerbose;
}

/**
 * Returns whether the processor is in verbose mode or not.
 */
bool Processor::isVerbose()
{
  return verbose;
}

/**
 * Constructor for the Processor class. 
 * Sets verbose level to false, and allocates space for 
 *  all images that will be used
 */
Processor::Processor()
{
  verbose = false;

  // Allocate space for all of the images that will be used
  calImgS = cvCreateImage(dsImgSize, IPL_DEPTH_16S, 3);

  //iH = cvCreateImage(bigImgSize, IPL_DEPTH_8U, 3);
  imgHsv = cvCreateImage(dsImgSize, IPL_DEPTH_8U, 3);

  diff = cvCreateImage(dsImgSize, IPL_DEPTH_16S, 3);
  imgHsvS = cvCreateImage(dsImgSize, IPL_DEPTH_16S, 3);

  matchH = cvCreateImage(dsImgSize, IPL_DEPTH_8U, 1);
  
  matchS = cvCreateImage(dsImgSize, IPL_DEPTH_8U, 1);
  matchV = cvCreateImage(dsImgSize, IPL_DEPTH_8U, 1);

  diffH = cvCreateImage(dsImgSize, IPL_DEPTH_16S, 1);
  diffHH = cvCreateImage(dsImgSize, IPL_DEPTH_16S, 1);
  diffS = cvCreateImage(dsImgSize, IPL_DEPTH_16S, 1);
  diffV = cvCreateImage(dsImgSize, IPL_DEPTH_16S, 1);

  alive = cvCreateImage(dsImgSize, IPL_DEPTH_8U, 1);

  Hvals = cvCreateImage(dsImgSize, IPL_DEPTH_16S, 1);
  Svals = cvCreateImage(dsImgSize, IPL_DEPTH_16S, 1);
  Vvals = cvCreateImage(dsImgSize, IPL_DEPTH_16S, 1);

  matchRes = cvCreateImage(roiSize, IPL_DEPTH_8U, 1);

  for (int i=0; i < NUM_COLORS; i++)
  {
    colors[i] = cvCreateImage(dsImgSize, IPL_DEPTH_8U, 1);
  }

  // Load in all of our template images into memory as well
  char buf[256];
  for (int i=0; i < NUM_TEMPLATES; i++)
  {
    sprintf(buf, "%s%d.png", TEMP_FILE_HEADER, i);
    templateImgs[i] = cvLoadImage(buf, CV_LOAD_IMAGE_GRAYSCALE);
    cvThreshold(templateImgs[i], templateImgs[i], 128, 255, CV_THRESH_BINARY);
  }
}

/**
 * Destructor for Processor class.
 * Simply releases all of the allocated images
 */
Processor::~Processor()
{
  cvReleaseImage(&calImgS);
  //cvReleaseImage(&iH);
  cvReleaseImage(&imgHsv);

  cvReleaseImage(&diff);
  cvReleaseImage(&imgHsvS);

  cvReleaseImage(&matchH);
  cvReleaseImage(&matchS);
  cvReleaseImage(&matchV);

  cvReleaseImage(&diffH);
  cvReleaseImage(&diffHH);
  cvReleaseImage(&diffS);
  cvReleaseImage(&diffV);

  cvReleaseImage(&alive);

  cvReleaseImage(&Hvals);
  cvReleaseImage(&Svals);
  cvReleaseImage(&Vvals);

  cvReleaseImage(&matchRes);
  
  for (int i=0; i < NUM_TEMPLATES; i++)
  {
    cvReleaseImage(&templateImgs[i]);
  }

  for (int i=0; i < NUM_COLORS; i++)
  {
    cvReleaseImage(&(colors[i]));
  }
}
