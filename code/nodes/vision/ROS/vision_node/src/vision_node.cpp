/**
 * @file vision_node.cpp
 *
 * This ROS node  controls the vision system for the simple hand project.
 * It must be run on the computer connected to the camera, since it stores and
 * processes all the images locally to eliminate the need for transferring
 * images.  This program can be run using:
 *
 *  rosrun vision_node vision_node
 *
 * All images are saved in /home/simplehands/Desktop/images.
 *
 * This system can handle 5 commands to it:
 *
 * 1. Capture an image - the system returns the filename it was saved as,
 *    so that the intelligence system can ask about this image later.
 * 2. Determine how many markers that are grasped by the hand, 
 *    along with their pose estimates.
 * 3. Calibrate vision - the system takes a picture of what it assumes to
 *    be an empty image, which it uses to calibrate color levels.
 * 4. Detects whether or not a marker has been placed properly
 * 5. Lets user Ping vision to show it is up and running
 *
 * @author Alex Zirbel
 * @author Robbie Paolini
 */

#include "vision_node.hpp"

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

VisionNode::VisionNode(ros::NodeHandle * n) : it_(*n)
{
  node = n;
}

VisionNode::~VisionNode()
{
}

/**
 * Attempts to save a text file in the current folder's /images directory
 * to make sure write permissions work
 */
void checkWritePermissions()
{
  // Change the path of the program to be the desktop
  char newPath[1000];
  char* homeDir = getenv("HOME");
  snprintf(newPath, sizeof(char)*1000, "%s/Desktop", homeDir);

  if (chdir(newPath) != 0)
  {
    printf("Failed to change directory to desktop\n");
    exit(EXIT_FAILURE);
  }

  // Since this application saves images in the current folder
  // we must ensure that we have permission to write to this folder.
  // If we do not have permission, fail right away.
  FILE* tempFile = fopen("images/test.txt", "w+");
  if (tempFile == NULL)
  {
    ROS_ERROR("Failed to create file in images subdirectory of "
        "current folder.  Please check permissions.");
    exit(EXIT_FAILURE);
  }
  fclose(tempFile);
  remove("images/test.txt");

  // Make folders that will be used
  mkdir("images/0/", 0777);
  mkdir("images/1/", 0777);
  mkdir("images/cal/", 0777);

  // Now move into the images directory
//  if (chdir("images") != 0)
//  {
//    printf("Failed to change directory to images\n");
//    exit(EXIT_FAILURE);
//  }
}

/**
 * Capture an Image from 'cameraNum' and save it to 'filename'
 * 
 * Saves the current time and sets class variables waiting for
 *  a call back to occur. Once the call back confirms that
 *  it has saved an image, we're done.
 */
bool VisionNode::captureImage(int cameraNum, char* filename)
{
  ros::Rate check_rate(CHECK_CAP_FREQ);

  // Remember the file name so the callback can save to it
  memcpy(saveFileName, filename, sizeof(char) * MAX_BUFFER);

  // Remember the requested time
  curTime = ros::Time::now().toSec();

  if (cameraNum == PLACE_DETECT)
  {
    // Unsubscribe and resubscribe to try and get a new image
    place_sub_.shutdown();
    place_sub_ = it_.subscribe("/vis_place/camera/image_raw", 1, 
        &VisionNode::placeCb, this);

    // Now set a flag and wait for the call back
    savePlaceImage = true;
    while (savePlaceImage)
    {
      check_rate.sleep();
    }
  }
  else if (cameraNum == GRASP_DETECT)
  {
    // Unsubscribe and resubscribe to try and get a new image
    hand_sub_.shutdown();
    hand_sub_ = it_.subscribe("/vis_hand/camera/image_raw", 1, 
        &VisionNode::handCb, this);

    // Now set a flag and wait for the call back
    saveHandImage = true;
    while (saveHandImage)
    {
      check_rate.sleep();
    }
  }
  else
  {
    return false;
  }

  return true;
}

/**
 * Callback that's called when we get a new image looking at the hand
 *
 * It updates that the camera has started, and only saves images to a file
 * when requested by captureImage
 */
void VisionNode::handCb(const sensor_msgs::ImageConstPtr& msg)
{
  // If we get a callback, we know that this camera1394 node is working
  if (!handCameraStarted)
    handCameraStarted = true;

  // Only save an image if requested by captureImage
  if (saveHandImage)
  {
    // Look at the message time stamp, and only save this image if it 
    // is later than the time a picture was requested
    double imgTime = msg->header.stamp.toSec();
    if (imgTime < curTime)
      return;

    // Copy the image from the image pipeline
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Now save the image to the requested file name
    imwrite( saveFileName, cv_ptr->image );

    // Tell captureImage that we're done
    saveHandImage = false;
  }
}

/**
 * Callback that's called when we get a new image looking at the placing loc
 *
 * It updates that the camera has started, and only saves images to a file
 * when requested by captureImage
 */
void VisionNode::placeCb(const sensor_msgs::ImageConstPtr& msg)
{
  // If we get a callback, we know that this camera1394 node is working
  if (!placeCameraStarted)
    placeCameraStarted = true;

  // Only save an image if requested by captureImage
  if (savePlaceImage)
  {
    // Look at the message time stamp, and only save this image if it
    // is later than the time a picture was requested
    double imgTime = msg->header.stamp.toSec();
    if (imgTime < curTime)
      return;

    // Copy the image from the image pipeline
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Now save the image to the requested file name
    imwrite(saveFileName, cv_ptr->image);
  
    // Tell captureImage that we're done
    savePlaceImage = false;
  }
}


/**
 * Vision Node Initialization
 *
 * First, checks write permissions to the folder it will use
 * Second, it makes sure the camera1394 node is initialized.
 * Finally, it advertises all of the services it offers.
 */
bool VisionNode::init()
{
  // Check write permissions before we do anything else
  checkWritePermissions();

  header = "vision";

  // Setup some picture taking state variables
  placeCameraStarted = false;
  handCameraStarted = false;
  saveHandImage = false;
  savePlaceImage = false;

  // Subscribe to the 2 camera1394 nodes
  hand_sub_ = it_.subscribe("/vis_hand/camera/image_raw", 1, 
      &VisionNode::handCb, this);
  place_sub_ = it_.subscribe("/vis_place/camera/image_raw", 1,
      &VisionNode::placeCb, this);

  // Wait to hear back from both cameras before continuing
  ROS_INFO("Waiting for cameras to initialize...");

  while (!placeCameraStarted || !handCameraStarted)
  {
    ros::spinOnce();
  }

  ROS_INFO("Cameras initialized");

  // Uncomment to show all debug output, including debug and labeled images.
  processor.setVerbose(1);

  // Advertise Services
  handle_calibrate_vision = 
    node->advertiseService("calibrate_vision", &VisionNode::calibrate_vision, this);
  handle_get_info = 
    node->advertiseService("get_info", &VisionNode::get_info, this);
  handle_capture_image = 
    node->advertiseService("capture_image", &VisionNode::capture_image, this);
  handle_ping_vision = 
    node->advertiseService("ping_vision", &VisionNode::ping_vision, this);
  handle_place_detect = 
    node->advertiseService("place_detect", &VisionNode::place_detect, this);
  handle_drop_detect = 
    node->advertiseService("drop_detect", &VisionNode::drop_detect, this);
  handle_insert_detect = 
    node->advertiseService("insert_detect", &VisionNode::insert_detect, this);
  return true;
}


/* 
 * Captures an Image from the requested camera and saves it to a file
 *
 * Choose the file name by getting the current date and time, and then
 * simply calls captureImage
 */
bool VisionNode::capture_image(vision_comm::CaptureImage::Request &req,
    vision_comm::CaptureImage::Response &res)
{
  ROS_INFO("Vision node: capturing image.");

  // send a filename
  char newFilename[MAX_BUFFER];
  time_t timer;
  timer=time(NULL);
  tm* today;
  today = localtime(&timer);
  sprintf(newFilename,"images/%d/%s__%d:%02d:%02d__%02d:%02d:%02d.png",
      (int)req.cameraNum,
      header.c_str(),
      today->tm_year+1900,
      today->tm_mon+1,
      today->tm_mday,
      today->tm_hour,
      today->tm_min,
      today->tm_sec);	

  captureImage(req.cameraNum, newFilename);

  res.filename = newFilename;

  ROS_INFO("Successfully captured image.");
  ROS_INFO("Vision node: ready to process images.");

  return true;
}


/**
 *  Calibrates the vision processor/system either according to the filename
 * passed in, or by capturing a new image
 */
bool VisionNode::calibrate_vision(vision_comm::CalibrateVision::Request &req,
    vision_comm::CalibrateVision::Response &res)
{
  ROS_INFO("Vision node: calibrating.");

  // If a new image is requested, take a picture and save it
  if(req.captureNew)
  {
    ROS_INFO("Capturing new image...");

    // Save the calibration image in the images folder
    char filename[MAX_BUFFER];
    sprintf(filename, "images/cal/%s", req.calibrationFilename.c_str());

    // Now capture an image from the requested camera
    captureImage(req.cameraNum, filename);

    // Set the appropriate processor's calibration image, and we're done!
    if (req.cameraNum == PLACE_DETECT)
    {
      placeD.setCalibrationImage(filename);
      res.response = "OK";
      ROS_INFO("Successful calibration to a new image.");
    }
    else if (req.cameraNum == GRASP_DETECT)
    {
      processor.setCalibrationImage(filename);
      res.response = "OK";
      ROS_INFO("Successful calibration to a new image.");
    }
    else
    {
      res.response = "ERROR: Invalid Camera Number";
      ROS_WARN("ERROR: Invalid Camera Number");
    }
  }
  else
  {
    // Otherwise, simply use an existing image
    ROS_INFO("Using exsting image: %s", req.calibrationFilename.c_str());

    // The calibration image should be in the images folder
    char filename[MAX_BUFFER];
    sprintf(filename, "images/%s", req.calibrationFilename.c_str());

    // Now simply set the appropriate processor's 
    //  calibration image, and we're done!
    if (req.cameraNum == PLACE_DETECT)
    {
      placeD.setCalibrationImage(filename);
      res.response = "OK";
      ROS_INFO("Successful calibration from specified file.");
    }
    else if (req.cameraNum == GRASP_DETECT)
    {
      processor.setCalibrationImage(filename);
      res.response = "OK";
      ROS_INFO("Successful calibration from specified file.");
    }
    else
    {
      res.response = "ERROR: Invalid Camera Number";
      ROS_WARN("ERROR: Invalid Camera Number");
    }
  }

  ROS_INFO("Vision node: ready to process images.");
  return 1;
}

/**
 * Computes the number of markers and their orientations in the hand. Note
 * that the pose of the returned marker is in tool frame coordinates, which is:
 * z towards camera, x to the right, y upwards.
 * 
 * Takes in a file name, and returns the pose of the marker with respect
 * to the hand.
 */
bool VisionNode::get_info(vision_comm::GetInfo::Request &req,
    vision_comm::GetInfo::Response &res)
{
  ROS_INFO("Vision node: processing image...");

  // Feed the file into our processor 
  OutputData *data = processor.processMarkers((char*)req.filename.c_str());
  int num_markers = data->numMarkers;

  // If we get a negative number of markers, something serious went wrong
  if(num_markers < 0)
  {
    ROS_ERROR("Error processing the image. "
        "Check filename and write permissions.");
  }

  // Save the number of markers and the marker length
  res.num_markers = num_markers;
  res.distance = (double)MARKER_LENGTH_MM;

  // Now, for each marker, save the pose information
  for(int i = 0; i < num_markers; i++)
  {
    res.certainty[i] = data->certainty[i];
    res.posx[i] = data->posx[i];
    res.posy[i] = data->posy[i];
    res.theta[i] = data->theta[i];
    res.alpha[i] = data->alpha[i];
    res.r[i] = data->r[i];
  }

  ROS_INFO("Vision node: ready to process images.");

  return 1;
}

/*
 * Ping Vision simply returns true, since the node is running.
 */
bool VisionNode::ping_vision(vision_comm::PingVision::Request &req,
    vision_comm::PingVision::Response &res)
{
  ROS_INFO("Vision Ping Received");
  return true;
}

/*
 * Detect whether a marker has been placed, 
 *  simply by calling a the place detector
 */
bool VisionNode::place_detect(vision_comm::PlaceDetect::Request &req,
    vision_comm::PlaceDetect::Response &res)
{
  ROS_INFO("Vision Node: Detecting whether placing was successful...");

  res.success = placeD.detectMarker((char*)req.filename.c_str());

  ROS_INFO("Vision Node: Ready to process images.");

  return true;
}

/*
 * Detect whether a marker has been dropped, 
 *  simply by calling a the place detector
 */
bool VisionNode::drop_detect(vision_comm::DropDetect::Request &req,
    vision_comm::DropDetect::Response &res)
{
  ROS_INFO("Vision Node: Detecting whether dropping was successful...");

  res.success = placeD.detectDrop((char*)req.filename.c_str());

  ROS_INFO("Vision Node: Ready to process images.");

  return true;
}

/*
 * Detect whether a marker has been inserted, 
 *  simply by calling a the place detector
 */
bool VisionNode::insert_detect(vision_comm::InsertDetect::Request &req,
    vision_comm::InsertDetect::Response &res)
{
  ROS_INFO("Vision Node: Detecting whether insertion was successful...");

  res.success = placeD.detectInsert((char*)req.filename.c_str());

  ROS_INFO("Vision Node: Ready to process images.");

  return true;
}



/** 
 * @brief Starts the vision node
 *
 * All the images are put into the same /images
 * folder in the directory the program is run from.
 */
int main(int argc, char **argv)
{
  cout << "Vision node connecting to master..." << endl;

  // Set up the ROS node
  ros::init(argc, argv, "vision_node");

  // Save the node handle
  ros::NodeHandle node;
  VisionNode vision(&node);

  // Initialize the vision node and connect to the cameras
  if(!vision.init())
  {
    ROS_ERROR("Problem initializing vision node.");
    exit(EXIT_FAILURE);
  }

  // Need a multi-thread spinner, because capture images has to wait for 
  // a callback to change a class variable
  ROS_INFO("Vision node: ready to process images.");
  ros::MultiThreadedSpinner spinner(2); // 2 "threads"
  spinner.spin();

  return 0;
}
