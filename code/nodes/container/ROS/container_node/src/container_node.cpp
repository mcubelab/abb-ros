/*
 * Container Node
 * Nick Stanley
 * Carnegie Mellon University
 * June 2012
 * 
 * This was developed for the Simple Hands Manipulation Lab, so that the ABB arm 
 * could visualize and detect objects in a bin. It is coupled with the 
 * Container_Comm node, which makes high level communication easy.
 */

#include "container_node.h"

ros::Timer loggerTimer;
ros::Timer scannerTimer;
ros::Time logStartTime;

using namespace std;
using namespace cv;
using namespace message_filters;
namespace enc = sensor_msgs::image_encodings;

double requestedTime;
bool newPointCloud;
bool saveImage;
bool savePoints;
bool saveRobotPoints;
char imageFileName [MAX_BUFFER];
char pointsFileName [MAX_BUFFER];
char robotPointsFileName [MAX_BUFFER];

// Latest points in Kinect frame and robot frame, respectively. 
pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr robot_points (new pcl::PointCloud<pcl::PointXYZ>);

// MutEx for point cloud.
bool cloudlock = false;

btScalar qw; // = -0.019950;
btScalar qx; // =  0.999725;
btScalar qy; // =  0.011978;
btScalar qz; // =  0.001225;
btScalar tx; // =  0.6227053;
btScalar ty; // =  0.3806433;
btScalar tz; // =  1.1309637;

tf::Transform TWR;

// Constructor
ContainerNode::ContainerNode(ros::NodeHandle * n)
{
  node = n;
}

ContainerNode::~ContainerNode()
{
}

/** 
 * Checks to make sure that we have permissions for writing the images. This is 
 * important for image_capture.
 * 
 * Parameters: none
 * Returns: none (void)
 */
void checkWritePermissions(){

  // Change path of program to Desktop.
  char newPath[1000];
  char * homeDir = getenv("HOME");
  snprintf(newPath, sizeof(char) * 1000, "%s/Desktop", homeDir);

  if (chdir(newPath) != 0){
    printf("Failed to change directory to Desktop.\n");
    exit(EXIT_FAILURE);
  }

  // Create a file, with both reading and writing permissions.
  FILE * tempFile = fopen("container_images/test.txt", "w+");
  if (tempFile == NULL){
    ROS_ERROR("Failed to create file in images subdirectory of current folder. Please check permissions.");
    exit(EXIT_FAILURE);
  }
  fclose(tempFile);
  remove("images/test.txt");

  ROS_INFO("We have write permissions.");

}

/**
 * Points callback. Saves the points to file and stores them in a global variable.
 */
void points_callback(const sensor_msgs::PointCloud2ConstPtr & msg)
{
  // Makes sure the program knows that we're receiving points.
  pointsInitialized = true;

  // Turns on the MutEx lock.
  cloudlock = true;

  // Converts to PCL and lets the program know that we have a fresh point cloud.
  pcl::fromROSMsg (*msg, *latest_cloud);
  newPointCloud = true;

  // If we are requested to save points...
  if (savePoints)
  {
    // ...check to make sure that the time is alright.
    double pointTime = msg->header.stamp.toSec();
    if (pointTime < requestedTime)
    {
      ROS_INFO("Points: Wrong time. Returning.");
      return;
    }
    // Then convert the message to PCL format...
    pcl::PointCloud<pcl::PointXYZ>::Ptr points (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg ( *msg, *points);

    // ...make sure that it is valid...
    /* if (!points->is_dense)
    {
      ROS_ERROR("Did not save. Contained invalid points.");
      return;
    } */

    // ...and save it to file.
    if (pcl::io::savePCDFile( pointsFileName, *points) != 0)
    {
      ROS_ERROR("Error saving file.");
      return;
    }
    
    ROS_INFO("Written to %s", pointsFileName);

    savePoints = false;
  }
  cloudlock = false;
}

// Similar to points_callback. 
void image_callback(const sensor_msgs::ImageConstPtr & msg)
{
  imageInitialized = true;

  // If we are requested to save an image...
  if (saveImage)
  {
    // ...check to make sure that the time is alright.
    double imgTime = msg->header.stamp.toSec();
    if (imgTime < requestedTime)
    {
      ROS_INFO("Wrong time. Returning.");
      return;
    }
    // Then convert the image to an OpenCV format...
    cv_bridge::CvImagePtr cv_ptr;

    // ...make sure that it is valid...
    try
    {
      ROS_INFO("Converting image.");
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception & e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // ...and save it to file.
    try
    {
    ROS_INFO("Writing image...");
    imwrite (imageFileName, cv_ptr->image); 
    }
    catch (runtime_error & ex)
    {
      ROS_ERROR("Exception converting image to PNG: %s", ex.what());
      return;
    }
    ROS_INFO("Written to %s", imageFileName);

    saveImage = false;
  }
}

/**
 * Gets approximately the same images, then deals with and saves both of them.
 */

void synthesized_callback( 
    const sensor_msgs::ImageConstPtr & image_msg,
    const sensor_msgs::PointCloud2ConstPtr & points_msg)
{
  points_callback(points_msg);
  image_callback(image_msg);
}

/**
 * Initializes the container node.
 */
bool ContainerNode::init(){

  // 1/5 of a second.
  ros::Rate waitTime(5); 

  // Check permissions.
  checkWritePermissions();

  header = "container";

  // Setup some picture taking state variables.

  pointsInitialized = false;
  imageInitialized = false;
  savePoints = false;
  saveImage = false;

  ROS_INFO("Waiting for topics to initialize.");

  while (!pointsInitialized || !imageInitialized)
  {
    waitTime.sleep();
  }

  ROS_INFO("Topics initialized.");

  // Handles the functions.
  handle_capture_image = node->advertiseService("container_CaptureImage", &ContainerNode::capture_image, this);
  handle_ping_container = node->advertiseService("container_Ping", &ContainerNode::ping_container, this);
  handle_get_average_height = node->advertiseService("container_GetAverageHeight", &ContainerNode::get_average_height, this);
  handle_transform_points = node->advertiseService("container_TransformPoints", &ContainerNode::transform_points, this);

return true;
}

/**
 * Sets everything up to save an image during image_callback, called from 
 * synthesized_callback.
 */
bool ContainerNode::capture_image(
    container_comm::container_capture_image::Request &req, 
    container_comm::container_capture_image::Response &res)
{

  ROS_INFO("Container node: capturing image.");

  // 1/15 of a second.
  ros::Rate waitTime(15);
  
  // Save a filename.
  time_t timer;
  timer = time(NULL);
  tm* today;
  today = localtime(&timer);
  sprintf(imageFileName, "/home/simplehands/Desktop/container_images/images/%s__%d:%02d:%02d__%02d:%02d:%02d.png",
      header.c_str(),
      today->tm_year + 1900,
      today->tm_mon + 1,
      today->tm_mday,
      today->tm_hour, 
      today->tm_min,
      today->tm_sec );
  sprintf(pointsFileName, "/home/simplehands/Desktop/container_images/points/%s__%d:%02d:%02d__%02d:%02d:%02d.pcd",
      header.c_str(),
      today->tm_year + 1900,
      today->tm_mon + 1,
      today->tm_mday,
      today->tm_hour, 
      today->tm_min,
      today->tm_sec );
  sprintf(robotPointsFileName, "/home/simplehands/Desktop/container_images/robot_points/%s__%d:%02d:%02d__%02d:%02d:%02d.pcd",
      header.c_str(),
      today->tm_year + 1900,
      today->tm_mon + 1,
      today->tm_mday,
      today->tm_hour, 
      today->tm_min,
      today->tm_sec );

  // Makes sure we get a valid image.
  requestedTime = ros::Time::now().toSec();

  // Returns the image file name.
  res.filename = imageFileName;

  saveImage = true;
  savePoints = true;
	saveRobotPoints = true;

  ROS_INFO("Successfully captured image.");
  ROS_INFO("Container node: ready to process images.");

  return true;
}

// Simply pings the node.
bool ContainerNode::ping_container(container_comm::container_Ping::Request &req,
    container_comm::container_Ping::Response &res){

  ROS_INFO("Container Ping received.");
  return true;
}

pcl::PointXYZ getNearestPoint(float x, float y, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int numSamples)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud (cloud);

	pcl::PointXYZ samples [numSamples];
	float minDistance = 1; // 1 meter. 

	std::vector<int> searchIndices(1);
	std::vector<float> searchSquaredDistances(1);

	pcl::PointXYZ result;
	result.x = 0;
	result.y = 0;
	result.z = 0;

	for (int i = 0; i < numSamples; i++)
	{
	  samples[i].x = x;
	  samples[i].y = y;
	  samples[i].z = (200.0 * (1.0 - (float) (i) / (float) (numSamples))) / 1000.0;

		if (kdtree.nearestKSearch(samples[i], 1, searchIndices, searchSquaredDistances) > 0)
		{
			pcl::PointXYZ nearestPoint = cloud->points[searchIndices[0]];
			float xDiff = nearestPoint.x - samples[i].x;
			float yDiff = nearestPoint.y - samples[i].y;
			float distance = xDiff * xDiff + yDiff * yDiff; 
			if (distance < minDistance)
			{
			  minDistance = distance;
				result = nearestPoint;
			}
		}
	}

  return result;
}

bool ContainerNode::get_average_height(container_comm::container_get_average_height::Request &req, 
    container_comm::container_get_average_height::Response &res)
{

  float radius = req.radius / 1000.0; // Measured in meters here.
  req.x /= 1000.0; // Converting from mm to m.
  req.y /= 1000.0; 

  // Point cloud to be used.
  cloudlock = true;
  pcl::PointCloud<pcl::PointXYZ>::Ptr localCloud = robot_points;
  cloudlock = false;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud (localCloud);

	pcl::PointXYZ searchPoint;
	searchPoint.x = req.x;
	searchPoint.y = req.y;
	
	pcl::PointXYZ nearestPoint = getNearestPoint(req.x, req.y, localCloud, 5);
	searchPoint.z = nearestPoint.z;	// Neighbors in radius search

  	// Radius defined at the top

	std::vector<int> pointIndices;
	std::vector<float> squaredDistances;

	double totalHeight = 0;
	unsigned int numValidPoints = 0;
	int leftOut = 0;

	if (kdtree.radiusSearch(searchPoint, radius, pointIndices, squaredDistances) > 0)
	{
		for (unsigned int i = 0; i < pointIndices.size(); ++i)
		{
			pcl::PointXYZ currentPoint = localCloud->points[pointIndices[i]];
			// Check for NaN.
			if (currentPoint.x == currentPoint.x && 0.05 <= currentPoint.z) 
			{	
				numValidPoints++;
				totalHeight += currentPoint.z;
			}
			else if (leftOut < 10)
			{
				ROS_INFO("Left out point <%f, %f, %f>.", 
					currentPoint.x, currentPoint.y, currentPoint.z);
				leftOut++;
			}
		}
		ROS_INFO("TotalHeight is %f.", totalHeight);
		ROS_INFO("NumValidPoints is %d", numValidPoints);
		ROS_INFO("Size of array is %d.", pointIndices.size());

    res.numPoints = numValidPoints;
  	res.avgHeight = totalHeight * 1000.0 / ((float) res.numPoints);
		ROS_INFO("Average height was %f.", res.avgHeight);
		// Check for NaN in result.
		if (res.avgHeight != res.avgHeight)
		{
			ROS_INFO("Invalid result.");
			return false;
		}
		else
    	return true;
	}
	else 
	{
		ROS_INFO("Didn't find points in radius.");
		return false;
	}
}

/**
 * Service. Currently transforms points but the points are inaccessible. Was made primarily as a test. 
 */
bool ContainerNode::transform_points (container_comm::container_transform_points::Request & req, 
    container_comm::container_transform_points::Response & res) 
{
  // Using latest_cloud
  ROS_INFO("Transforming points...");
  cloudlock = true;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);
  pcl_ros::transformPointCloud(*latest_cloud, *target, TWR);
  cloudlock = false;
	ROS_INFO("Points successfully transformed.");
	return true;
}

// Internal function. Exactly the same as the service, except stores them in the second parameter (which is a pointer).
bool transform_cloud (pcl::PointCloud<pcl::PointXYZ> & source, 
	pcl::PointCloud<pcl::PointXYZ> & target)
{
  pcl_ros::transformPointCloud(source, target, TWR);	
	return true;
}

int main (int argc, char ** argv){

  cout << "Container node connecting to master... " << endl;

  // Set up the ROS node.
  ros::init(argc, argv, "container_node");

  // Save the node handle.
  ros::NodeHandle node;
  ContainerNode container(&node);

  // Initialize the container node and connect to the topics.

  message_filters::Subscriber<sensor_msgs::Image> image_sub_ (node, "/camera/rgb/image_color", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> point_sub_ (node, "/camera/depth/points", 1);

  // Publishes points in robot frame.
  ros::Publisher robot_points_pub = node.advertise<sensor_msgs::PointCloud2>("/container/robot_points", 1000);
    
  // Synchronized spinner, synchronizes the image/points callbacks.
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub_, point_sub_);
  sync.registerCallback(boost::bind(&synthesized_callback, _1, _2));
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Initializes container. 
  ros::Rate waitTime(0.5);
  if (!container.init()){
    ROS_ERROR("Problem initializing vision node.");
    exit(EXIT_FAILURE);
  }

  ROS_INFO("Container node: ready to process images.");
  
  // Wait for topic initialization.
  while (!pointsInitialized || !imageInitialized)
  {
    waitTime.sleep();
    waitTime.sleep();
  }

	ROS_INFO("Making transform...");

	// Load parameters.
	if (!node.getParam("/tx", tx)) ROS_ERROR("Did not get tx.");
	else ROS_INFO("tx = %f.", tx);
	if (!node.getParam("/ty", ty)) ROS_ERROR("Did not get ty.");
	else ROS_INFO("ty = %f.", ty);
	if (!node.getParam("/tz", tz)) ROS_ERROR("Did not get tz.");
	else ROS_INFO("tz = %f.", tz);
	if (!node.getParam("/qw", qw)) ROS_ERROR("Did not get qw.");
	else ROS_INFO("qw = %f.", qw);
	if (!node.getParam("/qx", qx)) ROS_ERROR("Did not get qx.");
	else ROS_INFO("qx = %f.", qx);
	if (!node.getParam("/qy", qy)) ROS_ERROR("Did not get qy.");
	else ROS_INFO("qy = %f.", qy);
	if (!node.getParam("/qz", qz)) ROS_ERROR("Did not get qz.");
	else ROS_INFO("qz = %f.", qz);

	// Create transform.
  tf::Quaternion q (qx, qy, qz, qw);
  tf::Vector3 t (tx, ty, tz);
  tf::Transform TWR (q, t);

  // Pointer for points to be published.
  pcl::PointCloud<pcl::PointXYZ>::Ptr pubPoints (new pcl::PointCloud<pcl::PointXYZ>);

  ROS_INFO("Publishing robot points.");

  while (ros::ok())
  {
    // Continuously transforms kinect frame to robot frame
    // and publishes resulting points.
    // Also stores them in robot_points.
    if (newPointCloud && !cloudlock)
    {
      pcl_ros::transformPointCloud(*latest_cloud, *pubPoints, TWR);	
      robot_points = pubPoints;
      robot_points_pub.publish(pubPoints);
			if (saveRobotPoints)
			{
    		// Save it to file.
		    if (pcl::io::savePCDFile( robotPointsFileName, *pubPoints) != 0)
		    {
		      ROS_ERROR("Error saving file.");
    		}
		    else 
				{
				ROS_INFO("Written to %s", robotPointsFileName);
    		saveRobotPoints = false;
				}
			}
      newPointCloud = false;
      ros::spinOnce();
      waitTime.sleep();
    }
  }

  return 0;

}
