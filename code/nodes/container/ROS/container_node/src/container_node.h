/** 
 * @file container_node.h
 * 
 * Header file for the container node.
 * @author Nick Stanley
 */

#ifndef container_node
#define container_node

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <container_comm/container_comm.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <tf/transform_datatypes.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

// Standard C++ includes
#include <iostream>
#include <string>
#include <fstream>
#include <math.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <ctype.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

#define CHECK_CAP_FREQ 25.0
#define MAX_BUFFER 256

static bool pointsInitialized;
static bool imageInitialized;

class ContainerNode
{
  public:
    ContainerNode(ros::NodeHandle * n);
    virtual ~ContainerNode();
    bool init();

    bool capture_image(container_comm::container_capture_image::Request&,
        container_comm::container_capture_image::Response&);
    bool ping_container(container_comm::container_Ping::Request&,
        container_comm::container_Ping::Response&);
    bool get_average_height(container_comm::container_get_average_height::Request&, container_comm::container_get_average_height::Response&);
    bool transform_points(container_comm::container_transform_points::Request&, 
        container_comm::container_transform_points::Response&);

  private:

    ros::ServiceServer handle_capture_image;
    ros::ServiceServer handle_ping_container;
    ros::ServiceServer handle_get_average_height;
    ros::ServiceServer handle_transform_points;

    string header;

    ros::NodeHandle * node;

};

#endif
