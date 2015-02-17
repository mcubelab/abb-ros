#ifndef CONTAINER_COMM_H
#define CONTAINER_COMM_H

#include <ros/ros.h>
#include <string>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <container_comm/container_height.h>
#include <container_comm/container_Ping.h>
#include <container_comm/container_SaveMatrix.h>
#include <container_comm/container_height_area.h>
#include <container_comm/container_get_average_height.h>
#include <container_comm/container_capture_image.h>
#include <container_comm/container_transform_points.h>

#define BIN_WIDTH 100
#define BIN_HEIGHT 100

using namespace std;

class ContainerComm
{
  public: 
    ContainerComm();
    ContainerComm(ros::NodeHandle * np);
    ~ContainerComm();

    // Subscribe Function
    void subscribe(ros::NodeHandle * np);

    // Shuts down service clients
    void shutdown();

    bool Ping();
    bool CaptureImage(string &filename);
    bool GetAverageHeight(double x, double y, double radius);
		bool GetAverageHeight(double x, double y, double radius, double &surfaceHeight); 
    bool TransformPoints();

  private: 

    // ROS Service Clients
    ros::ServiceClient handle_container_PingContainer;
    ros::ServiceClient handle_container_CaptureImage;
    ros::ServiceClient handle_container_GetAverageHeight;
    ros::ServiceClient handle_container_TransformPoints;

    // ROS Services
    container_comm::container_Ping container_PingContainer_srv;
    container_comm::container_capture_image container_CaptureImage_srv;
    container_comm::container_get_average_height container_GetAverageHeight_srv;
    container_comm::container_transform_points container_TransformPoints_srv;
};

#endif // CONTAINER_COMM_H
