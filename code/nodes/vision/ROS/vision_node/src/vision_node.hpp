/**
 * @file vision_node.hpp
 *
 * Header file for the vision node
 * @author Alex Zirbel
 */

#ifndef VISION_NODE_H
#define VISION_NODE_H

// ROS includes, messages and services needed
#include <ros/ros.h>
#include <vision_comm/vision_comm.h>
#include <std_msgs/String.h>
//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

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

// Helper programs
#include "place_detect.hpp"
#include "processor.hpp"

#define CHECK_CAP_FREQ 25.0
#define MAX_BUFFER 256

class VisionNode
{
    public:
        VisionNode(ros::NodeHandle * n);
        virtual ~VisionNode();
        bool init();

        bool capture_image(vision_comm::CaptureImage::Request &req,
                vision_comm::CaptureImage::Response &res);
        bool calibrate_vision(vision_comm::CalibrateVision::Request &req,
                vision_comm::CalibrateVision::Response &res);
        bool get_info(vision_comm::GetInfo::Request &req,
                vision_comm::GetInfo::Response &res);
        bool ping_vision(vision_comm::PingVision::Request &req,
                vision_comm::PingVision::Response &res);
        bool place_detect(vision_comm::PlaceDetect::Request &req,
                vision_comm::PlaceDetect::Response &res);
        bool drop_detect(vision_comm::DropDetect::Request &req,
                vision_comm::DropDetect::Response &res);
        bool insert_detect(vision_comm::InsertDetect::Request &req,
                vision_comm::InsertDetect::Response &res);

        void handCb(const sensor_msgs::ImageConstPtr& msg);
        void placeCb(const sensor_msgs::ImageConstPtr& msg);

    private:
        bool captureImage(int cameraNum, char* filename);

        ros::ServiceServer handle_calibrate_vision;
        ros::ServiceServer handle_get_info;
        ros::ServiceServer handle_capture_image;
        ros::ServiceServer handle_ping_vision;
        ros::ServiceServer handle_place_detect;
        ros::ServiceServer handle_drop_detect;
        ros::ServiceServer handle_insert_detect;

        string header;

        Processor processor;
        PlaceDetect placeD;

        bool placeCameraStarted;
        bool handCameraStarted;
        bool savePlaceImage;
        bool saveHandImage;
        char saveFileName[MAX_BUFFER];

        double curTime;

        ros::NodeHandle *node;
        image_transport::ImageTransport it_;
        image_transport::Subscriber hand_sub_;
        image_transport::Subscriber place_sub_;
};

#endif
