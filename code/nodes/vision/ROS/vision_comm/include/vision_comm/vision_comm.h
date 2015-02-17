#ifndef VISION_COMM_H
#define VISION_COMM_H

#include <ros/ros.h>
#include <string>

#include <vision_comm/PingVision.h>
#include <vision_comm/CalibrateVision.h>
#include <vision_comm/CaptureImage.h>
#include <vision_comm/GetInfo.h>
#include <vision_comm/PlaceDetect.h>
#include <vision_comm/DropDetect.h>
#include <vision_comm/InsertDetect.h>

// Camera numbers
#define PLACE_DETECT 1
#define GRASP_DETECT 0

// Number of markers we will recognize
#define MAX_MARKERS 4

using namespace std;


class VisionComm
{
  public:
    VisionComm();
    VisionComm(ros::NodeHandle* np);
    ~VisionComm();

    // Subscribe Function
    void subscribe(ros::NodeHandle* np);

    // Shuts down service clients
    void shutdown();

    bool Ping();
    bool Calibrate(bool capNew, int camNum, string calFile);
    bool CaptureImage(int camNum, string &filename);
    bool CaptureImage(int camNum);
    bool GetInfo(string fileName, int &num_markers, 
        double certainty[MAX_MARKERS], double &distance, double posx[MAX_MARKERS], 
        double posy[MAX_MARKERS], double theta[MAX_MARKERS], 
        double alpha[MAX_MARKERS], double r[MAX_MARKERS]);
    bool GetInfo(string fileName, int &num_markers, double &certainty, 
        double &distance, double &posx, double &posy, double &theta, 
        double &alpha, double &r);
    bool PlaceDetect(string fileName, bool &success);
    bool DropDetect(string fileName, bool &success);
    bool InsertDetect(string fileName, bool &success);

  private:
    // ROS Service Clients
    ros::ServiceClient handle_vision_PingVision;
    ros::ServiceClient handle_vision_CalibrateVision;
    ros::ServiceClient handle_vision_CaptureImage;
    ros::ServiceClient handle_vision_GetInfo;
    ros::ServiceClient handle_vision_PlaceDetect;
    ros::ServiceClient handle_vision_DropDetect;
    ros::ServiceClient handle_vision_InsertDetect;

    // ROS services
    vision_comm::PingVision vision_PingVision_srv;
    vision_comm::CalibrateVision vision_CalibrateVision_srv;
    vision_comm::CaptureImage vision_CaptureImage_srv;
    vision_comm::GetInfo vision_GetInfo_srv;
    vision_comm::PlaceDetect vision_PlaceDetect_srv;
    vision_comm::DropDetect vision_DropDetect_srv;
    vision_comm::InsertDetect vision_InsertDetect_srv;
};

#endif //VISION_COMM_H
