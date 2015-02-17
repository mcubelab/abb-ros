#include "vision_comm.h"

VisionComm::VisionComm()
{
}

VisionComm::VisionComm(ros::NodeHandle* np)
{
  subscribe(np);
}

VisionComm::~VisionComm()
{
  shutdown();
}

void VisionComm::subscribe(ros::NodeHandle* np)
{
  handle_vision_PingVision = 
    np->serviceClient<vision_comm::PingVision>("ping_vision");
  handle_vision_CalibrateVision = 
    np->serviceClient<vision_comm::CalibrateVision>("calibrate_vision");
  handle_vision_CaptureImage = 
    np->serviceClient<vision_comm::CaptureImage>("capture_image");
  handle_vision_GetInfo = 
    np->serviceClient<vision_comm::GetInfo>("get_info");
  handle_vision_PlaceDetect = 
    np->serviceClient<vision_comm::PlaceDetect>("place_detect");
  handle_vision_DropDetect = 
    np->serviceClient<vision_comm::DropDetect>("drop_detect");
  handle_vision_InsertDetect = 
    np->serviceClient<vision_comm::InsertDetect>("insert_detect");
}

void VisionComm::shutdown()
{
  handle_vision_PingVision.shutdown();
  handle_vision_CalibrateVision.shutdown();
  handle_vision_CaptureImage.shutdown();
  handle_vision_GetInfo.shutdown();
  handle_vision_PlaceDetect.shutdown();
  handle_vision_DropDetect.shutdown();
  handle_vision_InsertDetect.shutdown();
}

bool VisionComm::Ping()
{
  return handle_vision_PingVision.call(vision_PingVision_srv);
}

bool VisionComm::Calibrate(bool capNew, int camNum, string calFile)
{
  bool ret;
  vision_CalibrateVision_srv.request.captureNew = capNew;
  vision_CalibrateVision_srv.request.cameraNum = camNum;
  vision_CalibrateVision_srv.request.calibrationFilename = calFile;
  ret = handle_vision_CalibrateVision.call(vision_CalibrateVision_srv);
  vision_CalibrateVision_srv.request.calibrationFilename.clear();
  return ret;
}

bool VisionComm::CaptureImage(int camNum, string &filename)
{
  vision_CaptureImage_srv.request.cameraNum = camNum;
  if (handle_vision_CaptureImage.call(vision_CaptureImage_srv))
  {
    filename = vision_CaptureImage_srv.response.filename;
    return true;
  }
  return false;
}

bool VisionComm::CaptureImage(int camNum)
{
  vision_CaptureImage_srv.request.cameraNum = camNum;
  return handle_vision_CaptureImage.call(vision_CaptureImage_srv);
}

bool VisionComm::GetInfo(string fileName, int &num_markers, 
    double certainty[MAX_MARKERS], double &distance, double posx[MAX_MARKERS], 
    double posy[MAX_MARKERS], double theta[MAX_MARKERS], 
    double alpha[MAX_MARKERS], double r[MAX_MARKERS])
{
  vision_GetInfo_srv.request.filename = fileName;
  if (handle_vision_GetInfo.call(vision_GetInfo_srv))
  {
    vision_GetInfo_srv.request.filename.clear();
    num_markers = vision_GetInfo_srv.response.num_markers;
    distance = vision_GetInfo_srv.response.distance;
    for (int i=0; i<num_markers; i++)
    {
      certainty[i] = vision_GetInfo_srv.response.certainty[i];
      posx[i] = vision_GetInfo_srv.response.posx[i];
      posy[i] = vision_GetInfo_srv.response.posy[i];
      theta[i] = vision_GetInfo_srv.response.theta[i];
      alpha[i] = vision_GetInfo_srv.response.alpha[i];
      r[i] = vision_GetInfo_srv.response.r[i];
    }
    return true;
  }
  else
  {
    vision_GetInfo_srv.request.filename.clear();
    return false;
  }
}

bool VisionComm::GetInfo(string fileName, int &num_markers, double &certainty, 
    double &distance, double &posx, double &posy, double &theta, 
    double &alpha, double &r)
{
  vision_GetInfo_srv.request.filename = fileName;
  if (handle_vision_GetInfo.call(vision_GetInfo_srv))
  {
    vision_GetInfo_srv.request.filename.clear();
    if (vision_GetInfo_srv.response.num_markers == 1)
    {
      num_markers = 1;
      certainty = vision_GetInfo_srv.response.certainty[0];
      distance = vision_GetInfo_srv.response.distance;
      posx = vision_GetInfo_srv.response.posx[0];
      posy = vision_GetInfo_srv.response.posy[0];
      theta = vision_GetInfo_srv.response.theta[0];
      alpha = vision_GetInfo_srv.response.alpha[0];
      r = vision_GetInfo_srv.response.r[0];
    }
    else
      {
	num_markers = vision_GetInfo_srv.response.num_markers;
	certainty = vision_GetInfo_srv.response.certainty[0];
      }
    return true;
  }

  vision_GetInfo_srv.request.filename.clear();
  return false;
}

bool VisionComm::PlaceDetect(string fileName, bool &success)
{
    vision_PlaceDetect_srv.request.filename = fileName;
    if (handle_vision_PlaceDetect.call(vision_PlaceDetect_srv))
    {
        vision_PlaceDetect_srv.request.filename.clear();
        success = vision_PlaceDetect_srv.response.success;
        return true;
    }

    success = false;
    vision_PlaceDetect_srv.request.filename.clear();
    return false;
}

bool VisionComm::DropDetect(string fileName, bool &success)
{
    vision_DropDetect_srv.request.filename = fileName;
    if (handle_vision_DropDetect.call(vision_DropDetect_srv))
    {
        vision_DropDetect_srv.request.filename.clear();
        success = vision_DropDetect_srv.response.success;
        return true;
    }

    success = false;
    vision_DropDetect_srv.request.filename.clear();
    return false;
}

bool VisionComm::InsertDetect(string fileName, bool &success)
{
    vision_InsertDetect_srv.request.filename = fileName;
    if (handle_vision_InsertDetect.call(vision_InsertDetect_srv))
    {
        vision_InsertDetect_srv.request.filename.clear();
        success = vision_InsertDetect_srv.response.success;
        return true;
    }

    success = false;
    vision_InsertDetect_srv.request.filename.clear();
    return false;
}
