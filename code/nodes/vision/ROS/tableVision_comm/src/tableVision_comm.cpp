#include "tableVision_comm.h"

// Simple constructors / destructor
TableVisionComm::TableVisionComm()
{
}

TableVisionComm::TableVisionComm(ros::NodeHandle* np)
{
  subscribe(np);
}
TableVisionComm::~TableVisionComm()
{
  shutdown();
}

// Function to subscribe to ROS services
void TableVisionComm::subscribe(ros::NodeHandle* np)
{
  handle_tableVision_GetPose = np->serviceClient<tableVision_comm::tableVision_GetPose>("tableVision_GetPose");
  handle_tableVision_GetHandPose = np->serviceClient<tableVision_comm::tableVision_GetHandPose>("tableVision_GetHandPose");
  handle_tableVision_GetHandPoseSpec = np->serviceClient<tableVision_comm::tableVision_GetHandPoseSpec>("tableVision_GetHandPoseSpec");
  handle_tableVision_GetObjAndPose = np->serviceClient<tableVision_comm::tableVision_GetObjAndPose>("tableVision_GetObjAndPose");
  handle_tableVision_SavePoints = np->serviceClient<tableVision_comm::tableVision_SavePoints>("tableVision_SavePoints");
}

// Shutdown service clients
void TableVisionComm::shutdown()
{
  handle_tableVision_GetPose.shutdown();
  handle_tableVision_GetHandPose.shutdown();
  handle_tableVision_GetHandPoseSpec.shutdown();
  handle_tableVision_GetObjAndPose.shutdown();
  handle_tableVision_SavePoints.shutdown();
}


// Services 
bool TableVisionComm::GetPose(const int obj, HomogTransf &objPose)
{
  std::string filename;
  return GetPose(obj, objPose, filename);  
}


bool TableVisionComm::GetPose(const int obj, HomogTransf &objPose, std::string &filename)
{
  double pose[7];
  bool success = GetPose(obj, pose, filename);  
  objPose.setPose(pose);
  return success;
}

bool TableVisionComm::GetPose(const int obj, double trans[3], double quat[4])
{
  double pose[7];
  bool success = GetPose(obj, pose);
  memcpy(trans, pose, sizeof(double)*3);
  memcpy(quat, pose+3, sizeof(double)*4);
  return success;
}

bool TableVisionComm::GetPose(const int obj, Vec &trans, Quaternion &quat)
{
  double pose[7];
  bool success = GetPose(obj, pose);
  trans = Vec(pose, 3);
  quat = Quaternion(pose+3);
  return success;
}


bool TableVisionComm::GetPose(const int obj, double pose[7])
{
  std::string filename;
  return GetPose(obj, pose, filename);
}


bool TableVisionComm::GetPose(const int obj, double pose[7], std::string &filename)
{
  tableVision_GetPose_srv.request.objNum = obj;
  bool success = handle_tableVision_GetPose.call(tableVision_GetPose_srv);
  geometry_msgs::Pose p = tableVision_GetPose_srv.response.result.pose;

  pose[0] = p.position.x;
  pose[1] = p.position.y;
  pose[2] = p.position.z;
  pose[3] = p.orientation.w;
  pose[4] = p.orientation.x;
  pose[5] = p.orientation.y;
  pose[6] = p.orientation.z;

  filename = tableVision_GetPose_srv.response.filename;

  return (success && tableVision_GetPose_srv.response.ret);
}

bool TableVisionComm::GetObjAndPose(int &obj, HomogTransf &objPose)
{
  std::string filename;
  return GetObjAndPose(obj, objPose, filename);  
}

bool TableVisionComm::GetObjAndPose(int &obj, HomogTransf &objPose, std::string &filename)
{
  double pose[7];
  bool success = GetObjAndPose(obj, pose, filename);  
  objPose.setPose(pose);
  return success;
}

bool TableVisionComm::GetObjAndPose(int &obj, double trans[3], double quat[4])
{
  double pose[7];
  bool success = GetObjAndPose(obj, pose);
  memcpy(trans, pose, sizeof(double)*3);
  memcpy(quat, pose+3, sizeof(double)*4);
  return success;
}

bool TableVisionComm::GetObjAndPose(int &obj, Vec &trans, Quaternion &quat)
{
  double pose[7];
  bool success = GetObjAndPose(obj, pose);
  trans = Vec(pose, 3);
  quat = Quaternion(pose+3);
  return success;
}


bool TableVisionComm::GetObjAndPose(int &obj, double pose[7])
{
  std::string filename;
  return GetObjAndPose(obj, pose, filename);
}


bool TableVisionComm::GetObjAndPose(int &obj, double pose[7], std::string &filename)
{
  //  tableVision_GetObjAndPose_srv.request.needed_obj = nobj;
  bool success = handle_tableVision_GetObjAndPose.call(tableVision_GetObjAndPose_srv);
  geometry_msgs::Pose p = tableVision_GetObjAndPose_srv.response.result.pose;

  obj = tableVision_GetObjAndPose_srv.response.objNum;
  pose[0] = p.position.x;
  pose[1] = p.position.y;
  pose[2] = p.position.z;
  pose[3] = p.orientation.w;
  pose[4] = p.orientation.x;
  pose[5] = p.orientation.y;
  pose[6] = p.orientation.z;

  filename = tableVision_GetObjAndPose_srv.response.filename;

  return (success && tableVision_GetObjAndPose_srv.response.ret);
}


bool TableVisionComm::GetHandPose(const int obj, HomogTransf &pose)
{
  tableVision_GetHandPose_srv.request.objNum = obj;
  bool success = handle_tableVision_GetHandPose.call(tableVision_GetHandPose_srv);
  geometry_msgs::Pose p = tableVision_GetHandPose_srv.response.result.pose;
  double posei[7];
  std::string filename;
  posei[0] = p.position.x;
  posei[1] = p.position.y;
  posei[2] = p.position.z;
  posei[3] = p.orientation.w;
  posei[4] = p.orientation.x;
  posei[5] = p.orientation.y;
  posei[6] = p.orientation.z;
  pose.setPose(posei);

  filename = tableVision_GetHandPose_srv.response.filename;

  return (success && tableVision_GetHandPose_srv.response.ret);

}

bool TableVisionComm::GetHandPoseSpec(const int obj, const HomogTransf handPose, HomogTransf &pose)
{
  tableVision_GetHandPoseSpec_srv.request.objNum = obj;

  Vec t(3);
  Quaternion q;
  t = handPose.getTranslation();
  q = handPose.getQuaternion();
  tableVision_GetHandPoseSpec_srv.request.handPose.pose.position.x = t[0];
  tableVision_GetHandPoseSpec_srv.request.handPose.pose.position.y = t[1];
  tableVision_GetHandPoseSpec_srv.request.handPose.pose.position.z = t[2];
  tableVision_GetHandPoseSpec_srv.request.handPose.pose.orientation.w = q[0];
  tableVision_GetHandPoseSpec_srv.request.handPose.pose.orientation.x = q[1];
  tableVision_GetHandPoseSpec_srv.request.handPose.pose.orientation.y = q[2];
  tableVision_GetHandPoseSpec_srv.request.handPose.pose.orientation.z = q[3];

  bool success = handle_tableVision_GetHandPoseSpec.call(tableVision_GetHandPoseSpec_srv);
  geometry_msgs::Pose p = tableVision_GetHandPoseSpec_srv.response.result.pose;
  double posei[7];
  std::string filename;
  posei[0] = p.position.x;
  posei[1] = p.position.y;
  posei[2] = p.position.z;
  posei[3] = p.orientation.w;
  posei[4] = p.orientation.x;
  posei[5] = p.orientation.y;
  posei[6] = p.orientation.z;
  pose.setPose(posei);

  filename = tableVision_GetHandPose_srv.response.filename;

  return (success && tableVision_GetHandPose_srv.response.ret);

}



bool TableVisionComm::SavePoints(std::string &filename, const bool use_filter, 
        const bool is_binary, const std::string cameraName)
{
  tableVision_SavePoints_srv.request.use_filter = use_filter;
  tableVision_SavePoints_srv.request.is_binary = is_binary;
  tableVision_SavePoints_srv.request.cameraName = cameraName;
  bool success = handle_tableVision_SavePoints.call(tableVision_SavePoints_srv);
  filename = tableVision_SavePoints_srv.response.filename;
  return success;
}


