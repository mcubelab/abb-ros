#include "handRec_comm.h"

// Simple constructors / destructor
HandRecComm::HandRecComm()
{
}

HandRecComm::HandRecComm(ros::NodeHandle* np)
{
  subscribe(np);
}
HandRecComm::~HandRecComm()
{
  shutdown();
}

// Function to subscribe to ROS services
void HandRecComm::subscribe(ros::NodeHandle* np)
{
  handle_handRec_GetPose = np->serviceClient<handRec_comm::handRec_GetPose>("handRec_GetPose");
  handle_handRec_GetPoseSpec = np->serviceClient<handRec_comm::handRec_GetPoseSpec>("handRec_GetPoseSpec");
  handle_handRec_SetPrefOrient = np->serviceClient<handRec_comm::handRec_SetPrefOrient>("handRec_SetPrefOrient");
  handle_handRec_SetGuess = np->serviceClient<handRec_comm::handRec_SetGuess>("handRec_SetGuess");
  handle_handRec_SetObject = np->serviceClient<handRec_comm::handRec_SetObject>("handRec_SetObject");
}

// Shutdown service clients
void HandRecComm::shutdown()
{
  handle_handRec_GetPose.shutdown();
  handle_handRec_GetPoseSpec.shutdown();
  handle_handRec_SetPrefOrient.shutdown();
  handle_handRec_SetGuess.shutdown();
  handle_handRec_SetObject.shutdown();
}


// Services 
bool HandRecComm::GetPose(int &obj, HomogTransf &handPose)
{
  std::vector<std::string> filenames;
  return GetPose(obj, handPose, filenames);  
}


bool HandRecComm::GetPose(int &obj, HomogTransf &handPose, std::vector<std::string> &filenames)
{
  double pose[7];
  bool success = GetPose(obj, pose, filenames);  
  handPose.setPose(pose);
  return success;
}

bool HandRecComm::GetPose(int &obj, double trans[3], double quat[4])
{
  double pose[7];
  bool success = GetPose(obj, pose);
  memcpy(trans, pose, sizeof(double)*3);
  memcpy(quat, pose+3, sizeof(double)*4);
  return success;
}

bool HandRecComm::GetPose(int &obj, Vec &trans, Quaternion &quat)
{
  double pose[7];
  bool success = GetPose(obj, pose);
  trans = Vec(pose, 3);
  quat = Quaternion(pose+3);
  return success;
}


bool HandRecComm::GetPose(int &obj, double pose[7])
{
  std::vector<std::string> filenames;
  return GetPose(obj, pose, filenames);
}


bool HandRecComm::GetPose(int &obj, double pose[7], std::vector<std::string> &filenames)
{
  bool success = handle_handRec_GetPose.call(handRec_GetPose_srv);
  geometry_msgs::Pose p = handRec_GetPose_srv.response.result.pose;

  pose[0] = p.position.x;
  pose[1] = p.position.y;
  pose[2] = p.position.z;
  pose[3] = p.orientation.w;
  pose[4] = p.orientation.x;
  pose[5] = p.orientation.y;
  pose[6] = p.orientation.z;

  obj = handRec_GetPose_srv.response.objNum;

  filenames = handRec_GetPose_srv.response.filenames;

  return success;
}


bool HandRecComm::GetPoseSpec(int &obj, HomogTransf &handPose, const std::vector<HomogTransf> poses, std::vector<std::string> &filenames)
{
  // Resize our pose array
  handRec_GetPoseSpec_srv.request.poseArray.poses.resize(poses.size());

  for (size_t i = 0; i < poses.size(); i++)
  {
    Quaternion quat = poses[i].getQuaternion();
    Vec trans = poses[i].getTranslation();

    handRec_GetPoseSpec_srv.request.poseArray.poses[i].position.x = trans[0];
    handRec_GetPoseSpec_srv.request.poseArray.poses[i].position.y = trans[1];
    handRec_GetPoseSpec_srv.request.poseArray.poses[i].position.z = trans[2];
    handRec_GetPoseSpec_srv.request.poseArray.poses[i].orientation.w = quat[0];
    handRec_GetPoseSpec_srv.request.poseArray.poses[i].orientation.x = quat[1];
    handRec_GetPoseSpec_srv.request.poseArray.poses[i].orientation.y = quat[2];
    handRec_GetPoseSpec_srv.request.poseArray.poses[i].orientation.z = quat[3];
  }
  handRec_GetPoseSpec_srv.request.poseArray.header.frame_id = "/wobj";

  
  bool success = handle_handRec_GetPoseSpec.call(handRec_GetPoseSpec_srv);
  geometry_msgs::Pose p = handRec_GetPoseSpec_srv.response.result.pose;

  double pose[7];
  pose[0] = p.position.x;
  pose[1] = p.position.y;
  pose[2] = p.position.z;
  pose[3] = p.orientation.w;
  pose[4] = p.orientation.x;
  pose[5] = p.orientation.y;
  pose[6] = p.orientation.z;

  obj = handRec_GetPoseSpec_srv.response.objNum;

  filenames = handRec_GetPoseSpec_srv.response.filenames;

  handPose.setPose(pose);

  return success;
}

bool HandRecComm::SetPrefOrient(const Quaternion quat)
{
  handRec_SetPrefOrient_srv.request.use_pref_orient = true;
  for (int i=0; i < 4; i++)
    handRec_SetPrefOrient_srv.request.quat[i] = quat[i];
  return handle_handRec_SetPrefOrient.call(handRec_SetPrefOrient_srv);
}

bool HandRecComm::ClearPrefOrient()
{
  handRec_SetPrefOrient_srv.request.use_pref_orient = false;
  return handle_handRec_SetPrefOrient.call(handRec_SetPrefOrient_srv);
}

bool HandRecComm::SetGuess(const HomogTransf handPose)
{
  handRec_SetGuess_srv.request.use_guess = true;
  Vec trans = handPose.getTranslation();
  Quaternion quat = handPose.getQuaternion();
  handRec_SetGuess_srv.request.guess.pose.position.x = trans[0];
  handRec_SetGuess_srv.request.guess.pose.position.y = trans[1];
  handRec_SetGuess_srv.request.guess.pose.position.z = trans[2];
  handRec_SetGuess_srv.request.guess.pose.orientation.w = quat[0];
  handRec_SetGuess_srv.request.guess.pose.orientation.x = quat[1];
  handRec_SetGuess_srv.request.guess.pose.orientation.y = quat[2];
  handRec_SetGuess_srv.request.guess.pose.orientation.z = quat[3];
  handRec_SetGuess_srv.request.guess.header.frame_id = "/hand";
  return handle_handRec_SetGuess.call(handRec_SetGuess_srv);
}

bool HandRecComm::ClearGuess()
{
  handRec_SetGuess_srv.request.use_guess = false;
  return handle_handRec_SetGuess.call(handRec_SetGuess_srv);
}

bool HandRecComm::SetObject(const int obj)
{
  handRec_SetObject_srv.request.use_object = true;
  handRec_SetObject_srv.request.objNum = obj;
  return handle_handRec_SetObject.call(handRec_SetObject_srv);
}

bool HandRecComm::ClearObject()
{
  handRec_SetObject_srv.request.use_object = false;
  return handle_handRec_SetObject.call(handRec_SetObject_srv);
}


