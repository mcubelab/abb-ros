#include "handRec_node.h"

HandRecognition::HandRecognition()
{
}

HandRecognition::HandRecognition(ros::NodeHandle *n)
{
  init(n);
}

HandRecognition::~HandRecognition()
{
}




bool HandRecognition::handRec_GetPose(handRec_comm::handRec_GetPose::Request& req, 
        handRec_comm::handRec_GetPose::Response& res)
{
  int obj;
  HomogTransf result;
  std::vector<std::string> filenames(defaultRecPoses.size());
  bool success = getPose(defaultRecPoses, obj, result, filenames);

  Vec trans = result.getTranslation();
  Quaternion quat = result.getQuaternion();

  res.objNum = obj;
  res.result.header.frame_id = "/hand";
  res.result.pose.position.x = trans[0];
  res.result.pose.position.y = trans[1];
  res.result.pose.position.z = trans[2];
  res.result.pose.orientation.w = quat[0];
  res.result.pose.orientation.x = quat[1];
  res.result.pose.orientation.y = quat[2];
  res.result.pose.orientation.z = quat[3];
  res.filenames = filenames;

  if (success)
  {
    res.ret = 1;
    res.msg = "";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "Failed to get object pose in hand.";
    return false;
  }
}

bool HandRecognition::handRec_GetPoseSpec(handRec_comm::handRec_GetPoseSpec::Request& req, 
        handRec_comm::handRec_GetPoseSpec::Response& res)
{
  std::vector<HomogTransf> poses;
  int obj;
  HomogTransf result;
  std::vector<std::string> filenames;

  size_t num_poses = req.poseArray.poses.size();

  poses.resize(num_poses);
  filenames.resize(num_poses);

  for (size_t i = 0; i < num_poses; i++)
  {
    Vec t(3);
    Quaternion q;

    t[0] = req.poseArray.poses[i].position.x;
    t[1] = req.poseArray.poses[i].position.y;
    t[2] = req.poseArray.poses[i].position.z;
    q[0] = req.poseArray.poses[i].orientation.w;
    q[1] = req.poseArray.poses[i].orientation.x;
    q[2] = req.poseArray.poses[i].orientation.y;
    q[3] = req.poseArray.poses[i].orientation.z;
    poses[i] = HomogTransf(q,t);
  }

  bool success = getPose(poses, obj, result, filenames);

  Vec trans = result.getTranslation();
  Quaternion quat = result.getQuaternion();

  res.objNum = obj;
  res.result.header.frame_id = "/hand";
  res.result.pose.position.x = trans[0];
  res.result.pose.position.y = trans[1];
  res.result.pose.position.z = trans[2];
  res.result.pose.orientation.w = quat[0];
  res.result.pose.orientation.x = quat[1];
  res.result.pose.orientation.y = quat[2];
  res.result.pose.orientation.z = quat[3];
  res.filenames = filenames;

  if (success)
  {
    res.ret = 1;
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "Failed to get object pose in hand.";
    return false;
  }
}

bool HandRecognition::handRec_SetPrefOrient(handRec_comm::handRec_SetPrefOrient::Request& req, 
        handRec_comm::handRec_SetPrefOrient::Response& res)
{
  use_pref_orient = req.use_pref_orient;
  if(use_pref_orient)
  {
    for (int i=0; i < 4; i++)
      pref_orient[i] = req.quat[i];

  }
  else
  {
    objRec.ClearPrefOrient();
  }
  res.ret = 1;
  return true;
}

bool HandRecognition::handRec_SetGuess(handRec_comm::handRec_SetGuess::Request& req, 
        handRec_comm::handRec_SetGuess::Response& res)
{
  use_guess = req.use_guess;
  if(use_guess)
  {
    Vec trans(3);
    Quaternion quat;
    trans[0] = req.guess.pose.position.x;
    trans[1] = req.guess.pose.position.y;
    trans[2] = req.guess.pose.position.z;
    quat[0] = req.guess.pose.orientation.w;
    quat[1] = req.guess.pose.orientation.x;
    quat[2] = req.guess.pose.orientation.y;
    quat[3] = req.guess.pose.orientation.z;
    guessPose = HomogTransf(quat, trans);
  }
  else
  {
    objRec.ClearGuess();
  }
  res.ret = 1;
  return true;
}

bool HandRecognition::handRec_SetObject(handRec_comm::handRec_SetObject::Request& req, 
        handRec_comm::handRec_SetObject::Response& res)
{
  if (req.use_object)
  {
    bool success = objRec.SetObject(req.objNum);
    if (success)
    {
      res.ret = 1;
      return true;
    }
    else
    {
      res.ret = 0;
      res.msg = "Unable to set preferred object.";
      return false;
    }
  }
  else
  {
    objRec.ClearObject();
    return true;
  }
}



bool HandRecognition::getPose(const std::vector<HomogTransf> &poses, int &obj, HomogTransf &handPose, std::vector<std::string> &filenames)
{
  obj = -1;

  // Store the current sate of the robot so we can reset the state
  // afterwards
  double old_tcp, old_ori;
  int old_zone;
  HomogTransf oldTool, oldWorkObj;
  double old_joints[6];
  robot.GetState(old_tcp, old_ori, old_zone, oldWorkObj, oldTool);
  robot.GetJoints(old_joints);


  // TODO: For speed purposes, we will assume that the work object and tool
  // object are the default ones in robotParams.yaml. If this function
  // starts getting used in settings where the work or tool object changes,
  // then we will add code to this method to save and restore the work and
  // tool.

  if(!robot.SetSpeed(REC_TCP, REC_ORI))
    return false;
  if(!robot.SetZone(0))
    return false;

  size_t num_views = poses.size();
  size_t num_finger_boxes = fingerBoxOffsets.size();

  // First, let's move to each position and take a picture
  for (size_t i=0; i < num_views; ++i)
  {
    // TODO: HACK. Since we are dramatically changing the pose of the hand
    // in our last move, we will first do a joint move that gets us close
    // to the right place, and then slide over to the correct pose.
    if (i == num_views -1)
    {
      robot.SetJoints(objRecFinalJ);
    }

    std::cout << "moving to: " << std::endl;
    std::cout << poses[i] << std::endl;
    if (!robot.SetCartesianJ(poses[i]))
      return false;
    ros::Duration(0.5).sleep();
    if (!objRec.SavePoints(filenames[i], false, true, "sideKinect"))
      return false;
  }

  // Now that we're done taking pictures, restore our state
  robot.SetJoints(old_joints);
  robot.SetSpeed(old_tcp, old_ori);
  robot.SetZone(old_zone);



  // Now let's set up everything so we can recognize the object

  // First, we'll compute where each of the fingers are, and add them to
  // our ignore regions

  // Find
  double motorAngle;
  double fingerAngle[NUM_FINGERS];
  hand.GetAngles(motorAngle, fingerAngle);

  std::vector<HomogTransf> ignoreRegions(NUM_FINGERS * num_finger_boxes + 1);
  std::vector<double> ignoreWidths;
  ignoreWidths.reserve((NUM_FINGERS * num_finger_boxes + 1) * 3);
  for (int i=0; i < NUM_FINGERS; ++i)
  {
    HomogTransf fingerAxis = poses[0] * fingerAxes[i];
    RotMat angle;
    angle.rotZ(fingerAngle[i]*PI/180.0);
    HomogTransf rotatedFingerAxis = fingerAxis * HomogTransf(angle, Vec(0.0, 3));

    for (size_t j=0; j < num_finger_boxes; ++j)
    {
      ignoreRegions[num_finger_boxes*i + j] = rotatedFingerAxis * fingerBoxOffsets[j];
      ignoreWidths.insert(ignoreWidths.end(),fingerBoxWidths[j].begin(), fingerBoxWidths[j].end());
    }
  }
  ignoreRegions[NUM_FINGERS * num_finger_boxes] = poses[0] * palmBoxOffset;
  ignoreWidths.insert(ignoreWidths.end(), palmBoxWidths, palmBoxWidths+3);

  // Compute our overall box bound, which is just offset from the current
  // hand pose
  HomogTransf handBoxBounds = poses[0] * handBoxOffset;

  // Now that we have our bounds and ignore regions, lets convert
  // everything from mm to m
  Vec t = handBoxBounds.getTranslation();
  t /= 1000.0;
  handBoxBounds.setTranslation(t);

  for (size_t i=0; i < ignoreRegions.size(); i++)
  {
    t = ignoreRegions[i].getTranslation();
    t /= 1000.0;
    ignoreRegions[i].setTranslation(t);
  }

  // Now actually set up our bounds and ignore regions
  if (!objRec.SetBounds(handBoxWidths, handBoxBounds))
    return false;
  if (!objRec.SetIgnoreRegions(ignoreWidths, ignoreRegions))
    return false;

  // Now, if we had an inital guess and or preferred orientation, transform
  // them to our initial hand pose, and then send them to our object
  // recognition node
  if (use_pref_orient)
  {
    Quaternion pref_quat = poses[0].getQuaternion() ^ pref_orient;
    if (!objRec.SetPrefOrient(pref_quat))
      return false;
  }
  if (use_guess)
  {
    HomogTransf guess = poses[0] * guessPose;
    t = guess.getTranslation();
    t /= 1000.0;
    guess.setTranslation(t);
    if (!objRec.SetGuess(guess))
      return false;
  }

  // Now, let's save the poses we moved our hand to, and actually call the
  // recognition function
  std::vector<double> trans, quat;
  trans.reserve(3*num_views);
  quat.reserve(4*num_views);

  for (size_t i = 0; i < num_views; ++i)
  {
    Quaternion q = poses[i].getQuaternion();
    Vec t = poses[i].getTranslation() / 1000.0;

    trans.insert(trans.end(), t.v, t.v+3);
    quat.insert(quat.end(), q.v, q.v+4);
  }

  HomogTransf resPose;

  if (!objRec.GetObjFromViews(obj, resPose, filenames, trans, quat, "sideKinect"))
    return false;

  t = resPose.getTranslation();
  t *= 1000.0;
  resPose.setTranslation(t);

  handPose = poses[0].inv() * resPose;

  // Save the result of this view in a text file
  return true;
}




bool HandRecognition::init(ros::NodeHandle *n)
{
  // Remember the pointer to our node
  node = n;

  // Subscribe to object recognition, robot, and hand, which we'll be using
  objRec.subscribe(n);
  robot.subscribe(n);
  hand.subscribe(n);

  // Initially, we have no guess or preferred orientation
  use_pref_orient = false;
  use_guess = false;


  // Load in our parameters which give us boundary boxes and ignore regions
  // for our hand, along with the default poses we want to move the robot
  // to when taking images with the kinect. Note that we convert all of the
  // bound and ignore region widths to meters from mm, as this is what
  // objRec_node expects, and this way we don't have to convert it each
  // time.

  XmlRpc::XmlRpcValue my_list;
  XmlRpc::XmlRpcValue inner_list;
  double pose[7];

  n->getParam("/handRec/handBoxOffset", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(my_list.size() == 7);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    pose[i] = static_cast<double>(my_list[i]);
  }
  handBoxOffset = HomogTransf(pose);

  n->getParam("/handRec/handBoxWidths", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(my_list.size() == 3);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    handBoxWidths[i] = static_cast<double>(my_list[i]) / 1000.0;
  }

  n->getParam("/handRec/palmBoxOffset", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(my_list.size() == 7);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    pose[i] = static_cast<double>(my_list[i]);
  }
  palmBoxOffset = HomogTransf(pose);

  n->getParam("/handRec/palmBoxWidths", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(my_list.size() == 3);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    palmBoxWidths[i] = static_cast<double>(my_list[i]) / 1000.0;
  }

  fingerAxes.resize(NUM_FINGERS);
  n->getParam("/handRec/fingerAxes", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(my_list.size() == NUM_FINGERS);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    inner_list = my_list[i];
    ROS_ASSERT(inner_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t j = 0; j < inner_list.size(); ++j)
    {
      ROS_ASSERT(inner_list[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      pose[j] = static_cast<double>(inner_list[j]);
    }
    fingerAxes[i] = HomogTransf(pose);
  }

  n->getParam("/handRec/fingerBoxOffsets", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  fingerBoxOffsets.resize(my_list.size());
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    inner_list = my_list[i];
    ROS_ASSERT(inner_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t j = 0; j < inner_list.size(); ++j)
    {
      ROS_ASSERT(inner_list[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      pose[j] = static_cast<double>(inner_list[j]);
    }
    fingerBoxOffsets[i] = HomogTransf(pose);
  }

  n->getParam("/handRec/fingerBoxWidths", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT((size_t)my_list.size() == fingerBoxOffsets.size());
  fingerBoxWidths.resize(fingerBoxOffsets.size());
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    inner_list = my_list[i];
    ROS_ASSERT(inner_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(inner_list.size() == 3);
    fingerBoxWidths[i].resize(3);
    for (int32_t j = 0; j < inner_list.size(); ++j)
    {
      ROS_ASSERT(inner_list[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      fingerBoxWidths[i][j] = static_cast<double>(inner_list[j]) / 1000.0;
    }
  }


  n->getParam("/handRec/defaultRecPoses", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  defaultRecPoses.resize(my_list.size());
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    inner_list = my_list[i];
    ROS_ASSERT(inner_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t j = 0; j < inner_list.size(); ++j)
    {
      ROS_ASSERT(inner_list[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      pose[j] = static_cast<double>(inner_list[j]);
    }
    defaultRecPoses[i] = HomogTransf(pose);
  }



  // Advertise our services
  handle_handRec_GetPose = 
    node->advertiseService("handRec_GetPose", &HandRecognition::handRec_GetPose, this);
  handle_handRec_GetPoseSpec = 
    node->advertiseService("handRec_GetPoseSpec", &HandRecognition::handRec_GetPoseSpec, this);
  handle_handRec_SetPrefOrient = 
    node->advertiseService("handRec_SetPrefOrient", &HandRecognition::handRec_SetPrefOrient, this);
  handle_handRec_SetGuess = 
    node->advertiseService("handRec_SetGuess", &HandRecognition::handRec_SetGuess, this);
  handle_handRec_SetObject = 
    node->advertiseService("handRec_SetObject", &HandRecognition::handRec_SetObject, this);

  ROS_INFO("handRec_node initialized");

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "handRec_node");

  ros::NodeHandle node;
  HandRecognition handRec(&node);
  ros::spin();

  return 0;
}

