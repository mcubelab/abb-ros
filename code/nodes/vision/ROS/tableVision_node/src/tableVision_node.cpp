#include "tableVision_node.h"

TableVision::TableVision()
{
}

TableVision::TableVision(ros::NodeHandle *n)
{
  init(n);
}

TableVision::~TableVision()
{
  serialPort.Close();
}



bool TableVision::tableVision_GetPose(tableVision_comm::tableVision_GetPose::Request& req, 
        tableVision_comm::tableVision_GetPose::Response& res)
{
  ros::Time begin = ros::Time::now();
  int obj;
  HomogTransf result;

  std::string filename;

  // Set our expected object
  objRec.SetObject(req.objNum);
  
  bool success = getTablePose(obj, result, filename);

  Vec trans = result.getTranslation();
  Quaternion quat = result.getQuaternion();

  res.result.header.frame_id = "/table";
  res.result.pose.position.x = trans[0];
  res.result.pose.position.y = trans[1];
  res.result.pose.position.z = trans[2];
  res.result.pose.orientation.w = quat[0];
  res.result.pose.orientation.x = quat[1];
  res.result.pose.orientation.y = quat[2];
  res.result.pose.orientation.z = quat[3];
  res.filename = filename;

  std::cout << "Time Elapsed: " << ros::Time::now() - begin << std::endl;

  if (success)
  {
    res.ret = 1;
    res.msg = "";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "Failed to get object pose on table.";
    return false;
  }
}

bool TableVision::tableVision_GetObjAndPose(tableVision_comm::tableVision_GetObjAndPose::Request& req, 
    tableVision_comm::tableVision_GetObjAndPose::Response& res)
{
  int obj;
  HomogTransf result;

  std::string filename;

  // Clear our initial guess for an object
  objRec.ClearObject();

  bool success = getTablePose(obj, result, filename);

  Vec trans = result.getTranslation();
  Quaternion quat = result.getQuaternion();

  res.objNum = obj;
  res.result.header.frame_id = "/table";
  res.result.pose.position.x = trans[0];
  res.result.pose.position.y = trans[1];
  res.result.pose.position.z = trans[2];
  res.result.pose.orientation.w = quat[0];
  res.result.pose.orientation.x = quat[1];
  res.result.pose.orientation.y = quat[2];
  res.result.pose.orientation.z = quat[3];
  res.filename = filename;

  if (success)
  {
    res.ret = 1;
    res.msg = "";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "Failed to get object pose on table.";
    return false;
  }
}



bool TableVision::tableVision_GetHandPose(tableVision_comm::tableVision_GetHandPose::Request& req, 
        tableVision_comm::tableVision_GetHandPose::Response& res)
{
  ros::Time begin = ros::Time::now();
  int obj;
  HomogTransf result;

  std::string filename;

  // Set our expected object
  objRec.SetObject(req.objNum);
  
  bool success = getHandPose(obj, result, filename, defaultHandPose);

  Vec trans = result.getTranslation();
  Quaternion quat = result.getQuaternion();

  res.result.header.frame_id = "/hand";
  res.result.pose.position.x = trans[0];
  res.result.pose.position.y = trans[1];
  res.result.pose.position.z = trans[2];
  res.result.pose.orientation.w = quat[0];
  res.result.pose.orientation.x = quat[1];
  res.result.pose.orientation.y = quat[2];
  res.result.pose.orientation.z = quat[3];
  res.filename = filename;

  std::cout << "Time Elapsed: " << ros::Time::now() - begin << std::endl;

  if (success)
  {
    res.ret = 1;
    res.msg = "";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "Failed to get object pose on table.";
    return false;
  }
}

bool TableVision::tableVision_GetHandPoseSpec(tableVision_comm::tableVision_GetHandPoseSpec::Request& req, 
        tableVision_comm::tableVision_GetHandPoseSpec::Response& res)
{
  ros::Time begin = ros::Time::now();
  int obj;
  HomogTransf result;

  std::string filename;

  // Set our expected object
  objRec.SetObject(req.objNum);

  HomogTransf handPose;
  Vec t(3);
  Quaternion q;

  t[0] = req.handPose.pose.position.x;
  t[1] = req.handPose.pose.position.y;
  t[2] = req.handPose.pose.position.z;
  q[0] = req.handPose.pose.orientation.w;
  q[1] = req.handPose.pose.orientation.x;
  q[2] = req.handPose.pose.orientation.y;
  q[3] = req.handPose.pose.orientation.z;
  handPose = HomogTransf(q,t);

  bool success = getHandPose(obj, result, filename, handPose);

  Vec trans = result.getTranslation();
  Quaternion quat = result.getQuaternion();

  res.result.header.frame_id = "/hand";
  res.result.pose.position.x = trans[0];
  res.result.pose.position.y = trans[1];
  res.result.pose.position.z = trans[2];
  res.result.pose.orientation.w = quat[0];
  res.result.pose.orientation.x = quat[1];
  res.result.pose.orientation.y = quat[2];
  res.result.pose.orientation.z = quat[3];
  res.filename = filename;

  std::cout << "Time Elapsed: " << ros::Time::now() - begin << std::endl;

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





bool TableVision::tableVision_SavePoints(tableVision_comm::tableVision_SavePoints::Request& req, 
    tableVision_comm::tableVision_SavePoints::Response& res)
{
  std::string filename;
  for (int i=0; i < numCameras; ++i)
  {
    if (cameraNames[i] == req.cameraName)
    {
      serialPort << (0x1 << servoNums[i]) << std::endl;
      ros::Duration(0.25).sleep();
      bool success = objRec.SavePoints(filename, req.use_filter, req.is_binary, cameraNames[i]);
      serialPort << 0x00 << std::endl;

      res.filename = filename;

      if (success)
      {
        res.ret = 1;
        return true;
      }
      else
      {
        res.ret = 0;
        res.msg = "Failed to save points from kinect.";
        return false;
      }
    }
  }

  res.ret = 0;
  res.msg = "Unrecognized camera name. Could not save points!";
  return false;
}

bool TableVision::getTablePose(int &obj, HomogTransf &objPose, std::string &filename)
{
  obj = -1;

  objRec.SetBounds(tableBoxWidths, tableBoxOffset);
  objRec.SetIgnoreRegions(ignoreWidths, ignoreOffsets);

  std::vector<std::string> filenames(numTableCameras);
  for (int i=0; i < numTableCameras; ++i)
  {
    for (int j=0; j < numCameras; ++j)
    {
      if (tableCameraNames[i] == cameraNames[j])
      {
        serialPort << (0x1 << servoNums[i]) << std::endl;
        ros::Duration(0.25).sleep();
        objRec.SavePoints(filenames[i], true, false, tableCameraNames[i]);
        break;
      }
    }
  }
  serialPort << 0x00 << std::endl;

  bool success = objRec.GetObjFromCameras(obj, objPose, filenames, tableCameraNames, true);

  // Convert from meters back to mm
  Vec t = objPose.getTranslation();
  t *= 1000.0;
  objPose.setTranslation(t);

  // TODO: Record stuff to 'filename'

  return success;
}

bool TableVision::getHandPose(int &obj, HomogTransf &objPose, std::string &filename, const HomogTransf &handPose)
{
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

  size_t num_finger_boxes = fingerBoxOffsets.size();

  if (!robot.SetCartesianJ(handPose))
    return false;

  // Now let's set up everything so we can recognize the object

  // First, we'll compute where each of the fingers are, and add them to
  // our ignore regions

  // Find
  double motorAngle;
  double fingerAngle[NUM_FINGERS];
  hand.GetAngles(motorAngle, fingerAngle);

  std::vector<HomogTransf> ignoreRegions(NUM_FINGERS * num_finger_boxes + 2);
  std::vector<double> ignoreWidths;
  ignoreWidths.reserve((NUM_FINGERS * num_finger_boxes + 2) * 3);
  for (int i=0; i < NUM_FINGERS; ++i)
  {
    HomogTransf fingerAxis = handPose * fingerAxes[i];
    RotMat angle;
    angle.rotZ(fingerAngle[i]*PI/180.0);
    HomogTransf rotatedFingerAxis = fingerAxis * HomogTransf(angle, Vec(0.0, 3));

    for (size_t j=0; j < num_finger_boxes; ++j)
    {
      ignoreRegions[num_finger_boxes*i + j] = rotatedFingerAxis * fingerBoxOffsets[j];
      ignoreWidths.insert(ignoreWidths.end(),fingerBoxWidths[j].begin(), fingerBoxWidths[j].end());
    }
  }
  ignoreRegions[NUM_FINGERS * num_finger_boxes] = handPose * palmBoxOffset;
  ignoreWidths.insert(ignoreWidths.end(), palmBoxWidths, palmBoxWidths+3);

  ignoreRegions[NUM_FINGERS * num_finger_boxes + 1] = tableIgnoreOffset;
  ignoreWidths.insert(ignoreWidths.end(), tableIgnoreWidths, tableIgnoreWidths+3);


  // Compute our overall box bound, which is just offset from the current
  // hand pose
  HomogTransf handBoxBounds = handPose * handBoxOffset;

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

  
  std::vector<std::string> filenames(numHandCameras);
  for (int i=0; i < numHandCameras; ++i)
  {
    for (int j=0; j < numCameras; ++j)
    {
      if (handCameraNames[i] == cameraNames[j])
      {
        serialPort << (0x1 << servoNums[i]) << std::endl;
        ros::Duration(0.25).sleep();
        objRec.SavePoints(filenames[i], true, false, handCameraNames[i]);
        break;
      }
    }
  }
  serialPort << 0x00 << std::endl;


  // Now that we're done taking pictures, restore our state
  robot.SetJoints(old_joints);
  robot.SetSpeed(old_tcp, old_ori);
  robot.SetZone(old_zone);


  /*
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
*/
/*
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
  */

  HomogTransf resPose;

  bool success = objRec.GetObjFromCameras(obj, resPose, filenames, handCameraNames, true);

  t = resPose.getTranslation();
  t *= 1000.0;
  resPose.setTranslation(t);

  objPose = handPose.inv() * resPose;

  // Save the result of this view in a text file
  return success;

}





bool TableVision::init(ros::NodeHandle *n)
{
  // Remember the pointer to our node
  node = n;

  // Subscribe to object recognition
  objRec.subscribe(n);
  hand.subscribe(n);
  robot.subscribe(n);


  // Load in our parameters which give us the boundary box and ignore
  // regions for the table. Note that we convert all of the distances in
  // these regions from mm to meters, as this is what objRec_node expects


  n->getParam("/tableVision/serialPortAddress", serialPortAddress);

  XmlRpc::XmlRpcValue my_list;
  XmlRpc::XmlRpcValue inner_list;
  double pose[7];

  n->getParam("/tableVision/cameraNames", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  numCameras = my_list.size();
  cameraNames.resize(numCameras);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    cameraNames[i] = static_cast<std::string>(my_list[i]);
  }

  n->getParam("/tableVision/tableCameras", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  numTableCameras = my_list.size();
  tableCameraNames.resize(numTableCameras);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    tableCameraNames[i] = static_cast<std::string>(my_list[i]);
  }

  n->getParam("/tableVision/handCameras", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  numHandCameras = my_list.size();
  handCameraNames.resize(numHandCameras);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    handCameraNames[i] = static_cast<std::string>(my_list[i]);
  }


  n->getParam("/tableVision/servoNums", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(my_list.size() == numCameras);
  servoNums.resize(my_list.size());
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    servoNums[i] = static_cast<int>(my_list[i]);
  }

  n->getParam("/tableVision/tableBoxOffset", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(my_list.size() == 7);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    pose[i] = static_cast<double>(my_list[i]);
    if (i < 3)
      pose[i] /= 1000.0;
  }
  tableBoxOffset = HomogTransf(pose);

  n->getParam("/tableVision/tableBoxWidths", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(my_list.size() == 3);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    tableBoxWidths[i] = static_cast<double>(my_list[i]) / 1000.0;
  }

  n->getParam("/tableVision/ignoreOffsets", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ignoreOffsets.resize(my_list.size());
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    inner_list = my_list[i];
    ROS_ASSERT(inner_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(inner_list.size() == 7);
    for (int32_t j = 0; j < inner_list.size(); ++j)
    {
      ROS_ASSERT(inner_list[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      pose[j] = static_cast<double>(inner_list[j]);
      if (j < 3)
        pose[j] /= 1000.0;
    }
    ignoreOffsets[i] = HomogTransf(pose);
  }

  n->getParam("/tableVision/ignoreWidths", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT((size_t)my_list.size() == ignoreOffsets.size());
  ignoreWidths.resize(my_list.size() * 3);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    inner_list = my_list[i];
    ROS_ASSERT(inner_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(inner_list.size() == 3);
    for (int32_t j = 0; j < inner_list.size(); ++j)
    {
      ROS_ASSERT(inner_list[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      ignoreWidths[3*i + j] = static_cast<double>(inner_list[j]) / 1000.0;
    }
  }

  n->getParam("/tableVision/handBoxOffset", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(my_list.size() == 7);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    pose[i] = static_cast<double>(my_list[i]);
  }
  handBoxOffset = HomogTransf(pose);

  n->getParam("/tableVision/handBoxWidths", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(my_list.size() == 3);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    handBoxWidths[i] = static_cast<double>(my_list[i]) / 1000.0;
  }

  n->getParam("/tableVision/palmBoxOffset", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(my_list.size() == 7);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    pose[i] = static_cast<double>(my_list[i]);
  }
  palmBoxOffset = HomogTransf(pose);

  n->getParam("/tableVision/palmBoxWidths", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(my_list.size() == 3);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    palmBoxWidths[i] = static_cast<double>(my_list[i]) / 1000.0;
  }

  fingerAxes.resize(NUM_FINGERS);
  n->getParam("/tableVision/fingerAxes", my_list);
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

  n->getParam("/tableVision/fingerBoxOffsets", my_list);
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

  n->getParam("/tableVision/fingerBoxWidths", my_list);
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

  n->getParam("/tableVision/tableIgnoreOffset", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(my_list.size() == 7);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    pose[i] = static_cast<double>(my_list[i]);
  }
  tableIgnoreOffset = HomogTransf(pose);

  n->getParam("/tableVision/tableIgnoreWidths", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(my_list.size() == 3);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    tableIgnoreWidths[i] = static_cast<double>(my_list[i]) / 1000.0;
  }


  n->getParam("/tableVision/defaultHandPose", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(my_list.size() == 7);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    pose[i] = static_cast<double>(my_list[i]);
  }
  defaultHandPose = HomogTransf(pose);



  // Connect to the servo control arduino
  serialPort.Open(serialPortAddress.c_str());
  serialPort.SetBaudRate(SerialStreamBuf::BAUD_9600);
  serialPort.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
  serialPort.SetNumOfStopBits(1);
  serialPort.SetParity(SerialStreamBuf::PARITY_NONE);
  serialPort.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);

  if (!serialPort.good())
  {
    ROS_ERROR("Couldn't connect to servo control arduino!");
    return false;
  }

  //Open all kinect projectors so they can initialize
  ROS_INFO("Opening up servos and waiting...");
  serialPort << 0xff << std::endl;
  ros::Duration(5.0).sleep();
  ROS_INFO("Done Waiting!");
  serialPort << 0x00 << std::endl;


  // Advertise our services
  handle_tableVision_GetPose = 
    node->advertiseService("tableVision_GetPose", &TableVision::tableVision_GetPose, this);
  handle_tableVision_GetHandPose = 
    node->advertiseService("tableVision_GetHandPose", &TableVision::tableVision_GetHandPose, this);
  handle_tableVision_GetHandPoseSpec = 
    node->advertiseService("tableVision_GetHandPoseSpec", &TableVision::tableVision_GetHandPoseSpec, this);
  handle_tableVision_GetObjAndPose = 
    node->advertiseService("tableVision_GetObjAndPose", &TableVision::tableVision_GetObjAndPose, this);
  handle_tableVision_SavePoints = 
    node->advertiseService("tableVision_SavePoints", &TableVision::tableVision_SavePoints, this);

  ROS_INFO("tableVision_node initialized");

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tableVision_node");

  ros::NodeHandle node;
  TableVision tableVision(&node);
  ros::spin();

  return 0;
}

