#include "util_comm.h"
#include <signal.h>
#include <cstdlib>
#include <ctime>

UtilComm::UtilComm()
{
  node = NULL;
}

UtilComm::UtilComm(ros::NodeHandle* np)
{
  subscribe(np);
}

UtilComm::~UtilComm()
{
  shutdown();
}

void UtilComm::shutdown()
{
  robot.shutdown();
  hand.shutdown();
  vision.shutdown();
  objRec.shutdown();
  logger.shutdown();
}

void UtilComm::subscribe(ros::NodeHandle* np)
{
  node = np;

  robot.subscribe(node);
  logger.subscribe(node);
  hand.subscribe(node);
  vision.subscribe(node);
  objRec.subscribe(node);

  srand(time(NULL));

  // Read in all necessary parameters
  node->getParam("/system/cameraJ1",visionJ[0]);
  node->getParam("/system/cameraJ2",visionJ[1]);
  node->getParam("/system/cameraJ3",visionJ[2]);
  node->getParam("/system/cameraJ4",visionJ[3]);
  node->getParam("/system/cameraJ5",visionJ[4]);
  node->getParam("/system/cameraJ6",visionJ[5]);

  node->getParam("/system/placeJ1",placeJ[0]);
  node->getParam("/system/placeJ2",placeJ[1]);
  node->getParam("/system/placeJ3",placeJ[2]);
  node->getParam("/system/placeJ4",placeJ[3]);
  node->getParam("/system/placeJ5",placeJ[4]);
  node->getParam("/system/placeJ6",placeJ[5]);

  node->getParam("/system/homeJ1",homeJ[0]);
  node->getParam("/system/homeJ2",homeJ[1]);
  node->getParam("/system/homeJ3",homeJ[2]);
  node->getParam("/system/homeJ4",homeJ[3]);
  node->getParam("/system/homeJ5",homeJ[4]);
  node->getParam("/system/homeJ6",homeJ[5]);

  node->getParam("/system/homeX", homeX);
  node->getParam("/system/homeY", homeY);
  node->getParam("/system/homeZ", homeZ);
  node->getParam("/system/homeQ0", homeQ0);
  node->getParam("/system/homeQX", homeQX);
  node->getParam("/system/homeQY", homeQY);
  node->getParam("/system/homeQZ", homeQZ);

  node->getParam("/system/safeCameraX", safeCameraX);
  node->getParam("/system/safeCameraY", safeCameraY);
  node->getParam("/system/safeCameraZ", safeCameraZ);
  node->getParam("/system/safeCameraQ0", safeCameraQ0);
  node->getParam("/system/safeCameraQX", safeCameraQX);
  node->getParam("/system/safeCameraQY", safeCameraQY);
  node->getParam("/system/safeCameraQZ", safeCameraQZ);

  node->getParam("/system/testHomeX", testHomeC[0]);
  node->getParam("/system/testHomeY", testHomeC[1]);
  node->getParam("/system/testHomeZ", testHomeC[2]);
  node->getParam("/system/testHomeQ0", testHomeC[3]);
  node->getParam("/system/testHomeQX", testHomeC[4]);
  node->getParam("/system/testHomeQY", testHomeC[5]);
  node->getParam("/system/testHomeQZ", testHomeC[6]);

  node->getParam("/system/testActionX", testActionC[0]);
  node->getParam("/system/testActionY", testActionC[1]);
  node->getParam("/system/testActionZ", testActionC[2]);
  node->getParam("/system/testActionQ0", testActionC[3]);
  node->getParam("/system/testActionQX", testActionC[4]);
  node->getParam("/system/testActionQY", testActionC[5]);
  node->getParam("/system/testActionQZ", testActionC[6]);

  node->getParam("/system/testHomeJ1",testHomeJ[0]);
  node->getParam("/system/testHomeJ2",testHomeJ[1]);
  node->getParam("/system/testHomeJ3",testHomeJ[2]);
  node->getParam("/system/testHomeJ4",testHomeJ[3]);
  node->getParam("/system/testHomeJ5",testHomeJ[4]);
  node->getParam("/system/testHomeJ6",testHomeJ[5]);

  node->getParam("/system/testActionJ1",testActionJ[0]);
  node->getParam("/system/testActionJ2",testActionJ[1]);
  node->getParam("/system/testActionJ3",testActionJ[2]);
  node->getParam("/system/testActionJ4",testActionJ[3]);
  node->getParam("/system/testActionJ5",testActionJ[4]);
  node->getParam("/system/testActionJ6",testActionJ[5]);

  node->getParam("/system/binMinX", bin_x_min);
  node->getParam("/system/binMaxX", bin_x_max);
  node->getParam("/system/binMinY", bin_y_min);
  node->getParam("/system/binMaxY", bin_y_max);

  bin_x_range = bin_x_max - bin_x_min;
  bin_y_range = bin_y_max - bin_y_min;


  double trans[3];
  double quat[4];

  node->getParam("/objRec/cameraFrameX", trans[0]);
  node->getParam("/objRec/cameraFrameY", trans[1]);
  node->getParam("/objRec/cameraFrameZ", trans[2]);
  node->getParam("/objRec/cameraFrameQ0", quat[0]);
  node->getParam("/objRec/cameraFrameQX", quat[1]);
  node->getParam("/objRec/cameraFrameQY", quat[2]);
  node->getParam("/objRec/cameraFrameQZ", quat[3]);

  cameraFrame.setTranslation(Vec(trans,3));
  cameraFrame.setRotation(Quaternion(quat).getRotMat());

  node->getParam("/system/objRecInitJ1",objRecInitJ[0]);
  node->getParam("/system/objRecInitJ2",objRecInitJ[1]);
  node->getParam("/system/objRecInitJ3",objRecInitJ[2]);
  node->getParam("/system/objRecInitJ4",objRecInitJ[3]);
  node->getParam("/system/objRecInitJ5",objRecInitJ[4]);
  node->getParam("/system/objRecInitJ6",objRecInitJ[5]);

  node->getParam("/system/objRecInitX", trans[0]);
  node->getParam("/system/objRecInitY", trans[1]);
  node->getParam("/system/objRecInitZ", trans[2]);
  node->getParam("/system/objRecInitQ0", quat[0]);
  node->getParam("/system/objRecInitQX", quat[1]);
  node->getParam("/system/objRecInitQY", quat[2]);
  node->getParam("/system/objRecInitQZ", quat[3]);

  objRecInitC.setTranslation(Vec(trans,3));
  objRecInitC.setRotation(Quaternion(quat).getRotMat());

}


// Drop a marker at the specified location. Note that this assumes we are 
// already in the dropping plane, so nothing weird happens when we do a 
// cartesian move. To be safe, one should always make sure that a call 
// to go_home() has occured before calling this function.
bool UtilComm::drop(double x, double y)
{
  if (node == NULL)
    return false;

  if (!set_defaults())
  {
    return false;
  }
  else
  {
    // Command robot to go to drop position
    if (!robot.SetCartesian(x, y, homeZ, 
          homeQ0, homeQX, homeQY, homeQZ))
    {
      return false;
    }
    else
    {
      drop();
      return true;
    }
  }
}


// Drops the marker at current location
bool UtilComm::drop()
{
  // Now command hand to drop the markers
  hand.SetSpeed(M_HAND_SPEED);
  hand.SetAngle(HAND_DROP);

  //Wait for hand to be opened
  hand.WaitRest(0.25);
  hand.SetRest();

  return true;
}

bool UtilComm::random_drop()
{
  if (node == NULL)
    return false;

  // Randomly choose a location to drop the markers
  double posX, posY;
  get_random_bin_pos(posX, posY);

  // Now drop the marker at that location
  return drop(posX, posY);
}

// Computes a random x y position that puts the 
//  hand inside of the bin area
bool UtilComm::get_random_bin_pos(double &x, double &y)
{
  if (node == NULL)
    return false;

  x = (rand() / ((double)RAND_MAX)) * bin_x_range + bin_x_min;
  y = (rand() / ((double)RAND_MAX)) * bin_y_range + bin_y_min;

  return true;
}

// Shows the hand to the grasp_detect camera, and calibrates 
//  both that camera and the place detect camera
bool UtilComm::calibrate_vision()
{
  if (node == NULL)
    return false;

  ROS_INFO("Checking for vision...");
  while (!vision.Ping()) ;
  ROS_INFO("Checking for hand...");
  while (!hand.Ping()) ;

  // Start moving fingers to vision calibration position
  hand.SetSpeed(M_HAND_SPEED);
  hand.SetForce(M_HAND_FORCE);
  hand.SetAngle(M_HAND_VIS);

  // Move arm to camera
  if (!go_safe_vision())
  {
    return false;
  }
  else
  {
    // Wait until our fingers have stopped moving
    hand.WaitRest(0.25);
    hand.SetRest();

    // Don't take calibration pictures until table has stopped shaking
    usleep(VIS_CAL_SETTLE_TIME);

    // Take pictures to calibrate both cameras
    if (!vision.Calibrate(true, GRASP_DETECT, vis_grasp_cal_file))
    {
      return false;
    }
    else if (!vision.Calibrate(true, PLACE_DETECT, vis_place_cal_file))
    {
      return false;
    }
    // Now move the robot back home
    else if (!go_home_from_vision())
    {
      return false;
    }
    else
    {
      return true;
    }
  }
}


/*
// Shows the hand to the grasp_detect camera, and calibrates 
//  both that camera and the place detect camera
bool UtilComm::calibrate_vision()
{
  if (node == NULL)
    return false;

  ROS_INFO("Checking for vision...");
  while (!vision.Ping()) ;
  ROS_INFO("Checking for hand...");
  while (!hand.Ping()) ;

  // Start moving fingers to vision calibration position
  hand.SetSpeed(M_HAND_SPEED);
  hand.SetForce(M_HAND_FORCE);
  hand.SetAngle(M_HAND_VIS);

  // Move arm to camera
  if (!go_vision())
  {
    return false;
  }
  else
  {
    // Wait until our fingers have stopped moving
    hand.WaitRest(0.25);
    hand.SetRest();

    // Don't take calibration pictures until table has stopped shaking
    usleep(VIS_CAL_SETTLE_TIME);

    // Take pictures to calibrate both cameras
    if (!vision.Calibrate(true, GRASP_DETECT, vis_grasp_cal_file))
    {
      return false;
    }
    else if (!vision.Calibrate(true, PLACE_DETECT, vis_place_cal_file))
    {
      return false;
    }
    // Now move the robot back home
    else if (!robot.SetJoints(homeJ[0], homeJ[1], homeJ[2],
          homeJ[3], homeJ[4], homeJ[5]))
    {
      return false;
    }
    else
    {
      return true;
    }
  }
}
*/

// Goes to the home position and calibrates the hand
bool UtilComm::calibrate_hand(bool fast)
{
  if (node == NULL)
    return false;

  ROS_INFO("Checking for hand...");
  while (!hand.Ping()) ;

  if (go_home())
  {
    hand.Calibrate(fast);
    return true;
  }
  else
    return false;
}

// Does a joint move to our home position
bool UtilComm::go_home()
{
  if (node == NULL)
    return false;

  if (!set_defaults())
  {
    return false;
  }
  else if (!robot.SetJoints(homeJ[0], homeJ[1], homeJ[2],
        homeJ[3], homeJ[4], homeJ[5]))
  {
    return false;
  }
  else
    return true;
}


// Does a joint move to our home position
bool UtilComm::go_test_home()
{
  if (node == NULL)
    return false;

  if (!set_defaults())
  {
    return false;
  }
  else if (!robot.SetJoints(testHomeJ[0], testHomeJ[1], testHomeJ[2],
        testHomeJ[3], testHomeJ[4], testHomeJ[5]))
  {
    return false;
  }
  else
    return true;
}


// Does a joint move to our home position
bool UtilComm::go_test_action()
{
  if (node == NULL)
    return false;

  if (!set_defaults())
  {
    return false;
  }
  else if (!robot.SetJoints(testActionJ[0], testActionJ[1], testActionJ[2],
        testActionJ[3], testActionJ[4], testActionJ[5]))
  //else if (!robot.SetCartesian(testActionC[0], testActionC[1], testActionC[2],
  //      testActionC[3], testActionC[4], testActionC[5], testActionC[6]))
  {
    return false;
  }
  else
    return true;
}


// Does a cartesian move to our home position
bool UtilComm::go_home_from_test()
{
  if (node == NULL)
    return false;

  if (!set_defaults())
  {
    return false;
  }
  else if (!robot.SetCartesian(testHomeC[0], testHomeC[1], testHomeC[2],
        testHomeC[3], testHomeC[4], testHomeC[5], testHomeC[6]))
  {
    return false;
  }
  else
    return true;
}




bool UtilComm::go_home_from_vision()
{
  if (node == NULL)
    return false;

  if (!set_defaults())
  {
    return false;
  }
  else if(!robot.SetCartesian(safeCameraX, safeCameraY, safeCameraZ, 
        safeCameraQ0, safeCameraQX, safeCameraQY, safeCameraQZ))
  {
    return false;
  }
  else if(!robot.SetCartesian(homeX, homeY, homeZ, 
        homeQ0, homeQX, homeQY, homeQZ))
  {
    return false;
  }
  else
    return true;
}

bool UtilComm::set_defaults()
{
  if (node == NULL)
    return false;

  // Make sure our robot communication is blocking
  if (!robot.SetComm(BLOCKING))
  {
    return false;
  }
  // Make sure that our robot's work object is set up
  else if (!robot.SetWorkObject(U_WORK_X, U_WORK_Y, U_WORK_Z, 
		      U_WORK_Q0, U_WORK_QX, U_WORK_QY, U_WORK_QZ))
  {
    return false;
  }
  // Make sure our tool frame is correctly set up
  else if (!robot.SetTool(U_TOOL_X, U_TOOL_Y, U_TOOL_Z, 
		      U_TOOL_Q0, U_TOOL_QX, U_TOOL_QY, U_TOOL_QZ))
  {
    return false;
  }
  // Set our default speed limits
  else if (!robot.SetSpeed(U_TCP, U_ORI))
  {
    return false;
  }
  // Set the default "zone" of our robot (amount of interpolation we allow)
  else if (!robot.SetZone(U_ZONE))
  {
    return false;
  }
  else
  {
    return true;
  }
}

// Move arm to camera smoothly but quickly
bool UtilComm::go_safe_vision()
{
  // First, go to the home position so we know where we are
  if (!go_home())
  {
    return false;
  }
  else if(!robot.SetCartesian(safeCameraX, safeCameraY, safeCameraZ, 
        safeCameraQ0, safeCameraQX, safeCameraQY, safeCameraQZ))
  {
    return false;
  }
  else
  {
    /*
    double interJ[NUM_JOINTS];

    // Go almost all of the way there at high speed
    for (int i=0; i<NUM_JOINTS; i++)
    {
      interJ[i] = (visionJ[i] - homeJ[i])*VIS_INTERP_LEVEL+homeJ[i];
    }

    robot.SetZone(U_INTERP_ZONE);

    robot.SetJoints(interJ[0], interJ[1], interJ[2],
          interJ[3], interJ[4], interJ[5]);

    // Once we're close, slow down and go the rest of the way
    robot.SetSpeed(U_SLOW_TCP, U_SLOW_ORI);
    robot.SetZone(U_ZONE);

    */

    // Command our robot to move to the camera position
    if (!robot.SetJoints(visionJ[0], visionJ[1], visionJ[2],
          visionJ[3], visionJ[4], visionJ[5]))
    {
      return false;
    }
    else
    {
      // Restore default speed, and return
      robot.SetSpeed(U_TCP, U_ORI);
      return true;
    }
  }
}



// Move arm to camera smoothly but quickly
bool UtilComm::go_vision()
{
  if (!set_defaults())
  {
    return false;
  }
  else
  {
    double interJ[NUM_JOINTS];

    // Go almost all of the way there at high speed
    for (int i=0; i<NUM_JOINTS; i++)
    {
      interJ[i] = (visionJ[i] - homeJ[i])*VIS_INTERP_LEVEL+homeJ[i];
    }

    robot.SetZone(U_INTERP_ZONE);

    robot.SetJoints(interJ[0], interJ[1], interJ[2],
          interJ[3], interJ[4], interJ[5]);

    // Once we're close, slow down and go the rest of the way
    robot.SetSpeed(U_SLOW_TCP, U_SLOW_ORI);
    robot.SetZone(U_ZONE);

    // Command our robot to move to the camera position
    if (!robot.SetJoints(visionJ[0], visionJ[1], visionJ[2],
          visionJ[3], visionJ[4], visionJ[5]))
    {
      return false;
    }
    else
    {
      // Restore default speed, and return
      robot.SetSpeed(U_TCP, U_ORI);
      return true;
    }
  }
}


// Move arm to placing location
bool UtilComm::go_place()
{
  if (!set_defaults())
  {
    return false;
  }
  else
  {
    // Command our robot to move to the camera position
    return robot.SetJoints(placeJ[0], placeJ[1], placeJ[2],
          placeJ[3], placeJ[4], placeJ[5]);
  }
}


bool UtilComm::getObjectInHand(int &obj, double trans[3], double quat[4])
{
  Vec t(3);
  Quaternion q;
  bool success = getObjectInHand(obj, t, q);
  for (int i=0; i<3; i++)
    trans[i] = t[i];
  for (int i=0; i<4; i++)
    quat[i] = q[i];
  return success;
}

bool UtilComm::getObjectInHand(int &obj, Vec trans, Quaternion quat)
{
  HomogTransf handPose;
  bool success = getObjectInHand(obj, handPose);
  trans = handPose.getTranslation();
  quat = handPose.getRotation().getQuaternion();
  return success;
}

bool UtilComm::getObjectInHand(int &obj, HomogTransf &handPose)
{
  // First get the current robot pose
  HomogTransf robotPose;
  if(!robot.GetCartesian(robotPose))
    return false;

  // Now compute the center of the box where we should be looking for objects is
  HomogTransf boxBounds = robotPose * handBoxOffset;
  int numIgnore = numFingers*numBoxesPerFinger;
  HomogTransf fingerBounds[numIgnore];
  for (int i=0; i<numIgnore; i++)
    fingerBounds[i] = robotPose * fingerBoxOffset[i];

  // Convert mm to m
  Vec trans = boxBounds.getTranslation();
  trans /= 1000.0;
  boxBounds.setTranslation(trans);
  for (int i=0; i<numIgnore; i++)
    { 
      Vec ft = fingerBounds[i].getTranslation();
      ft /= 1000.0;
      fingerBounds[i].setTranslation(ft);
    }

  // Set the region to look for objects
  if (!objRec.SetBounds(boxWidths, boxBounds))
    return false;
  if (!objRec.SetIgnoreRegions(numIgnore, fingerWidths, fingerBounds))
    return false;

  // Now find the location of the object in the camera frame
  HomogTransf objPose;
  bool success = objRec.GetObject(obj, objPose);

  // Convert m to mm
  trans = objPose.getTranslation();
  trans *= 1000.0;
  objPose.setTranslation(trans);

  // Convert this location back to the pose of the robot
  handPose = robotPose.inv() * objPose;
  
  return success;
}

bool UtilComm::getObjectFromCamera(int &obj, HomogTransf &objPose)
{

  // First get the current robot pose
  HomogTransf robotPose;
  if(!robot.GetCartesian(robotPose))
    return false;

  // Now compute the center of the box where we should be looking for objects is
  HomogTransf boxBounds = robotPose * handBoxOffset;
  HomogTransf fingerBounds[3];
  for (int i=0; i<3; i++)
    fingerBounds[i] = robotPose * fingerBoxOffset[i];

  // Convert mm to m
  Vec trans = boxBounds.getTranslation();
  trans /= 1000.0;
  boxBounds.setTranslation(trans);
  for (int i=0; i<3; i++)
    { 
      Vec ft = fingerBounds[i].getTranslation();
      ft /= 1000.0;
      fingerBounds[i].setTranslation(ft);
    }

  /*
  HomogTransf temp(Quaternion("1.0 0.0 0.0 0.0").getRotMat(), Vec("0.0 0.0 0.9",3));
  double tempWidth[3] = {0.6,0.6,0.6};

  // Set the region to look for objects
  if (!objRec.SetBounds(tempWidth, temp))
    return false;
    */

  // Set the region to look for objects
  if (!objRec.SetBounds(boxWidths, boxBounds))
    return false;
  if (!objRec.SetIgnoreRegions(3, fingerWidths, fingerBounds))
    return false;

  // Now find the location of the object in the world frame
  bool success = objRec.GetObject(obj, objPose);

  // Convert m to mm
  trans = objPose.getTranslation();
  trans *= 1000.0;
  objPose.setTranslation(trans);
  
  return success;
}

bool UtilComm::getObjMulti(int &obj, double trans[3], double quat[4])
{
  Vec t(3);
  Quaternion q;
  bool success = getObjMulti(obj, t, q);
  for (int i=0; i<3; i++)
    trans[i] = t[i];
  for (int i=0; i<4; i++)
    quat[i] = q[i];
  return success;
}

bool UtilComm::getObjMulti(int &obj, Vec trans, Quaternion quat)
{
  HomogTransf handPose;
  if (getObjMulti(obj, handPose))
  {
    trans = handPose.getTranslation();
    quat = handPose.getRotation().getQuaternion();
    return true;
  }
  return false;
}

bool UtilComm::getObjMulti(int &obj, HomogTransf &handPose)
{
  std::string filenames[NUM_OBJREC_VIEWS];
  return getObjMulti(obj, handPose, filenames);
}

bool UtilComm::getObjMulti(int &obj, HomogTransf &handPose, const HomogTransf poses[NUM_OBJREC_VIEWS])
{
  std::string filenames[NUM_OBJREC_VIEWS];
  return getObjMulti(obj, handPose, filenames, poses);
}

bool UtilComm::getObjMulti(int &obj, HomogTransf &handPose, std::string filenames[NUM_OBJREC_VIEWS])
{
  return getObjMulti(obj, handPose, filenames, objRecPoses);
}

bool UtilComm::getObjMulti(int &obj, HomogTransf &handPose, 
    std::string filenames[NUM_OBJREC_VIEWS], const HomogTransf poses[NUM_OBJREC_VIEWS])
{
  // Move the robot to the first view
  if (!robot.SetJoints(objRecInitJ))
    return false;

  if (!robot.SetSpeed(100.0, 40.0))
    return false;
  if (!robot.SetZone(0))
    return false;

  for (int i=0; i < NUM_OBJREC_VIEWS; i++)
  {
    // Change the orientation of the hand
    if (i == NUM_OBJREC_VIEWS - 1)
    {
      if (!robot.SetJoints(objRecFinalJ))
        return false;
    }

    if (!robot.SetCartesian(poses[i]))
      return false;
    ros::Duration(0.25).sleep();

    // Now save the points at this location
    if (!objRec.SavePoints(filenames[i]))
      return false;
  }

  if (!robot.SetSpeed(50.0, 20.0))
    return false;

  return getObjMultiFiles(obj, handPose, filenames, poses);
}

bool UtilComm::getObjMultiFiles(int &obj, HomogTransf &handPose, const std::string filenames[NUM_OBJREC_VIEWS])
{
  return getObjMultiFiles(obj, handPose, filenames, objRecPoses);
}

bool UtilComm::getObjMultiFiles(int &obj, HomogTransf &handPose, const std::string filenames[NUM_OBJREC_VIEWS], const HomogTransf poses[NUM_OBJREC_VIEWS])
{
  double trans[3 * NUM_OBJREC_VIEWS];
  double quat[4 * NUM_OBJREC_VIEWS];

  for (int i=0; i < NUM_OBJREC_VIEWS; i++)
  {
    Quaternion q = poses[i].getRotation().getQuaternion();
    Vec t = poses[i].getTranslation() / 1000.0;

    memcpy(trans + 3 * i, t.v, sizeof(double)*3);
    memcpy(quat + 4 * i, q.v, sizeof(double)*4);
  }

  // Now compute the center of the box where we should be looking for objects is
  HomogTransf boxBounds = poses[0] * handBoxOffset;

  // compute the center of the box for each finger
  // find the finger angle transformation
  double motorAngle;
  double fingerAngle[3];
  hand.GetAngles(motorAngle, fingerAngle);
  // convert to radians
  for (int i=0; i<3; i++)
    fingerAngle[i] = fingerAngle[i]*(3.14159/180);

  // transform 
  HomogTransf fingerBounds[4];
  for (int i=0; i<3; i++)
    {
      fingerBounds[i] = poses[0] * fingerAxes[i];

      // Transform due to finger angle (about the z axis)
      RotMat rot = fingerBounds[i].getRotation();
      RotMat angle;
      angle.rotZ(fingerAngle[i]);
      fingerBounds[i].setRotation(rot * angle);

      // translate finger position relative to axes
      std::cout << fingerBounds[i] << endl;
      fingerBounds[i] = fingerBounds[i] * fingerBoxOffset[0];
      std::cout << fingerBounds[i] << endl;
    }

  fingerBounds[3] = poses[0] * palmBoxOffset[0];
  

  // Convert mm to m
  Vec t = boxBounds.getTranslation();
  t /= 1000.0;
  boxBounds.setTranslation(t);
  for (int i=0; i<4; i++)
    { 
      Vec ft = fingerBounds[i].getTranslation();
      ft /= 1000.0;
      fingerBounds[i].setTranslation(ft);
    }

  // Set the region to look for objects
  if (!objRec.SetBounds(boxWidths, boxBounds))
    return false;

  /*
  double lala[7];
  t = boxBounds.getTranslation();
  Quaternion q = boxBounds.getRotation().getQuaternion();
  lala[0] = t[0];
  */  
  // if (!objRec.SetIgnoreRegions(1, fingerWidths, &boxBounds))
  //   return false;

  double ignore_widths[3*4];
  memcpy(ignore_widths, fingerWidths, sizeof(double)*9);
  memcpy(ignore_widths+9, palmWidths, sizeof(double)*3);

  if (!objRec.SetIgnoreRegions(4, ignore_widths, fingerBounds))
   return false;

  // Now find the location of the object in the world frame 
  //  (using the camera frame stored in the parameters)
  HomogTransf objPose;
  bool success;
  success = objRec.GetObjFromViews(obj, objPose, NUM_OBJREC_VIEWS, filenames, trans, quat);

  // Convert m to mm
  t = objPose.getTranslation();
  t *= 1000.0;
  objPose.setTranslation(t);

  // Convert this location back to the pose of the robot
  handPose = poses[0].inv() * objPose;

  cout << "HAND POSE: " << handPose << endl;
  
  return success;
}

// Set guess for object recognition of the object in the world 
//  frame given its pose with resepct to the robot hand.
// Note: handPose in mm
bool UtilComm::setGuessInHand(const HomogTransf handPose)
{
  // Get the robot's current position
  HomogTransf robotPose;
  robot.GetCartesian(robotPose);

  return setGuessInHand(handPose, robotPose);
}

// Set guess for recognition of object in the world frame given its pose 
//  with respect to the robot hand, and a given robot position. 
// Note: both poses in mm
bool UtilComm::setGuessInHand(const HomogTransf handPose, const HomogTransf robotPose)
{
  // Compute the world pose of the object given its pose in the hand
  HomogTransf guessPose = robotPose * handPose;

  // Convert from mm to m, which objRec_node needs
  Vec t = guessPose.getTranslation();
  t /= 1000.0;
  guessPose.setTranslation(t);

  // Set our guess to be the object's pose in the world
  return objRec.SetGuess(guessPose);
}


// Do everything getObjMulti does, but also put a 
//  guess on where the object is in the hand.
bool UtilComm::getObjMultiWithGuess(int &obj, HomogTransf &handPose, const HomogTransf handGuess)
{
  // Set our guess to be with respect to the first pose we will move the robot to
  if (!setGuessInHand(handGuess, objRecPoses[0]))
    return false;

  // Now move the robot to different poses and compute the actual object pose
  std::string filenames[NUM_OBJREC_VIEWS];
  return getObjMulti(obj, handPose, filenames, objRecPoses);
}

