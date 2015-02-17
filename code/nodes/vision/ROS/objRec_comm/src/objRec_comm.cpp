//
// File Name: objRec_comm.cpp
//
// Author: Robbie Paolini
//
// Last Modified: 10/15/2013
// 
// This is the shorthand library class implementation file for the object
// recognition node. See the comments in objRec_node.h for more information
//

#include "objRec_comm.h"

// Simple constructors / destructor
ObjRecComm::ObjRecComm()
{
}

ObjRecComm::ObjRecComm(ros::NodeHandle* np)
{
  subscribe(np);
}
ObjRecComm::~ObjRecComm()
{
  shutdown();
}

// Function to subscribe to ROS services
void ObjRecComm::subscribe(ros::NodeHandle* np)
{
  handle_objRec_GetObject = np->serviceClient<objRec_comm::objRec_GetObject>("objRec_GetObject");
  handle_objRec_GetObjFromViews = np->serviceClient<objRec_comm::objRec_GetObjFromViews>("objRec_GetObjFromViews");
  handle_objRec_GetObjFromCameras = np->serviceClient<objRec_comm::objRec_GetObjFromCameras>("objRec_GetObjFromCameras");
  handle_objRec_SetBounds = np->serviceClient<objRec_comm::objRec_SetBounds>("objRec_SetBounds");
  handle_objRec_SetStream = np->serviceClient<objRec_comm::objRec_SetStream>("objRec_SetStream");
  handle_objRec_SavePoints = np->serviceClient<objRec_comm::objRec_SavePoints>("objRec_SavePoints");
  handle_objRec_SetParams = np->serviceClient<objRec_comm::objRec_SetParams>("objRec_SetParams");
  handle_objRec_SetGuess = np->serviceClient<objRec_comm::objRec_SetGuess>("objRec_SetGuess");
  handle_objRec_SetCamera = np->serviceClient<objRec_comm::objRec_SetCamera>("objRec_SetCamera");
  handle_objRec_SetIgnoreRegions = np->serviceClient<objRec_comm::objRec_SetIgnoreRegions>("objRec_SetIgnoreRegions");
  handle_objRec_SetPrefOrient = np->serviceClient<objRec_comm::objRec_SetPrefOrient>("objRec_SetPrefOrient");
  handle_objRec_SetObject = np->serviceClient<objRec_comm::objRec_SetObject>("objRec_SetObject");
}

// Shutdown service clients
void ObjRecComm::shutdown()
{
  handle_objRec_GetObject.shutdown();
  handle_objRec_GetObjFromViews.shutdown();
  handle_objRec_GetObjFromCameras.shutdown();
  handle_objRec_SetBounds.shutdown();
  handle_objRec_SetStream.shutdown();
  handle_objRec_SavePoints.shutdown();
  handle_objRec_SetParams.shutdown();
  handle_objRec_SetGuess.shutdown();
  handle_objRec_SetCamera.shutdown();
  handle_objRec_SetIgnoreRegions.shutdown();
  handle_objRec_SetPrefOrient.shutdown();
  handle_objRec_SetObject.shutdown();
}

/////////////////////////////////////
// Services

// Set rectangular prisms to to ignore (corresponding to the fingers) 
// widths: list of box widths: [x1, y1, z1, x2, y2, z2 ... ]
// trans: list of translations for each box: [x1, y1, z1, x2, y2, z2 ... ]
// quat: list of quaternions for each box: [q0_1, qx_1, qy_1, qz_1 ... ]
bool ObjRecComm::SetIgnoreRegions(const int num_regions, const double *widths, const double *trans, const double *quats)
{
  objRec_SetIgnoreRegions_srv.request.num_regions = num_regions;
  objRec_SetIgnoreRegions_srv.request.widths.resize(3 * num_regions);
  objRec_SetIgnoreRegions_srv.request.trans.resize(3 * num_regions);
  objRec_SetIgnoreRegions_srv.request.quats.resize(4 * num_regions);

  for (int i=0; i<num_regions; i++)
    {
      for (int j=0; j<3;j++)
	{
	  objRec_SetIgnoreRegions_srv.request.widths[3*i+j] = widths[3*i+j];
	  objRec_SetIgnoreRegions_srv.request.trans[3*i+j] = trans[3*i+j];
	}
      for (int j=0; j<4; j++)
	  objRec_SetIgnoreRegions_srv.request.quats[4*i+j] = quats[4*i+j];
    }

  return handle_objRec_SetIgnoreRegions.call(objRec_SetIgnoreRegions_srv);
}

bool ObjRecComm::SetIgnoreRegions(const int num_regions, const double *widths, const Vec *trans, const Quaternion *quats)
{
  objRec_SetIgnoreRegions_srv.request.num_regions = num_regions;
  objRec_SetIgnoreRegions_srv.request.widths.resize(3 * num_regions);
  objRec_SetIgnoreRegions_srv.request.trans.resize(3 * num_regions);
  objRec_SetIgnoreRegions_srv.request.quats.resize(4 * num_regions);

  for (int i=0; i<num_regions; i++)
    {
      for (int j=0; j<3;j++)
	{
	  objRec_SetIgnoreRegions_srv.request.widths[3*i+j] = widths[3*i+j];
	  objRec_SetIgnoreRegions_srv.request.trans[3*i+j] = trans[i][j];
	}
      for (int j=0; j<4; j++)
	  objRec_SetIgnoreRegions_srv.request.quats[4*i+j] = quats[i][j];
    }

  return handle_objRec_SetIgnoreRegions.call(objRec_SetIgnoreRegions_srv);
}

bool ObjRecComm::SetIgnoreRegions(const int num_regions, const double *widths, const HomogTransf *t)
{
  Vec trans[num_regions];
  Quaternion quat[num_regions];

  for (int i=0; i<num_regions; i++)
    {
      Vec itrans = t[i].getTranslation();
      Quaternion iquat = t[i].getRotation().getQuaternion();
      trans[i] = Vec(3);
      for (int j=0;j<3; j++)
  	trans[i][j] = itrans[j];
      for (int j=0; j<4; j++)
  	quat[i][j] = iquat[j];
    }

  return SetIgnoreRegions(num_regions, widths, trans, quat);
}


bool ObjRecComm::SetIgnoreRegions(const std::vector<double> widths, 
    const std::vector<HomogTransf> t)
{
  int num_ignore = t.size();
  objRec_SetIgnoreRegions_srv.request.trans.resize(num_ignore*3);
  objRec_SetIgnoreRegions_srv.request.quats.resize(num_ignore*4);
  for (int i=0; i<num_ignore; ++i)
  {
    Vec trans = t[i].getTranslation();
    Quaternion quat = t[i].getQuaternion();

    for (int j=0; j<4; ++j)
    {
      if (j < 3)
      {
        objRec_SetIgnoreRegions_srv.request.trans[3*i+j] = trans[j];
      }
      objRec_SetIgnoreRegions_srv.request.quats[4*i+j] = quat[j];
    }
  }

  objRec_SetIgnoreRegions_srv.request.widths = widths;
  objRec_SetIgnoreRegions_srv.request.num_regions = num_ignore;

  return handle_objRec_SetIgnoreRegions.call(objRec_SetIgnoreRegions_srv);
}

// Set rectangular prism size and rigid-body transform for where to look
// for objects. See header file for more info
bool ObjRecComm::SetBounds(const double widths[3], const double trans[3], const double quat[4])
{
  for (int i=0; i < 3; i++)
    objRec_SetBounds_srv.request.widths[i] = widths[i];
  for (int i=0; i < 3; i++)
    objRec_SetBounds_srv.request.trans[i] = trans[i];
  for (int i=0; i < 4; i++)
    objRec_SetBounds_srv.request.quat[i] = quat[i];

  return handle_objRec_SetBounds.call(objRec_SetBounds_srv);
}

// Set rectangular prism size and rigid-body transform for where to look
// for objects. See header file for more info
bool ObjRecComm::SetBounds(const double widths[3], const Vec trans, const Quaternion quat)
{
  for (int i=0; i < 3; i++)
    objRec_SetBounds_srv.request.widths[i] = widths[i];
  for (int i=0; i < 3; i++)
    objRec_SetBounds_srv.request.trans[i] = trans[i];
  for (int i=0; i < 4; i++)
    objRec_SetBounds_srv.request.quat[i] = quat[i];

  return handle_objRec_SetBounds.call(objRec_SetBounds_srv);
}


// Set rectangular prism size and rigid-body transform for where to look
// for objects. See header file for more info
bool ObjRecComm::SetBounds(const double widths[3], const HomogTransf t)
{
  Vec trans = t.getTranslation();
  Quaternion quat = t.getRotation().getQuaternion();
  return SetBounds(widths, trans, quat);
}

// Take a picture of the object with the kinect and return the world pose
// of an object
bool ObjRecComm::GetObject(int &obj, double trans[3], double quat[4], const std::string cameraName, bool onTable)
{
  objRec_GetObject_srv.request.cameraName = cameraName;
  objRec_GetObject_srv.request.onTable = onTable;
  bool success = handle_objRec_GetObject.call(objRec_GetObject_srv);
  obj = objRec_GetObject_srv.response.objNum;
  for (int i=0; i < 3; i++)
    trans[i] = objRec_GetObject_srv.response.trans[i];
  for (int i=0; i < 4; i++)
    quat[i] = objRec_GetObject_srv.response.quat[i];

  return success;
}


// Take a picture of the object witht he kinect and return the world pose
// of an object
bool ObjRecComm::GetObject(int &obj, Vec &trans, Quaternion &quat, const std::string cameraName, bool onTable)
{
  objRec_GetObject_srv.request.cameraName = cameraName;
  objRec_GetObject_srv.request.onTable = onTable;
  bool success = handle_objRec_GetObject.call(objRec_GetObject_srv);
  obj = objRec_GetObject_srv.response.objNum;
  for (int i=0; i < 3; i++)
    trans[i] = objRec_GetObject_srv.response.trans[i];
  for (int i=0; i < 4; i++)
    quat[i] = objRec_GetObject_srv.response.quat[i];
  return success;
}


// Take a picture of the object witht he kinect and return the world pose
// of an object
bool ObjRecComm::GetObject(int &obj, HomogTransf &t, const std::string cameraName, bool onTable)
{
  Vec trans(3);
  Quaternion quat;

  bool success = GetObject(obj, trans, quat, cameraName, onTable);
  t.setTranslation(trans);
  t.setRotation(quat.getRotMat());
  return success;
}

// Get the world pose of an object from a number of saved point clouds.
// 
// num_views: number of views
// filenames: list of filenames
// trans: list of translations for each frame: [x1, y1, z1, x2, y2, z2 ... ]
// quat: list of quaternions for each frame: [q0_1, qx_1, qy_1, qz_1 ... ]
bool ObjRecComm::GetObjFromViews(int &obj, double t[3], double q[4], const int num_views, 
    const std::string *filenames, const double *trans, const double *quat, const std::string cameraName)
{
  Vec tr(3);
  Quaternion qu;

  // Call a different version of the overloaded function, and convert
  // vectors to c-arrays
  bool success = GetObjFromViews(obj, tr, qu, num_views, filenames, trans, quat, cameraName);
  memcpy(t, tr.v, sizeof(double)*3);
  memcpy(q, qu.v, sizeof(double)*4);
  return success;
}


// Get the world pose of an object from a number of saved point clouds.
// 
// num_views: number of views
// filenames: list of filenames
// trans: list of translations for each frame: [x1, y1, z1, x2, y2, z2 ... ]
// quat: list of quaternions for each frame: [q0_1, qx_1, qy_1, qz_1 ... ]
bool ObjRecComm::GetObjFromViews(int &obj, Vec &t, Quaternion &q, const int num_views, 
    const std::string *filenames, const double *trans, const double *quat, const std::string cameraName)
{
  // Store the number of views, and resize our vector arrays to hold the
  // correct amount of data
  objRec_GetObjFromViews_srv.request.num_views = num_views;
  objRec_GetObjFromViews_srv.request.cameraName = cameraName;
  objRec_GetObjFromViews_srv.request.filenames.resize(num_views);
  objRec_GetObjFromViews_srv.request.trans.resize(3 * num_views);
  objRec_GetObjFromViews_srv.request.quat.resize(4 * num_views);

  // Copy over each of our file names
  for (int i = 0; i < num_views; i++)
  {
    objRec_GetObjFromViews_srv.request.filenames[i] = filenames[i];
  }
  // Copy over the translations and orientations
  for (int i = 0; i < num_views; i++)
  {
    for (int j = 0; j < 3; j++)
      objRec_GetObjFromViews_srv.request.trans[3*i+j] = trans[3*i+j];
    for (int j = 0; j < 4; j++)
      objRec_GetObjFromViews_srv.request.quat[4*i+j] = quat[4*i+j];
  }

  // Call our service to get the object position
  bool success = handle_objRec_GetObjFromViews.call(objRec_GetObjFromViews_srv);
  
  // Recover the results from the service
  obj = objRec_GetObjFromViews_srv.response.objNum;
  for (int i=0; i < 3; i++)
    t[i] = objRec_GetObjFromViews_srv.response.trans[i];
  for (int i=0; i < 4; i++)
    q[i] = objRec_GetObjFromViews_srv.response.quat[i];
  return success;
}

// Get the world pose of an object from a number of saved point clouds.
// 
// num_views: number of views
// filenames: list of filenames
// trans: list of translations for each frame: [x1, y1, z1, x2, y2, z2 ... ]
// quat: list of quaternions for each frame: [q0_1, qx_1, qy_1, qz_1 ... ]
bool ObjRecComm::GetObjFromViews(int &obj, HomogTransf &t, const int num_views, 
    const std::string *filenames, const double *trans, const double *quat, const std::string cameraName)
{
  Vec tr(3);
  Quaternion qu;

  // Call a different version of our overloaded function, and convert the
  // quaternion and translation result into a homogeneous transform
  bool success = GetObjFromViews(obj, tr, qu, num_views, filenames, trans, quat, cameraName);
  t.setTranslation(tr);
  t.setRotation(qu.getRotMat());
  return success;
}

bool ObjRecComm::GetObjFromViews(int &obj, HomogTransf &t,
        const std::vector<std::string> filenames,
        const std::vector<double> trans,
        const std::vector<double> quat, const std::string cameraName)
{
  double pose[7];
  bool success = GetObjFromViews(obj, pose, filenames, trans, quat, cameraName);
  t = HomogTransf(pose);
  return success;
}


bool ObjRecComm::GetObjFromViews(int &obj, double pose[7], 
        const std::vector<std::string> filenames,
        const std::vector<double> trans,
        const std::vector<double> quat, const std::string cameraName)
{
  objRec_GetObjFromViews_srv.request.num_views = filenames.size();
  objRec_GetObjFromViews_srv.request.cameraName = cameraName;
  objRec_GetObjFromViews_srv.request.trans = trans;
  objRec_GetObjFromViews_srv.request.quat = quat;
  objRec_GetObjFromViews_srv.request.filenames = filenames;

  bool success = handle_objRec_GetObjFromViews.call(objRec_GetObjFromViews_srv);

  pose[0] = objRec_GetObjFromViews_srv.response.trans[0];
  pose[1] = objRec_GetObjFromViews_srv.response.trans[1];
  pose[2] = objRec_GetObjFromViews_srv.response.trans[2];
  pose[3] = objRec_GetObjFromViews_srv.response.quat[0];
  pose[4] = objRec_GetObjFromViews_srv.response.quat[1];
  pose[5] = objRec_GetObjFromViews_srv.response.quat[2];
  pose[6] = objRec_GetObjFromViews_srv.response.quat[3];

  obj = objRec_GetObjFromViews_srv.response.objNum;

  return success;
}

bool ObjRecComm::GetObjFromCameras(int &obj, HomogTransf &t,
        const std::vector<std::string> filenames,
        const std::vector<std::string> cameraNames,
        bool onTable)
{
  double pose[7];
  bool success = GetObjFromCameras(obj, pose, filenames, cameraNames, onTable);
  t = HomogTransf(pose);
  return success;
}


bool ObjRecComm::GetObjFromCameras(int &obj, double pose[7], 
        const std::vector<std::string> filenames,
        const std::vector<std::string> cameraNames,
        bool onTable)
{
  objRec_GetObjFromCameras_srv.request.num_cameras = filenames.size();
  objRec_GetObjFromCameras_srv.request.filenames = filenames;
  objRec_GetObjFromCameras_srv.request.cameraNames = cameraNames;
  objRec_GetObjFromCameras_srv.request.onTable = onTable;

  bool success = handle_objRec_GetObjFromCameras.call(objRec_GetObjFromCameras_srv);

  pose[0] = objRec_GetObjFromCameras_srv.response.trans[0];
  pose[1] = objRec_GetObjFromCameras_srv.response.trans[1];
  pose[2] = objRec_GetObjFromCameras_srv.response.trans[2];
  pose[3] = objRec_GetObjFromCameras_srv.response.quat[0];
  pose[4] = objRec_GetObjFromCameras_srv.response.quat[1];
  pose[5] = objRec_GetObjFromCameras_srv.response.quat[2];
  pose[6] = objRec_GetObjFromCameras_srv.response.quat[3];

  obj = objRec_GetObjFromCameras_srv.response.objNum;

  return success;
}

// Set up the node to constantly stream the object pose or not
bool ObjRecComm::SetStream(const bool streaming, const std::string cameraName)
{
  objRec_SetStream_srv.request.streaming = streaming;
  objRec_SetStream_srv.request.cameraName = cameraName;
  return handle_objRec_SetStream.call(objRec_SetStream_srv);
}

// Save an RGB-D image from the kinect. Specify whether or not you want
// the point cloud to be binary, and the function returns the name and
// location of the file the node saved the point cloud file to
bool ObjRecComm::SavePoints(std::string &filename, const bool use_filter, const bool is_binary, const std::string cameraName)
{
  objRec_SavePoints_srv.request.is_binary = is_binary;
  objRec_SavePoints_srv.request.use_filter = use_filter;
  objRec_SavePoints_srv.request.cameraName = cameraName;
  if (handle_objRec_SavePoints.call(objRec_SavePoints_srv))
  {
    filename = objRec_SavePoints_srv.response.filename;
    return true;
  }
  return false;
}

// Set up parameters used in feature matching. This is useful when tuning
bool ObjRecComm::SetParams(const double normal_radius, const double feature_radius,
    const int num_samples, const double min_sample_dist, 
    const int k_correspondences)
{
  objRec_SetParams_srv.request.normal_radius = normal_radius;
  objRec_SetParams_srv.request.feature_radius = feature_radius;
  objRec_SetParams_srv.request.num_samples = num_samples;
  objRec_SetParams_srv.request.min_sample_dist = min_sample_dist;
  objRec_SetParams_srv.request.k_correspondences = k_correspondences;

  return handle_objRec_SetParams.call(objRec_SetParams_srv);
}

// Set a guess for where the object is in world coordinates. Doing this
// will cause the node to not use feature matching as its first step in
// the localization pipeline
bool ObjRecComm::SetGuess(const double trans[3], const double quat[4])
{
  objRec_SetGuess_srv.request.use_guess = true;

  for (int i=0; i < 3; i++)
    objRec_SetGuess_srv.request.trans[i] = trans[i];
  for (int i=0; i < 4; i++)
    objRec_SetGuess_srv.request.quat[i] = quat[i];

  return handle_objRec_SetGuess.call(objRec_SetGuess_srv);
}

// Set a guess for where the object is in world coordinates. Doing this
// will cause the node to not use feature matching as its first step in
// the localization pipeline
bool ObjRecComm::SetGuess(const Vec trans, const Quaternion quat)
{  
  objRec_SetGuess_srv.request.use_guess = true;

  for (int i=0; i < 3; i++)
    objRec_SetGuess_srv.request.trans[i] = trans[i];
  for (int i=0; i < 4; i++)
    objRec_SetGuess_srv.request.quat[i] = quat[i];

  return handle_objRec_SetGuess.call(objRec_SetGuess_srv);
}

// Set a guess for where the object is in world coordinates. Doing this
// will cause the node to not use feature matching as its first step in
// the localization pipeline
bool ObjRecComm::SetGuess(const HomogTransf t)
{
  return SetGuess(t.getTranslation(), t.getRotation().getQuaternion());
}

// This function will clear the current guess, and get the node to
// start using feature matching again
bool ObjRecComm::ClearGuess()
{
  objRec_SetGuess_srv.request.use_guess = false;

  return handle_objRec_SetGuess.call(objRec_SetGuess_srv);
}

// Dynamically set the camera frame. This is useful when doing camera
// calibration
bool ObjRecComm::SetCamera(const double trans[3], const double quat[4], const std::string cameraName)
{
  for (int i=0; i < 3; i++)
    objRec_SetCamera_srv.request.trans[i] = trans[i];
  for (int i=0; i < 4; i++)
    objRec_SetCamera_srv.request.quat[i] = quat[i];

  objRec_SetCamera_srv.request.cameraName = cameraName;

  return handle_objRec_SetCamera.call(objRec_SetCamera_srv);
}

// Dynamically set the camera frame. This is useful when doing camera
// calibration
bool ObjRecComm::SetCamera(const Vec trans, const Quaternion quat, const std::string cameraName)
{  
  return SetCamera(trans.v, quat.v, cameraName);
}

// Dynamically set the camera frame. This is useful when doing camera
// calibration
bool ObjRecComm::SetCamera(const HomogTransf t, const std::string cameraName)
{
  return SetCamera(t.getTranslation().v, t.getRotation().getQuaternion().v, cameraName);
}

// Set the preferred orientation of the object with respect to the
// world frame. This is useful when there are symmetries in the object
// that allow the vision system to return a number of different poses
bool ObjRecComm::SetPrefOrient(const double quat[4])
{  
  objRec_SetPrefOrient_srv.request.use_pref_orient = true;
  for (int i=0; i < 4; i++)
    objRec_SetPrefOrient_srv.request.quat[i] = quat[i];
  return handle_objRec_SetPrefOrient.call(objRec_SetPrefOrient_srv);
}

// Set the preferred orientation of the object with respect to the
// world frame. This is useful when there are symmetries in the object
// that allow the vision system to return a number of different poses
bool ObjRecComm::SetPrefOrient(const Quaternion quat)
{  
  return SetPrefOrient(quat.v);
}

// Clear any orientation preference
bool ObjRecComm::ClearPrefOrient()
{
  objRec_SetPrefOrient_srv.request.use_pref_orient = false;
  return handle_objRec_SetPrefOrient.call(objRec_SetPrefOrient_srv);
}

// Tell the vision system what object to look for. If this is not set,
// then the vision system will try to match every object in its library
// with the point cloud, and then return the best one.
bool ObjRecComm::SetObject(const int obj)
{
  objRec_SetObject_srv.request.use_object = true;
  objRec_SetObject_srv.request.objNum = obj;

  return handle_objRec_SetObject.call(objRec_SetObject_srv);
}

// Clear any object preference
bool ObjRecComm::ClearObject()
{
  objRec_SetObject_srv.request.use_object = false;

  return handle_objRec_SetObject.call(objRec_SetObject_srv);
}

