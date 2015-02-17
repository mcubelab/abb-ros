//
// File Name: objRec_comm.h
//
// Author: Robbie Paolini
//
// Last Modified: 10/15/2013
// 
// This is the shorthand library class definition file for the object
// recognition node. See the comments in objRec_node.h for more information
//

#ifndef OBJREC_COMM_H
#define OBJREC_COMM_H

#include <ros/ros.h>
#include <string>
#include <vector>

#include <objRec_comm/objRec_GetObject.h>
#include <objRec_comm/objRec_GetObjFromViews.h>
#include <objRec_comm/objRec_GetObjFromCameras.h>
#include <objRec_comm/objRec_SetBounds.h>
#include <objRec_comm/objRec_SetStream.h>
#include <objRec_comm/objRec_SavePoints.h>
#include <objRec_comm/objRec_SetParams.h>
#include <objRec_comm/objRec_SetGuess.h>
#include <objRec_comm/objRec_SetCamera.h>
#include <objRec_comm/objRec_SetIgnoreRegions.h>
#include <objRec_comm/objRec_SetPrefOrient.h>
#include <objRec_comm/objRec_SetObject.h>

#include <objRec_comm/objRec_ObjPos.h>

#include <matVec/matVec.h>

// Object recognition namespace so we don't pollute the global namespace
namespace RecObj
{
  // An enumerated list of objects to recognize
  enum Type
  {
    ARCH = 0,
    BIG_SPHERE,
    BIG_TRIANGLE,
    OLD_TRIANGLE,
    CYLINDER,
    LONG_BLOCK,
    RECTANGLE,
    SMALL_BLOCK,
    SMALL_CUBE,
    SMALL_SPHERE,
    SMALL_TRIANGLE,
    SQUARE_BLOCK,
    NUM_REC_OBJ
  };
  
  // Object description file names for our known objects
  static const std::string fileNames[NUM_REC_OBJ] = 
  {
    "arch.objd",            // ARCH
    "big_sphere.objd",      // BIG_SPHERE
    "big_triangle.objd",    // BIG_TRIANGLE
    "old_triangle.objd",    // OLD_TRIANGLE
    "cylinder.objd",        // CYLINDER
    "long_block.objd",      // LONG_BLOCK
    "rectangle.objd",       // RECTANGLE
    "small_block.objd",     // SMALL_BLOCK
    "small_cube.objd",      // SMALL_CUBE
    "small_sphere.objd",    // SMALL_SPHERE
    "small_triangle.objd",  // SMALL_TRIANGLE
    "square_block.objd"     // SQUARE_BLOCK
  };
}


class ObjRecComm
{
  public:
    ObjRecComm();
    ObjRecComm(ros::NodeHandle* np);
    ~ObjRecComm();

    // Function to subscribe to ROS services
    void subscribe(ros::NodeHandle* np);

    // Shutdown service clients
    void shutdown();

    /////////////////////////////////////
    // Services

    bool SetIgnoreRegions(const int num_regions, const double *widths, 
			  const double *trans, const double *quats);
    bool SetIgnoreRegions(const int num_regions, const double *widths, 
			  const Vec *trans, const Quaternion *quats);
    bool SetIgnoreRegions(const int num_regions, const double *widths, 
			  const HomogTransf *t);

    bool SetIgnoreRegions(const std::vector<double> widths, 
        const std::vector<HomogTransf> t);

    // Set the bounds of a rectangular prism where we should look for points 
    // in our point cloud. The rectangular prism in 3d world coordinates 
    // is specified by: a rectangular prism centered at the origin, with a
    // corner in the positive quadrant specified (x,y,z) ['widths']. Then, the
    // prism is moved by a rigid-body transform (['trans', 'quat'], ['t']).
    // See the comments in objRec_node.cpp for more details.
    bool SetBounds(const double widths[3], const double trans[3], const double quat[4]);
    bool SetBounds(const double widths[3], const Vec trans, const Quaternion quat);
    bool SetBounds(const double widths[3], const HomogTransf t);
    
    // Take a depth picture with the kinect, and return the world pose of
    // an object. 
    bool GetObject(int &obj, double trans[3], double quat[4], const std::string cameraName="", bool onTable = false);
    bool GetObject(int &obj, Vec &trans, Quaternion &quat, const std::string cameraName="", bool onTable = false);
    bool GetObject(int &obj, HomogTransf &t, const std::string cameraName="", bool onTable = false);

    // Get the world pose of an object from a number of saved point clouds.
    // It's assumed that the camera is fixed, so each view represents a
    // rigid body transformation of the object in space. By sending a list
    // of point cloud files, and the transforms of the object for each of
    // the views, the node will calculate the pose of the object in world
    // coordinates, at the time of the first view.
    bool GetObjFromViews(int &obj, double t[3], double q[4], const int num_views, 
        const std::string *filenames, const double *trans, const double *quat, const std::string cameraName="");

    bool GetObjFromViews(int &obj, Vec &t, Quaternion &q, const int num_views, 
        const std::string *filenames, const double *trans, const double *quat, const std::string cameraName="");

    bool GetObjFromViews(int &obj, HomogTransf &t, const int num_views, 
        const std::string *filenames, const double *trans, const double *quat, const std::string cameraName="");

    // std::vector form. filenames is of length "num_views", trans is of
    // length num_views*3, and quat is of length num_views*4
    bool GetObjFromViews(int &obj, HomogTransf &t, 
        const std::vector<std::string> filenames,
        const std::vector<double> trans,
        const std::vector<double> quats, const std::string cameraName="");
    bool GetObjFromViews(int &obj, double pose[7], 
        const std::vector<std::string> filenames,
        const std::vector<double> trans,
        const std::vector<double> quats, const std::string cameraName="");

    bool GetObjFromCameras(int &obj, HomogTransf &t, 
        const std::vector<std::string> filenames, const std::vector<std::string> cameraNames, bool onTable = false);
    bool GetObjFromCameras(int &obj, double pose[7], 
        const std::vector<std::string> filenames, const std::vector<std::string> cameraNames, bool onTable = false);

    // Set up the node to constantly stream the object pose or not
    bool SetStream(const bool streaming, const std::string cameraName="");

    // Save an RGB-D image from the kinect. Specify whether or not you want
    // the point cloud to be binary, and the function returns the name and
    // location of the file the node saved the point cloud file to
    bool SavePoints(std::string &filename, const bool use_filter=false, const bool is_binary=true, const std::string cameraName="");

    // Set up parameters used in feature matching. This is useful when
    // tuning
    bool SetParams(const double normal_radius, const double feature_radius,
        const int num_samples, const double min_sample_dist, 
        const int k_correspondences);

    // Set a guess for where the object is in world coordinates. Doing this
    // will cause the node to not use feature matching as its first step in
    // the localization pipeline
    bool SetGuess(const double trans[3], const double quat[4]);
    bool SetGuess(const Vec trans, const Quaternion quat);
    bool SetGuess(const HomogTransf t);
    // This function will clear the current guess, and get the node to
    // start using feature matching again
    bool ClearGuess();

    // Dynamically set the camera frame. This is useful when doing camera
    // calibration
    bool SetCamera(const double trans[3], const double quat[4], const std::string cameraName="");
    bool SetCamera(const Vec trans, const Quaternion quat, const std::string cameraName="");
    bool SetCamera(const HomogTransf t, const std::string cameraName="");

    // Set the preferred orientation of the object with respect to the
    // world frame. This is useful when there are symmetries in the object
    // that allow the vision system to return a number of different poses
    bool SetPrefOrient(const double quat[4]);
    bool SetPrefOrient(const Quaternion quat);
    // Clear any orientation preference
    bool ClearPrefOrient();

    // Tell the vision system what object to look for. If this is not set,
    // then the vision system will try to match every object in its library
    // with the point cloud, and then return the best one.
    bool SetObject(const int obj);
    // Clear any object preference
    bool ClearObject();

  private:
    // ROS Shorthand
    ros::ServiceClient handle_objRec_GetObject;
    ros::ServiceClient handle_objRec_GetObjFromViews;
    ros::ServiceClient handle_objRec_GetObjFromCameras;
    ros::ServiceClient handle_objRec_SetBounds;
    ros::ServiceClient handle_objRec_SetStream;
    ros::ServiceClient handle_objRec_SavePoints;
    ros::ServiceClient handle_objRec_SetParams;
    ros::ServiceClient handle_objRec_SetGuess;
    ros::ServiceClient handle_objRec_SetCamera;
    ros::ServiceClient handle_objRec_SetIgnoreRegions;
    ros::ServiceClient handle_objRec_SetPrefOrient;
    ros::ServiceClient handle_objRec_SetObject;

    objRec_comm::objRec_GetObject objRec_GetObject_srv;
    objRec_comm::objRec_GetObjFromViews objRec_GetObjFromViews_srv;
    objRec_comm::objRec_GetObjFromCameras objRec_GetObjFromCameras_srv;
    objRec_comm::objRec_SetBounds objRec_SetBounds_srv;
    objRec_comm::objRec_SetStream objRec_SetStream_srv;
    objRec_comm::objRec_SavePoints objRec_SavePoints_srv;
    objRec_comm::objRec_SetParams objRec_SetParams_srv;
    objRec_comm::objRec_SetGuess objRec_SetGuess_srv;
    objRec_comm::objRec_SetCamera objRec_SetCamera_srv;
    objRec_comm::objRec_SetIgnoreRegions objRec_SetIgnoreRegions_srv;
    objRec_comm::objRec_SetPrefOrient objRec_SetPrefOrient_srv;
    objRec_comm::objRec_SetObject objRec_SetObject_srv;
};

#endif // OBJREC_COMM_H
