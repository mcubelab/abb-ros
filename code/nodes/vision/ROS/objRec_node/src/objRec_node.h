//
// File Name: objRec_node.h
//
// Author: Robbie Paolini
//
// Last Modified: 2/12/2014
// 
// This is the class definition for the object recognition node.
//
// The object recognition node uses point cloud data to determine 
// the pose of a known object within the cloud. Initially, known object 
// models are loaded into the system, and relevant features are computed. 
// The recognition pipeline is the following:
//
// 1) Filter the point cloud based on a bounding box and 
//     ignore regions to make it more reasonable
// 2) Compute features on the point cloud, and match it to 
//     features on our object to get a good initial guess
// 3) Use iterative closest point (ICP) to refine our guess 
//     and come up with the final pose estimate of the object
//
// The node subscribes to point cloud data, and allows the user to save point 
// clouds to a folder for later use. In addition, the user can specify 
// multiple point cloud files from different views, which can be stitched 
// together to form one large point cloud. The bounding box, and an initial 
// guess can be set if desired. Finally, the node allows feature parameters 
// and the camera frame to be changed, which is useful during calibration. 
//
// Note: This node is designed to be used by a fixed camera, that will never 
// move in relation to the world frame. Unless specified otherwise, all units 
// in this node are in meters.


#ifndef OBJREC_NODE_H
#define OBJREC_NODE_H

//////////////////////////////////////////////////////////////////////////
// Define statements that allow us to change what the code does slightly

//#define OBJREC_ROUGH_ESTIMATE // Only use SAC-IA. Don't use ICP to refine our estimate

//#define OBJREC_DEBUG  // Show different steps in the analysis process

#define SHOW_MULTI  // Show how each view is fused together

#define OBJREC_SHOW_RES // Show the results of our analysis

//////////////////////////////////////////////////////////////////////////
// Include files

#include <pthread.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <objRec_comm/objRec_comm.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <matlab_comm/matlab_comm.h>
#include <matVec/matVec.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>

#include <limits>
#include <fstream>
#include <vector>
//#include <array>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>



#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>



#include <pcl/registration/registration.h>

#include <boost/array.hpp>
#include <string>
#include <ctime>

// Includes needed for visualizing point clouds
#if defined(OBJREC_DEBUG) || defined(OBJREC_SHOW_RES) || defined(SHOW_MULTI)

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#endif

#include "RecognitionObject.h"

using namespace std;


//////////////////////////////////////////////////////////////////////////
// Definition of some constants

// Maximum file name length
#define MAX_FILE_BUFFER 512

// Thresholds to determine if ICP converged properly
#define CLOSE_RAD 0.004       // How close a point has to be between object and point cloud to be considered a match

// NOTE: For these 2 thresholds, if either is satisfied, then we return
// success.
#define MODEL_MATCH_THRESH 0.5 // Percentage of object model points that must match the point cloud for us to consider ICP to have converged
#define CLOUD_MATCH_THRESH 0.5 // Percentage of point cloud points that must match the object model for us to consider ICP to have converged

// Minimum number of points we will look at before doing matching. TODO: Remove this hack
#define MIN_POINT_THRESH 1000

// Voxel size when filtering multiple point clouds
#define VOXEL_SIZE 0.004f


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// Class definition for object recognition node. Handles service calls 
//  to compute object pose from a single or multiple views, set up bounds 
//  for where to look for an object, save point clouds, and change the 
//  camera pose and parameters, which is useful during calibration
class ObjectRecognition
{
  public:
    // Constructor/Destructors
    ObjectRecognition();
    ObjectRecognition(ros::NodeHandle *n);
    virtual ~ObjectRecognition();

    // Services
    // Get pose of object from a single point cloud
    bool objRec_GetObject(objRec_comm::objRec_GetObject::Request& req, 
        objRec_comm::objRec_GetObject::Response& res);

    // Get pose of an object from multiple point clouds 
    //  representing different views
    bool objRec_GetObjFromViews(objRec_comm::objRec_GetObjFromViews::Request& req, 
        objRec_comm::objRec_GetObjFromViews::Response& res);

    // Get pose of an object from multiple point clouds 
    //  representing different views
    bool objRec_GetObjFromCameras(objRec_comm::objRec_GetObjFromCameras::Request& req, 
        objRec_comm::objRec_GetObjFromCameras::Response& res);

    // Set a rectangular box bound in 3d space for where to look for object
    bool objRec_SetBounds(objRec_comm::objRec_SetBounds::Request& req, 
        objRec_comm::objRec_SetBounds::Response& res);

    // Set a list of rectangular prisms to ignore (the hand's fingers)
    bool objRec_SetIgnoreRegions(objRec_comm::objRec_SetIgnoreRegions::Request& req,
				 objRec_comm::objRec_SetIgnoreRegions::Response& res);

    // Choose whether or not to stream object pose from data (TODO: Make much faster)
    bool objRec_SetStream(objRec_comm::objRec_SetStream::Request& req, 
        objRec_comm::objRec_SetStream::Response& res);

    // Service to save points seen from system to a point cloud file
    bool objRec_SavePoints(objRec_comm::objRec_SavePoints::Request& req,
        objRec_comm::objRec_SavePoints::Response& res);

    // Set parameters used for feature matching
    bool objRec_SetParams(objRec_comm::objRec_SetParams::Request& req,
        objRec_comm::objRec_SetParams::Response& res);

    // Set guess of where we think the object is. Bypasses feature matching
    bool objRec_SetGuess(objRec_comm::objRec_SetGuess::Request& req,
        objRec_comm::objRec_SetGuess::Response& res);

    // Set the camera frame. Useful for calibration
    bool objRec_SetCamera(objRec_comm::objRec_SetCamera::Request& req,
        objRec_comm::objRec_SetCamera::Response& res);

    // Set the preferred orientation. Useful for getting consistent results
    // when the object contains symmetries
    bool objRec_SetPrefOrient(objRec_comm::objRec_SetPrefOrient::Request& req,
        objRec_comm::objRec_SetPrefOrient::Response& res);

    // Set the object to look for. Can also clear the object, in which case
    // the vision system will look through all objects to find the best
    // match
    bool objRec_SetObject(objRec_comm::objRec_SetObject::Request& req,
        objRec_comm::objRec_SetObject::Response& res);

    // Callback function for use with point cloud device
    void points_callback(const sensor_msgs::PointCloud2ConstPtr& msg, size_t camNum);

    // Initialization of node
    bool init(ros::NodeHandle *n);


  private:
    // Get pose of object given a point cloud
    bool getObjectPose(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        int &objNum, double trans[3], double quat[4], size_t curCamera, unsigned int min_points=0, bool onTable = false);


    // Filter a point cloud based on the current bounds and ignore regions
    // Returns a much smaller depth point cloud, that we can look for an object in
    bool filterPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_cloud, 
        pcl::PointCloud<pcl::PointXYZ> &out_cloud, size_t curCamera);

    // Filter a point cloud based on bounds and ignore regions we specify
    bool filterPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_cloud, 
        pcl::PointCloud<pcl::PointXYZ> &out_cloud,
        double box_widths[3], double box_trans[3], double box_quat[4],
        int num_regions, std::vector<double> ignore_widths, std::vector<double> ignore_trans, std::vector<double> ignore_quats, size_t curCamera);
    bool filterPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_cloud, 
        pcl::PointCloud<pcl::PointXYZ> &out_cloud,
        double box_widths[3], double box_trans[3], double box_quat[4], size_t curCamera);

    // Filter a point cloud based on bounding planes and ignore planes which we specify
    bool filterPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_cloud, 
        pcl::PointCloud<pcl::PointXYZ> &out_cloud,
        double bound_planes[3][4], double bound_dists[3],
        int num_ignore, std::vector<std::vector<std::vector<double> > > ignore_planes, std::vector<double> ignore_dists);
    bool filterPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_cloud, 
        pcl::PointCloud<pcl::PointXYZ> &out_cloud,
        double bound_planes[3][4], double bound_dists[3]);


    // Based on the widths of our rectangular prism and its transform, 
    //  compute 3 planes representing the center axes of the bounding 
    //  box in 3d space, and the maximum distance away from each plane 
    //  where we will allow points to be.
    bool getBounds(double widths[3], double trans[3], double quat[4], 
        double bound_planes[3][4], double bound_dists[3], size_t curCamera);

    bool getIgnoreRegions(int num_regions, std::vector<double> widths, std::vector<double> trans, std::vector<double> quats, 
			  std::vector<std::vector<std::vector<double> > >& ignoreRegions_planes, std::vector<double>& ignoreRegions_dists, size_t curCamera);

    size_t getCameraIdx(std::string cameraName);

#if defined(OBJREC_DEBUG) || defined(OBJREC_SHOW_RES)
    void visualize_match(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr registration_output, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr Final, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr Final2);
#endif
#ifdef SHOW_MULTI
    void drawBoundaries(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, size_t curCamera);
#endif

    /*
  public:
    class PointsCallBack : public ObjectRecognition
    {
      friend class ObjectRecognition;
      public:
        void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
        size_t camNum_;
    };
    */

  private:
    // Booleans used to coordinate call back from point cloud device
    std::vector<bool> use_points;
    std::vector<bool> points_ready;
    std::vector<ros::Time> ask_time;

    // Most recent points from point cloud data stream
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cur_points;
    
    // Whether or not we will stream object position
    bool streaming;
    size_t streamingCam;
    
    // Locations for where to store saved points and where 
    //  our object definition files are located 
    string pointsFolder;

    // Our object library. This contains everything we need to know about
    // the objects we're trying to recognize
    std::vector<RecognitionObject> recObjs;

    // Frame of the camera with respect to the world. 
    // Used to convert points to and from the world frame.
    size_t numCameras;
    std::vector<HomogTransf> cameraFrames;
    std::vector<std::string> cameraNames;
    //Eigen::Matrix4f cameraFrame;

    // Current guess in the world frame
    HomogTransf curGuess;
    // Whether or not to use this guess
    bool use_guess;

    // Current preferred orienation
    Quaternion curOrient;
    // Whether or not to use this preferred orientation
    bool use_pref_orient;

    // Current object to look for
    int curObject;
    // Whether or not to only look for this object
    bool use_object;

    // Bounding box
    // We will store this as 3 planes intersecting at the center of the
    //  box, and then the distance to the edge of the box for each plane. 
    //  If we make the normal vector of the plane a unit vector, we can 
    //  check if a point is inside the box by doing 1 dot product, 
    //  1 addition, and 2 comparisons for each plane.
    //
    // Note that the bounding planes are in the camera frame.
    
    //double cur_planes[3][4];
    //double cur_dists[3];


    double cur_widths[3];
    double cur_trans[3];
    double cur_quat[4];

    int num_ignore_regions;
    //std::vector<std::vector<std::vector<double> > > cur_ignore_planes;
    //std::vector<double> cur_ignore_dists;
    
    std::vector<double> cur_ignore_widths;
    std::vector<double> cur_ignore_trans;
    std::vector<double> cur_ignore_quats;

    // ROS shorthand
    ros::NodeHandle *node;
    ros::Publisher handle_objRec_ObjPos;
    ros::ServiceServer handle_objRec_GetObject;
    ros::ServiceServer handle_objRec_GetObjFromViews;
    ros::ServiceServer handle_objRec_GetObjFromCameras;
    ros::ServiceServer handle_objRec_SetBounds;
    ros::ServiceServer handle_objRec_SetIgnoreRegions;
    ros::ServiceServer handle_objRec_SetStream;
    ros::ServiceServer handle_objRec_SavePoints;
    ros::ServiceServer handle_objRec_SetParams;
    ros::ServiceServer handle_objRec_SetGuess;
    ros::ServiceServer handle_objRec_SetCamera;
    ros::ServiceServer handle_objRec_SetPrefOrient;
    ros::ServiceServer handle_objRec_SetObject;
    std::vector<ros::Subscriber> sub_points;

    // Mutex for bounding box. Makes sure we aren't using 
    //  and changing the bounding box at the same time
    pthread_mutex_t boundsMutex;
    pthread_mutex_t ignoreRegionsMutex;



};

#endif // OBJREC_NODE_H
