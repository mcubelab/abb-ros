
#include <ros/ros.h>
#include <matVec/matVec.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>

#include <limits>
#include <fstream>
#include <vector>
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

#include <string>
#include <ctime>


#include <tableVision_comm/tableVision_comm.h>
#include <objRec_comm/objRec_comm.h>


#define NUM_IMAGES 3

//#define VISUALIZE

#ifdef VISUALIZE
// Includes needed for visualizing point clouds
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#endif

static const std::string trueFrameKinect = "upperLeftKinect";
static const std::string sharedKinect = "upperCenterKinect";

// Bounds in meters
static const double tableBoundWidths[3] = {0.6, 0.3, 0.14};
static const double tableBoundTrans[3] = {0.605, 0.305, 0.120};
static const double tableBoundQuat[4] = {1.0, 0.0, 0.0, 0.0};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "static_calibration");

  ros::NodeHandle node;

  TableVisionComm tableVision(&node);
  ObjRecComm objRec(&node);

  int numCameras;
  std::vector<std::string> cameraNames;
  std::vector<int> cameraIdxs;
  std::vector<HomogTransf> initialFrames;
  std::vector<HomogTransf> finalFrames;
  int sharedKinectIdx;
  int trueKinectIdx;
  std::vector<std::vector<std::string> > pcdFiles;

  // First, read in parameters for names of the kinects
  XmlRpc::XmlRpcValue my_list;
  XmlRpc::XmlRpcValue inner_list;
  double pose[7];

  node.getParam("/tableVision/cameraNames", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  numCameras = my_list.size();
  cameraNames.resize(numCameras);
  cameraIdxs.resize(numCameras);
  initialFrames.resize(numCameras);
  finalFrames.resize(numCameras);
  pcdFiles.resize(numCameras);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    cameraNames[i] = static_cast<std::string>(my_list[i]);
    pcdFiles[i].resize(NUM_IMAGES);
    std::cout << cameraNames[i] << std::endl;
    if (cameraNames[i] == sharedKinect)
    {
      sharedKinectIdx = i;
      std::cout << "shared kinect: " << cameraNames[i] << std::endl;
    }
    if (cameraNames[i] == trueFrameKinect)
    {
      trueKinectIdx = i;
      std::cout << "true kinect: " << cameraNames[i] << std::endl;
    }
  }

  // Next, get the initial pose guess for each of the kinects
  node.getParam("/objRec/cameraNames", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < my_list.size(); ++i) 
  {
    std::string cam_name;
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    cam_name = static_cast<std::string>(my_list[i]);
    for (int j=0; j < numCameras; ++j)
    {
      if (cameraNames[j] == cam_name)
      {
        cameraIdxs[j] = i;
      }
    }
  }

  // Now, get the initial guesses for the frames of the kinects
  node.getParam("objRec/cameraFrames", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (size_t i = 0; i < my_list.size(); ++i)
  {
    int curCamera = -1;
    for (int j=0; j < numCameras; ++j)
    {
      if (cameraIdxs[j] == i)
      {
        curCamera = j;
        break;
      }
    }
    if (curCamera == -1)
      continue;

    inner_list = my_list[i];
    ROS_ASSERT(inner_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(inner_list.size() == 7);
    for (int32_t j = 0; j < inner_list.size(); ++j)
    {
      ROS_ASSERT(inner_list[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      pose[j] = static_cast<double>(inner_list[j]);
    }
    initialFrames[curCamera] = HomogTransf(pose);
  }


  // Let's set up our desired region of interest
  objRec.SetBounds(tableBoundWidths, tableBoundTrans, tableBoundQuat);


  // Now, let's take images from each of our table kinects
  for (int i=0; i < NUM_IMAGES; ++i)
  {
    for (int j=0; j < numCameras; ++j)
    {
      tableVision.SavePoints(pcdFiles[j][i], true, true, cameraNames[j]);
    }
  }



  


/*

  //HomogTransf trueLeftKinect(Quaternion("0.2587, 0.6602, 0.6609, 0.2460"), Vec("-0.1596, 0.3891, 0.7070",3));
  HomogTransf trueLeftKinect(Quaternion("1 0 0 0"), Vec("0 0 0",3));
  Quaternion q = trueLeftKinect.getQuaternion();
  q.normalize();
  trueLeftKinect.setQuaternion(q);


*/

  
  //pcd_files[0] = "/home/simplehands/Desktop/extrinsic_cal/upperCenterKinectImage.pcd";
  //pcd_files[1] = "/home/simplehands/Desktop/extrinsic_cal/upperRightKinectImage.pcd";
  //pcd_files[2] = "/home/simplehands/Desktop/extrinsic_cal/upperLeftKinectImage.pcd";
  //pcd_files[0] = "/home/simplehands/Desktop/extrinsic_cal/points__2014_06_05__17_04_44_208.pcd";
  //pcd_files[1] = "/home/simplehands/Desktop/extrinsic_cal/points__2014_06_05__17_04_19_280.pcd";
  //pcd_files[2] = "/home/simplehands/Desktop/extrinsic_cal/points__2014_06_05__17_04_30_763.pcd";

  /*
  pcd_files[0][0] = "/home/simplehands/Desktop/objRec/pointsFolder/points__2014_06_18__09_58_29_679.pcd";
  pcd_files[0][1] = "/home/simplehands/Desktop/objRec/pointsFolder/points__2014_06_18__09_58_33_810.pcd";
  pcd_files[0][2] = "/home/simplehands/Desktop/objRec/pointsFolder/points__2014_06_18__09_58_35_539.pcd";
  pcd_files[1][0] = "/home/simplehands/Desktop/objRec/pointsFolder/points__2014_06_18__09_59_13_796.pcd";
  pcd_files[1][1] = "/home/simplehands/Desktop/objRec/pointsFolder/points__2014_06_18__09_59_16_435.pcd";
  pcd_files[1][2] = "/home/simplehands/Desktop/objRec/pointsFolder/points__2014_06_18__09_59_20_635.pcd";
  pcd_files[2][0] = "/home/simplehands/Desktop/extrinsic_cal/points__2014_06_05__17_32_34_425.pcd";
  pcd_files[2][1] = "/home/simplehands/Desktop/extrinsic_cal/points__2014_06_05__17_32_37_384.pcd";
  pcd_files[2][2] = "/home/simplehands/Desktop/extrinsic_cal/points__2014_06_05__17_32_40_086.pcd";

  HomogTransf initTrans[NUM_KINECTS];
  initTrans[0] = HomogTransf(Quaternion("1 0 0 0"), Vec("0 0 0",3));
  initTrans[1] = HomogTransf(Quaternion("1 0 0 0"), Vec("0 0 0",3));
  initTrans[2] = HomogTransf(Quaternion("1 0 0 0"), Vec("0 0 0",3));

  */
  //initTrans[0] = HomogTransf(Quaternion("0.00730802 0.0197376 0.964314 0.263923"), Vec("0.589978 -0.189724 0.915814",3));
  //initTrans[1] = HomogTransf(Quaternion("-0.252077 -0.649369 0.666064 0.266712"), Vec("1.38481 0.358726 0.702303",3));
  //initTrans[2] = HomogTransf(Quaternion("0.2587, 0.6602, 0.6609, 0.2460"), Vec("-0.1596, 0.3891, 0.7070",3));

  //initTrans[0] = HomogTransf(Quaternion("-0.0205 0.0105 0.9652 0.2606"), Vec("0.6146 -0.1785 0.9006",3));
  //initTrans[1] = HomogTransf(Quaternion("0.2534 0.6521 -0.6620 -0.2690"), Vec("1.3730 0.3699 0.7090",3));
  //initTrans[2] = HomogTransf(Quaternion("0.2601 0.6543 0.6619 0.2571"), Vec("-0.1619 0.4021 0.7054",3));

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> trans_pts(numCameras);

  Quaternion q;
  for (int i=0; i < numCameras; ++i)
  {
    q = initialFrames[i].getQuaternion();
    q.normalize();
    initialFrames[i].setQuaternion(q);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pts (new pcl::PointCloud<pcl::PointXYZ>);

    for (int j=0; j < NUM_IMAGES; ++j)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr tp (new pcl::PointCloud<pcl::PointXYZ>);
      // Open the point cloud file
      ROS_INFO("Processing file: %s", pcdFiles[i][j].c_str());
      if( pcl::io::loadPCDFile<pcl::PointXYZ> (pcdFiles[i][j].c_str(), *tp) == -1)
      {
        ROS_ERROR("Unable to process file!");
        return false;
      }
      else
      {
        ROS_INFO("Successfully read file");
      }
      *pts += *tp;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filt_pts (new pcl::PointCloud<pcl::PointXYZ>);


    // ... and downsampling the point cloud
    const float voxel_grid_size = 0.004f;
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setInputCloud (pts);
    vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
    //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
    vox_grid.filter (*tempCloud);
    filt_pts = tempCloud;


    pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>);
    trans_pts[i] = temp;

    if (i != sharedKinectIdx)
    {
      HomogTransf overallTransf = initialFrames[sharedKinectIdx].inv() * initialFrames[i];

      // Use our computed transform to move all of our points 
      //  into the first point cloud's frame
      Eigen::Matrix4f transform; 
      for (int j=0; j<4;j++)
        for (int k=0; k<4;k++)
          transform(j,k) = overallTransf[j][k];

      std::cout << transform << std::endl;

      pcl::transformPointCloud(*filt_pts, *trans_pts[i], transform);
    }
    else
    {
      // If this is the original point cloud, there's no change
      trans_pts[i] = filt_pts;
    }

    ROS_INFO("Successfully transformed");
  }

  for (int i=0; i < numCameras; ++i)
  {
    if (i == sharedKinectIdx)
      continue;

    // Now that we have a guess, we will use iterative closest point to refine our estimate
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(trans_pts[i]);
    icp.setInputTarget(trans_pts[sharedKinectIdx]);

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.03);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (500);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    icp.setRANSACOutlierRejectionThreshold (0.01);
    icp.setRANSACIterations (500);

    std::cout << "ICP TIME!!!" << std::endl;
    // Come up with the best final alignment
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*Final);

    std::cout << "ICP Results" << std::endl;
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
      icp.getFitnessScore() << std::endl;

    // Obtain the transformation that aligned our initial guess to the final 
    // transform. Note that we multiply by the initial transform to get the 
    // full transform of where the object is.
    Eigen::Matrix4f refined_T = icp.getFinalTransformation ();

    std::cout << refined_T << std::endl;

    RotMat r;
    double trans[3];
    for (int j=0; j<3; j++)
    {
      trans[j] = refined_T(j,3);
      for (int k=0; k<3; k++)
        r[j][k] = refined_T(j,k);
    }

    q = r.getQuaternion();
    q.normalize();

    // Compute the object pose with respect to the world
    HomogTransf ref_cal(q.getRotMat(), Vec(trans,3));
    finalFrames[i] =  initialFrames[sharedKinectIdx] * ref_cal * initialFrames[sharedKinectIdx].inv() * initialFrames[i];

    std::cout << "final frame: " << finalFrames[i] << std::endl;
    std::cout << "Quat: " << finalFrames[i].getQuaternion() << " Vec: " << finalFrames[i].getTranslation() << std::endl;

#ifdef VISUALIZE
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    int v1(0);
    viewer->createViewPort(0.0,0.0,0.5,1.0,v1);
    viewer->setBackgroundColor(255,255,255,v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_target_v1 (trans_pts[sharedKinectIdx],0,255,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_source_v1 (trans_pts[i],255,0,0);
    viewer->addPointCloud<pcl::PointXYZ> (trans_pts[sharedKinectIdx],color_target_v1,"target cloud v1",v1);
    viewer->addPointCloud<pcl::PointXYZ> (trans_pts[i],color_source_v1,"source cloud v1",v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"target cloud v1");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"source cloud v1");

    viewer->addCoordinateSystem (0.2,v1);

    int v2(0);
    viewer->createViewPort(0.5,0.0,1.0,1.0,v2);
    viewer->setBackgroundColor(255,255,255,v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_target_v2 (trans_pts[sharedKinectIdx],0,255,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_source_v2 (Final,255,0,0);
    viewer->addPointCloud<pcl::PointXYZ> (trans_pts[sharedKinectIdx],color_target_v2,"target cloud v2",v2);
    viewer->addPointCloud<pcl::PointXYZ> (Final,color_source_v2,"source cloud v2",v2);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"target cloud v2");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"source cloud v2");

    viewer->addCoordinateSystem (0.2,v2);

    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

#endif

  }

  finalFrames[sharedKinectIdx] = initialFrames[sharedKinectIdx];

  HomogTransf trueFrameInitial = initialFrames[trueKinectIdx];
  HomogTransf trueFrameFinal = finalFrames[trueKinectIdx];


  for (int i=0; i < numCameras; ++i)
  {
    finalFrames[i] = trueFrameInitial * trueFrameFinal.inv() * finalFrames[i];
    Vec t_f = finalFrames[i].getTranslation();
    Quaternion q_f = finalFrames[i].getQuaternion();
    std::cout << i << "(" << cameraNames[i] << "): [" << t_f[0] << ", " << t_f[1] << ", " << t_f[2] << ", " << q_f[0] << ", " << q_f[1] << ", " << q_f[2] << ", " << q_f[3] << "]" << std::endl;
    //std::cout << i << "(" << cameraNames[i] << "): " << finalFrames[i].getTranslation() << finalFrames[i].getQuaternion() << std::endl;
  }

  return 0;
}
