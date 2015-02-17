//
// File Name: objRec_node.cpp
//
// Author: Robbie Paolini
//
// Last Modified: 10/15/2013
//
// Implementation for object recognition node. 
// See objRec_node.h for detailed description of node


// Include class definition
#include "objRec_node.h"

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// Simple constructors and destructors
ObjectRecognition::ObjectRecognition()
{
}

ObjectRecognition::ObjectRecognition(ros::NodeHandle *n)
{
  init(n);
}

ObjectRecognition::~ObjectRecognition()
{
  pthread_mutex_destroy(&boundsMutex);
}


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// Call back for subscription to a point cloud topic. 
//
// If we're streaming object pose, compute the object position 
//  and publish our estimate.
//
// If we're not streaming, only save the point cloud if 
//  we're using the points.
//
void ObjectRecognition::points_callback(const sensor_msgs::PointCloud2ConstPtr& msg, size_t camNum)
//void ObjectRecognition::points_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg, size_t camNum)
{
  if (this->streaming && streamingCam == camNum)
  {
    double trans[3];
    double quat[4];
    int objNum;

    // First, filter our cloud to make it less unwieldy 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr orig_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *orig_cloud);

    filterPoints(orig_cloud, *cloud_filtered, streamingCam);
    ROS_INFO("filtered");

    // Now, find the object in our point cloud
    if (getObjectPose(cloud_filtered, objNum, trans, quat, streamingCam))
    {
      objRec_comm::objRec_ObjPos msgObj;
      for (int i = 0; i < 3; i++)
        msgObj.trans[i] = trans[i];
      for (int i = 0; i < 4; i++)
        msgObj.quat[i] = quat[i];
      msgObj.objNum = objNum;

      // If we succeeded, publish the current pose
      handle_objRec_ObjPos.publish(msgObj);
    }
  }

  if (use_points[camNum])
  {
    std::cout << "message time: " << msg->header.stamp << std::endl; 
    std::cout << "ask time: " << ask_time[camNum] << std::endl; 
  }

  // If we have a request to save the points, do so
  if (use_points[camNum] && msg->header.stamp > ask_time[camNum])
  {
    ROS_INFO("KINECT");

    // Save the points
    pcl::fromROSMsg(*msg, *(cur_points[camNum]));
    //cur_points[camNum] = msg;

    // We no longer need points, and alert our other 
    // thread that the points are ready
    use_points[camNum] = false;
    points_ready[camNum] = true;
  }
}


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// Services

//////////////////////////////////////////////////////////////////////////
// Service to get object pose
// 
// This service will either analyze a specified point cloud file, or save
// an point cloud from the kinect and analyze that.
//
// It returns what object it found, and its 3d pose
//
bool ObjectRecognition::objRec_GetObject(objRec_comm::objRec_GetObject::Request& req, 
    objRec_comm::objRec_GetObject::Response& res)
{
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pts (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr file_pts (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  double trans[3];
  double quat[4];
  int objNum;

  size_t curCamera = getCameraIdx(req.cameraName);

  // First, let's acquire the desired point cloud file
  if (req.filename.length() != 0)
  {
    // If a file is requested, load it.
    ROS_INFO("Processing file: %s", req.filename.c_str());
    if( pcl::io::loadPCDFile<pcl::PointXYZ> (req.filename.c_str(), *file_pts) == -1)
    {
      res.ret = 0;
      res.msg = "Could not read points file. Unable to process";
      return false;
    }
    pts = file_pts;
  }
  else
  {
    // If not, take a picture with the kinect.
    points_ready[curCamera] = false;
    use_points[curCamera] = true;
    ask_time[curCamera] = ros::Time::now();

    ros::Rate loop_rate(60);

    while(ros::ok() && !points_ready[curCamera])
    {
      loop_rate.sleep();
    }

    pts = cur_points[curCamera];
  }

  // Now that we have our points, let's first filter the point cloud, and 
  //  then determine what object is in it and where the object is
  filterPoints(pts, *cloud_filtered, curCamera);
  ROS_INFO("filtered");

  if (!getObjectPose(cloud_filtered, objNum, trans, quat, curCamera, MIN_POINT_THRESH, req.onTable))
  {
    res.ret = 0;
    res.msg = "Unable to get object pose";
    return false;
  }

  // Save our results and return them
  res.objNum = objNum;
  for (int i = 0; i < 3; i++)
    res.trans[i] = trans[i];
  for (int i = 0; i < 4; i++)
    res.quat[i] = quat[i];

  res.ret = 1;
  return true;
}


//////////////////////////////////////////////////////////////////////////
// Service to get object pose from multiple views
// 
// This service takes in a specified number of point cloud files, as well 
// as the "frame" that each of the views were taken from, and determines 
// the pose of the object with respect to the first frame. This service is 
// perfectly suited for use when a robot moves an object around, and saves 
// point clouds at different positions. Each frame would be the tool frame 
// of the robot, and the returned object pose would be its pose in the 
// world frame, based on where it was in the first point cloud file. 
//
// The filenames are in an array, and then there are 2 other arrays, 
//  for the translation and orientation of each frame. The translation
//  array is of length 3 x N ([x1 y1 z1 x2 y2 z2 ...]) and the orientation
//  array is of length 4 x N ([q0_1 qx_1 qy_1 qz_1 ...]) where each 4
//  element block is the quaternion for that frame
//
// It returns what object it found, and its 3d pose
//

bool ObjectRecognition::objRec_GetObjFromViews(objRec_comm::objRec_GetObjFromViews::Request& req, 
    objRec_comm::objRec_GetObjFromViews::Response& res)
{
  clock_t begin_clk = clock();

  pcl::PointCloud<pcl::PointXYZ>::Ptr all_pts (new pcl::PointCloud<pcl::PointXYZ>);

#ifdef SHOW_MULTI
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_pts (new pcl::PointCloud<pcl::PointXYZRGB>);
#endif

  double trans[3];
  double quat[4];
  int objNum;

  size_t curCamera = getCameraIdx(req.cameraName);


  // Store the current bounds, in case they change while we're running 
  //  this function, to make sure we use the same bounding box 
  //  throughout all of our calculations
  double box_widths[3];
  double bound_planes[3][4];
  double bound_dists[3];
  pthread_mutex_lock(&boundsMutex);
  getBounds(cur_widths, cur_trans, cur_quat, bound_planes, bound_dists, curCamera);
  //memcpy(bound_planes, cur_planes, sizeof(double)*12);
  //memcpy(bound_dists, cur_dists, sizeof(double)*3);
  memcpy(box_widths, cur_widths, sizeof(double)*3);
  HomogTransf orig_box(Quaternion(cur_quat).getRotMat(), Vec(cur_trans,3));
  pthread_mutex_unlock(&boundsMutex);

  // Store the current ignore regions, in case they change while we're
  // running this function, to make sure we use the same ignore regions
  // throughout all of our calculations
  int num_regions;
  std::vector<std::vector<std::vector<double> > > ignore_planes;
  std::vector<double> ignore_dists;
  std::vector<double> ignore_widths;
  pthread_mutex_lock(&ignoreRegionsMutex);
  num_regions = num_ignore_regions;
  getIgnoreRegions(num_regions, cur_ignore_widths, cur_ignore_trans, cur_ignore_quats, ignore_planes, ignore_dists, curCamera);
  //ignore_planes = cur_ignore_planes;
  //ignore_dists = cur_ignore_dists;
  ignore_widths = cur_ignore_widths;
  std::vector<HomogTransf> orig_regions(num_regions);
  for (int i=0; i < num_regions; i++)
  {
    double t[3];
    double q[4];
    std::copy(cur_ignore_trans.begin()+i*3, cur_ignore_trans.begin()+(i+1)*3, t);
    std::copy(cur_ignore_quats.begin()+i*4, cur_ignore_quats.begin()+(i+1)*4, q);
    orig_regions[i] = HomogTransf(Quaternion(q).getRotMat(), Vec(t,3));
  }
  pthread_mutex_unlock(&ignoreRegionsMutex);

  // First, get our base transform, which is the frame of the first view
  for (int j=0; j < 3; j++)
    trans[j] = req.trans[j];
  for (int j=0; j < 4; j++)
    quat[j] = req.quat[j];

  HomogTransf orig(Quaternion(quat).getRotMat(), Vec(trans,3));


#ifdef SHOW_MULTI
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  int *v = new int[req.num_views];
#endif

  // First, we filter each point cloud file and aggregate it into a single 
  //  point cloud, making sure to do the transformations correctly
  for (int i=0; i < req.num_views; i++)
  {
    // Open the point cloud file
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts (new pcl::PointCloud<pcl::PointXYZ>);
    ROS_INFO("Processing file: %s", req.filenames[i].c_str());
    if( pcl::io::loadPCDFile<pcl::PointXYZ> (req.filenames[i].c_str(), *pts) == -1)
    {
      res.ret = 0;
      res.msg = "Could not read points file. Unable to process";
      return false;
    }
    else
    {
      ROS_INFO("Successfully read file");
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filt_pts (new pcl::PointCloud<pcl::PointXYZ>);

    // Use box filter to greatly reduce points
    if (i != 0)
    {
      // Given the initial bounding box, compute where the new bounding box 
      // must be based on the transformation required to move this point 
      // cloud into the original point cloud's frame

      for (int j=0; j < 3; j++)
        trans[j] = req.trans[3*i+j];
      for (int j=0; j < 4; j++)
        quat[j] = req.quat[4*i+j];

      HomogTransf h(Quaternion(quat).getRotMat(), Vec(trans,3));
      HomogTransf new_box = h * orig.inv() * orig_box;

      Quaternion q = new_box.getRotation().getQuaternion();
      Vec t = new_box.getTranslation();

      std::vector<double> ignore_trans(num_regions*3);
      std::vector<double> ignore_quats(num_regions*4);
      for (int k=0; k < num_regions; k++)
      {
        HomogTransf new_reg = h * orig.inv() * orig_regions[k];
        Quaternion iq = new_reg.getRotation().getQuaternion();
        Vec it = new_reg.getTranslation();
        std::copy(it.v, it.v+3, ignore_trans.begin()+k*3);
        std::copy(iq.v, iq.v+4, ignore_quats.begin()+k*4);
      }

      // Now that we have the new bounding box, filter our points
      filterPoints(pts, *filt_pts, box_widths, t.v, q.v, 
          num_regions, ignore_widths, ignore_trans, ignore_quats, curCamera);
    }
    else
    {
      // If this is the first point cloud, use the current bounds
      filterPoints(pts, *filt_pts, bound_planes, bound_dists, 
          num_regions, ignore_planes, ignore_dists);
    }

    ROS_INFO("Successfully filtered");

    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_pts (new pcl::PointCloud<pcl::PointXYZ>);

    // Now transform our filtered points back into the initial frame
    if (i != 0)
    {
      for (int j=0; j < 3; j++)
      {
        trans[j] = req.trans[3*i+j];
        cout << trans[j] << ", ";
      }
      cout << endl;
      for (int j=0; j < 4; j++)
      {
        quat[j] = req.quat[4*i+j];
        cout << quat[j] << ", ";
      }
      cout << endl;

      // Our overall transform first moves the points into the world frame, 
      //  then into the frame where they were taken, then back to the 
      //  world frame but moved by how the points have moved, and 
      //  finally back into the camera frame.
      HomogTransf h(Quaternion(quat).getRotMat(), Vec(trans,3));
      HomogTransf overallTransf = cameraFrames[curCamera].inv() * orig * h.inv() * cameraFrames[curCamera];

      // Use our computed transform to move all of our points 
      //  into the first point cloud's frame
      Eigen::Matrix4f transform; 
      for (int j=0; j<4;j++)
        for (int k=0; k<4;k++)
          transform(j,k) = overallTransf[j][k];

      cout << transform << endl;

      pcl::transformPointCloud(*filt_pts, *trans_pts, transform);
    }
    else
    {
      // If this is the original point cloud, there's no change
      trans_pts = filt_pts;
    }

    ROS_INFO("Successfully transformed");

    // Aggregate all of our points into 1 bigger point cloud
    *all_pts += *trans_pts;

    ROS_INFO("Successfully concatenated");

#ifdef SHOW_MULTI
    // If desired, create a viewer which shows the filtered point cloud from 
    // each view. Also, set up to show a colored aggregated point cloud

    char title[128];

    sprintf(title, "view %d", i);

    viewer3->createViewPort(1.0/req.num_views*i,0.0,1.0/req.num_views*(i+1),1.0,v[i]);  
    viewer3->setBackgroundColor (0, 0, 0, v[i]);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(trans_pts, 0, 255, 0);
    viewer3->addPointCloud<pcl::PointXYZ> (trans_pts, color1, title,v[i]);
    viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, title);
    viewer3->addCoordinateSystem (0.1,v[i]);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::copyPointCloud(*trans_pts, *temp);

    for (size_t j=0; j < temp->points.size(); j++)
    {
      if (i == 0)
      {
        temp->points[j].r = 255;
        temp->points[j].g = 0;
        temp->points[j].b = 0;
      }
      else if (i == 1)
      {
        temp->points[j].r = 0;
        temp->points[j].g = 255;
        temp->points[j].b = 0;
      }
      else if (i == 2)
      {
        temp->points[j].r = 0;
        temp->points[j].g = 0;
        temp->points[j].b = 255;
      }
      else if (i == 3)
      {
        temp->points[j].r = 255;
        temp->points[j].g = 255;
        temp->points[j].b = 255;
      }
      else
      {
        temp->points[j].r = 255;
        temp->points[j].g = 0;
        temp->points[j].b = 255;
      }
    }

    *color_pts += *temp;
#endif

  }

#ifdef SHOW_MULTI
  /*
  // Find center point
  double center[3];
  for (int i=0; i<3; i++)
  {
    center[i] = 0;
    for(int j=0; j<3; j++)
    {
      center[i] -= bound_planes[j][i] * bound_planes[j][3];
    }
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);   
  pcl::PointXYZRGB pt;  

  out_cloud->height = 1;
  out_cloud->is_dense = false;
  out_cloud->points.resize(200*200*200);

  bool pt_valid;
  double dist;
  int nr_points  = 200*200*200;

  int nr_p = 0;

  for (int m=0; m < 200; m++)
  {
    for (int n=0; n<200; n++)
    {
      for (int p=0; p<200; p++)
      {
        pt.x = center[0] - 0.5 + 0.005*m;
        pt.y = center[1] - 0.5 + 0.005*n;
        pt.z = center[2] - 0.5 + 0.005*p;

        for (int i=0; i<num_regions; i++)
        {
          pt_valid = true;
          for (int j=0; j<3;j++)
          {
            // Compute the distance of the point away from each 
            //  plane using a simple dot product
            dist = ignore_planes[i][j][3];
            for (int k=0; k<3;k++)
            {
              dist += ignore_planes[i][j][k]*pt.data[k];
            }

            // If we are outside the region, we will keep this point
            if (fabs(dist) > ignore_dists[i*3+j])
            {
              pt_valid = false;
              break;
            }
          }
          // If the point was inside one of the ignore regions, we're done
          if (pt_valid)
            break;
        }

        if (pt_valid)
        {
          pt.r = 255;
          pt.g = 0;
          pt.b = 255;
          out_cloud->points[nr_p] = pt;
          nr_p++;
        }
      }
    }
  }
  out_cloud->width = nr_p;
  out_cloud->points.resize (nr_p);

  *color_pts += *out_cloud;

  */

  // If desired, show a colored point cloud representing all of our points aggregated into a single point cloud
  while (!viewer3->wasStopped ())
  {
    viewer3->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  delete [] v;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(color_pts);
  viewer->addPointCloud<pcl::PointXYZRGB> (color_pts, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
  viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters ();

  drawBoundaries(viewer, curCamera);

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
#endif

  // Now that we have our aggregated point cloud, try to fit an object to it
  // TODO: The MIN_POINT_THRESH is currently a hack. We should do a better job
  //  of recognizing if we're holding an object or not in the future.
  if (!getObjectPose(all_pts, objNum, trans, quat, curCamera, MIN_POINT_THRESH))
  {
    res.ret = 0;
    res.msg = "Unable to get object pose";
    return false;
  }

  for (int i = 0; i < 3; i++)
    res.trans[i] = trans[i];
  for (int i = 0; i < 4; i++)
    res.quat[i] = quat[i];

  clock_t end_clk = clock();
  double elapsed_secs = double(end_clk - begin_clk) / CLOCKS_PER_SEC;

  cout << elapsed_secs << " elapsed." << endl;

  res.ret = 1;
  return true;
}



















bool ObjectRecognition::objRec_GetObjFromCameras(objRec_comm::objRec_GetObjFromCameras::Request& req, 
    objRec_comm::objRec_GetObjFromCameras::Response& res)
{
  clock_t begin_clk = clock();

  pcl::PointCloud<pcl::PointXYZ>::Ptr all_pts (new pcl::PointCloud<pcl::PointXYZ>);

#ifdef SHOW_MULTI
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_pts (new pcl::PointCloud<pcl::PointXYZRGB>);
#endif

  int objNum;
  double trans[3];
  double quat[4];

  double bound_widths[3];
  double bound_trans[3];
  double bound_quat[4];

  pthread_mutex_lock(&boundsMutex);
  memcpy(bound_widths, cur_widths, sizeof(double)*3);
  memcpy(bound_trans, cur_trans, sizeof(double)*3);
  memcpy(bound_quat, cur_quat, sizeof(double)*4);
  pthread_mutex_unlock(&boundsMutex);

  int num_regions;
  std::vector<double> ignore_widths;
  std::vector<double> ignore_trans;
  std::vector<double> ignore_quats;

  pthread_mutex_lock(&ignoreRegionsMutex);
  num_regions = num_ignore_regions;
  ignore_widths = cur_ignore_widths;
  ignore_trans = cur_ignore_trans;
  ignore_quats = cur_ignore_quats;
  pthread_mutex_unlock(&ignoreRegionsMutex);



#ifdef SHOW_MULTI
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  int *v = new int[req.num_cameras];
#endif

  size_t baseCamera = getCameraIdx(req.cameraNames[0]);

  // First, we filter each point cloud file and aggregate it into a single 
  //  point cloud, making sure to do the transformations correctly
  for (int i=0; i < req.num_cameras; i++)
  {
    // Open the point cloud file
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts (new pcl::PointCloud<pcl::PointXYZ>);
    ROS_INFO("Processing file: %s", req.filenames[i].c_str());
    if( pcl::io::loadPCDFile<pcl::PointXYZ> (req.filenames[i].c_str(), *pts) == -1)
    {
      res.ret = 0;
      res.msg = "Could not read points file. Unable to process";
      return false;
    }
    else
    {
      ROS_INFO("Successfully read file");
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filt_pts (new pcl::PointCloud<pcl::PointXYZ>);

    // Use box filter to greatly reduce points
    size_t curCamera = getCameraIdx(req.cameraNames[i]);

    filterPoints(pts, *filt_pts, bound_widths, bound_trans, bound_quat, 
        num_regions, ignore_widths, ignore_trans, ignore_quats, curCamera);

    ROS_INFO("Successfully filtered");
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_pts (new pcl::PointCloud<pcl::PointXYZ>);

    if (i != 0)
    {
      HomogTransf overallTransf = cameraFrames[baseCamera].inv() * cameraFrames[curCamera];

      // Use our computed transform to move all of our points 
      //  into the first point cloud's frame
      Eigen::Matrix4f transform; 
      for (int j=0; j<4;j++)
        for (int k=0; k<4;k++)
          transform(j,k) = overallTransf[j][k];

      cout << transform << endl;

      pcl::transformPointCloud(*filt_pts, *trans_pts, transform);
    }
    else
    {
      // If this is the original point cloud, there's no change
      trans_pts = filt_pts;
    }

    ROS_INFO("Successfully transformed");

    // Aggregate all of our points into 1 bigger point cloud
    *all_pts += *trans_pts;

    ROS_INFO("Successfully concatenated");




#ifdef SHOW_MULTI
    // If desired, create a viewer which shows the filtered point cloud from 
    // each view. Also, set up to show a colored aggregated point cloud

    char title[128];

    sprintf(title, "view %d", i);

    viewer3->createViewPort(1.0/req.num_cameras*i,0.0,1.0/req.num_cameras*(i+1),1.0,v[i]);  
    viewer3->setBackgroundColor (0, 0, 0, v[i]);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(trans_pts, 0, 255, 0);
    viewer3->addPointCloud<pcl::PointXYZ> (trans_pts, color1, title,v[i]);
    viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, title);
    viewer3->addCoordinateSystem (0.1,v[i]);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::copyPointCloud(*trans_pts, *temp);

    for (size_t j=0; j < temp->points.size(); j++)
    {
      if (i == 0)
      {
        temp->points[j].r = 255;
        temp->points[j].g = 0;
        temp->points[j].b = 0;
      }
      else if (i == 1)
      {
        temp->points[j].r = 0;
        temp->points[j].g = 255;
        temp->points[j].b = 0;
      }
      else if (i == 2)
      {
        temp->points[j].r = 0;
        temp->points[j].g = 0;
        temp->points[j].b = 255;
      }
      else if (i == 3)
      {
        temp->points[j].r = 255;
        temp->points[j].g = 255;
        temp->points[j].b = 255;
      }
      else
      {
        temp->points[j].r = 255;
        temp->points[j].g = 0;
        temp->points[j].b = 255;
      }
    }

    *color_pts += *temp;
#endif

  }

#ifdef SHOW_MULTI
  // If desired, show a colored point cloud representing all of our points aggregated into a single point cloud
  while (!viewer3->wasStopped ())
  {
    viewer3->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  delete [] v;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(color_pts);
  viewer->addPointCloud<pcl::PointXYZRGB> (color_pts, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
  viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters ();

  drawBoundaries(viewer, baseCamera);

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
#endif

  // Now that we have our aggregated point cloud, try to fit an object to it
  // TODO: The MIN_POINT_THRESH is currently a hack. We should do a better job
  //  of recognizing if we're holding an object or not in the future.
  if (!getObjectPose(all_pts, objNum, trans, quat, baseCamera, MIN_POINT_THRESH, req.onTable))
  {
    res.ret = 0;
    res.msg = "Unable to get object pose";
    return false;
  }

  for (int i = 0; i < 3; i++)
    res.trans[i] = trans[i];
  for (int i = 0; i < 4; i++)
    res.quat[i] = quat[i];

  clock_t end_clk = clock();
  double elapsed_secs = double(end_clk - begin_clk) / CLOCKS_PER_SEC;

  cout << elapsed_secs << " elapsed." << endl;

  res.ret = 1;
  return true;
}









//////////////////////////////////////////////////////////////////////////
// Service to set regions to ignore
//
// This service allows the user to set a list rectangular prisms (in the 
// world frame) to ignore which correspond to the hand's fingers.  
//
bool ObjectRecognition::objRec_SetIgnoreRegions(objRec_comm::objRec_SetIgnoreRegions::Request& req,
    objRec_comm::objRec_SetIgnoreRegions::Response& res)
{
  num_ignore_regions = req.num_regions;
  // Convert all of our input to C-arrays
  //int n = req.num_regions;
  std::vector<double> widths;
  std::vector<double> trans;
  std::vector<double> quats;

  widths = req.widths;
  trans = req.trans;
  quats = req.quats;

  pthread_mutex_lock(&ignoreRegionsMutex);
  cur_ignore_widths = widths;
  cur_ignore_trans = trans;
  cur_ignore_quats = quats;
  pthread_mutex_unlock(&ignoreRegionsMutex);

  res.ret = 1;
  return true;
  /*
  // Now attempt to calculate the bounds and save them
  pthread_mutex_lock(&ignoreRegionsMutex);

  if (getIgnoreRegions(n, widths, trans, quats, cur_ignore_planes, cur_ignore_dists))
  {

  // If successful, also remember our current box pose, 
  //  as this will be useful later
  cur_ignore_widths = widths;
  cur_ignore_trans = trans;
  cur_ignore_quats = quats;

  pthread_mutex_unlock(&ignoreRegionsMutex);

  res.ret = 1;
  return true;
  }

  // If there was something wrong with our bounds 
  //  specification, return failure
  pthread_mutex_unlock(&ignoreRegionsMutex);
  res.ret = 0;
  res.msg = "SetIgnoreRegions failed.";
  return false;

   */

}


//////////////////////////////////////////////////////////////////////////
// Service to set bounding box for filter
//
// This service allows the user to set the bounding box for where the 
//  object is expected to be in the world frame. It is specified by choosing 
//  the (x,y,z) corner of a rectangular prism centered at the origin 
//  [widths], and then specifying the rigid body transform of this prism to 
//  somewhere in 3d space [trans, quat].
//
// The service takes in the specification and converts it into 3 planes 
//  and allowable distances from each plane. This allows filtering to be 
//  done extremely efficiently.
//
bool ObjectRecognition::objRec_SetBounds(objRec_comm::objRec_SetBounds::Request& req, 
    objRec_comm::objRec_SetBounds::Response& res)
{
  // Convert all of our input to C-arrays
  double widths[3];
  double trans[3];
  double quat[4];

  for (int i=0; i < 3; i++)
  {
    widths[i] = req.widths[i];
    trans[i] = req.trans[i];
  }
  for (int j = 0; j < 4; j++)
  {
    quat[j] = req.quat[j];
  }

  pthread_mutex_lock(&boundsMutex);

  memcpy(cur_widths, widths, sizeof(double)*3);
  memcpy(cur_trans, trans, sizeof(double)*3);
  memcpy(cur_quat, quat, sizeof(double)*4);

  pthread_mutex_unlock(&boundsMutex);

  res.ret = 1;
  return true;

  /*
  // Now attempt to calculate the bounds and save them
  if (getBounds(widths, trans, quat, cur_planes, cur_dists))
  {
    // If successful, also remember our current box pose, 
    //  as this will be useful later
    memcpy(cur_widths, widths, sizeof(double)*3);
    memcpy(cur_trans, trans, sizeof(double)*3);
    memcpy(cur_quat, quat, sizeof(double)*4);

    pthread_mutex_unlock(&boundsMutex);

    res.ret = 1;
    return true;
  }

  // If there was something wrong with our bounds 
  //  specification, return failure
  pthread_mutex_unlock(&boundsMutex);
  res.ret = 0;
  res.msg = "SetBounds failed.";
  return false;
  */
}


//////////////////////////////////////////////////////////////////////////
// Service to set whether or not to stream object pose
// 
// Simply sets whether or not to stream object pose as fast as possible
// Note that setting 'streaming' to true will put a significant load 
// on the system
// 
bool ObjectRecognition::objRec_SetStream(objRec_comm::objRec_SetStream::Request& req, 
    objRec_comm::objRec_SetStream::Response& res)
{
  streaming = req.streaming;
  streamingCam = getCameraIdx(req.cameraName);

  res.ret = 1;
  return true;
}


//////////////////////////////////////////////////////////////////////////
// Service to save point cloud data
//
// This service lets the user save point cloud data seen from the point 
// cloud topic we are subscribed to. All point clouds will be saved 
// in the .pcd format, and the user has the option to save it as a binary 
// or ascii file.
//
// The service automatically generates a file name and saves it in a 
// predefined folder for future use. The name of the file is returned.
//
bool ObjectRecognition::objRec_SavePoints(objRec_comm::objRec_SavePoints::Request& req,
    objRec_comm::objRec_SavePoints::Response& res)
{
  size_t curCamera = getCameraIdx(req.cameraName);

  // Acquire point cloud from the topic we are subscribed to
  points_ready[curCamera] = false;
  use_points[curCamera] = true;
  ask_time[curCamera] = ros::Time::now();

  ros::Rate loop_rate(60);

  while(ros::ok() && !points_ready[curCamera])
  {
    loop_rate.sleep();
  }
  // Once points are ready, they should be saved to the 'cur_points' variable

  // Generate a file name based on the current time of day
  char newFilename[MAX_FILE_BUFFER];
  ros::Time curTime = ros::Time::now();
  /*time_t timer;
  timer=time(NULL);
  tm* today;
  today = localtime(&timer);*/
  time_t t = curTime.sec;
  tm* today = localtime(&t);
  int today_msec = curTime.nsec / 1000000;
  sprintf(newFilename,"%s/points__%d_%02d_%02d__%02d_%02d_%02d_%03d.pcd",
      pointsFolder.c_str(),
      today->tm_year+1900,
      today->tm_mon+1,
      today->tm_mday,
      today->tm_hour,
      today->tm_min,
      today->tm_sec,
      today_msec);	


  if (req.use_filter)
  {
    pcl::PointCloud<pcl::PointXYZ> out_cloud;
    filterPoints(cur_points[curCamera], out_cloud, curCamera);
    // Save our point cloud, letting the user decide whether or not it is binary
    if (pcl::io::savePCDFile(string(newFilename), out_cloud, req.is_binary) == -1)
    {
      res.ret = 0;
      res.msg = "Failed to save filtered point cloud file.";
      return false;
    }
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZ> out_cloud;
    pcl::PointXYZ pt;

    // Compute the total number of points
    int nr_points  = cur_points[curCamera]->width * cur_points[curCamera]->height;

    out_cloud.height       = 1;
    out_cloud.is_dense     = false;
    out_cloud.points.resize (cur_points[curCamera]->points.size ());

    int nr_p = 0;

    for (int cp = 0; cp < nr_points; ++cp)
    {
      pt = cur_points[curCamera]->points[cp];

      // Check if the point is invalid. If so, don't bother saving it.
      if (std::isnan (pt.x) || std::isnan (pt.y) || std::isnan (pt.z))
      {
        continue;
      }

      out_cloud.points[nr_p] = pt;
      nr_p++;
    }

    // Resize our point cloud to reflect the invalid points we didn't add.
    out_cloud.width = nr_p;
    out_cloud.points.resize (nr_p);

    // Save our point cloud, letting the user decide whether or not it is binary
    if (pcl::io::savePCDFile(string(newFilename), out_cloud, req.is_binary) == -1)
    {
      res.ret = 0;
      res.msg = "Failed to save point cloud file.";
      return false;
    }
  }

  // Return the filename we used.
  res.filename = newFilename;
  res.ret = 1;
  return true;
}

//////////////////////////////////////////////////////////////////////////
// Service to set parameters for feature matching
//
// Simple service for setting parameters used for feature matching. Useful 
// when wanting to try out different parameters without needing to recompile.
//
// Note that if the feature parameters change, we recompute all of our known
//  object features
//
bool ObjectRecognition::objRec_SetParams(objRec_comm::objRec_SetParams::Request& req,
    objRec_comm::objRec_SetParams::Response& res)
{
  RecognitionObject::Guess_SACIA::RecParams params;

  params.normal_radius = req.normal_radius;
  params.feature_radius = req.feature_radius;
  params.num_samples = req.num_samples;
  params.min_sample_dist = req.min_sample_dist;
  params.k_correspondences = req.k_correspondences;

  ROS_INFO("Parameters have changed. Recomputing normals and features for our objects...");
  for (int i=0; i < RecObj::NUM_REC_OBJ; i++)
    recObjs[i].computeObjectFeatures(params);

  // recomputeRecObjFeatures(params);

  res.ret = 1;
  return true;
}

//////////////////////////////////////////////////////////////////////////
// Service to set a guess object pose
//
// This service lets the user specify a guess for the object transform in 
// the world. If a guess is set, when recognizing an object, no feature
// matching will be done, and ICP will immediately be used from the initial
// guess. This service can also be used to disable the guess.
//
bool ObjectRecognition::objRec_SetGuess(objRec_comm::objRec_SetGuess::Request& req,
    objRec_comm::objRec_SetGuess::Response& res)
{
  // Save whether or not to use a guess
  use_guess = req.use_guess;

  // If we do want to use a guess, store it
  if (use_guess)
  {
    Vec trans(3);
    Quaternion quat;

    for (int i = 0; i < 3; i++)
      trans[i] = req.trans[i];
    for (int i=0; i<4; i++)
      quat[i] = req.quat[i];

    // Convert our guess (translation & quaternion) into a Homogeneous Transform
    HomogTransf guessFrame(quat.getRotMat(), trans);

    // Store this guess relative to the world frame
    curGuess = guessFrame;

    // Store our guess in relation to the camera frame
    // curGuess = cameraFrame.inv() * guessFrame;
  }

  res.ret = 1;
  return true;
}


//////////////////////////////////////////////////////////////////////////
// Service to set the camera frame
//
// This service lets the user specify the location of the camera in 3d
// space. Useful when trying to refine the camera calibration.
//
bool ObjectRecognition::objRec_SetCamera(objRec_comm::objRec_SetCamera::Request& req,
    objRec_comm::objRec_SetCamera::Response& res)
{
  Vec trans(3);
  Quaternion quat;

  for (int i = 0; i < 3; i++)
    trans[i] = req.trans[i];
  for (int i=0; i<4; i++)
    quat[i] = req.quat[i];

  size_t curCamera = getCameraIdx(req.cameraName);

  cameraFrames[curCamera].setTranslation(trans);
  cameraFrames[curCamera].setRotation(quat.getRotMat());

  res.ret = 1;
  return true;
}


//////////////////////////////////////////////////////////////////////////
// Service to set a preferred orientation 
//
// This service lets the user specify a preferred orientation to return the
// object pose with respect to the world frame. This is helpful when using
// an object with symmetries and trying to get consistent results of
// reported pose across runs. Note that this service can also be used to
// disable a preferred orientation as well.
//
bool ObjectRecognition::objRec_SetPrefOrient(objRec_comm::objRec_SetPrefOrient::Request& req,
    objRec_comm::objRec_SetPrefOrient::Response& res)
{
  // Save whether or not to use a preferred orientation
  use_pref_orient = req.use_pref_orient;

  // If we do want to use a preferred orienatation, store it
  if (use_pref_orient)
  {
    for (int i = 0; i < 4; i++)
      curOrient[i] = req.quat[i];
  }

  res.ret = 1;
  return true;
}

//////////////////////////////////////////////////////////////////////////
// Service to set what object to look for
//
// This service lets the user specify which object the vision system should
// be looking for. If this option is disabled (Which can also be done with
// this service), then the vision system will try fitting each of the
// objects in its library, and return the best fit. If you know what object
// you're looking for, this service should be used to greatly speed up the
// recognition process
//
bool ObjectRecognition::objRec_SetObject(objRec_comm::objRec_SetObject::Request& req,
    objRec_comm::objRec_SetObject::Response& res)
{
  // Save whether or not to look for only a particular object
  use_object = req.use_object;

  // If we do want to look for a particular object, save it
  if (use_object)
  {
    curObject = req.objNum;
  }

  res.ret = 1;
  return true;
}




//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// Private functions - used by services


//////////////////////////////////////////////////////////////////////////
// Get pose of object from a filtered point cloud. 
//
// First, we use feature matching to come up with an initial guess of 
//  where the object is. (Can override this and use a guess instead). 
// Then, we use ICP to refine our estimate and come up with the final pose.
// Finally, we convert the pose of the object into the world frame.
//
// Takes in a point cloud, and optionally, the minimum number of points 
//  it needs to actually do the analysis
//
// Returns the identity of the object it found, and it's 3d pose in 
//  world coordinates
//
bool ObjectRecognition::getObjectPose(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
    int &objNum, double trans[3], double quat[4], size_t curCamera, unsigned int min_points, bool onTable)
{
  ROS_INFO("IN getObjectPose");




  // First, check if we have enough points. If not, we're done. 
  // TODO: Possibly eliminate this hack
  if (cloud->points.size() < min_points)
  {
    ROS_INFO("Number of points: %ld. Min threshold: %u", cloud->points.size(), min_points);
    ROS_WARN("Have not detected enough points. No object in this point cloud.");
    objNum = -1;
    return false;
  }

  // TODO: Currently, we simply specify the object we're looking for in here. 
  // Try and come up with a clever way (perhaps a service) to let the user 
  // select the object they're looking for, or alternatively, to ask the 
  // system to find if any of the objects in its library exist in this 
  // point cloud.
  if (use_object)
    objNum = curObject;
  else
    objNum = RecObj::BIG_TRIANGLE;

  pcl::PointCloud<pcl::PointXYZ>::Ptr registration_output (new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Matrix4f initial_T;

  // If no guess is specified, we will use the guess function specific to
  // this object
  if (!use_guess)
  {
    if (onTable)
    {
      ROS_INFO("Table Guess!");
      // For our guess, let's voxelize our point cloud, in case certain 
      // areas of the point cloud are more dense than others
      pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
      vox_grid.setInputCloud (cloud);
      vox_grid.setLeafSize (VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
      pcl::PointCloud<pcl::PointXYZ>::Ptr voxCloud (new pcl::PointCloud<pcl::PointXYZ>); 
      vox_grid.filter (*voxCloud);

      ROS_INFO("Successful voxel filtering");
      if (!recObjs[objNum].getTableGuess(voxCloud, registration_output, initial_T))
      {
        ROS_WARN("Get Table Guess Failed. Returning false");
        return false;
      }
    }
    else
    {
      ROS_INFO("Normal Guess!");
      if (!recObjs[objNum].getGuess(cloud, registration_output, initial_T))
      {
        ROS_WARN("Get Table Guess Failed. Returning false");
        return false;
      }
    }

    std::cout << initial_T << std::endl;
  }
  else
  {
    // If we have secified a guess, don't do any feature matching. 
    // Save the transform, and transform our object model
    // Convert our guess into the camera frame
    HomogTransf camGuess = cameraFrames[curCamera].inv() * curGuess;
    for (int i=0; i < 4; i++)
      for (int j=0; j < 4; j++)
        initial_T(i,j) = camGuess[i][j];

    pcl::transformPointCloud(*(recObjs[objNum].getPointCloud ()), *registration_output, initial_T);
    std::cout << initial_T << std::endl;
  }



  // Only do ICP if more than a rough estimate is required
#ifndef OBJREC_ROUGH_ESTIMATE

  // Now that we have a guess, we will use iterative closest point to refine our estimate
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(registration_output);
  icp.setInputTarget(cloud);

  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (0.03);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (500);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (1e-8);
  icp.setRANSACOutlierRejectionThreshold (0.005);
  icp.setRANSACIterations (500);

  // Come up with the best final alignment
  pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
  icp.align(*Final);

  std::cout << "ICP Results" << std::endl;
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;

  // Obtain the transformation that aligned our initial guess to the final 
  // transform. Note that we multiply by the initial transform to get the 
  // full transform of where the object is.
  Eigen::Matrix4f refined_T = icp.getFinalTransformation () * initial_T;

  std::cout << refined_T << std::endl;

#else
  // If we're only doing a rough estimate, don't do ICP. Just set our 
  // guess as the final transform
  pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
  Final = registration_output;
  Eigen::Matrix4f refined_T = initial_T;

#endif

#if defined(OBJREC_DEBUG) || defined(OBJREC_SHOW_RES)
  pcl::PointCloud<pcl::PointXYZ>::Ptr Final2 (new pcl::PointCloud<pcl::PointXYZ>);
#endif

  RotMat r;
  for (int i=0; i<3; i++)
  {
    trans[i] = refined_T(i,3);
    for (int j=0; j<3; j++)
      r[i][j] = refined_T(i,j);
  }

  // Compute the object pose with respect to the world
  HomogTransf objPos(r, Vec(trans,3));
  HomogTransf worldPos = cameraFrames[curCamera] * objPos;


  // If this object is symmetric, make sure we always pick the orientation 
  //  of the object that is closer to the preferred orientation
  if (use_pref_orient && recObjs[objNum].isSymmetric())
  {
    worldPos = recObjs[objNum].getClosestTransf(worldPos, curOrient);

#if defined(OBJREC_DEBUG) || defined(OBJREC_SHOW_RES)
    // If desired, transform the object into its symmetric pose, 
    //  just to confirm that we have transformed it correctly
    HomogTransf symT = cameraFrames[curCamera].inv() * worldPos;
    Eigen::Matrix4f transform; 
    for (int j=0; j<4;j++)
      for (int k=0; k<4;k++)
        transform(j,k) = symT[j][k];

    cout << transform << endl;

    pcl::transformPointCloud(*(recObjs[objNum].getPointCloud ()), *Final2, transform);
#endif

  }
#if defined(OBJREC_DEBUG) || defined(OBJREC_SHOW_RES)
  else
  {
    // Nothing to do if the object doesn't have any symmetries
    Final2 = Final;
  }
#endif

#if defined(OBJREC_DEBUG) || defined(OBJREC_SHOW_RES)
  // If desired, show the results of our matching
  visualize_match(cloud, registration_output, Final, Final2);
#endif


  // Extract the translation and orientation for our object transform
  Quaternion q = worldPos.getRotation().getQuaternion();
  Vec t = worldPos.getTranslation();
  for (int i=0; i<4; i++)
    quat[i] = q[i]; 
  for (int i=0; i<3; i++)
    trans[i] = t[i];


  // As a final step, let's check if we have succeeded or not by seeing how
  // many points are close by. Note that we will look at 2 different
  // measures in order to be robust. First, we will look at what percentage
  // of our model points matches our cloud. This is useful when the cloud
  // has a lot of extraneous information, and we should only care about how
  // well our model has matched the part of the cloud we care about.
  // Second, we will look at what percentage of our cloud points match our
  // model points. This is useful when we only have a small number of cloud
  // points because of occlusion, but we still do a good job in matching
  // the object. In this case, we only care about how many of the cloud
  // points match, even if it's a really small portion of the entire object
  // model. We will report failure only if BOTH of these criteria fail

  int num_model_matches = 0, num_cloud_matches = 0;

#ifdef OBJREC_ROUGH_ESTIMATE
  // If we are only doing a rough estimate, don't bother with checking if
  // the points agree
  num_model_matches = Final->points.size();
  num_cloud_matches = cloud->points.size();
#else

  // Calculate how many points on our object are 
  //  near a place on our input cloud
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);
  std::vector<int> pointIdx;
  std::vector<float> pointDist;

  for (size_t i=0; i < Final->points.size(); i++)
  {
    if (kdtree.radiusSearch (Final->points[i], CLOSE_RAD, pointIdx, pointDist) > 0)
      num_model_matches++;
  }

  kdtree.setInputCloud(Final);
  for (size_t i=0; i < cloud->points.size(); i++)
  {
    if (kdtree.radiusSearch (cloud->points[i], CLOSE_RAD, pointIdx, pointDist) > 0)
      num_cloud_matches++;
  }

#endif

  ROS_INFO("num_model_matches: %d, Total Points: %ld, ratio: %f", 
      num_model_matches, Final->points.size(), 
      ((double)num_model_matches)/((double) Final->points.size()));
  ROS_INFO("num_cloud_matches: %d, Total Points: %ld, ratio: %f", 
      num_cloud_matches, cloud->points.size(), 
      ((double)num_cloud_matches)/((double) cloud->points.size()));


  // If our model does not match the cloud in enough places, AND the cloud
  // does not match the model in enough places, then we have failed
  if (num_model_matches < MODEL_MATCH_THRESH * Final->points.size() && 
      num_cloud_matches < CLOUD_MATCH_THRESH * cloud->points.size())
  {
    ROS_INFO("Failed to find enough matches. Our object pose guess is probably incorrect.");
    return false;
  }

  // Otherwise, we probably do have a good match. Alert the user.
  ROS_INFO("Found enough matches. Our object pose guess is probably correct.");
  return true;
}


//////////////////////////////////////////////////////////////////////////
// Filter a point cloud using current bounds and ignore regions
//
// This function takes in a point cloud, and uses the current bounds and 
// ignore regions to generate a much smaller, and much more relevant point 
// cloud
//
bool ObjectRecognition::filterPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_cloud, 
    pcl::PointCloud<pcl::PointXYZ> &out_cloud, size_t curCamera)
{
  // Get the current bounding planes
  double bound_planes[3][4];
  double bound_dists[3];
  int num_regions;
  std::vector<std::vector<std::vector<double> > > ignore_planes;
  std::vector<double> ignore_dists;

  // Get the current bounding planes
  pthread_mutex_lock(&boundsMutex);
  getBounds(cur_widths, cur_trans, cur_quat, bound_planes, bound_dists, curCamera);
  //memcpy(bound_planes, cur_planes, sizeof(double)*12);
  //memcpy(bound_dists, cur_dists, sizeof(double)*3);
  pthread_mutex_unlock(&boundsMutex);

  // Get the current ignore regions
  pthread_mutex_lock(&ignoreRegionsMutex);
  num_regions = num_ignore_regions;
  getIgnoreRegions(num_regions, cur_ignore_widths, cur_ignore_trans, cur_ignore_quats, ignore_planes, ignore_dists, curCamera);
  //ignore_planes = cur_ignore_planes;
  //ignore_dists = cur_ignore_dists;
  pthread_mutex_unlock(&ignoreRegionsMutex);

  // Filter our data
  return filterPoints(in_cloud, out_cloud, bound_planes, bound_dists, 
      num_regions, ignore_planes, ignore_dists);
}


//////////////////////////////////////////////////////////////////////////
// Filter a point cloud using bounds and ignore regions
//
// This function takes in a point cloud, and specifications for bounds, 
// and uses those bounds and ignore regions to generate a much smaller, and
// much more relevant point cloud
//
bool ObjectRecognition::filterPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_cloud, 
    pcl::PointCloud<pcl::PointXYZ> &out_cloud,
    double box_widths[3], double box_trans[3], double box_quat[4], size_t curCamera)
{
  return filterPoints(in_cloud, out_cloud, box_widths, box_trans, box_quat, 0, std::vector<double>(),  std::vector<double>(), std::vector<double>(), curCamera); 
}

bool ObjectRecognition::filterPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_cloud, 
    pcl::PointCloud<pcl::PointXYZ> &out_cloud,
    double box_widths[3], double box_trans[3], double box_quat[4],
    int num_regions, std::vector<double> ignore_widths, std::vector<double> ignore_trans, std::vector<double> ignore_quats, size_t curCamera)
{
  double bound_planes[3][4];
  double bound_dists[3];
  std::vector<std::vector<std::vector<double> > > ignore_planes;
  std::vector<double> ignore_dists;

  // Compute bounds given the specifications
  if (getBounds(box_widths, box_trans, box_quat, bound_planes, bound_dists, curCamera) &&
      getIgnoreRegions(num_regions, ignore_widths, ignore_trans, ignore_quats, 
        ignore_planes, ignore_dists, curCamera))
  {
    // Use those bounds to filter our points
    return filterPoints(in_cloud, out_cloud, bound_planes, bound_dists,
        num_regions, ignore_planes, ignore_dists);
  }
  return false;
}


//////////////////////////////////////////////////////////////////////////
// Filter an RGB-D point cloud using bounds and ignore regions
//
// This function takes in a point cloud, desired bounds, and desired ignore
// regions, and generates a much smaller, and more 
// relevant point cloud
//
bool ObjectRecognition::filterPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_cloud, 
    pcl::PointCloud<pcl::PointXYZ> &out_cloud,
    double bound_planes[3][4], double bound_dists[3])
{
  return filterPoints(in_cloud, out_cloud, bound_planes, bound_dists, 0, std::vector<std::vector<std::vector<double> > >(), std::vector<double>());
}

bool ObjectRecognition::filterPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_cloud, 
    pcl::PointCloud<pcl::PointXYZ> &out_cloud,
    double bound_planes[3][4], double bound_dists[3], 
    int num_ignore, std::vector<std::vector<std::vector<double> > > ignore_planes, std::vector<double> ignore_dists)
{
  double dist;
  bool pt_valid;
  pcl::PointXYZ pt;

  // Make sure we have actually have input
  if (!in_cloud)
  {
    out_cloud.width = out_cloud.height = 0;
    out_cloud.points.clear ();
    return false;
  }

  // Allocate enough space for our filtered point cloud
  int nr_points  = in_cloud->width * in_cloud->height;
  out_cloud.height       = 1;   // filtering breaks the organized structure
  out_cloud.is_dense     = false;
  out_cloud.points.resize (in_cloud->points.size ());

  int nr_p = 0;

  // Go through all of our points, and only keep the ones we want
  for (int cp = 0; cp < nr_points; ++cp)
  {
    pt.x = in_cloud->points[cp].x;
    pt.y = in_cloud->points[cp].y;
    pt.z = in_cloud->points[cp].z;

    // First, check if the point is invalid
    if (std::isnan (pt.x) || std::isnan (pt.y) || std::isnan (pt.z))
    {
      continue;
    }

    // Now check if point is inside of our boundary box
    pt_valid = true;
    for (int i=0; i<3;i++)
    {
      // Compute the distance of the point away from each 
      //  plane using a simple dot product
      dist = bound_planes[i][3];
      for (int j=0; j<3;j++)
      {
        dist += bound_planes[i][j]*pt.data[j];
      }

      // If we are too far away, we will not keep this point
      if (fabs(dist) > bound_dists[i])
      {
        pt_valid = false;
        break;
      }
    }

    if (!pt_valid)
      continue;

    // Now make sure point is outside of an ignored region
    for (int i=0; i<num_ignore; i++)
    {
      pt_valid = false;
      for (int j=0; j<3;j++)
      {
        // Compute the distance of the point away from each 
        //  plane using a simple dot product
        dist = ignore_planes[i][j][3];
        for (int k=0; k<3;k++)
        {
          dist += ignore_planes[i][j][k]*pt.data[k];
        }

        // If we are outside the region, we will keep this point
        if (fabs(dist) > ignore_dists[i*3+j])
        {
          pt_valid = true;
          break;
        }
      }
      // If the point was inside one of the ignore regions, we're done
      if (!pt_valid)
        break;
    }

    // If we got this far, we have a point that is inside our boundary box
    // and outside of our ignore regions, so copy all of the fields
    if (pt_valid)
    {
      out_cloud.points[nr_p] = pt;
      nr_p++;
    }
  }

  // Resize our point cloud to the number of points we kept
  out_cloud.width = nr_p;
  out_cloud.points.resize (nr_p);

  ROS_INFO("IN_POINTS: %d, OUT_POINTS: %d", nr_points, nr_p);

  return true;
}

bool ObjectRecognition::getIgnoreRegions(int num_regions, 
    std::vector<double> widths, std::vector<double> trans, 
    std::vector<double> quats, 
    std::vector<std::vector<std::vector<double> > >& ignoreRegions_planes, 
    std::vector<double>& ignoreRegions_dists, size_t curCamera)
{
  ignoreRegions_dists.resize(num_regions*3);
  ignoreRegions_planes.resize(num_regions);

  for (int i=0; i < num_regions; i++)
  {
    double bound_planes[3][4];
    double bound_dists[3];
    double w[3];
    double t[3];
    double q[4];
    std::copy(widths.begin()+i*3, widths.begin()+(i+1)*3, w);
    std::copy(trans.begin()+i*3, trans.begin()+(i+1)*3, t);
    std::copy(quats.begin()+i*4, quats.begin()+(i+1)*4, q);
    if (getBounds(w,t,q,bound_planes, bound_dists, curCamera))
    {
      ignoreRegions_planes[i].resize(3);
      for (int j=0; j < 3; j++)
      {
        ignoreRegions_planes[i][j].resize(4);
        for (int k=0; k < 4; k++)
        {
          ignoreRegions_planes[i][j][k] = bound_planes[j][k];
        }
        ignoreRegions_dists[i*3+j] = bound_dists[j];
      }
    }
    else
    {
      return false;
    }
  }

  // Print out our results. We're done!
  for (int k=0; k<num_regions; k++)
  {
    for (int i=0; i<3;i++)
    {
      printf("ip[%d] = [",i);
      for (int j=0; j<4; j++)
      {
        printf("%f,", ignoreRegions_planes[k][i][j]);
      }
      printf("%f]\n", ignoreRegions_dists[k*3+i]);
    }
  }

  return true;

  /*

  // First, make sure that all of our bounding box widths are positive
  for (int i = 0; i < (3*num_regions); i++)
  {
  if (widths[i] <= 0)
  {
  ROS_WARN("Cannot handle non-positive box width");
  return false;
  }
  }

  // Next make sure that we have a unit quaternions
  for (int k=0;k<num_regions; k++)
  {      
  double mag = 0;
  for (int i = 0; i < 4; i++)
  mag += quat[k*num_regions+i] * quat[k*num_regions+i];

  if (mag == 0)
  {
  ROS_WARN("Invalid rotation. Quaternion magnitude 0.");
  return false;
  }
  for (int i = 0; i < 4; i++)
  quat[k*num_regions+i] /= sqrt(mag);
  }

  // Transform our box into the camera frame
  for (int k=0;k<num_regions; k++)
  {
  double reg_trans[3];
  double reg_quat[4]; 
  for (int i=0;i<3;i++)
  reg_trans[i] = trans[k*num_regions+i];
  for (int i=0;i<4;i++)
  reg_quat[i] = quat[k*num_regions+i];

  HomogTransf boxTrans(Quaternion(reg_quat).getRotMat(), Vec(reg_trans,3));
  HomogTransf modifiedFrame = cameraFrame.inv() * boxTrans;

  RotMat r;
  r = modifiedFrame.getRotation();
  Vec t = modifiedFrame.getTranslation();

  // Note that [A,B,C] in the equation Ax+By+Cz+D=0 represent the vector 
  // perpendicular to the plane. At the origin, the 3 planes centered in 
  // the box have normal vectors [1,0,0], [0,1,0], and [0,0,1]. Hence the 
  // new normal vectors for our rotated box will simply be the rows of our 
  // rotation matrix! Also note that [A,B,C] will also automatically be a 
  // unit vector, from the definition of a rotation matrix. Nice!
  for (int i=0; i<3; i++)
  for(int j=0; j<3; j++)
  ignoreRegions_planes[k][i][j] = r[k][i][j];

  // Now let's find the correct translation component.
  // The origin of the box is being transformed, so we know that the new
  // origin of the box (which is exactly our translation vector) must be
  // on each of our 3 planes. Hence, D = - [A,B,C] * [x;y;z], 
  // where [x;y;z] is our translation vector.
  for (int i = 0; i < 3; i++)
  {
  ignoreRegions_planes[k][i][3] = 0;
  for (int j = 0; j < 3; j++)
  ignoreRegions_planes[k][i][3] += -1 * ignoreRegions_planes[k][i][j] * t[j];

  // Also, the widths of our box (x,y,z), will exactly be the maximum
  // allowable distance from each of our 3 planes that were transformed 
  // from unit vectors [1,0,0], [0,1,0], and [0,0,1].
  ignoreRegions_dists[k][i] = widths[3*k+i];
  }
}

// Print out our results. We're done!
for (int k=0; k<num_regions; k++)
{
  for (int i=0; i<3;i++)
  {
    printf("bp[%d] = [",i);
    for (int j=0; j<4; j++)
    {
      printf("%f,", ignoreRegions_planes[k][i][j]);
    }
    printf("%f]\n", ignoreRegions_dists[k][i]);
  }
}

return true;
*/
}



//////////////////////////////////////////////////////////////////////////
// Compute bounds (bounding planes) given a specification
//
// This function takes in a rectangular prism centered at the world origin 
// and a desired rigid-body transform, which will move it to somewhere in 3d 
// space, and generates 3 planes along the 3 axes of the box in the camera 
// frame so that filtering can be done much more efficiently,
//
// As input, we take in the (x,y,z) corner of the box (widths), and the 
// rigid body transform (translation and quaternion)
//
// As output, this function returns the 3 planes going through the center 
// of the box, and the corresponding maximum distances away from each 
// plane. Each row of 'bound_planes' contains a plane [A,B,C,D], and 
// represent the equation Ax + By + Cz + D = 0. Note that we make 
// norm([A,B,C]) = 1, so that when calculating a distance of a point to a 
// plane, we can use the following equation: dist = [A,B,C] * [x;y;z] + D. 
// (See http://mathworld.wolfram.com/Point-PlaneDistance.html). This is 
// extremely fast to calculate when filtering point clouds, and is why 
// we have implemented this function.
//
// Note that the specification for the bounds (widths, trans, quat) are in 
// the world frame, but the actual generated bounding planes are in the 
// camera frame, again for speed. Filtering can be done without 
// transforming the points taken by the camera into the world frame.
//
bool ObjectRecognition::getBounds(double widths[3], double trans[3], double quat[4], 
    double bound_planes[3][4], double bound_dists[3], size_t curCamera)
{
  // First, make sure that all of our bounding box widths are positive
  for (int i = 0; i < 3; i++)
  {
    if (widths[i] <= 0)
    {
      ROS_WARN("Cannot handle non-positive box width");
      return false;
    }
  }

  // Next make sure that we have a unit quaternion
  double mag = 0;
  for (int i = 0; i < 4; i++)
    mag += quat[i] * quat[i];

  if (mag == 0)
  {
    ROS_WARN("Invalid rotation. Quaternion magnitude 0.");
    return false;
  }
  for (int i = 0; i < 4; i++)
    quat[i] /= sqrt(mag);

  // Transform our box into the camera frame
  HomogTransf boxTrans(Quaternion(quat).getRotMat(), Vec(trans,3));
  HomogTransf modifiedFrame = cameraFrames[curCamera].inv() * boxTrans;

  RotMat r;
  r = modifiedFrame.getRotation();
  Vec t = modifiedFrame.getTranslation();

  // Note that [A,B,C] in the equation Ax+By+Cz+D=0 represent the vector 
  // perpendicular to the plane. At the origin, the 3 planes centered in 
  // the box have normal vectors [1,0,0], [0,1,0], and [0,0,1]. Hence the 
  // new normal vectors for our rotated box will simply be the columns of our 
  // rotation matrix! Also note that [A,B,C] will also automatically be a 
  // unit vector, from the definition of a rotation matrix. Nice!
  for (int i=0; i<3; i++)
    for(int j=0; j<3; j++)
      bound_planes[i][j] = r[j][i];

  // Now let's find the correct translation component.
  // The origin of the box is being transformed, so we know that the new
  // origin of the box (which is exactly our translation vector) must be
  // on each of our 3 planes. Hence, D = - [A,B,C] * [x;y;z], 
  // where [x;y;z] is our translation vector.
  for (int i = 0; i < 3; i++)
  {
    bound_planes[i][3] = 0;
    for (int j = 0; j < 3; j++)
      bound_planes[i][3] += -1 * bound_planes[i][j] * t[j];

    // Also, the widths of our box (x,y,z), will exactly be the maximum
    // allowable distance from each of our 3 planes that were transformed 
    // from unit vectors [1,0,0], [0,1,0], and [0,0,1].
    bound_dists[i] = widths[i];
  }

  // Print out our results. We're done!
  for (int i=0; i<3;i++)
  {
    printf("bp[%d] = [",i);
    for (int j=0; j<4; j++)
    {
      printf("%f,", bound_planes[i][j]);
    }
    printf("%f]\n", bound_dists[i]);
  }

  return true;
}

size_t ObjectRecognition::getCameraIdx(std::string cameraName)
{
  for (size_t i = 0; i < numCameras; ++i)
  {
    if (cameraName == cameraNames[i])
      return i;
  }
  ROS_WARN("Unrecognized camera name: %s. Defaulting to %s", cameraName.c_str(), cameraNames[0].c_str());
  return 0;
}

#if defined(OBJREC_DEBUG) || defined(OBJREC_SHOW_RES)

void ObjectRecognition::visualize_match(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr registration_output, pcl::PointCloud<pcl::PointXYZ>::Ptr Final, pcl::PointCloud<pcl::PointXYZ>::Ptr Final2)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  int v1(0);
  viewer->createViewPort(0.0,0.0,0.5,1.0,v1);
  viewer->setBackgroundColor(255,255,255,v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_target_v1 (cloud,0,255,0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_source_v1 (registration_output,255,0,0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud,color_target_v1,"target cloud v1",v1);
  viewer->addPointCloud<pcl::PointXYZ> (registration_output,color_source_v1,"source cloud v1",v1);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"target cloud v1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"source cloud v1");

  viewer->addCoordinateSystem (0.2,v1);

  int v2(0);
  viewer->createViewPort(0.5,0.0,1.0,1.0,v2);
  viewer->setBackgroundColor(255,255,255,v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_target_v2 (cloud,0,255,0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_source_v2 (Final,0,0,255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_source2_v2 (Final2,255,0,0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud,color_target_v2,"target cloud v2",v2);
  viewer->addPointCloud<pcl::PointXYZ> (Final,color_source_v2,"source cloud v2",v2);
  viewer->addPointCloud<pcl::PointXYZ> (Final2,color_source2_v2,"source2 cloud v2",v2);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"target cloud v2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"source cloud v2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"source2 cloud v2");
  viewer->addCoordinateSystem (0.2,v2);


  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

}

#endif

#ifdef SHOW_MULTI

void ObjectRecognition::drawBoundaries(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, size_t curCamera)
{
  pthread_mutex_lock(&boundsMutex);

  HomogTransf boxTrans(Quaternion(cur_quat).getRotMat(), Vec(cur_trans,3));
  HomogTransf modifiedFrame = cameraFrames[curCamera].inv() * boxTrans;

  Quaternion q = modifiedFrame.getQuaternion();
  Vec t = modifiedFrame.getTranslation();

  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back(t[0]);
  coeffs.values.push_back(t[1]);
  coeffs.values.push_back(t[2]);
  coeffs.values.push_back(q[1]);
  coeffs.values.push_back(q[2]);
  coeffs.values.push_back(q[3]);
  coeffs.values.push_back(q[0]);
  coeffs.values.push_back(cur_widths[0]*2);
  coeffs.values.push_back(cur_widths[1]*2);
  coeffs.values.push_back(cur_widths[2]*2);
  pthread_mutex_unlock(&boundsMutex);
  viewer->addCube(coeffs, "boundary_box");

  pthread_mutex_lock(&ignoreRegionsMutex);

  for (int i=0; i < num_ignore_regions; ++i)
  {
    boxTrans.setTranslation(Vec(&cur_ignore_trans[i*3], 3));
    boxTrans.setQuaternion(Quaternion(&cur_ignore_quats[i*4]));
    modifiedFrame = cameraFrames[curCamera].inv() * boxTrans;
    q = modifiedFrame.getQuaternion();
    t = modifiedFrame.getTranslation();
    coeffs.values.clear();
    coeffs.values.push_back(t[0]);
    coeffs.values.push_back(t[1]);
    coeffs.values.push_back(t[2]);
    coeffs.values.push_back(q[1]);
    coeffs.values.push_back(q[2]);
    coeffs.values.push_back(q[3]);
    coeffs.values.push_back(q[0]);
    coeffs.values.push_back(cur_ignore_widths[i*3]*2);
    coeffs.values.push_back(cur_ignore_widths[i*3+1]*2);
    coeffs.values.push_back(cur_ignore_widths[i*3+2]*2);
    char name[20];
    sprintf(name, "ignore_%d", i);
    viewer->addCube(coeffs, name);
  }
  pthread_mutex_unlock(&ignoreRegionsMutex);
}

#endif




//////////////////////////////////////////////////////////////////////////
// Initialize the object recognition node
//
bool ObjectRecognition::init(ros::NodeHandle *n)
{
  // Remember the pointer to our node
  node = n;


  //RecognitionObject::Guess_SACIA::RecParams params;

  // Remember the folder to save pcd files to
  node->getParam("objRec/pointsFolder", pointsFolder);

  // Read in the values of parameters we will use for our matching algorithms
  //node->getParam("objRec/normal_radius", params.normal_radius);
  //node->getParam("objRec/feature_radius", params.feature_radius);
  //node->getParam("objRec/num_samples", params.num_samples);
  //node->getParam("objRec/min_sample_dist", params.min_sample_dist);
  //node->getParam("objRec/k_correspondences", params.k_correspondences);

  // Get the frame of the camera with respect to the world

  XmlRpc::XmlRpcValue my_list;
  XmlRpc::XmlRpcValue inner_list;
  double pose[7];

  node->getParam("/objRec/cameraNames", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  numCameras = my_list.size();
  cameraNames.resize(numCameras);
  for (size_t i = 0; i < numCameras; ++i)
  {
    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    cameraNames[i] = static_cast<std::string>(my_list[i]);
  }

  node->getParam("objRec/cameraFrames", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT((size_t)my_list.size() == numCameras);
  cameraFrames.resize(numCameras);
  for (size_t i = 0; i < numCameras; ++i)
  {
    inner_list = my_list[i];
    ROS_ASSERT(inner_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(inner_list.size() == 7);
    for (int32_t j = 0; j < inner_list.size(); ++j)
    {
      ROS_ASSERT(inner_list[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      pose[j] = static_cast<double>(inner_list[j]);
    }
    cameraFrames[i] = HomogTransf(pose);
  }

  /*

  double trans[3];
  double quat[4];
  node->getParam("/objRec/cameraFrameX", trans[0]);
  node->getParam("/objRec/cameraFrameY", trans[1]);
  node->getParam("/objRec/cameraFrameZ", trans[2]);
  node->getParam("/objRec/cameraFrameQ0", quat[0]);
  node->getParam("/objRec/cameraFrameQX", quat[1]);
  node->getParam("/objRec/cameraFrameQY", quat[2]);
  node->getParam("/objRec/cameraFrameQZ", quat[3]);

  // Normalize the quaternion, just in case
  double mag = 0;
  for (int i=0; i < 4; i++)
    mag += quat[i] * quat[i];
  if (fabs(mag - 1) > 0.01)
    ROS_WARN("Camera frame quaternion does not have unit magnitude. Normalizing and continuing anyways...");
  for (int i=0; i < 4; i++)
    quat[i] /= sqrt(mag);


  // Now convert our frame into a homogeneous matrix, and save it
  cameraFrame.setTranslation(Vec(trans,3));
  cameraFrame.setRotation(Quaternion(quat).getRotMat());

  */

  // Initialize everything we need to do with all of our objects, and store
  // the objects in 'recObjs'
  recObjs.clear();
  recObjs.reserve(RecObj::NUM_REC_OBJ);

  string pkg_path = ros::package::getPath("objRec_node");
  string objectFolder = pkg_path + "/objectFolder";

  for (int i = 0; i < RecObj::NUM_REC_OBJ; i++)
  {
    recObjs.push_back(RecognitionObject(node, RecObj::fileNames[i], objectFolder));
  }
  //initializeRecObjs(node, objectFolder, params, recObjs);

  // Set up our arrays for ignored regions
  num_ignore_regions = 0;

  // Initialize our mutex for setting point bounds
  pthread_mutex_init(&boundsMutex, NULL);
  pthread_mutex_init(&ignoreRegionsMutex, NULL);

  // Subscribe to point cloud data
  sub_points.resize(numCameras);
  use_points.resize(numCameras);
  ask_time.resize(numCameras);
  points_ready.resize(numCameras);
  pcl::PointCloud<pcl::PointXYZ>::Ptr default_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  cur_points.resize(numCameras, default_cloud);
  for (size_t i=0; i < numCameras; ++i)
  {
    use_points[i] = false;
    points_ready[i] = false;
    sub_points[i] = node->subscribe<sensor_msgs::PointCloud2>("/" + cameraNames[i] + "/depth/points", 5, boost::bind(&ObjectRecognition::points_callback, this, _1, i));
  }

  // Advertise our topics
  handle_objRec_ObjPos = 
    node->advertise<objRec_comm::objRec_ObjPos>("objRec_ObjPos", 10);

  // Advertise our services
  handle_objRec_GetObject = 
    node->advertiseService("objRec_GetObject", &ObjectRecognition::objRec_GetObject, this);
  handle_objRec_GetObjFromViews = 
    node->advertiseService("objRec_GetObjFromViews", &ObjectRecognition::objRec_GetObjFromViews, this);
  handle_objRec_GetObjFromCameras = 
    node->advertiseService("objRec_GetObjFromCameras", &ObjectRecognition::objRec_GetObjFromCameras, this);
  handle_objRec_SetBounds = 
    node->advertiseService("objRec_SetBounds", &ObjectRecognition::objRec_SetBounds, this);
  handle_objRec_SetIgnoreRegions = 
    node->advertiseService("objRec_SetIgnoreRegions", &ObjectRecognition::objRec_SetIgnoreRegions, this); 
  handle_objRec_SetStream = 
    node->advertiseService("objRec_SetStream", &ObjectRecognition::objRec_SetStream, this);
  handle_objRec_SavePoints = 
    node->advertiseService("objRec_SavePoints", &ObjectRecognition::objRec_SavePoints, this);
  handle_objRec_SetParams = 
    node->advertiseService("objRec_SetParams", &ObjectRecognition::objRec_SetParams, this);
  handle_objRec_SetGuess = 
    node->advertiseService("objRec_SetGuess", &ObjectRecognition::objRec_SetGuess, this);
  handle_objRec_SetCamera = 
    node->advertiseService("objRec_SetCamera", &ObjectRecognition::objRec_SetCamera, this);
  handle_objRec_SetPrefOrient = 
    node->advertiseService("objRec_SetPrefOrient", &ObjectRecognition::objRec_SetPrefOrient, this);
  handle_objRec_SetObject = 
    node->advertiseService("objRec_SetObject", &ObjectRecognition::objRec_SetObject, this);

  // Set up internal variables
  streaming = false;
  streamingCam = 0;

  use_guess = false;
  use_pref_orient = false;
  use_object = false;

  return true;
}

//////////////////////////////////////////////////////////////////////////
// Main Node. Creates object recognition node and spins.
//
int main(int argc, char **argv)
{
  ros::init(argc, argv, "objRec_node");

  ros::NodeHandle node;
  ObjectRecognition objRec(&node);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
