#include "RecognitionObject.h"
#include <fstream>
#include <cctype>
#include <cstdio>
#include <cstdlib>

RecognitionObject::GuessFunc::~GuessFunc(){}
RecognitionObject::SymFunc::~SymFunc(){}

RecognitionObject::Guess_SACIA::Guess_SACIA(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, RecParams p) : search_method_xyz_ (new SearchMethod)
{
  // Store the parameters to use for sacia, and compute the normals and
  // features for this object using these parameters
  xyz_ = cloud;
  updateParams(p);
}


bool RecognitionObject::Guess_SACIA::updateParams(RecParams p)
{
  sacia_params = p;
  computeObjectNormals(xyz_, normals_);
  computeLocalFeatures(xyz_, normals_, features_);
  return true;
}

// Compute the surface normals of a point cloud
void RecognitionObject::Guess_SACIA::computeSurfaceNormals (PointCloud::Ptr cloud, SurfaceNormals::Ptr n)
{
  n = SurfaceNormals::Ptr (new SurfaceNormals);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
  norm_est.setInputCloud (cloud);
  norm_est.setSearchMethod (search_method_xyz_);
  norm_est.setRadiusSearch (sacia_params.normal_radius);
  norm_est.compute (*n);
}

// Compute the surface normals of a specific object.
// Do this by first setting the viewpoint to be inside 
//  the object, and then flip all of the normals.
void RecognitionObject::Guess_SACIA::computeObjectNormals (PointCloud::Ptr cloud, SurfaceNormals::Ptr n)
{
  n = SurfaceNormals::Ptr (new SurfaceNormals);

  // Setup our normal estimation
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
  norm_est.setInputCloud (cloud);
  norm_est.setSearchMethod (search_method_xyz_);
  norm_est.setRadiusSearch (sacia_params.normal_radius);

  // Set the viewpoint to be inside the object. Assume that the object
  // is convex, so if we find the centroid, we can compute all normals
  // towards the center, and then flip them to get all outward facing
  // normals
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  norm_est.setViewPoint(centroid[0], centroid[1], centroid[2]);

  // Actually compute all of the inward facing normals
  norm_est.compute (*n);

  // Now flip all of our normals so they're facing outwards
  for (size_t i=0; i < n->points.size(); ++i)
  {
    n->points[i].normal_x *= -1;
    n->points[i].normal_y *= -1;
    n->points[i].normal_z *= -1;
  }
}

// Compute the local feature descriptors
void RecognitionObject::Guess_SACIA::computeLocalFeatures (PointCloud::Ptr cloud, SurfaceNormals::Ptr n, LocalFeatures::Ptr f)
{
  f = LocalFeatures::Ptr (new LocalFeatures);

  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
  fpfh_est.setInputCloud (cloud);
  fpfh_est.setInputNormals (n);
  fpfh_est.setSearchMethod (search_method_xyz_);
  fpfh_est.setRadiusSearch (sacia_params.feature_radius);
  fpfh_est.compute (*f);
}



bool RecognitionObject::Guess_SACIA::getGuess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &reg_out, Eigen::Matrix4f &initial_T)
{
  // Compute relevant features in our point cloud
  SurfaceNormals::Ptr target_normals;
  LocalFeatures::Ptr target_features;
  computeSurfaceNormals(cloud, target_normals);
  computeLocalFeatures(cloud, target_normals, target_features);

#ifdef VISUALIZE_GUESS
  // If desired, visualize what the object's surface normals look like versus the point clouds
  visualize_normals(cloud, target_normals);
#endif

  // We will use SAC-IA to match features
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;

  // Set up SAC-IA
  sac_ia_.setMinSampleDistance (sacia_params.min_sample_dist);
  sac_ia_.setNumberOfSamples (sacia_params.num_samples);
  sac_ia_.setMaximumIterations (500);
  sac_ia_.setCorrespondenceRandomness(sacia_params.k_correspondences);

  sac_ia_.setInputCloud (xyz_);
  sac_ia_.setSourceFeatures (features_);

  sac_ia_.setInputTarget (cloud);
  sac_ia_.setTargetFeatures (target_features);

  // Use SAC-IA to come up with a guess. Note that in PCL's implementation 
  // of SAC-IA, it transforms the input cloud (our known object) to its 
  // best guess of where it is in the target cloud (the point cloud we're 
  // trying to analyze)
  sac_ia_.align (*reg_out);
  initial_T = sac_ia_.getFinalTransformation ();

  float fitness_score = (float) sac_ia_.getFitnessScore (0.01f*0.01f);
  std::cout << "has converged:" << sac_ia_.hasConverged() << " score: " <<
    fitness_score << std::endl;

  return true;
}

#ifdef VISUALIZE_GUESS

void RecognitionObject::Guess_SACIA::visualize_normals(PointCloud::Ptr cloud, SurfaceNormals::Ptr normals)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  int v3(0);
  viewer3->createViewPort(0.0,0.0,0.5,1.0,v3);  
  viewer3->setBackgroundColor (0, 0, 0, v3);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(xyz_, 0, 255, 0);
  viewer3->addPointCloud<pcl::PointXYZ> (xyz_, color1, "obj cloud",v3);
  viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "obj cloud");
  viewer3->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (xyz_, normals_, 1, 0.005, "obj normals",v3);
  viewer3->addCoordinateSystem (0.1,v3);

  int v4(0);
  viewer3->createViewPort(0.5,0.0,1.0,1.0,v4);  
  viewer3->setBackgroundColor (0, 0, 0, v4);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cloud, 255, 0, 0);
  viewer3->addPointCloud<pcl::PointXYZ> (cloud, color2, "targ cloud",v4);
  viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "targ cloud");
  viewer3->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 1, 0.005, "targ normals",v4);
  viewer3->addCoordinateSystem (0.1,0,0,1,v4);

  while (!viewer3->wasStopped ())
  {
    viewer3->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

#endif




RecognitionObject::Guess_Planes::Guess_Planes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, char p[ARG_CHAR_LENGTH], char i[ARG_CHAR_LENGTH], ros::NodeHandle *n)
{
  // Subscribe to the matlab_node, which we will use later
  matlab.subscribe(n);

  // Also be sure to add this node's matlab scripts folder to the package
  // path of MATLAB
  std::string pkg_path = ros::package::getPath("objRec_node");
  pkg_path += "/matlab_scripts";

  ros::Rate loop_rate(10);
  while (!matlab.Ping())
  {
    loop_rate.sleep();
  }


  matlab.addPath(pkg_path.c_str());

  object_cloud = cloud;

  // Store the planes and intersections for this object
  memcpy(planes, p, sizeof(char)*ARG_CHAR_LENGTH);
  memcpy(intersections, i, sizeof(char)*ARG_CHAR_LENGTH);
}


bool RecognitionObject::Guess_Planes::getGuess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &reg_out, Eigen::Matrix4f &initial_T)
{

  /*
  // First, find the number of points in this point cloud
  int num_points = cloud->width;

  // Allocate enough space for all of our points
  double* points = new double[num_points * 3];

  // Copy all of the points over to a c-array
  for (int i=0; i < num_points; i++)
  {
  for (int j=0; j < 3; j++)
  {
  points[i * 3 + j] = cloud->points[i].data[j];
  }
  }

  // Send the points to matlab
  matlab.sendMat("points", num_points, 3, points);
  matlab.sendCommand("save('/home/simplehands/Documents/hands/code/nodes/vision/ROS/objRec_node/matlab_scripts/saved_points.mat', 'points');");

  // Call our function to compute our guess of the object location
  char buf[1024];
  sprintf(buf, "pose = findObj(%s, %s, points);", planes, intersections);
  matlab.sendCommand(buf);

  // Extract the guess pose from matlab
  Mat pose = matlab.getMat("pose");

  // Convert to Eigen::Matrix
  for (int i=0; i<4; i++)
  {
  for (int j=0; j<4; j++)
  {
  initial_T(i,j) = pose[i][j];
  }
  }

  // Transform our object point cloud by this transform
  pcl::transformPointCloud(*(obj.getPointCloud ()), *reg_out, initial_T);
  return true;

   */


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::copyPointCloud(*cloud, *cloud_filtered);

  Eigen::Vector4f centroid;
  Eigen::Vector4f obj_centroid;

  pcl::compute3DCentroid(*cloud_filtered, centroid);
  pcl::compute3DCentroid(*object_cloud, obj_centroid);

  std::cout << "centroid = " << centroid << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (PLANE_THRESH);

  std::vector<std::vector<float> > fitted_planes;

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int num_planes = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 10% of the original cloud is still there
  while (cloud_filtered->points.size () > MIN_POINTS_FOR_PLANE)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);

    ROS_INFO("Plane inliers: %d", inliers->indices.size());

    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
    else if (inliers->indices.size() < MIN_POINTS_IN_PLANE )
    {
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);

    fitted_planes.push_back(coefficients->values);


    // First, find the number of points in this point cloud
    int num_points = cloud_p->points.size();

    // Allocate enough space for all of our points
    double* points = new double[num_points * 3];

    // Copy all of the points over to a c-array
    for (int i=0; i < num_points; i++)
    {
      for (int j=0; j < 3; j++)
      {
        points[i * 3 + j] = cloud_p->points[i].data[j];
      }
    }

    // Send the points to matlab
    char buf[1024];
    sprintf(buf, "points_%d", num_planes);
    matlab.sendMat(buf, num_points, 3, points);

    sprintf(buf, "save('/home/simplehands/Documents/hands/code/nodes/vision/ROS/objRec_node/matlab_scripts/saved_points_%d.mat', 'points_%d');", num_planes, num_planes);
    matlab.sendCommand(buf);

    delete[] points;

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    num_planes++;
  }

  double* cplanes = new double[num_planes * 4];


  for (int i=0; i < num_planes; i++)
  {
    std::cout << "plane" << i << "= ";
    for (int j=0; j < 4; j++)
    {
      cplanes[i*4 + j] = fitted_planes[i][j];
      std::cout << cplanes[i*4+j] << " ";
    }
    std::cout << std::endl;

    // Make sure all of our normals are facing outwards
    double dist = cplanes[i*4 + 3];
    for (int j=0; j<3; j++)
    {
      dist += cplanes[i*4 + j] * centroid[j];
    }

    std::cout << "dist = " << dist << std::endl;

    if (dist > 0)
    {
      for (int j=0; j<4; j++)
      {
        cplanes[i*4 + j] *= -1;
      }
    }
    std::cout << "new_plane" << i << "= ";
    for (int j=0; j < 4; j++)
    {
      std::cout << cplanes[i*4+j] << " ";
    }
    std::cout << std::endl;
  }


  // Send the points to matlab
  matlab.sendMat("fitted_planes", num_planes, 4, cplanes);
  matlab.sendCommand("save('/home/simplehands/Documents/hands/code/nodes/vision/ROS/objRec_node/matlab_scripts/saved_planes.mat', 'fitted_planes');");

  /*
  // Call our function to compute our guess of the object location
  char buf2[1024];
  sprintf(buf2, "pose = findObj(%s, %s, fitted_planes);", planes, intersections);
  matlab.sendCommand(buf2);

  // Extract the guess pose from matlab
  Mat pose = matlab.getMat("pose");
   */

  // Call our function to compute our guess of the object location
  char buf2[1024];
  sprintf(buf2, "[transforms, M, valid_transforms] = findObjMulti(%s, %s, fitted_planes);", planes, intersections);
  matlab.sendCommand(buf2);

  // It's possible that we will come up with more than one possible
  // transform. Let's figure out how many
  int num_transforms = matlab.getValue("M");

  Vec valid_transforms = matlab.getVec("valid_transforms");

  // Calculate how many points on our object are 
  //  near a place on our input cloud for each of the transforms
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree(false);
  kdtree.setInputCloud(cloud);
  std::vector<int> pointIdx;
  std::vector<float> pointDist;

#ifdef VISUALIZE_GUESS
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> diff_transforms(num_transforms);
  std::vector<int> neighbor_list(num_transforms);
#endif

  int most_neighbors = 0;
  int best_idx = -1;
  // Now, go through each transform, and determine how many points are
  // close by if we transform the point cloud in this way
  for (int kk=0; kk<num_transforms; kk++)
  {
    // Extract the guess pose from matlab
    sprintf(buf2, "pose = transforms{%d};", kk+1);
    matlab.sendCommand(buf2);
    Mat pose = matlab.getMat("pose");


    // Convert to Eigen::Matrix
    for (int i=0; i<4; i++)
    {
      for (int j=0; j<4; j++)
      {
        initial_T(i,j) = pose[i][j];
      }
    }

    // If a valid transformation was not found, our rotation is specified
    // correctly, but because we couldn't find any plane intersections, we
    // have no way of determining the translation. In this case, we will
    // use the centroid of both point clouds as an approximation for the
    // translation
    if (valid_transforms[kk] == 0)
    {
      // Compute the rotated centroid of the object
      double rotated_centroid[3];
      for (int i=0; i<3; i++)
      {
        rotated_centroid[i] = 0;
        for (int j=0; j<3; j++)
        {
          rotated_centroid[i] += pose[i][j] * obj_centroid[j];
        }
      }

      // Compute the translation between the rotated object centroid and
      // the object seen by the camera
      for (int i=0; i<3; i++)
      {
        initial_T(i,3) = centroid[i] - rotated_centroid[i];
      }
    }


    // Now transform our object point cloud by this transform
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*object_cloud, *transformed_cloud, initial_T);


    // Determine the number of data points close by
    int num_neighbors = 0;
    std::cout << "transformed point cloud size: " << transformed_cloud->points.size() << std::endl;
    for (size_t j=0; j < transformed_cloud->points.size(); j++)
    {
      if (kdtree.radiusSearch (transformed_cloud->points[j], INITIAL_RAD, pointIdx, pointDist, 1) > 0)
        num_neighbors++;
    }

#ifdef VISUALIZE_GUESS
    diff_transforms[kk] = transformed_cloud;
    neighbor_list[kk] = num_neighbors;
#endif

    // If this is the most neighbors found so far, remember it
    std::cout << "num_neighbors: " << num_neighbors << std::endl;
    if (num_neighbors > most_neighbors)
    {
      best_idx = kk;
      most_neighbors = num_neighbors;
    }
  }
  std::cout << std::flush;

#ifdef VISUALIZE_GUESS
  visualize_options(diff_transforms, neighbor_list, cloud);
#endif


  if (best_idx == -1)
  {
    ROS_WARN("No suitable transform was found! Simply returning identity as our guess");
    initial_T = Eigen::Matrix4f::Identity();
  }
  else
  {
    // Extract the best pose from matlab
    sprintf(buf2, "pose = transforms{%d};", best_idx+1);
    matlab.sendCommand(buf2);
    Mat best_pose = matlab.getMat("pose");

    // Convert to Eigen::Matrix
    for (int i=0; i<4; i++)
    {
      for (int j=0; j<4; j++)
      {
        initial_T(i,j) = best_pose[i][j];
      }
    }



    // If a valid transformation was not found, our rotation is specified
    // correctly, but because we couldn't find any plane intersections, we
    // have no way of determining the translation. In this case, we will
    // use the centroid of both point clouds as an approximation for the
    // translation
    if (valid_transforms[best_idx] == 0)
    {
      // Compute the rotated centroid of the object
      double rotated_centroid[3];
      for (int i=0; i<3; i++)
      {
        rotated_centroid[i] = 0;
        for (int j=0; j<3; j++)
        {
          rotated_centroid[i] += best_pose[i][j] * obj_centroid[j];
        }
      }

      // Compute the translation between the rotated object centroid and
      // the object seen by the camera
      for (int i=0; i<3; i++)
      {
        initial_T(i,3) = centroid[i] - rotated_centroid[i];
      }
    }
  }

  // Transform our object point cloud by this transform
  pcl::transformPointCloud(*object_cloud, *reg_out, initial_T);
  return true;
}






#ifdef VISUALIZE_GUESS

void RecognitionObject::Guess_Planes::visualize_options(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> diff_transforms, std::vector<int> neighbor_list, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  for (size_t i=0; i<neighbor_list.size(); i++)
  {
    char name[20];
    sprintf(name, "transform_%d", (int)i);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr color;
    switch (i)
    {
      case 0:
        color = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(diff_transforms[i], 255, 0, 0));
        break;
      case 1:
        color = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(diff_transforms[i], 0, 255, 0));
        break;
      case 2:
        color = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(diff_transforms[i], 0, 0, 255));
        break;
      case 3:
        color = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(diff_transforms[i], 255, 255, 0));
        break;
      case 4:
        color = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(diff_transforms[i], 255, 0, 255));
        break;
      case 5:
        color = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(diff_transforms[i], 0, 255, 255));
        break;
      default:
        color = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(diff_transforms[i], 128, 128, 128));
        break;
    }

    viewer->addPointCloud<pcl::PointXYZ> (diff_transforms[i], *color, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
  }

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_col(cloud, 255,255,255);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, cloud_col, "main_cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "main_cloud");
  viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters ();

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
#endif









RecognitionObject::Guess_Ellipsoid::Guess_Ellipsoid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, SymType sym, bool rem_outliers)
{
  object_cloud = cloud;
  symmetry = sym;
  remove_outliers = rem_outliers;
  computeCentroidAndEigenVectors(cloud, centroid, eigen_vectors, eigen_values, false);
}

bool RecognitionObject::Guess_Ellipsoid::getGuess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr &reg_out, 
    Eigen::Matrix4f &initial_T)
{
  // First, let's compute the centroid and eigenvectors of our target
  // object
  Eigen::Vector3f targ_centroid;
  Eigen::Matrix3f targ_eigen_vecs;
  Eigen::Vector3f targ_eigen_vals;
  computeCentroidAndEigenVectors(cloud, targ_centroid, targ_eigen_vecs, targ_eigen_vals, remove_outliers);

  std::cout << "eig_vecs: " << eigen_vectors << std::endl;
  std::cout << "eig_vals: " << eigen_values << std::endl;
  std::cout << "targ_eig_vecs: " << targ_eigen_vecs << std::endl;
  std::cout << "targ_eig_vals: " << targ_eigen_vals << std::endl;


  // First, compare our eigen values to make sure that nothing terrible is
  // going to happen
  for (int i=0; i < 3; i++)
  {
    double quotient = eigen_values[i] / targ_eigen_vals[i];
    printf("sqrt(ev[%d]) = %f, sqrt(tev[%d]) = %f, q = %f\n", i, sqrt(eigen_values[i]), i, sqrt(targ_eigen_vals[i]), sqrt(quotient));
    if (fabs(quotient - 1) > EIG_VAL_FAC)
    {
      ROS_WARN("Eigen values are different. We may have too much occlusion "
          "to have a good guess. Trying anyways... ev[%d] = %f, tev[%d] = %f, q = %f",
          i, eigen_values[i], i, targ_eigen_vals[i], quotient);
    }
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree(false);
  kdtree.setInputCloud(cloud);
  std::vector<int> pointIdx;
  std::vector<float> pointDist;

  int most_neighbors = 0;

  initial_T = Eigen::Matrix4f::Identity();


  // Now, since the eigenvectors are not necessarily facing the correct
  // direction, try the 4 possible combinations and get the best fitting
  // guess

  Eigen::Vector3f u1 = targ_eigen_vecs.col(0);
  Eigen::Vector3f u2 = targ_eigen_vecs.col(1);
  Eigen::Vector3f u3 = u1.cross(u2);

  Eigen::Matrix3f U;
  U.col(0) = u1;
  U.col(1) = u2;
  U.col(2) = u3;

  for (int i=0; i < 2; i++)
  {
    Eigen::Vector3f v1;
    Eigen::Vector3f v2;
    switch(i)
    {
      case 0:
        v1 = eigen_vectors.col(0);
        v2 = eigen_vectors.col(1);
        break;
      case 1:
        v1 = -eigen_vectors.col(0);
        v2 = eigen_vectors.col(1);
        break;
      case 2:
        v1 = eigen_vectors.col(0);
        v2 = -eigen_vectors.col(1);
        break;
      case 3:
        v1 = -eigen_vectors.col(0);
        v2 = -eigen_vectors.col(1);
        break;
    }

    // Based on our chosen orientations, find the best rotation matrix.
    // Note that because we found eigenvectors of the correlation matrix,
    // which is symmetric, our vectors will necessarily be perpendicular

    Eigen::Vector3f v3 = v1.cross(v2);

    Eigen::Matrix3f V;
    V.col(0) = v1;
    V.col(1) = v2;
    V.col(2) = v3;



    // If there is occlusion, we'll do a local grid search along the
    //eigenvector directions that are troublesome.


    printf("num_steps:");

    int steps[3];
    for (int j=0; j < 3; j++)
    {
      steps[j] = ceil((sqrt(eigen_values[j]) - sqrt(targ_eigen_vals[j])) * MAGIC_FACTOR / STEP_SIZE);
      if (steps[j] < 0)
        steps[j] = 0;
      printf(" %d", steps[j]);
    }
    printf("\n");

    for (int a = -steps[0]; a <= steps[0]; a++)
    {
      for (int b = -steps[1]; b <= steps[1]; b++)
      {
        for (int c = -steps[2]; c <= steps[2]; c++)
        {
          Eigen::Vector3f cent_diff = v1*a*STEP_SIZE + v2*b*STEP_SIZE + v3*c*STEP_SIZE;

          Eigen::Matrix4f temp_T = Eigen::Matrix4f::Identity();
          temp_T.block<3,3>(0,0) = U * V.transpose();
          temp_T.block<3,1>(0,3) = - U * V.transpose() * (centroid + cent_diff) + targ_centroid;

          // Now transform our object point cloud by this transform
          pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
          pcl::transformPointCloud(*object_cloud, *transformed_cloud, temp_T);

          // Determine the number of data points close by
          int num_neighbors = 0;
          std::cout << "transformed point cloud size: " << transformed_cloud->points.size() << std::endl;
          for (size_t j=0; j < transformed_cloud->points.size(); j++)
          {
            if (kdtree.radiusSearch (transformed_cloud->points[j], CLOSE_RAD, pointIdx, pointDist, 1) > 0)
              num_neighbors++;
          }

          // If this is the most neighbors found so far, remember it
          std::cout << "num_neighbors: " << num_neighbors << std::endl;
          if (num_neighbors > most_neighbors)
          {
            most_neighbors = num_neighbors;
            initial_T = temp_T;
          }
        }
      }
    }

    // If this object looks like a cylinder, then there's no need to check
    // any other eigenvector combinations
    if (symmetry == SYM_CYLINDER)
      break;
    // If this object looks like a cone, then once we've checked 2, there's
    // no need to check any other eigenvector combinations
    if (symmetry == SYM_CONE && i == 1)
      break;
  }


  // Transform our object point cloud by this transform
  pcl::transformPointCloud(*object_cloud, *reg_out, initial_T);
  return true;
}



void RecognitionObject::Guess_Ellipsoid::computeCentroidAndEigenVectors(pcl::PointCloud<pcl::PointXYZ>::Ptr c, 
    Eigen::Vector3f &cent, Eigen::Matrix3f &eigen_vecs, Eigen::Vector3f &eigen_vals, bool rem_outliers)
{
  // First, compute the mean and covariance matrix of our points
  Eigen::Vector4f mean;
  Eigen::Matrix3f covariance_matrix;

  pcl::compute3DCentroid(*c, mean);

  if (rem_outliers)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = false;
    size_t num_points = c->points.size();
    filtered_cloud->points.resize(num_points);
    int np = 0;
    for (size_t i = 0; i < num_points; ++i)
    {
      if ((mean[0] - c->points[i].x)*(mean[0] - c->points[i].x) + (mean[1] - c->points[i].y)*(mean[1] - c->points[i].y) + (mean[2] - c->points[i].z)*(mean[2] - c->points[i].z) < OUTLIER_DIST_THRESH2)
      {
        filtered_cloud->points[np] = c->points[i];
        np++;
      }
    }
    filtered_cloud->width = np;
    filtered_cloud->points.resize(np);

#ifdef VISUALIZE_GUESS

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  int v3(0);
  viewer3->createViewPort(0.0,0.0,0.5,1.0,v3);  
  viewer3->setBackgroundColor (0, 0, 0, v3);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(c, 0, 255, 0);
  viewer3->addPointCloud<pcl::PointXYZ> (c, color1, "obj cloud",v3);
  viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "obj cloud");
  viewer3->addCoordinateSystem (0.1,v3);

  int v4(0);
  viewer3->createViewPort(0.5,0.0,1.0,1.0,v4);  
  viewer3->setBackgroundColor (0, 0, 0, v4);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(filtered_cloud, 255, 0, 0);
  viewer3->addPointCloud<pcl::PointXYZ> (filtered_cloud, color2, "targ cloud",v4);
  viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "targ cloud");
  viewer3->addCoordinateSystem (0.1,0,0,1,v4);

  while (!viewer3->wasStopped ())
  {
    viewer3->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

#endif

    // Now that we have removed outliers, recompute the mean
    pcl::compute3DCentroid(*filtered_cloud, mean);
    c = filtered_cloud;
  }

  pcl::computeCovarianceMatrixNormalized(*c,mean,covariance_matrix);


  // Now, compute the eigenvectors of the covariance matrix (Note that
  // since our matrix is symmetric and full rank, the eigen decomposition
  // is the same as the SVD decomposition
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariance_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // Save the eigenvectors, eigenvalues, and centroid
  eigen_vals = svd.singularValues();
  eigen_vecs = svd.matrixV();

  cent = mean.head<3>();
}













RecognitionObject::SymOrder::SymOrder(int num_syms, double* axes, double* points, int* orders)
{
  num_symmetries = num_syms;
  symTransf.resize(num_syms);
  sym_orders.resize(num_syms);
  for (int i=0; i < num_syms; i++)
  {
    symTransf[i].resize(orders[i]);
    sym_orders[i] = orders[i];

    // Set up the transform to the axis
    Vec w(&axes[3*i], 3);
    Vec p(&points[3*i], 3);

    Vec Z = w;
    Vec Y = w ^ Vec("1.0 0.0 0.0", 3);
    Vec X = Y ^ Z;

    Z.normalize();
    Y.normalize();
    X.normalize();

    // This transforms points into the symmetry frame, so they can then be
    // rotated
    HomogTransf T(RotMat(X,Y,Z), p);

    for (int j=0; j < orders[i]; j++)
    {
      // Set up our rotation about the axis of symmetry
      RotMat rot;
      rot.rotZ(2*PI / orders[i] * (j + 1));

      // This is our rotation about the symmetry frame
      HomogTransf R(rot, Vec("0.0 0.0 0.0",3));

      // Now our symmetry transform involves transforming points into
      //  the symmetry frame, rotating them by 360/n degrees, and then
      //  transforming back
      symTransf[i][j] = T*R*T.inv();
    }
  }
}


HomogTransf RecognitionObject::SymOrder::getClosestTransf(const HomogTransf orig, const Quaternion quat)
{
  double min_dist = 2.0;
  HomogTransf bestTransf = orig;
  switch (num_symmetries)
  {
    case 0:
      break;

    case 1:
      for (int i=0; i < sym_orders[0]; i++)
      {
        HomogTransf tempTransf = orig * symTransf[0][i];
        double dist = quat.dist(tempTransf.getRotation().getQuaternion());

        if (dist < min_dist)
        {
          bestTransf = tempTransf;
          min_dist = dist;
        }
      }
      break;

    case 2:
      for (int i=0; i < sym_orders[0]; i++)
      {
        HomogTransf tempTransf = orig * symTransf[0][i];
        for (int j=0; j < sym_orders[1]; j++)
        {
          tempTransf = tempTransf * symTransf[1][j];

          double dist = quat.dist(tempTransf.getRotation().getQuaternion());

          if (dist < min_dist)
          {
            bestTransf = tempTransf;
            min_dist = dist;
          }
        }
      }
      break;

    case 3:
      for (int i=0; i < sym_orders[0]; i++)
      {
        HomogTransf tempTransf = orig * symTransf[0][i];
        for (int j=0; j < sym_orders[1]; j++)
        {
          tempTransf = tempTransf * symTransf[1][j];
          for (int k=0; k < sym_orders[2]; k++)
          {
            tempTransf = tempTransf * symTransf[2][k];
            double dist = quat.dist(tempTransf.getRotation().getQuaternion());

            if (dist < min_dist)
            {
              bestTransf = tempTransf;
              min_dist = dist;
            }
          }
        }
      }
      break;
  }

  return bestTransf;
}


RecognitionObject::SymCylinder::SymCylinder(double axis[3], double point[3], bool sym_order)
{
  // Keep track of whether there is a plane of symmetry perpendicular to
  // the axis of rotation as well
  order_symmetry = sym_order;

   // Set up the transform to the axis
    Vec w(axis, 3);
    Vec p(point, 3);

    Vec Z = w;
    Vec Y = w ^ Vec("1.0 0.0 0.0", 3);
    Vec X = Y ^ Z;

    Z.normalize();
    Y.normalize();
    X.normalize();

    // This transforms points into the symmetry frame, so they can then be
    // rotated
    T.setRotation(RotMat(X,Y,Z));
    T.setTranslation(p);
}

HomogTransf RecognitionObject::SymCylinder::getClosestTransf(const HomogTransf orig, const Quaternion quat)
{
  // First, let's figure out what rotation gets us the closest to the
  // desired orientation:
  RotMat r = quat.getRotMat();
  RotMat tr = T.getRotation();

  // Given that our final transform will be: orig * T * R * T.inv(), we can
  // find the desired rotation in the symmetry frame by inverting everything 
  // and multiplying it to the desired quaternion in the world frame
  RotMat desired_rot = tr.inv() * orig.getRotation().inv() * r * tr;
  Quaternion desired_q = desired_rot.getQuaternion();

  std::cout << "desired_q: " << desired_q << std::endl;
  
  // Since we can only rotate about the z axis in this frame, let's find
  // the quaternion that gets as close as possible to the desired one.
  // After some math, we can use the expressions below to find the closest
  // quaternion:
  double q02 = desired_q[0] * desired_q[0];
  double qz2 = desired_q[3] * desired_q[3];

  Quaternion best_q;
  best_q[0] = desired_q[0] / sqrt(q02 + qz2);
  best_q[1] = 0;
  best_q[2] = 0;
  best_q[3] = desired_q[3] / sqrt(q02 + qz2);

  std::cout << "best_q: " << best_q << std::endl;

  if (order_symmetry)
  {
    // If we also have a symmetric plane perpendicular to our axis, then
    // there's one more quaternion value to calculate, which is if we
    // flipped the z axis around
    RotMat rotx;
    rotx.rotX(PI);
    RotMat desired_rot2 = desired_rot * rotx.inv();
    Quaternion desired_q2 = desired_rot2.getQuaternion();

    std::cout << "desired_q2: " << desired_q2 << std::endl;
    
    double q022 = desired_q2[0] * desired_q2[0];
    double qz22 = desired_q2[3] * desired_q2[3];

    Quaternion best_q2;
    best_q2[0] = desired_q2[0] / sqrt(q022 + qz22);
    best_q2[1] = 0;
    best_q2[2] = 0;
    best_q2[3] = desired_q2[3] / sqrt(q022 + qz22);

    std::cout << "best_q2: " << best_q2 << std::endl;

    double dist1 = desired_q.dist(best_q);
    double dist2 = desired_q2.dist(best_q2);

    // If we can get closer by flipping the frame along our plane, then we
    // will use this configuration
    if (dist2 < dist1)
    {
      best_q = best_q2 ^ rotx.getQuaternion();
      std::cout << "res best_q: " << best_q << std::endl;
      ROS_WARN("Flipping!");
    }
  }


  // Now that we have the best quaternion, let's compute our final symmetry
  // transform to get the closest possible transform
  HomogTransf bestTransf = orig * T * HomogTransf(best_q.getRotMat(), Vec("0 0 0",3)) * T.inv();

  std::cout << "PrefOrient: " << quat << std::endl;
  std::cout << "Result: " << bestTransf.getRotation().getQuaternion() << std::endl;

  return bestTransf;
}


RecognitionObject::SymSphere::SymSphere(double center[3])
{
  memcpy(sphere_center, center, sizeof(double)*3);
}


HomogTransf RecognitionObject::SymSphere::getClosestTransf(const HomogTransf orig, const Quaternion quat)
{
  // First, compute the desired rotation by
  RotMat r = quat.getRotMat();

  RotMat desired_rot = orig.getRotation().inv() * r;

  // Since we have spherical symmetry, we will just use this orientation as
  // our desired one
  HomogTransf T;
  T.setTranslation(Vec(sphere_center,3));

  HomogTransf bestTransf = orig * T * HomogTransf(desired_rot, Vec("0 0 0",3)) * T.inv();

  return bestTransf;
}









RecognitionObject::RecognitionObject()
{
}

RecognitionObject::RecognitionObject(ros::NodeHandle *n, std::string configFile, std::string objectFolder)
{
  initialize(n, configFile, objectFolder);
}

RecognitionObject::~RecognitionObject()
{
}

bool RecognitionObject::getGuess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &reg_out, Eigen::Matrix4f &initial_T)
{
  return guessFunc->getGuess(cloud, reg_out, initial_T);
}

bool RecognitionObject::getTableGuess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &reg_out, Eigen::Matrix4f &initial_T)
{
  return tableGuessFunc->getGuess(cloud, reg_out, initial_T);
}

HomogTransf RecognitionObject::getClosestTransf(const HomogTransf orig, const Quaternion quat)
{
  return symFunc->getClosestTransf(orig, quat);
}










/*
   bool RecObj::initializeRecObjs(ros::NodeHandle *n, std::string objectFolder, RecParams params, std::vector<RecObj> &recObjs)
   {
// Assign the node pointer for all of our objects
ros::NodeHandle RecObj::node = node;

// Read in all of our objects and save them into point cloud files
recObjs.resize(NUM_REC_OBJ);
for (int i=0; i < NUM_REC_OBJ; i++)
{
char newFilename[MAX_FILE_BUFFER];
sprintf(newFilename,"%s/%s",
objectFolder.c_str(),
recObjFileNames[i].c_str());	

printf("%s\n", newFilename);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

if( pcl::io::loadPCDFile<pcl::PointXYZ> (newFilename, *cloud) == -1)
{
ROS_WARN("Could not read points file. Unable to process... Continuing anyways...");
}
else
{
recObjs[i].setInputCloud(cloud);

switch (i)
{
//
case LONG_BLOCK:
recObjs[i].guessFunc = new Guess_SACIA(cloud, params);
recObjs[i].setSymmetry(false);
break;


case ARCH_BLOCK:
recObjs[i].guessFunc = new Guess_SACIA(cloud, params);
recObjs[i].setSymmetry(false);
break;


case BIG_TRIANGLE:
recObjs[i].guessFunc = new Guess_Planes(cloud, "[0 0 1 -0.035;0 0 -1 0;-1 0 0 0;0 -1 0 0;1 1 0 -0.097]", "[1 3 5; 1 3 4; 1 4 5; 2 3 5; 2 3 4; 2 4 5]", node);
recObjs[i].setSymmetry(true);
double axis[3] = {0.7071, 0.7071, 0.0};
double point[3] = {0.0, 0.0, 0.0175};
int order[1] = {2};
recObjs[i].getClosestTransf = new SymOrder(1, axis, point, order);
break;


default:
ROS_WARN("Warning, guess function and/or symmetries not set up for RecObj[%d]!", i);
break;
}
}
}

return true;
}
 */


bool RecognitionObject::computeObjectFeatures(Guess_SACIA::RecParams params)
{
  if (gtype != GUESS_FEATURES)
  {
    return false;
  }

  return ((Guess_SACIA *)guessFunc)->updateParams(params);
}

bool RecognitionObject::initialize(ros::NodeHandle *n, std::string configFile, std::string objectFolder)
{
  // Save our node pointer
  node = n;

  // Read in our configuration file
  /*
     char newFilename[MAX_FILE_BUFFER];
     sprintf(newFilename,"%s/%s",
     objectFolder.c_str(),
     configFile.c_str());	

     printf("%s\n", newFilename);
   */

  has_symmetry = false;

  stype = SYMMETRY_UNKNOWN;
  gtype = GUESS_UNKNOWN;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  object_cloud = cloud;

  std::ifstream config_file;
  std::string filename = objectFolder + "/object_descriptions/" + configFile;
  config_file.open(filename.c_str());

  std::string line;

  if (!config_file.is_open())
  {
    ROS_WARN("Error: Can't open file: %s", filename.c_str());
    return false;
  }

  bool have_name = false, have_pcdfile = false;

  while (getline(config_file, line))
  {
    // If this is an empty line, a comment, or a space, skip it
    if (line.empty() || line[0] == '#' || isspace(line[0]))
      continue;

    // The first non-ignored line is the name of the block
    if (!have_name)
    {
      obj_name = line;
      have_name = true;
      printf("Object Name: %s\n", obj_name.c_str());
      continue;
    }

    // The second non-ignored line is the point cloud file name, located in
    // the 'pcd_files' folder
    if (!have_pcdfile)
    {
      pcd_file = objectFolder + "/pcd_files/" + line;
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file.c_str(), *object_cloud) == -1)
      {
        ROS_WARN("Error: Can't open pcd file: %s", pcd_file.c_str());
        config_file.close();
        return false;
      }
      have_pcdfile = true;
      printf("pcdfile: %s\n", pcd_file.c_str());
      continue;
    }

    // If we ever see an exclamation mark at the beginning of a line, this
    // signifies the end of the configuration file
    if (line[0] == '!')
    {
      printf("***************************************************\n");
      break;
    }

    // If a '$' is found, this means the symmetry for an object is about to
    // be declared
    if (line[0] == '$')
    {
      int sym_type = strtol(line.substr(1,std::string::npos).c_str(),NULL,10);
      switch (sym_type)
      {
        case SYMMETRY_NONE:
          stype = SYMMETRY_NONE;
          printf("No symmetry\n");
          break;

        case SYMMETRY_ORDER:
          {
            stype = SYMMETRY_ORDER;
            has_symmetry = true;
            getline(config_file, line);
            int num_syms = (int)strtol(line.c_str(),NULL,10);

            double *axes = new double[num_syms * 3];
            double *points = new double[num_syms * 3];
            int *orders = new int[num_syms];

            for (int i=0; i < num_syms; i++)
            {
              getline(config_file, line);
              char *cstr = new char[line.size()];
              cstr[line.size()] = '\0';
              memcpy(cstr, line.c_str(), sizeof(char)*line.size());
              char *str;
              axes[3*i] = strtod(cstr, &str);
              axes[3*i+1] = strtod(str, &str);
              axes[3*i+2] = strtod(str, &str);
              points[3*i] = strtod(str, &str);
              points[3*i+1] = strtod(str, &str);
              points[3*i+2] = strtod(str, &str);
              orders[i] = (int)strtol(str, NULL, 10);
              delete[] cstr;
            }

            printf("order symmetry: \n");

            for (int i=0; i<num_syms; i++)
            {
              printf("%f %f %f %f %f %f %d\n", axes[3*i], axes[3*i+1], axes[3*i+2], points[3*i], points[3*i+1], points[3*i+2], orders[i]);
            }

            symFunc = new SymOrder(num_syms, axes, points, orders);
            delete[] axes;
            delete[] points;
            delete[] orders;

            break;
          }

        case SYMMETRY_CYLINDER:
          {
            stype = SYMMETRY_CYLINDER;
            has_symmetry = true;

            getline(config_file, line);

            double axis[3];
            double point[3];
            bool order_sym;
            char *cstr = new char[line.size()];
            cstr[line.size()] = '\0';
            memcpy(cstr, line.c_str(), sizeof(char)*line.size());
            char *str;
            axis[0] = strtod(cstr, &str);
            axis[1] = strtod(str, &str);
            axis[2] = strtod(str, &str);
            point[0] = strtod(str, &str);
            point[1] = strtod(str, &str);
            point[2] = strtod(str, &str);
            order_sym = (strtol(str, NULL, 10) != 0);

            printf("cylindrical symmetry: \n");
            printf("%f %f %f %f %f %f %d\n", axis[0], axis[1], axis[2], point[0], point[1], point[2], order_sym);

            symFunc = new SymCylinder(axis, point, order_sym);
            
            delete[] cstr;

            break;
          }

        case SYMMETRY_SPHERE:
          {
            stype = SYMMETRY_SPHERE;
            has_symmetry = true;

            getline(config_file, line);

            double center[3];
            char *cstr = new char[line.size()];
            cstr[line.size()] = '\0';
            memcpy(cstr, line.c_str(), sizeof(char)*line.size());
            char *str;
            center[0] = strtod(cstr, &str);
            center[1] = strtod(str, &str);
            center[2] = strtod(str, NULL);

            printf("spherical symmetry: \n");
            printf("%f %f %f\n", center[0], center[1], center[2]);
            symFunc = new SymSphere(center);

            delete[] cstr;

            break;
          }

        default:
          ROS_WARN("Unrecognized symmetry type: %d", sym_type);
      }

      continue;
    }

    if (line[0] == '&')
    {
      int guess_type = strtol(line.substr(1,std::string::npos).c_str(),NULL,10);
      switch(guess_type)
      {
        case GUESS_NONE:
          gtype = GUESS_NONE;
          printf("No guess\n");
          break;

        case GUESS_ELLIPSOID:
          {
            gtype = GUESS_ELLIPSOID;

            getline(config_file, line);
            int symnum = (int)strtol(line.c_str(), NULL, 10);
            Guess_Ellipsoid::SymType esym;
            switch(symnum)
            {
              case Guess_Ellipsoid::SYM_NONE:
                esym = Guess_Ellipsoid::SYM_NONE;
                break;
              case Guess_Ellipsoid::SYM_CONE:
                esym = Guess_Ellipsoid::SYM_CONE;
                break;
              case Guess_Ellipsoid::SYM_CYLINDER:
                esym = Guess_Ellipsoid::SYM_CYLINDER;
                break;
              default:
                ROS_WARN("Unknown symmetry type for ellipsoid guess: %d", symnum);
                esym = Guess_Ellipsoid::SYM_NONE;
            }

            printf("Guess ellipsoid: %d\n", (int)esym);

            guessFunc = new Guess_Ellipsoid(object_cloud, esym, false);
            break;
          }

        case GUESS_PLANES:
          {
            gtype = GUESS_PLANES;

            char planes[Guess_Planes::ARG_CHAR_LENGTH];
            char intersections[Guess_Planes::ARG_CHAR_LENGTH];

            getline(config_file, line);

            unsigned int len = line.size();
            if (len > Guess_Planes::ARG_CHAR_LENGTH - 1)
              len = Guess_Planes::ARG_CHAR_LENGTH - 1;

            planes[len] = '\0';
            memcpy(planes, line.c_str(), len);

            getline(config_file, line);

            len = line.size();
            if (len > Guess_Planes::ARG_CHAR_LENGTH - 1)
              len = Guess_Planes::ARG_CHAR_LENGTH - 1;

            intersections[len] = '\0';
            memcpy(intersections, line.c_str(), len);

            printf("guess planes: \n");
            printf("%s\n", planes);
            printf("%s\n", intersections);

            guessFunc = new Guess_Planes(object_cloud, planes, intersections, n);
            break;
          }

        case GUESS_FEATURES:
          {
            gtype = GUESS_FEATURES;

            Guess_SACIA::RecParams sacia_params;

            getline(config_file, line);

            char *cstr = new char[line.size()];
            cstr[line.size()] = '\0';
            memcpy(cstr, line.c_str(), sizeof(char)*line.size());
            char *str;
            sacia_params.normal_radius = strtod(cstr, &str);
            sacia_params.feature_radius = strtod(str, &str);
            sacia_params.num_samples = (int)strtol(str, &str, 10);
            sacia_params.min_sample_dist = strtod(str, &str);
            sacia_params.k_correspondences = (int)strtol(str, NULL, 10);

            printf("guess features: \n");
            printf("%f %f %d %f %d\n", sacia_params.normal_radius, sacia_params.feature_radius, sacia_params.num_samples, sacia_params.min_sample_dist, sacia_params.k_correspondences);
            guessFunc = new Guess_SACIA(object_cloud, sacia_params);

            delete[] cstr;
            break;
          }
        default:
          ROS_WARN("Unrecognized guess type: %d", guess_type);
      }
      continue;
    }

    // If we get here, we've encountered a line that we weren't expecting
    ROS_WARN("Unrecognized line: %s", line.c_str());
  }
  config_file.close();

  if (stype == SYMMETRY_UNKNOWN)
  {
    ROS_WARN("Unknown symmetry type. Assuming no symmetry");
    stype = SYMMETRY_NONE;
  }
  if (gtype == GUESS_UNKNOWN)
  {
    ROS_WARN("Unknown guess type. Assuming ellipsoid");
    gtype = GUESS_ELLIPSOID;
    guessFunc = new Guess_Ellipsoid(object_cloud, Guess_Ellipsoid::SYM_NONE, false);
  }


  // If we have read in a pcd file, let's do some processing on the cloud so that we are prepared when we need to give a guess on the table
  if (have_pcdfile)
  {
    tableGuessFunc = new Guess_Ellipsoid(object_cloud, Guess_Ellipsoid::SYM_NONE, true);
  }

  return (have_name && have_pcdfile);
}
