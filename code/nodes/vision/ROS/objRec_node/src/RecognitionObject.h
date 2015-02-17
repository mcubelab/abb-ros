// Class for objects we're trying to recognize. Has link to the point cloud
// file, a function that deals with symmetries, planes and intersections,
// etc. 

#ifndef RECOBJ_H
#define RECOBJ_H

#include <ros/ros.h>
#include <ros/package.h>
#include <objRec_comm/objRec_comm.h>
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


//#define VISUALIZE_GUESS
//#define VISUALIZE_SYMMETRY


// Includes needed for visualizing point clouds
#if defined(VISUALIZE_GUESS) || defined(VISUALIZE_SYMMETRY)

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#endif

// Class for objects we're trying to recognize. 

class RecognitionObject
{
  public:

    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    // Guess Functions
    // These are different functions that can be used to guess the initial pose
    // of the object. Each object will select one of these to use. A guess
    // function will have its own constructor, and then will call a
    // 'getGuess' function, which takes in the point cloud, and outputs the
    // point cloud of the object in the frame of the given cloud, and the
    // transform that moved the object to that place in the cloud
    class GuessFunc
    {
      public:
        virtual bool getGuess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
            pcl::PointCloud<pcl::PointXYZ>::Ptr &reg_out, 
            Eigen::Matrix4f &initial_T) = 0;
        virtual ~GuessFunc() = 0;
    };

    /////////////////////////////////////////////////////////////////////
    // SAmple Consensus Initial Alignment guess function
    class Guess_SACIA : public GuessFunc
    {
      public:
        // Short hand that will be used during feature matching
        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
        typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
        typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
        typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

        // RecParams
        // A structure which contains all of the parameters we need to tune for feature matching
        struct RecParams
        {
          // FPFH Feature parameters
          double normal_radius;    // Radius to use when computing surface normals
          double feature_radius;   // Radius to use when computing nearest neighbors for features

          // SAC-IA matching parameters
          int num_samples;        // Number of sample points to use for estimating transform
          double min_sample_dist;  // Minimum allowable distance between selected points
          int k_correspondences;  // Amount of randomness when finding nearest feature in output cloud
        };

        // Constructor for feature matching takes in the object point cloud
        // and the feature matching parameters to use
        Guess_SACIA(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, RecParams p);
        virtual bool getGuess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
            pcl::PointCloud<pcl::PointXYZ>::Ptr &reg_out, 
            Eigen::Matrix4f &initial_T);

        // Updates the parameters for feature matching and recomputes
        // features
        bool updateParams(RecParams p);

      private:
        // Compute normals and features needed for feature matching
        void computeSurfaceNormals(PointCloud::Ptr cloud, SurfaceNormals::Ptr n);
        void computeObjectNormals(PointCloud::Ptr cloud, SurfaceNormals::Ptr n);
        void computeLocalFeatures(PointCloud::Ptr cloud, SurfaceNormals::Ptr n, LocalFeatures::Ptr f);

        // A way to visualize normals
#ifdef VISUALIZE_GUESS
        void visualize_normals(PointCloud::Ptr cloud, SurfaceNormals::Ptr normals);
#endif

        // Local variables needed for feature matching
        RecParams sacia_params;                 // Parameters for feature matching
        PointCloud::Ptr xyz_;                   // Object point cloud
        SurfaceNormals::Ptr normals_;           // Object normals
        LocalFeatures::Ptr features_;           // Object features
        SearchMethod::Ptr search_method_xyz_;   // Search method to use for features
    };

    /////////////////////////////////////////////////////////////////////
    // Guess function that uses planes
    class Guess_Planes : public GuessFunc
  {
    public:
      // Max length of an argument that we'll send to MATLAB
      static const unsigned int ARG_CHAR_LENGTH = 256;
      
      // Our constructor takes in the point cloud, a list of planes, a list
      // of planes that intersect and form a point, and a node handle so
      // that we can access the matlab node
      Guess_Planes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
          char p[ARG_CHAR_LENGTH], char i[ARG_CHAR_LENGTH], ros::NodeHandle *n);
      virtual bool getGuess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
          pcl::PointCloud<pcl::PointXYZ>::Ptr &reg_out, 
          Eigen::Matrix4f &initial_T);
    
    private:
      // Minimum number of points remaining in the point 
      //  cloud before we stop looking for planes
      static const unsigned int MIN_POINTS_FOR_PLANE = 500;
      static const unsigned int MIN_POINTS_IN_PLANE = 400;
      // How close a point has to be between object and point cloud to 
      // be considered a match during the initial guessing stage (mm)
      static const double INITIAL_RAD = 0.004; 

      // Threshold for how far away points can be from our guessed plane in
      // order to be considered an inlier (mm)
      static const double PLANE_THRESH = 0.004;

      // The maximum number of planes to look for in an image
      static const int MAX_PLANES = 5;

#ifdef VISUALIZE_GUESS
      void visualize_options(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> diff_transforms, std::vector<int> neighbor_list, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
#endif

      
      // Save our planes and intersections that we'll be sending to matlab
      char planes[ARG_CHAR_LENGTH];
      char intersections[ARG_CHAR_LENGTH];

      // Save our interface to matlab
      MatlabComm matlab;

      // Save the point cloud so we can resolve conflicts among multiple
      // guesses
      pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud;
  };

    /////////////////////////////////////////////////////////////////////
    // Guess function that computes eigenvectors to form an initial guess
    // Note that this function is highly sensitive to occlusions
    class Guess_Ellipsoid : public GuessFunc
  {
    public:
      // The type of symmetry that exists in this ellipsoid (Can save some
      // computation time)
      enum SymType
      {
        SYM_NONE = 0,
        SYM_CONE,
        SYM_CYLINDER
      };


      // Our constructor takes in the object point cloud so it can compute
      // the eigenvectors it needs
      Guess_Ellipsoid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, SymType sym, bool rem_outliers);
      virtual bool getGuess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
          pcl::PointCloud<pcl::PointXYZ>::Ptr &reg_out, 
          Eigen::Matrix4f &initial_T);


    private:
      void computeCentroidAndEigenVectors(pcl::PointCloud<pcl::PointXYZ>::Ptr c, 
          Eigen::Vector3f &cent, Eigen::Matrix3f &eigen_vecs, Eigen::Vector3f &eigen_vals, bool rem_outliers);

      // How much eigenvalues from our object and target can differ before
      // we sound the alarm
      static const double EIG_VAL_FAC = 0.00001;
      static const double CLOSE_RAD = 0.005;
      static const double MAGIC_FACTOR = 3.5; // factor from sqrt(eig) to outside edge of object, taking into account spurious noise, etc
      static const double STEP_SIZE = 0.005; // m

      // If computing outliers, if the squared distance from a point to the mean of
      // the cloud is above this value, we will remove it (m^2)
      static const double OUTLIER_DIST_THRESH2 = 0.02;

      Eigen::Vector3f centroid;
      Eigen::Matrix3f eigen_vectors;
      Eigen::Vector3f eigen_values;
      pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud;
      SymType symmetry;
      bool remove_outliers;
  };


    /////////////////////////////////////////////////////////////////////
    // Symmetry functions
    // If an object has a symmetry, we will use one of these functions to
    // determine the optimal transform that gets us as close as possible to a
    // preferred orientation

    class SymFunc
    {
      public:
        virtual HomogTransf getClosestTransf(const HomogTransf orig, const Quaternion quat) = 0;
        virtual ~SymFunc() = 0;
    };

    class SymOrder : public SymFunc
  {
    public:
      SymOrder(int num_syms, double* axes, double* points, int* orders);
      virtual HomogTransf getClosestTransf(const HomogTransf orig, const Quaternion quat);
    private:
      // Holds all of the transforms defined by our object symmetries. Note
      // that these are designed to be concatenated together
      std::vector<std::vector<HomogTransf> > symTransf;
      int num_symmetries; // Holds the number of different order symmetries that exist for this object
      // Holds the order of each of the symmetries of this object
      std::vector<int> sym_orders;
  };

    class SymCylinder : public SymFunc
  {
    public:
      SymCylinder(double axis[3], double point[3], bool sym_order);
      virtual HomogTransf getClosestTransf(const HomogTransf orig, const Quaternion quat);
    private:
      // Transform that gets us into the symmetry frame (where any rotation
      // about the z-axis is symmetric
      HomogTransf T;
      bool order_symmetry;
  };

    class SymSphere : public SymFunc
  {
    public:
      SymSphere(double center[3]);
      virtual HomogTransf getClosestTransf(const HomogTransf orig, const Quaternion quat);
    private:
      double sphere_center[3];
  };



  public:
    // Constructors
    RecognitionObject();
    RecognitionObject(ros::NodeHandle *n, std::string configFile, std::string objectFolder);
    virtual ~RecognitionObject();

    // Initialize the object via a configuration file
    bool initialize(ros::NodeHandle *n, std::string configFile, std::string objectFolder); 

    // Compute object features when necessary (When using guess
    bool computeObjectFeatures(Guess_SACIA::RecParams params);

    // Get the best guess of where the object might be given a point cloud
    bool getGuess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &reg_out, Eigen::Matrix4f &initial_T);

    // Get a guess for the object when the object is on a table and no occlusions are assumed
   bool getTableGuess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &reg_out, Eigen::Matrix4f &initial_T);

    // If our object is symmetric, get the closest transform it can be at
    // to a preferred orientation 
    HomogTransf getClosestTransf(const HomogTransf orig, const Quaternion quat);

    // Accessor functions to get point cloud of object, and whether or not
    // it has any symmetries
    inline pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud() const {return object_cloud;}
    inline bool isSymmetric() const {return has_symmetry;}

    // Type of symmetry that the object might have
    enum SymType
    {
      SYMMETRY_NONE = 0,
      SYMMETRY_ORDER,
      SYMMETRY_CYLINDER,
      SYMMETRY_SPHERE,
      SYMMETRY_UNKNOWN
    };

    // Type of guess we'll use as an initial pose of the object
    enum GuessType
    {
      GUESS_NONE = 0,
      GUESS_ELLIPSOID,
      GUESS_PLANES,
      GUESS_FEATURES,
      GUESS_UNKNOWN
    };

  private:
    // All objects will share the same node pointer
    //static ros::NodeHandle *node;
    ros::NodeHandle *node;
    SymFunc* symFunc;
    GuessFunc* guessFunc;
    GuessFunc* tableGuessFunc;
    std::string pcd_file;
    std::string obj_name;
    bool has_symmetry;
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud;
    SymType stype;
    GuessType gtype;
};

#endif // RECOBJ_H
