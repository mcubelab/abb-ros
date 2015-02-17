#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

int main(int argc, char** argv)
{
/*
   // Original Depth
  double in[3][3] = {
    {578.102690998601, 0, 333.385560430016}, 
    {0, 581.454225325842, 254.968962116837}, 
    {0, 0, 1}};

  double dist[5] = {-0.10271643697228, 0.162795773366185, -0.00334691672264735, 0.00567588667441946, 0};
  */

  /*
  // New Depth
  double in[3][3] = {
    {586.80836, 0, 324.23359}, 
    {0, 588.73242, 239.94186}, 
    {0, 0, 1}};

  double dist[5] = {-0.09784, 0.34524, -0.00195, 0.00826, 0.00000};
  */

  // New RGB
  double in[3][3] = {
    {527.60326, 0, 315.34733}, 
    {0, 528.83841, 249.26472}, 
    {0, 0, 1}};

  double dist[5] = {0.16459, -0.26875, -0.00052, 0.00730, 0.00000};
  Mat intrinsics = Mat(3,3,CV_64F,in);
  Mat distortion = Mat(1,5,CV_64F,dist);

  std::cout << intrinsics << std::endl;
  std::cout << distortion << std::endl;



  CvSize size = cvSize(640,480);
  double a = 0.0;
  Mat P;

  P = getOptimalNewCameraMatrix(intrinsics, distortion, size, a);

  std::cout << P << std::endl;


  return 0;
}
