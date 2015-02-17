#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pc_utils.h"

/* Description

parameters:
returns: 
*/

pcl::PointXYZ convertRGBtoPCPoint(unsigned int imageHeight, unsigned int imageWidth, pcl::PointCloud<pcl::PointXYZ> * xyzPointCloud, unsigned int x, unsigned int y){
  
  pcl::PointXYZ res;

  // Check for size correspondence of image and point cloud.
  if ( imageHeight * imageWidth != xyzPointCloud->size() ){
    ROS_ERROR("Sizes not equal.");
    return res;
  }
  
  int loc = imageWidth * y + x; 

  // Assign to corresponding point in the point cloud

  res = xyzPointCloud->points[loc];
  return res;
}

void getPosVectorOfIthPoint(unsigned int imageHeight, unsigned int imageWidth, pcl::PointCloud<pcl::PointXYZ>* xyzPointCloud, unsigned int imageX, unsigned int imageY, float vecX, float  vecY, float vecZ){

  if (imageHeight * imageWidth != xyzPointCloud->size() ){
    ROS_ERROR("Sizes not equal.");
    return;
  }

  int loc = imageWidth * imageY + imageX;

  vecX = xyzPointCloud->points[loc].x;
  vecY = xyzPointCloud->points[loc].y;
  vecZ = xyzPointCloud->points[loc].z;

  ROS_INFO("Position vector (before exiting function) are <%f, %f, %f>.", vecX, vecY, vecZ);

}

void getColors(pcl::PointXYZRGB point, uint8_t * r, uint8_t * g, uint8_t * b){
  
  uint32_t rgb = *reinterpret_cast<int*>(&point.rgb);
  *r = (rgb >> 16) & 0x0000ff;
  *g = (rgb >> 8)  & 0x0000ff;
  *b = (rgb)       & 0x0000ff;

}

uint8_t* getColorVector(pcl::PointXYZRGB point){
  uint8_t colorVector [3];
  getColors(point, &colorVector[0], &colorVector[0] + 1, &colorVector[0] + 2);
  return &colorVector[0];
}
