#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/imp/point_types.hpp>

/* Description

parameters:
returns: 
*/

pcl::PointXYZRGB convertRGBtoPCPoint(IplImage &rgbIm, pcl::PointCloud<pcl::PointXYZRGB> &rgbPointCloud, int x, int y){
  
  // Check for size correspondence of image and point cloud.
  if ( rgbIm.imageSize != rgbPointCloud.size() ){
    return false;
  }
  
  int loc = rgbIm.width * y + x; 

  // TODO: convert rgbIm to usable format?

  /*rgbPointCloud.points[loc].b = rgbIm.imageData[rgbIm.widthStep * y + rgbIm.nChannels * x + 0];
  rgbPointCloud.points[loc].g = rgbIm.imageData[rgbIm.widthStep * y + rgbIm.nChannels * x + 1];
  rgbPointCloud.points[loc].r = rgbIm.imageData[rgbIm.widthStep * y + rgbIm.nChannels * x + 2];*/

  // assign to corresponding point in the point cloud

  return rgbPointCloud.points[loc];
}
