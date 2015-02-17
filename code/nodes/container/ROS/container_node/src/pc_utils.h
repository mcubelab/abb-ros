#ifndef pc_utils
#define pc_utils

#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

pcl::PointXYZ convertRGBtoPCPoint(unsigned int, unsigned int, pcl::PointCloud<pcl::PointXYZ>*, unsigned int, unsigned int);

void getPosVectorOfIthPoint(unsigned int, unsigned int, pcl::PointCloud<pcl::PointXYZ>*, unsigned int, unsigned int, float, float, float);

void getColors(pcl::PointXYZRGB, uint8_t *, uint8_t *, uint8_t *);

uint8_t* getColorVector(pcl::PointXYZRGB);

#endif
