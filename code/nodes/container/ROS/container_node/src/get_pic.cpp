#include "container_node.h"
#include "parameters.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

int i;
int ii;
int j;
int bh;
int bw;
int size;
short pdata[480*640];
double p_data[12];
double k_data[9];
double r_data[9];

bool written;

int test = 123;
/*copy bin data from kinect to desinated memory*/
/*
margin demo:
 _____________
|      1      |
|   _______   |
|0 |  bin  |2 |
|  |_______|  |
|      3      |
|_____________|

*/
char fileName[LOG_BUFFER];

void get_bin(short* data, int margin0, int margin1,
             int margin2, int margin3){
  char completeFileName[LOG_BUFFER];
  time_t timer;

  // Name of the file where to save it
  timer=time(NULL);
  tm* today;
  today = localtime(&timer);
  sprintf(fileName,"Kinect__Log__%d:%02d:%02d__%02d:%02d:%02d.txt",
      today->tm_year+1900,
      today->tm_mon+1,
      today->tm_mday,
      today->tm_hour,
      today->tm_min,
      today->tm_sec);

  // Complete filename
  sprintf(completeFileName,"/home/simplehands/Desktop/container/%s",fileName);

  //Open file
  FILE * fLog = fopen(completeFileName,"w+");
    
  bh = 480-margin1-margin3;
  bw = 640-margin0-margin2;
  size = bw*bh;
  int bin[size];
  j=0;
  for (i=margin1; i<(480-margin3); i++){
    for (ii = margin0; ii < (640 - margin2); ii++){
      bin[j] = data[i*640+ii];
      fprintf(fLog, "%d ", data[i*640+ii]);
      j++;
    }
    fprintf(fLog, "\n");
  }
  fprintf(fLog, "\n");
  fclose(fLog);
  return;
}

int in;

bool get_image(container_comm::container_SaveMatrix::Request &req, container_comm::container_SaveMatrix::Response &res){
  if (!written)
    get_bin(pdata, M0, M1, M2, M3);
  written = true;
  res.filename = fileName;
  written = false;
  printf("%d: k, r, p\n");
  for (in = 0; in < 9; in++){
    printf("d: %f, %f, %f\n", k_data[in], r_data[in], p_data[in]);
  }
  return true;
}

/*subscribe to kinect message*/
void callback(const sensor_msgs::ImageConstPtr & msg){
  memcpy(pdata, &msg->data[0], msg->step*msg->height);
  return;
}
  
//just get matrices
  void callback2(const sensor_msgs::CameraInfo::ConstPtr & msg){
  memcpy(k_data, &msg->K[0],8*9);
 memcpy(r_data, &msg->R[0],8*9);
 memcpy(p_data, &msg->P[0],8*12);
}

int main(int argc, char **argv){
  written = false;
  ros::init(argc, argv, "container_node");
  ros::init(argc, argv, "get_depth_points_server");
  
  //start get_depth_area
  ros::init(argc, argv, "get_depth_area_server");
  
  //ros::NodeHandle m;
  
  ros::NodeHandle n;
  ros::NodeHandle m;

  //start get_depgh_area
  matlab.subscribe(&n);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("/camera/depth/image_raw", 1, callback);

 //start
  ros::Subscriber sub2 = m.subscribe("/camera/depth/camera_info", 1000, callback2);
  ros::ServiceServer service0 = n.advertiseService("container_SaveMatrix", get_image);
  ros::ServiceServer service1 = n.advertiseService("get_depth_points", get_height_p);
  
  //start get_depth_area
  ros::ServiceServer service2 = n.advertiseService("get_depth_area", get_height_a);

  ros::spin();
  return 1;
}
