#include "get_depth_area_server.h"
MatlabComm matlab;


//int area_buf[640*480*4];
void read_matrix(char *file);
int area_len = 3;

/*x and y are tranformed coordinates in 
  the container matrix, this function uses
  x and y to compute the points that are
  the closest to it and update int array p */
void get_points_area(double x, double y, int col, int row, int** p){
  int first_i = ((int)y * row + (int)x) - row*(area_len - 1) - (area_len - 1);
  for (int i=0; i<2*area_len; i++){
    for (int j=0; j<2*area_len; j++){
      p[i][j] = first_i + row*i + j;
    }
  }
  return;
}


bool get_height_a (container_comm::container_height_area::Request &req, container_comm::container_height_area::Response &res){
  /*get file name and coordinates from request*/
  /*convert coordinates into x and y*/
  char file[1000];

  char command[1000];
  strcpy(file, req.filename.c_str()); 
  //read_matrix(file);
  char newname[1000];
  sprintf(newname, "/home/simplehands/Desktop/container/Kinect__Log__2012:02:27__03:17:04.txt");
  read_matrix(newname);

  
  int xcor = req.x.front();
  int ycor = req.y.front();
  area_len = req.area_length.front();
    int col = 480 - M0 - M2;
  int row = 640 - M1 - M3;
  double* new_coor = conversion_data(xcor, ycor);

  int pix_num = area_len + 1;
  int p[pix_num][pix_num];

  int mat[pix_num][pix_num];
  
  for (int i = 0; i < pix_num; i++){
    for (int j = 0; j < pix_num; j++){
      mat[i][j] = 5;
    }
  }

  //get_points_area(new_coor[0], new_coor[1], col, row, mat);


  //matlab commend
  Mat matlab_mtrx((const char*)mat, pix_num, pix_num);
  matlab.sendMat("a", matlab_mtrx);
  double sigma = pix_num / 6.0;//standard diviation
  // matlab command for average using gaussian filter 
  sprintf(command, "h = fspecial('gaussian', %d, %e);", pix_num, sigma); 

  matlab.sendCommand(command);
  matlab.sendCommand("res=sum(sum(a.*h));");
  res.height = matlab.getValue("res");


  return true;
}

