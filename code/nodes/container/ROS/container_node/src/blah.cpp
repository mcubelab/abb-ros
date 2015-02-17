#include "get_depth_points_server.h"
#include "parameters.h"
#include "conversion.cpp"
using namespace std;

void get_points(float x, float y, int col, int row);
int p[4];
int receive[2];
int buf[480*640];
void read_matrix(char *file);
int sum;

/*read matrix from logger file*/
void read_matrix(char *file){
  FILE* pfile;
  pfile = fopen(file, "r");
  if (pfile == NULL){
    fputs ("File error", stderr); 
    exit(1);
  }
  int s;
  int count = 0;
  int r;
  while((r = fscanf(pfile, "%d", &s))>0){
    buf[count] = s;
    count++;
  }
  fclose(pfile);
}

/*x and y are tranformed coordinates in 
  the container matrix, this function uses
  x and y to compute the four points that are
  the closest to it and update int array p */
void get_points(double x, double y, int col, int row){
  p[0] = (int)y * row + (int)x;//get the top-left point
  p[1] = p[0] + 1;
  p[2] = p[0] + row;
  p[3] = p[1] + row;
  return;
}

int get_height(){
  sum = 0;
  for (int i=0; i<4; i++){
    sum += buf[p[i]];
  }
  return (1138 - (sum / 4));
}

/*subscribe to the point whose height is requested*/
bool get_height_p(container_comm::container_height::Request &req,container_comm::container_height::Response &res){

  /*get file name and coordinates from request*/
  /*convert coordinates into x and y*/
  char file[1000];
  int xcoor[1];
  int ycoor[1];
  strcpy(file, req.filename.c_str());
  
  int xcor = req.x.front();
  //ROS_INFO("init x is %d\n", xcor);
  int ycor = req.y.front();
  int col = 480 - M0 - M2;
  int row = 640 - M1 - M3;

  double* new_coor = conversion_data(xcor, ycor);
  char newname[1000];
  
  sprintf(newname, "/home/simplehands/Desktop/container/Kinect__Log__2012:02:29__15:02:19.txt");
  
  read_matrix(newname);
  
  get_points(new_coor[0], new_coor[1], col, row);
  res.height = get_height();

  char tempname[1000]; 
  sprintf(tempname, "/home/simplehands/Desktop/container/test.txt");

  FILE * fe = fopen(tempname, "w+");
  
  for (int xx = 0; xx < 600; xx++){
    for (int yy = 0; yy < 600; yy++){
      double* tcoor = conversion_data(yy, xx);
      get_points(tcoor[0],tcoor[1], col, row);
      int h = get_height();
      fprintf(fe, "%d ", h);
    }
    fprintf(fe, "\n");
  }
  fclose(fe);

  return true;
}

