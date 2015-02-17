/*the function conversion takes the length(mm)/pixel ration, the coordinates
 of the robot and returns an int array containing two ints, the x and y
 coordinates on the coresponding kinect depth image*/
double converCoor[2];

void conversion(int x, int y){
  converCoor[0] = CX + (x/RATIO);
  converCoor[1] = 480 - CY - (y/RATIO);
  return;
}

/*the function conversion_data takes the coordinates of the robot, the ratio,
  the robot coordinates that correspond to the left-bottom corner and returns
 an int array containing two ints, the x and y coordinates on the correspoding
 data map that has been cut to bin-size*/
double* conversion_data(int robot_x, int robot_y){
  conversion(robot_x, robot_y);
  converCoor[0] = converCoor[0] - M0;
  converCoor[1] = converCoor[1] - M1;
  //ROS_INFO("local x %f\n", converCoor[0]);
  if (converCoor[0] < 0 || converCoor[1] < 0)
    return 0; 
  return converCoor;
}
