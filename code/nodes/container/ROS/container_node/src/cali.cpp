int n_boards = 0;
const int board_dt = 4;
int board_w;
int board_h;
double[10][24] corner_x;
double[10][24] corner_y;
double[24] result_x;
double[24] result_y;
double robo_x;
double robo_y;
double robo_z;

bool start;

FILE* fd;

int blah = 0;

namespace enc = sensor_msgs::image_encodings;

void cv(const sensor::ImageConstPtr & msg){
  if (!start){
    return;
  }
  if (blah == 10){
    int t1;
    int t2;
    for (t1=0;t1<10;t1++){
      for (t2=0;t2<24;t2++){
        result_x[t1][t2] += corner_x[t1][t2]/10.0;
        result_y[t1][t2] += corner_y[t1][t2]/10.0;
      }
    }
    start = false;
    char filename[200];
    sprintf(filename, "", <F8>);
    fopen();
    fclose(fd);
    return;
  }

  cv_bridge::CvImagePtr cim = cv_bridge::toCvCopy(msg, enc::BAYER_GRBG8);

  board_w = 4; // Board width in squares
	board_h = 6; // Board height 
	n_boards = 1; // Number of boards
	int board_n = board_w * board_h;
	CvSize board_sz = cvSize( board_w, board_h );
  
  CvCapture* capture = cvCreateCameraCapture(1);
	//assert( capture );

	cvNamedWindow( "Calibration" );
	// Allocate Sotrage
	CvMat* image_points		= cvCreateMat( n_boards*board_n, 2, CV_32FC1 );
	CvMat* object_points		= cvCreateMat( n_boards*board_n, 3, CV_32FC1 );
	CvMat* point_counts			= cvCreateMat( n_boards, 1, CV_32SC1 );
	CvMat* intrinsic_matrix		= cvCreateMat( 3, 3, CV_32FC1 );
	CvMat* distortion_coeffs	= cvCreateMat( 5, 1, CV_32FC1 );

	CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];
	int corner_count;
	int successes = 0;
	int step, frame = 0;

	//IplImage *image = cvQueryFrame( capture );
  IplImage preimage((cim->image));
  IplImage* image = &preimage;
	IplImage *gray_image = cvCreateImage( cvGetSize(image), 8, 1 );

	// Capture Corner views loop until we've got n_boards
	// succesful captures (all corners on the board are found)
  int ttt = 0;
	//while( successes < n_boards ){
  while(ttt < 1){
    ttt ++;
		// Skp every board_dt frames to allow user to move chessboard
		if( frame++ % board_dt == 0 ){

			// Find chessboard corners:
			int found = cvFindChessboardCorners( image, board_sz, corners,
				&corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );

      //assert(!found);
			// Get subpixel accuracy on those corners

      cvFindCornerSubPix( gray_image, corners, corner_count, cvSize( 11, 11 ), 
				cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

			// Draw it
			cvDrawChessboardCorners( image, board_sz, corners, corner_count, found );

      cvShowImage( "Calibration", image );

			// If we got a good board, add it to our data
			if( corner_count == board_n ){
				step = successes*board_n;
				for( int i=step, j=0; j < board_n; ++i, ++j ){
					CV_MAT_ELEM( *image_points, float, i, 0 ) = corners[j].x;
					CV_MAT_ELEM( *image_points, float, i, 1 ) = corners[j].y;
					CV_MAT_ELEM( *object_points, float, i, 0 ) = j/board_w;
					CV_MAT_ELEM( *object_points, float, i, 1 ) = j%board_w;
					CV_MAT_ELEM( *object_points, float, i, 2 ) = 0.0f;
          ROS_INFO("coner.x %f, corner.y %f", corners[j].x, corners[j].y);
          corner_x[blah][j] = corners[j].x;
          corner_y[blah][j] = corners[j].y;
				}
        blah++;
				CV_MAT_ELEM( *point_counts, int, successes, 0 ) = board_n;
				successes++;
			}
		} 
		// Handle pause/unpause and ESC
		int c = cvWaitKey( 15 );
		if( c == 'p' ){
      
			c = 0;
			while( c != 'p' && c != 27 ){
				c = cvWaitKey( 250 );
			}
		}
		if( c == 27 ){
      fclose(fd);
			return;
    }
      //return 0;
		image = cvQueryFrame( capture ); // Get next image
	} // End collection while loop

  return; 
}


void calibration(container_comm::container_cali::Request &req, container_comm::container_cali::Response &res){
  start = true;
  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "cali");

  ros::NodelHandle n;
  image_transport::ImageTransport it(n);

  image_transport::Subsriber sb = it.subscribe("/camera/rgb/image_raw", 1, cv);

  ros::ServiceServer service0 = n.advertiseService("calibration", calibration)

  ros::spin();
  return 1;
}
