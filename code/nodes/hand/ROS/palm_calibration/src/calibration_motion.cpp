// calibration_motion.cpp : service to move the robot arm through a series of movements to press the palm against the force sensor
// Copyright (c) 2012 Garth Zeglin

// This handles the robot arm movement to calibrate the palm
// force sensor.  It operates the ABB arm with attached P3 hand
// to press the palm against an ATI mini-40 force sensor mounted
// on the arm table.

// This is a separate node for ease of coding a linear motion
// sequence without requiring threading.  It also helps with
// testing the data and control node without any risk of moving
// the robot.

// Import basic ROS API.
#include <ros/ros.h>

// The MLAB node API to drive the ABB arm and receive data from the server. This
// is a thin C++ wrapper over ROS publish, subscribe, and service calls,
// robot_node does the actual work.
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>

// The MLAB math utilities
#include <matVec/matVec.h>

// Include the message declarations for this package.

//  topics:
#include <palm_calibration/progress.h>

//  service definitions:
#include <palm_calibration/start.h>

/****************************************************************/
// The following handles are stored globals so they are available
// to service callbacks.

static RobotComm *robot = NULL;    // Robot API handle.  If null, then in dry run mode.
static HandComm *hand = NULL;     // Hand API handle.  If null, then in dry run mode.
static ros::Publisher progressPub; // progress reporting topic
static int motion_requested = 0;   // true if a start call has been received
static int motion_active = 0;      // true if currently generating movement

/****************************************************************/
// Speed of the robot.  All speeds specify TCP linear or
// rotational speed in mm/sec or deg/sec.

#define ROT_SPEED     15.0   // general rotational speed, shouldn't matter since pose has constant orientation
#define PREPARE_SPEED 30.0   // linear speed from safe pose to approach point
#define APPROACH_SPEED 15.0  // linear speed from approach point to contact hold point
#define TOUCH_SPEED     1.5  // linear speed while touching the force sensor

// Zone of the robot
#define ZONE 0 //0 means every commanded position is a fine position. (No fly-by)

/****************************************************************/
// Define kinematic constants for the position and orientation of
// the palm and force sensor. (These values might instead be
// stored as ROS parameters.)

// Define a safe joint-space pose for beginning and ending the movement.
static double safe_pose[6] = { 0.0, 0.0, 0.0, 0.0, 90.0, 0.0 };

// Define the robot TCP location which places the palm center
// exactly at the tip of the force sensor probe (the ruby
// sphere).  Ideally, this should register no force signal but
// have no gap.

// The center point is measured by jogging the edge of the palm
// against the force sensor at multiple points (at least three)
// spaced around the radius and fitting a circle. (See
// ../octave/force_sensor_center.m).  The palm radius is
// nominally 50.8mm and the ruby sphere radius is precisely
// 2.50mm so the computed radius should be about 50.8+2.5 = 53.3.
// Z is measured at the center of the palm by (carefully) jogging
// down observing the first inflection of the force sensor
// output.
// static Vec forceSensorCenter = Vec("106.06 800.57 146.3", 3);
// static Vec forceSensorCenter = Vec("106.76 800.30 146.1", 3);  for previous hand
   static Vec forceSensorCenter = Vec("106.76 800.30 148.05", 3);

// Safe location in the center of the robot workspace.
static Vec safePosition = Vec("612 360 540", 3); 

// Orient the palm facing exactly down.  This corresponds to
// rotation by pi around (0.7071 0.7071 0.0) from the work object
// Cartesian pose with the palm pointing up, i.e. the palm normal
// oriented along world Z.  This orientation is the same as in
// the safe joint-space pose [0 0 0 0 90 0].

static Quaternion safeOrient = Quaternion("0.0 0.7071067811865476 0.7071067811865476 0.0");

// This places the palm TCP Z (palm normal) along work frame -Z
// (down), the palm +Y toward work frame +X (right), and the palm
// +X toward work frame +Y (toward the robot base).  This is
// correct when the hand is installed normally with the circuit
// board and motor mounted to the right and the straight finger
// to the left.

// The palm origin is the center of the plate, with -Y toward the
// straight finger and +Y away.  In palm coordinates, the fingers
// are thus at 30, 150, and 270 degrees around Z measured from
// the X axis.

// The  suspension  springs  and  optical sensors  are  nominally
// located as follows:
//  S1 :  90 degrees (+Y)
//  S2 : 330 degrees
//  S3 : 210 degrees

// FIXME: I'm not sure the numerical assigment is correct, but it doesn't matter for now:
static const double finger_angles[3] = { 30  * (M_PI/180.0), 
					 150 * (M_PI/180.0), 
					 270 * (M_PI/180.0) };

// Transit location on the way to the force sensor.
// static Vec transitPosition = Vec("612 360 350", 3); 

// Safe location in midair around which to rotate while
// performing the gravity check motion.
static Vec gravityCheckPosition = Vec("550 300 640", 3); 

/****************************************************************/
// Utility function to move to a TCP point with default rotation.
// This seems trivial, but is intended to compress the notation below.
// Should return true on failure.

static int move( Vec *tcp, double dx, double dy, double dz)
{
  if (robot) {
    return !robot->SetCartesian( (*tcp)[0] + dx, (*tcp)[1] + dy, (*tcp)[2] + dz, 
				 safeOrient[0], safeOrient[1], safeOrient[2], safeOrient[3]);  
  } else {
    ROS_INFO("palm calibration_motion dry run moving to offset %f %f %f", dx, dy, dz );
    return 0;
  }
}

/****************************************************************/
// Utility function to return the current time as a float64.
static inline double now(void)
{
  return ros::Time::now().toSec();
}

/****************************************************************/
// Make sure the robot node is ready to operate.
// Returns zero on success, or an error code on error.
static int prepare_calibration_motion(void)
{
  // Make sure robot communication is blocking
  if (robot && !robot->SetComm(BLOCKING)) {
    ROS_INFO("can't set blocking mode");
    return 100;
  }
  
  // Set default speed limits
  if (robot && !robot->SetSpeed( PREPARE_SPEED, ROT_SPEED ))  {
    ROS_INFO("can't set arm speed");
    return 200;
  }
  
  // Set the default "zone" of our robot (amount of interpolation we allow)
  if (robot && !robot->SetZone(ZONE)) {
    ROS_INFO("can't set arm zone");
    return 300;
  }
  return 0;
}
/****************************************************************/
// Move the robot through the calibration sequence, which takes some time.
// Returns zero on success, or an error code on error.
static int do_calibration_motion_sequence( void )
{
  
  ROS_INFO("Beginning calibration motion sequence...");
  motion_active = 1;
  /****************************************************************/
  // All Cartesian moves are with respect to the proximal left corner of the table.
  // (where the default work object is located)
  // X-axis along the edge that goes left to right (facing the robot)
  // Y-axis along the edge that goes front to back
  // Z-axis pointing up.

  // Move to the center of the robot workspace
  move( &safePosition, 0.0, 0.0, 0.0 );

  // Calibrate the hand. This will move the fingers out of the way of the palm
  hand->Calibrate();

  // Move above force sensor avoiding obstacles in the workspace
  robot->SetJoints(-90.0, 0.0, 0.0, 0.0, 90.0, 0.0);

  // Move to a point above the force sensor.
  double z_offset = 150.0; // distance in mm of palm above probe
  move( &forceSensorCenter, 0.0, 0.0, z_offset);  

  // Descend to the probe.  The fingers must be open already, and
  // the probe will enter between them to touch the palm.
  const double clearance_height = 2.0; // safe distance in mm between probe and palm

  // This is set to zero for normal operation, but can be increased for testing without contact.
  z_offset = 0.0; // 10.0;

  if (robot) robot->SetSpeed( APPROACH_SPEED, ROT_SPEED );
  move( &forceSensorCenter, 0.0, 0.0, z_offset + clearance_height);

  // Define exactly how many steps to take in each dimension; the
  // minimum value for each dim is 2, which samples just the min
  // and max points.
  const int radial_steps  = 9;
  const int angular_steps = 12;
  const int depth_steps   = 3;

  const double min_radius = 5.0;    // inclusive bound
  const double max_radius = 45.0;   // inclusive bound

  const double min_angle = 0.0;     // inclusive bound
  const double max_angle = 2*PI;    // exclusive bound

  const double min_depth = -0.33;   // inclusive bound (negative is probe pressing into the palm)
  const double max_depth = -1.0;    // inclusive bound

  ROS_INFO("beginning calibration sequence at %d points\n", radial_steps*angular_steps*depth_steps);

  // Now move in a spiral pattern of touches of varying depth.
  int r, a, d;
  for ( r = 0; r < radial_steps; r++ ) {
    for ( a = 0; a < angular_steps; a++ ) {
      for ( d = 0; d < depth_steps; d++ ) {
	// Interpolate a sampling location, defined in the palm coordinate frame.
	// (Yes, I know this recomputes unnecessarily, but it is easier to read this way.)
	double radius = min_radius + r * (max_radius - min_radius) / (radial_steps  - 1);
	double angle  = min_angle  + a * (max_angle  - min_angle ) / (angular_steps);  // exclusive bound
	double depth  = min_depth  + d * (max_depth  - min_depth ) / (depth_steps  - 1);
	double dx     = radius * cos(angle);
	double dy     = radius * sin(angle);

	// Skip positions corresponding to the gaps around the fingers.
	if (radius > 28.0) { // if outside the inner edge of the gaps
	  // check whether within the angle subtended by the inner edges
	  if ( fabs(angle - finger_angles[0]) < 0.25 ||
	       fabs(angle - finger_angles[1]) < 0.25 ||
	       fabs(angle - finger_angles[2]) < 0.25 ) {
	    ROS_INFO("calibration motion skipping position at radius %f angle %f depth %f", radius, angle, depth );
	    break; // leave this depth loop, move to next angular position
	  }
	}
	// Begin a single probe motion.  See the notes above about orientation: 
	//  motion of the hand in world X moves the probe in palm -Y
	//  motion of the hand in world Y moves the probe in palm -X
	//  motion of the hand in world Z moves the probe in palm Z

	ROS_INFO("calibration motion beginning probe at radius %f angle %f depth %f", radius, angle, depth );
	palm_calibration::progress progress;
	progress.dx = dx;
	progress.dy = dy;
	progress.dz = depth;

	// move above the probe location
	move( &forceSensorCenter, -dy, -dx, z_offset + clearance_height );
	progress.timestamp = now(); progress.phase = 1; progressPub.publish( progress );

	// press down against the probe
	if (robot) robot->SetSpeed( TOUCH_SPEED, ROT_SPEED );
	move( &forceSensorCenter, -dy, -dx, z_offset + depth );

	// emit a signal to the data collection node to start averaging
	progress.timestamp = now(); progress.phase = 2; progressPub.publish( progress );
	
	// delay slightly in this pose
	ros::Duration(0.5).sleep();

	// emit the sampling signal
	progress.timestamp = now(); progress.phase = 3; progressPub.publish( progress );

	// delay slightly in this pose
	ros::Duration(0.5).sleep();

	// then return upwards
	if (robot) robot->SetSpeed( APPROACH_SPEED, ROT_SPEED );
	move( &forceSensorCenter, -dy, -dx, z_offset + clearance_height );
      }	
    }
  }    

  // return to the safe upper position
  z_offset = 150.0;
  if (robot) robot->SetSpeed( APPROACH_SPEED, ROT_SPEED );
  move( &forceSensorCenter, 0.0, 0.0, z_offset );	

  // and return to safe center pose
  if (robot) robot->SetSpeed( PREPARE_SPEED, ROT_SPEED );
  move( &safePosition, 0.0, 0.0, 0.0 );

  // and return to safe starting pose
  if (robot) robot->SetJoints(safe_pose[0], safe_pose[1], safe_pose[2], safe_pose[3], safe_pose[4], safe_pose[5] );

  ROS_INFO("calibration movement sequence completed.");
  motion_active = 0;
  return 0;
}
/****************************************************************/
// set quaternion vector to axis-angle rotation
void quat_set_axis_angle( double q[4], double angle, double v0, double v1, double v2 ) 
{ 
  double sn = sin(angle / 2);
  Vec axis = Vec(3);
  axis[0] = v0;
  axis[1] = v1;
  axis[2] = v2;
  axis.normalize();
  q[0] = cos(angle / 2);
  q[1] = sn * axis[0];
  q[2] = sn * axis[1];
  q[3] = sn * axis[2];
}
/****************************************************************/
// Move the robot through the gravity self-test motion sequence, which takes some time.
// Returns zero on success, or an error code on error.
static int do_gravity_check_motion_sequence( void )
{
  
  ROS_INFO("Beginning gravity check motion sequence...");
  motion_active = 1;

  /****************************************************************/
  // All Cartesian moves are with respect to the proximal left corner of the table.
  // (where the default work object is located)
  // X-axis along the edge that goes left to right (facing the robot)
  // Y-axis along the edge that goes front to back
  // Z-axis pointing up.

  // Move to the center of the robot workspace
  move( &safePosition, 0.0, 0.0, 0.0 );

  // Go a little faster while testing
  // if (robot) robot->SetSpeed( 4*APPROACH_SPEED, 4*ROT_SPEED );

  // Now compose a series of rotations to test different gravity vectors.
  // This grids a 2D set of force vectors along longitude lines.
  // N.B. This is not a uniform sampling of rotations.
  
  // latitude of M_PI/2 has the palm facing straight down, i.e., gravity pulling normal to the palm
  // for latitude=0, 
  //   longitude of      0: gravity is pulling along palm -Y
  //   longitude of M_PI/2: gravity is pulling along palm -X
  // 15 degree increments -> steps of (13, 24)
  // 30 degree increments -> steps of ( 7, 12)
  const int latitude_steps  =  7; // 13; // 7;
  const int longitude_steps = 12; // 24; // 8;
  
  int lat, lon;
  for (lat = 0; lat < latitude_steps; lat++) {
    for (lon = 0; lon < longitude_steps; lon++) {
      // Interpolate a sampling location.
      const double min_lat = -0.5*M_PI;
      const double max_lat =  0.5*M_PI;  // inclusive, this angle is reached
      const double min_lon = -1.50*M_PI;
      const double max_lon =  0.50*M_PI; // exclusive, this angle not reached

      // move through elevation
      double lat_ang = max_lat + lat * ( min_lat - max_lat) / (latitude_steps  - 1);

      // move through azimuth and back again
      int lon_idx = (lat & 1) ? (longitude_steps - 1 - lon) : lon;
      double lon_ang = min_lon + lon_idx * ( max_lon - min_lon) / (longitude_steps);

      palm_calibration::progress progress;
      progress.latitude = lat_ang;
      progress.longitude = lon_ang;

#if 0
      // Compose quaternions.  The neutral pose has the palm +Z
      // facing 'forward' along -Y, with gravity pulling along
      // the 'equator', i.e. parallel to the palm face.  The (0,
      // 0) polar position corresponds to gravity pulling
      // parallel to the palm along palm -Y (e.g. straight finger
      // on bottom, +Y up), which is joint pose [0 0 0 0 0 90],
      // work frame orientation [0.707 0.707 0 0].

      // double nq1[4], nq2[4];
      // quat_set_axis_angle( nq1, -0.5*M_PI, 0.0, 0.0, 1.0 ); // rotate default frame -90 around Z
      // quat_set_axis_angle( nq2, 0.5*M_PI, 0.0, 1.0, 0.0 );  // rotate frame 90 around the local Y
      // Quaternion qnq1 = Quaternion( nq1 );
      // Quaternion qnq2 = Quaternion( nq2 );
      Quaternion neutral = Quaternion("0.7071067811865476 0.7071067811865476 0.0 0.0");

      double latq[4], lonq[4];
      quat_set_axis_angle( latq, lat_ang, 0.0, 1.0, 0.0 );  // the local Y 'nods' the hand through elevation
      quat_set_axis_angle( lonq, lon_ang, 0.0, 0.0, 1.0 );  // the local Z rotates the hand through azimuth
      Quaternion qlat = Quaternion( latq );
      Quaternion qlon = Quaternion( lonq );

      // Quaternion orient = qnq1 ^ qnq2 ^ qlat ^ qlon; 
      Quaternion orient = neutral ^ qlat ^ qlon; 
      ROS_INFO("gravity motion moving to lat %f lon %f (%d, %d)", lat_ang, lon_ang, lat, lon );

      if (robot) {
	// bias the hand downward for positive latitudes to help with self-clearance
	double zoffset = -200*lat_ang;
	progress.timestamp = now(); progress.phase = 1; progressPub.publish( progress );
	robot->SetCartesian( gravityCheckPosition[0], gravityCheckPosition[1], gravityCheckPosition[2] + zoffset, 
			     orient[0], orient[1], orient[2], orient[3] );  
      } else {
	Vec axis = orient.getAxis();
	double angle = orient.getAngle();
	ROS_INFO("palm gravity_check_motion dry run moving to rotation %f around %f %f %f", angle, axis[0], axis[1], axis[2] );
      }
#else
      // rather than get lost in the quaternions, just program this in joint space for now
      if (robot) {
	robot->SetJoints( 0.0, 0.0, 0.0, 0.0, 
			  lat_ang * (180.0 / M_PI),         // joint 5 controls elevation (latitude)
			  90.0 + lon_ang * (180.0 / M_PI)); // joint 6 controls azimuth (longitude)

      };
#endif

      // phase 2 is settling
      progress.timestamp = now(); progress.phase = 2; progressPub.publish( progress );

      // delay slightly in this pose
      ros::Duration(0.5).sleep();

      // emit the sampling signal
      progress.timestamp = now(); progress.phase = 3; progressPub.publish( progress );

      // delay slightly in this pose
      ros::Duration(0.5).sleep();

      // additional delay for debugging
      // ros::Duration(2.0).sleep();
    }
  }      

  // and return to safe starting pose
  if (robot) robot->SetSpeed( PREPARE_SPEED, ROT_SPEED );
  if (robot) robot->SetJoints(safe_pose[0], safe_pose[1], safe_pose[2], safe_pose[3], safe_pose[4], safe_pose[5] );

  ROS_INFO("gravity check movement sequence completed.");
  motion_active = 0;
  return 0;
}

/****************************************************************/
// Check whether the robot is exactly in the safe pose for beginning the sequence.
// Returns true with a numeric error code on failure.  Returns zero if pose matches.

static int check_robot_safe_pose(void)
{
  double pose[6];
  // N.B. GetJoints is call-by-reference, this fills in the array values
  bool success = robot->GetJoints( pose[0], pose[1], pose[2], pose[3], pose[4], pose[5] );

  if (!success) {
    ROS_INFO("palm calibration_motion unable to read arm joint angles");
    return 1;
  }

  int i;
  for (i = 0; i < 6; i++) if ( fabs(pose[i] - safe_pose[i]) > 0.1 ) return 2;
  return 0;
}

/****************************************************************/
// handle the startCalibrationMotion service call
static bool startCalibrationMotion( palm_calibration::start::Request  &request, 
				    palm_calibration::start::Response &response )
{
  if (motion_active) {
    response.failure = 1000;
    response.msg = "currently running calibration cycle";
    return true;
    
  }
  // verify preconditions for movement
  int err = (robot != NULL) && check_robot_safe_pose();

  if (err) {
    response.failure = err;
    response.msg = "arm not in safe initial pose";
    return true;
  }

  err = prepare_calibration_motion();
  if (err) {
    response.failure = err;
    response.msg     = "unable to prepare robot arm"; 
    return true;
  }

  // else ready to go, this will trigger the main event loop motion
  motion_requested = 1;
  response.failure = 0;
  response.msg = "calibration motion ready to start";
  return true; // return true to indicate that response is valid
}

/****************************************************************/
/****************************************************************/
// handle the startGravityCheckMotion service call
static bool startGravityCheckMotion( palm_calibration::start::Request  &request, 
				     palm_calibration::start::Response &response )
{
  if (motion_active) {
    response.failure = 1000;
    response.msg = "currently running calibration movement";
    return true;
    
  }
  // verify preconditions for movement
  int err = (robot != NULL) && check_robot_safe_pose();

  if (err) {
    response.failure = err;
    response.msg = "arm not in safe initial pose";
    return true;
  }

  err = prepare_calibration_motion();
  if (err) {
    response.failure = err;
    response.msg     = "unable to prepare robot arm"; 
    return true;
  }

  // else ready to go, this will trigger the main event loop motion
  motion_requested = 2;
  response.failure = 0;
  response.msg = "gravity check motion ready to start";
  return true; // return true to indicate that response is valid
}

/****************************************************************/

int main(int argc, char** argv)
{
  // Start the ROS connection.
  ros::init(argc, argv, "calibration_motion");
  ros::NodeHandle node;

  // check arguments
  int dryrun = 0;

  for (int arg = 1; arg < argc; arg++ ) {
    if (!strcmp( "--dryrun", argv[arg])  || !strcmp( "-d", argv[arg] )) dryrun = 1;
  }

  // Set up a publisher to indicate status.
  progressPub = node.advertise<palm_calibration::progress>("palm_calibration/progress", 100 );

  // Advertise services.
  ros::ServiceServer calibrationMotionSrv = node.advertiseService( "palm_calibration/startCalibrationMotion", startCalibrationMotion );
  ros::ServiceServer gravityCheckMotionSrv = node.advertiseService( "palm_calibration/startGravityCheckMotion", startGravityCheckMotion );

  if (dryrun) { 
    ROS_INFO("palm calibration_motion starting up in dry run mode.");

  } else { 
    // Initialize the C++ API wrapper around the ABB arm node service calls.
    robot = new RobotComm(&node);
    if (!robot) {
      ROS_INFO("Error: palm calibration motion service unable to initialize robot arm API.");
      exit(1);
    }
    hand = new HandComm(&node);
    if (!hand) {
      ROS_INFO("Error: palm calibration motion service unable to initialize hand API.");
      exit(1);
    }

  }

  // Event loop.
  ROS_INFO("Palm calibration motion service entering event loop.");
  int rate = 1; // Hz
  static ros::Rate loop_rate(rate);
  while ( ros::ok() ) {
    ros::spinOnce();
    loop_rate.sleep();

    // The actual motion is performed from the main event loop so
    // that the service request which initiates movement can
    // return right away without blocking the calling process.
    // The movement can take some time.

    switch (motion_requested) {
    case 1: 
      {
	motion_requested = 0;
	int err = do_calibration_motion_sequence();
	if (err) ROS_INFO("palm calibration_motion failed with code %d", err);
	break;
      }
      
    case 2:
      {
	motion_requested = 0;
	int err = do_gravity_check_motion_sequence();
	if (err) ROS_INFO("palm gravity_check_motion failed with code %d", err);
	break;
      }

    default:      
      {
	// otherwise emit a periodic heartbeat
	palm_calibration::progress progress;
	progress.dx = 0.0;
	progress.dy = 0.0;
	progress.dz = 0.0;
	progress.phase = 0;
	progress.timestamp = now();
	progressPub.publish( progress );
	break;
      }
    }
  }

  ROS_INFO("Palm calibration motion service exiting event loop.");

  if (robot) robot->shutdown();
  if (hand) hand->shutdown();

  return 0;
}
