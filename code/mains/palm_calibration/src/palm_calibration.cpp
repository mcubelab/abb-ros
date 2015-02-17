// palm_calibration.cpp : record data from the hand and force sensor during a calibration motion sequence
// Copyright (c) 2012 Garth Zeglin

// This is a task-level script node to calibrate the palm force
// sensor.  It operates the ABB arm with attached P3 hand to press
// the palm against an ATI mini-40 force sensor mounted on the arm
// table.

// This node handles collecting and recording data and
// coordinating the process.  The motion is handled by a second
// node to simplify scripting.

/****************************************************************/
// for getpid()
#include <sys/types.h>
#include <unistd.h>

// basic ROS API
#include <ros/ros.h>

// The force sensor uses a standard message type to report force
// data.  This has changed a bit with ROS.  Previously, this was here:
//   /opt/ros/diamondback/stacks/common_msgs/geometry_msgs/msg_gen/cpp/include/geometry_msgs/WrenchStamped.h
//
// Now see:
//   /opt/ros/fuerte/include/geometry_msgs/WrenchStamped.h
//   /opt/ros/fuerte/share/geometry_msgs/msg/WrenchStamped.msg

#include <geometry_msgs/WrenchStamped.h>

// The MLAB node API to drive the ABB arm and receive data from the server. This
// is a thin C++ wrapper over ROS publish, subscribe, and service calls,
// robot_node does the actual work.  This library is receiving these messages
// from robot_node: code/nodes/robot/ROS/robot_comm/msg/robot_CartesianLog.msg
#include <robot_comm/robot_comm.h>

// The MLAB node API to drive the P3 hand and receive data from the Arduino.
// This is a thin C++ wrapper over ROS publish, subscribe, and service calls,
// hand_node does the actual work.
#include <hand_comm/hand_comm.h>

// The MLAB math utilities
#include <matVec/matVec.h>

// Include the message declarations for this package.
//  topics:
#include <palm_calibration/progress.h>  // calibration motion progress reports

//  service definitions:
//  e.g. code/mains/palm_calibration/srv_gen/cpp/include/palm_calibration/ping.h
#include <palm_calibration/ping.h>
#include <palm_calibration/start.h>

/****************************************************************/
// Global state to be available to service callbacks.
static ros::ServiceClient calibrationMotionSrv;
static ros::ServiceClient gravityCheckMotionSrv;
static FILE *logfile = NULL;
static FILE *datafile = NULL;
static char *dataname = NULL;

/****************************************************************/
// Define a blackboard structure to hold the most recent state
// estimate.  This structure roughly corresponds to a single
// record and one line of the output stream file.

static struct calibrator_state_t {
  double t;                       // ROS clock time as determined by this node

  double force_t;                 // ROS clock time of most recent force data
  double fx, fy, fz, tx, ty, tz;  // force sensor reading

  double motion_t;                // ROS clock time of most recent movement generator data
  double dx, dy, dz;              // force probe position relative to palm
  double lat, lon;                // latitude and longitude for gravity check

  double palm_t;                  // ROS clock time of most recent palm data
  double f1, f2, f3;              // raw hand force sensor readings
  double f1s, f2s, f3s;           // filtered (smoothed) hand force sensor readings

  double arm_t;                   // ROS clock time of most recent Cartesian arm data
  double tcp[3];                  // TCP location
  double quat[4];                 // TCP orientation

  double hand_t;                  // ROS clock time of most recent hand data
  double q1, q2, q3;              // finger angles
  double motor;                   // motor angle

  // number of messages received from each source.
  int force_count;                // force sensor node
  int arm_count;                  // arm node
  int motion_count;               // motion generator node
  int hand_count;                 // hand node
  int palm_count;                 // palm data from hand node

  // calibration state machine indices
  int calibration_active;         // true while calibration process is underway
  int points_calibrated;          // number of calibration points processed this cycle
  int motion_phase;               // as reported in calibration progress messages

} state;

/****************************************************************/
// The output file is a ASCII matrix defined by this mapping.  This bit of metadata decouples the 
// internal structure from the log format, and allows putting a descriptive comment into the file.

static struct file_mapping_t {
  const char *name; 
  double *value;
} log_mapping[] = {
  { "t",       &state.t       },
  { "force_t", &state.force_t },
  { "fx",      &state.fx      },
  { "fy",      &state.fy      },
  { "fz",      &state.fz      },
  { "tx",      &state.tx      },
  { "ty",      &state.ty      },
  { "tz",      &state.tz      },
  { "motion_t",&state.motion_t},
  { "dx",      &state.dx      },
  { "dy",      &state.dy      },
  { "dz",      &state.dz      },
  { "palm_t",  &state.palm_t  },
  { "f1",      &state.f1      },
  { "f2",      &state.f2      },
  { "f3",      &state.f3      },
  { "f1s",     &state.f1s     },
  { "f2s",     &state.f2s     },
  { "f3s",     &state.f3s     },
  { "arm_t",   &state.arm_t   },
  { "tcp.x",   &state.tcp[0]  },  
  { "tcp.y",   &state.tcp[1]  },  
  { "tcp.z",   &state.tcp[2]  },  
  { "quat.0",  &state.quat[0] },  
  { "quat.x",  &state.quat[1] },  
  { "quat.y",  &state.quat[2] },  
  { "quat.z",  &state.quat[3] },  
  { "hand_t",  &state.hand_t  },
  { "q1",      &state.q1      },
  { "q2",      &state.q2      },
  { "q3",      &state.q3      },
  { "motor",   &state.motor   },
  { "lat",     &state.lat     },
  { "lon",     &state.lon     },
};

#define NUM_LOGVARS ((int)(sizeof(log_mapping)/sizeof(struct file_mapping_t)))

/****************************************************************/
// emit a comment line for the output file
static void emit_comment_line( FILE *stream )
{
  if (stream) {
    fprintf(stream, "# palm_calibration data.  file format from version: " __DATE__ " " __TIME__ "\n");
    fprintf(stream, "#");
    int i;
    for (i = 0; i < NUM_LOGVARS; i++) fprintf( stream, " %d:%s", i+1, log_mapping[i].name );
    fprintf(stream, "\n");
  }
}

// emit a data record for the output file
static void emit_data_record( FILE *stream )
{
  if (stream) {
    int i;
    for (i = 0; i < NUM_LOGVARS; i++) fprintf( stream, " %f", *(log_mapping[i].value) );
    fprintf(stream, "\n");
    fflush(stream);
  }
}
/****************************************************************/
static void print_status_report(void)
{
  ROS_INFO( "calib: palm %d force %d arm %d motion %d, palm force %d %d %d, calibration is at (%f %f %f)(%f %f) %d", 
	    state.palm_count, state.force_count, state.arm_count, state.motion_count,
	    (int)state.f1, (int)state.f2, (int)state.f3,
	    state.dx, state.dy, state.dz, 
	    state.lat, state.lon,
	    state.motion_phase );
}
/****************************************************************/
// Open a file for specific calibration samples used for
// computing the calibration function.
static void open_data_file( const char *basename )
{
  // Open a data file for calibration samples.
  if (asprintf( &dataname, "/tmp/%s_%s_%d_%d.data", 
		basename,
		getenv("USER"), 
		getpid(),
		(int)(floor( ros::Time::now().toSec()))  ) > 0 ) {
    ROS_INFO( "palm_calibration opening data file %s", dataname );
    datafile = fopen( dataname, "w" );
    emit_comment_line( datafile );
  }
}
static void close_data_file(void)
{
  if (datafile) fclose(datafile);
  if (dataname) free(dataname);
  datafile = NULL;
  dataname = NULL;
}

/****************************************************************/
static void state_updated(void) 
{
  // log all data if active
  if ( state.calibration_active ) {
    state.t = ros::Time::now().toSec();
    emit_data_record( logfile );
  }
}

/****************************************************************/
// Callback function for force data.  This data arrives
// frequently, so not every sample gets saved.  By default apply
// a basic low-pass filter.

static void update_force_filter( double *s, double input )
{
  *s += 0.25 * (input - *s);
}
static void forceDataCallback( const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  update_force_filter( &state.fx, msg->wrench.force.x );
  update_force_filter( &state.fy, msg->wrench.force.y );
  update_force_filter( &state.fz, msg->wrench.force.z );
  update_force_filter( &state.tx, msg->wrench.torque.x );
  update_force_filter( &state.ty, msg->wrench.torque.y );
  update_force_filter( &state.tz, msg->wrench.torque.z );

  state.force_t = msg->header.stamp.toSec();
  state.force_count++;

  // for now, don't trigger data logging on every single force point, it seems to be updating at about 100Hz.
  // state_updated();
}

/****************************************************************/
// Callback function for arm data.
static void arm_data_callback( const robot_comm::robot_CartesianLogConstPtr& msg )
{
  // string date       // appears to be parsed from server messages
  // string time       // appears to be parsed from server messages
  // float64 timeStamp // appears to be parsed from server messages
  // float64 x
  // float64 y
  // float64 z
  // float64 q0
  // float64 qx
  // float64 qy
  // float64 qz

  state.tcp[0] = msg->x;
  state.tcp[1] = msg->y;
  state.tcp[2] = msg->z;
  state.quat[0]   = msg->q0;
  state.quat[1]   = msg->qx;
  state.quat[2]   = msg->qy;
  state.quat[3]   = msg->qz;
  state.arm_t  = msg->timeStamp;
  state.arm_count++;

  state_updated();
  // printf("received arm data, z is %f\n", msg->z);
}

/****************************************************************/
// Hand callbacks

// code/nodes/hand/ROS/hand_comm/msg/hand_AnglesLog.msg
// float64 timeStamp      # ROS time-stamp
// float64 angleMotor     # Motor angle
// float64[] angle        # Finger angles


static void hand_data_callback( const hand_comm::hand_AnglesLog::ConstPtr& msg )
{
  state.hand_t = msg->timeStamp;
  state.q1 = (double) msg->angle[0];
  state.q2 = (double) msg->angle[1];
  state.q3 = (double) msg->angle[2];
  state.motor = (double) msg->angleMotor;
  state.hand_count++;
  state_updated();
}

// code/nodes/hand/ROS/hand_comm/msg/hand_EncodersLog.msg
// float64 timeStamp    # ROS time-stamp
// int64 encMotor       # Motor encoder
// int64[] encFinger    # Finger encoders

#if 0
static void hand_raw_data_callback( const hand_comm::hand_EncodersLog::ConstPtr& msg )
{
}
#endif

/****************************************************************/
// Receive and process raw palm force sensor data.

// hand_RawForcesLogConstPtr:
// float64 timeStamp      # ROS time-stamp
// int64[] forces        # Palm sensor values

static void update_palm_force_filter( double *s, double input )
{
  *s += 0.25 * (input - *s);
}

// Set the filter to the most recent values to reduce convergence time after a step input.
static void reset_palm_force_filter( void )
{
  state.f1s = state.f1;
  state.f2s = state.f2;
  state.f3s = state.f3;
}

static void hand_raw_forces_callback ( const hand_comm::hand_RawForcesLogConstPtr& msg)
{
  // save the current values
  state.palm_t = msg->timeStamp;
  state.f1 = (double) msg->forces[0];
  state.f2 = (double) msg->forces[1];
  state.f3 = (double) msg->forces[2];

  // update the filters
  update_palm_force_filter( &state.f1s, msg->forces[0] );
  update_palm_force_filter( &state.f2s, msg->forces[1] );
  update_palm_force_filter( &state.f3s, msg->forces[2] );

  state.palm_count++;
  state_updated();
}

/****************************************************************/
static void calibProgressCallback( const palm_calibration::progress::ConstPtr& msg )
{
  state.dx = msg->dx;
  state.dy = msg->dy;
  state.dz = msg->dz;
  state.lat = msg->latitude;
  state.lon = msg->longitude;
  state.motion_t = msg->timestamp;
  state.motion_count++;
  state.motion_phase = msg->phase;

  state_updated();

  if ( msg->phase ==  0) {
    // If phase is zero, the motion is not active, and has either
    // ended or not yet begun: there is a race condition, since a
    // zero can be received prior to the start of motion.

    if ( state.calibration_active && state.points_calibrated > 0) {
      state.calibration_active = 0;
      ROS_INFO("calibration cycle complete, data is in %s", dataname);
      close_data_file();
      // FIXME: should recompute the calibration matrix here and post it
    }

  } else {
    // else phase is non-zero, and the motion is mid-sequence

    if ( msg->phase == 2) {
      // begin sampling contact state 
      reset_palm_force_filter();

    } else if ( msg->phase == 3) {      
      // end sampling contact state

      // record one full sample of the calibration function here
      emit_data_record( datafile );

      state.points_calibrated++;
    }
  }
  print_status_report();
}

/****************************************************************/
// handle the ping service call
static bool ping( palm_calibration::ping::Request &request, 
		  palm_calibration::ping::Response &response )
{
  ROS_INFO("palm_calibrate/ping received");
  response.failure = 0;
  response.msg = "palm calibration service running";
  return true; // return true to indicate that response is valid
}

/****************************************************************/
// Check whether the data and services are available for calibration.
// Returns true with a numeric error code on failure. 
static int check_calibration_readiness(void)
{
  // This just checks that some data has been received from every
  // required source.  It could do more, i.e., check timestamps
  // on data, check hand pose to verify finger position, or other
  // sanity checks.

  int err = 0;
  if ( state.motion_count == 0 ) {
    ROS_ERROR("calibration_motion node doesn't appear to be running");
    err++;
  } 

  if ( state.force_count == 0 ) {
    ROS_ERROR("force sensor node doesn't appear to be running");
    err++;
  } 

  if ( state.arm_count == 0 ) {
    ROS_ERROR("arm node doesn't appear to be running");
    err++;
  } 

  if ( state.hand_count == 0 ) {
    ROS_ERROR("warning: hand node not sending angles data");
    // FIXME: this should eventually be a real error
  } else {
#if 0
    // check that the fingers are in the correct pose
    // FIXME: correct these values and comparisons
    if ( state.q1 > 1.0 || state.q2 > 1.0 || state.q3 > 1.0 ) {
      ROS_ERROR("fingers are not in a safe position for palm calibration.");
      err++;
    }
#endif
  }

  if (state.palm_count == 0 ) {
    ROS_ERROR("hand node doesn't appear to be running, no palm data arriving");
    err++;
  }

  return err;
}


/****************************************************************/
// handle the startCalibration service call
static bool startCalibration( palm_calibration::start::Request  &request, 
			      palm_calibration::start::Response &response )
{

  int err = check_calibration_readiness();
  if (err) {
    response.failure = err;
    response.msg = "conditions not met to start calibration";

  } else {
    palm_calibration::start startMotion;
    if ( calibrationMotionSrv.call(startMotion) ) {

      if ( startMotion.response.failure ) {
	response.failure = startMotion.response.failure;
	response.msg = startMotion.response.msg;
	ROS_ERROR("startCalibrationMotion reported error");

      } else {
	open_data_file("palm_calibration");
	state.calibration_active = 1;
	state.points_calibrated = 0;
	response.failure = 0;
	response.msg = "starting calibration process";
      }

    } else {
      ROS_ERROR("startCalibrationMotion service failed to respond");
      response.failure = 100;
      response.msg = "startCalibrationMotion service failed";
    }    
  }
  if (response.failure) ROS_INFO("WARNING: palm calibrator refusing to start calibration");
  else                  ROS_INFO("palm calibrator starting calibration");

  return true; // return true to indicate that response is valid
}

/****************************************************************/
// handle the startGravityCheck service call
static bool startGravityCheck( palm_calibration::start::Request  &request, 
			       palm_calibration::start::Response &response )
{

  int err = check_calibration_readiness();
  if (err) {
    response.failure = err;
    response.msg = "conditions not met to start calibration";

  } else {
    palm_calibration::start startMotion;
    if ( gravityCheckMotionSrv.call(startMotion) ) {

      if ( startMotion.response.failure ) {
	response.failure = startMotion.response.failure;
	response.msg = startMotion.response.msg;
	ROS_ERROR("startGravityCheckMotion reported error");

      } else {
	open_data_file("gravity_check");
	state.calibration_active = 1;
	state.points_calibrated = 0;
	response.failure = 0;
	response.msg = "starting gravityCheck process";
      }

    } else {
      ROS_ERROR("startGravityCheckMotion service failed to respond");
      response.failure = 100;
      response.msg = "startGravityCheckMotion service failed";
    }    
  }
  if (response.failure) ROS_INFO("WARNING: palm calibrator refusing to start gravity check");
  else                  ROS_INFO("palm calibrator starting gravity check");

  return true; // return true to indicate that response is valid
}

/****************************************************************/
int main(int argc, char** argv)
{
  // Start the ROS connection.
  ros::init(argc, argv, "palm_calibration");
  ros::NodeHandle node;

  // Note: the following sequence will succeed even if none of
  // the other nodes are running yet.

  // Connect to the force sensor node.
  ros::Subscriber forceDataSub = node.subscribe("/netft_data", 100, forceDataCallback );

  // Connect to the calibration progress reports.
  ros::Subscriber calibProgressSub = node.subscribe("/palm_calibration/progress", 100, calibProgressCallback );

  // Initialize the C++ API wrapper around the ABB arm node service calls.
  RobotComm robot(&node);

  // Initialize the C++ API wrapper around the P3 hand node service calls.
  HandComm hand(&node);

  // Subscribe to the robot Cartesian position data.
  robot.subscribeCartesian( &node, 100, arm_data_callback );

  // Subscribe to the hand raw finger and motor data.
  // hand.subscribeEncoders( &node, 100, hand_raw_data_callback );

  // Subscribe to the hand calibrated data.
  hand.subscribeAngles( &node, 100, hand_data_callback );

  // Subscribe to the palm force sensor raw data.
  hand.subscribeRawForces( &node, 100, hand_raw_forces_callback );

  // Initialize connection to the calibration motion service.
  calibrationMotionSrv  = node.serviceClient<palm_calibration::start>("palm_calibration/startCalibrationMotion");
  gravityCheckMotionSrv = node.serviceClient<palm_calibration::start>("palm_calibration/startGravityCheckMotion");

  // Advertise services.
  ros::ServiceServer calibrationService  = node.advertiseService( "palm_calibration/startCalibration",  startCalibration );
  ros::ServiceServer gravityCheckService = node.advertiseService( "palm_calibration/startGravityCheck", startGravityCheck );
  ros::ServiceServer pingService         = node.advertiseService( "palm_calibration/ping", ping );

  // Open a logfile for raw data.
  char *logname;
  if (asprintf( &logname, "/tmp/palm_calibration_%s_%d_%d.log", getenv("USER"), getpid(), (int)(floor( ros::Time::now().toSec())) ) > 0 ) {
    ROS_INFO( "palm_calibration opening log file %s", logname );
    logfile = fopen( logname, "w" );
    emit_comment_line( logfile );
    free( logname );
  }

  // Begin the event loop to process requests and data.
  ROS_INFO("Palm calibration control service entering event loop.  Issue a /palm_calibration/startCalibration service call to initiate action.");
  int rate = 20; // Hz
  static ros::Rate loop_rate(rate);
  while ( ros::ok() ) {
    ros::spinOnce();
    loop_rate.sleep();

    // check for timeouts
    if (state.calibration_active) {
      double now = ros::Time::now().toSec();
      if (( now - state.motion_t ) > 60.0 ) {
	ROS_ERROR("palm_calibration timeout, not receiving progress messages.");
	state.calibration_active = 0;
      }
    }
  }

  ROS_INFO("Palm calibration control service exiting event loop.");
  if (logfile) fclose(logfile);
  close_data_file();

  return 0;
}
/****************************************************************/
