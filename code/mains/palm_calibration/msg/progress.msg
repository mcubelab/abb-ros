# Define a basic report on the calibration movement the locations
# are reported relative the nominal center contact position of
# the palm against the force sensor

# The timestamp is the value returned by ros::Time::now().toSec().

# The phase is an index of the state of the current touch sequence.
float64 timestamp
float64 dx
float64 dy
float64 dz
int32 phase

# the latitude and longitude are specified during the gravity
# check rotation motions and specify the direction of the gravity
# vector relative to the palm (where Z is the north pole)
float64 latitude
float64 longitude