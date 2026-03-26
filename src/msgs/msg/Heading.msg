# heading: ROS convention (0=East, CCW, radians). Converted from compass bearing,
#          but does not account for magnetic declination.
# compass_bearing: raw magnetometer reading (0=magnetic North, CW, degrees).

std_msgs/Header header
float64 heading
float64 heading_acc
float64 compass_bearing
