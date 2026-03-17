# Heading in ROS convention: 0 = East, increases counter-clockwise, radians.
# Note: this differs from compass bearing (0 = North, clockwise, degrees).
# The driver converts from compass bearing via: heading = (90 - bearing_deg) * pi / 180

std_msgs/Header header
float64 heading
float64 heading_acc
