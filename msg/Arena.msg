# An Arena defines a volume of space with all features useful for global planning

std_msgs/Header header

# The arena volume. z_min is always 0
float64 x_min
float64 x_max
float64 y_min
float64 y_max
float64 z_max

# Each drone has a landing pad
geometry_msgs/Point[] landing_pads

# Location of ArUco markers
flock_vlam_msgs/Map map