# Freespace
Header header
uint32 width
uint32 height
float32 resolution
geometry_msgs/Pose origin
# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
uint8[] data