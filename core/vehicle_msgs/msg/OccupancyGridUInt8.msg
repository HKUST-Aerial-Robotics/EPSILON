Header header

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# The map resolution [m/cell]
float32 resolution

# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
uint8[] data
