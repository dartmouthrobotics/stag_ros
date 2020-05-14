Header header
uint32 id
float32 reprojection_error # sum of the squared reprojection error of each corner
geometry_msgs/PoseStamped pose # pose from solvePnP
geometry_msgs/Point[] corners # corners from marker detection in image coordinates
