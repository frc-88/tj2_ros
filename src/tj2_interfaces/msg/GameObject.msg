# object label
string label

# object label index
int16 class_index

# object confidence (0..1)
float32 confidence

# object 3D pose
geometry_msgs/Pose pose

# 2D bounding box in pixel space
tj2_interfaces/UVBoundingBox bounding_box_2d

# 3D bounding box in camera world frame
tj2_interfaces/XYZBoundingBox bounding_box_3d

# 3D dimensions (width, height, lenght)
float32[3] dimensions_3d
