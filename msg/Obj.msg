# This is a message to hold data for objects found during segmentation

Header header

int32 object_id                            # Each objects gets different number.
bool is_supporting_surface                 # Can be a table or floor or an object on it.  
geometry_msgs/Pose pose                    # Pose represented in world frame.
float64[3] scale                           # Size in x, y and z direction in their orientation.
sensor_msgs/PointCloud2 cloud              # Point Normal of the object (position & normals).
shape_msgs/Mesh mesh                       # Shape Mesh of the object.
float32[] objectness