string id
int32 timestamp
string meta_data

geometry_msgs/Pose pose # centroid in map co-ordinates
sensor_msgs/PointCloud2 map_cloud   #cloud in map co-ordinates
sensor_msgs/PointCloud2 camera_cloud # cloud in camera co-ordinates
sensor_msgs/PointCloud2 room_cloud  # cloud aligned to meta-room

sensor_msgs/Image rgb_cropped
sensor_msgs/Image depth_cropped
sensor_msgs/Image rgb_masked
sensor_msgs/Image depth_masked
