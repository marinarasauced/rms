# PointCloudStamped.msg
# This message contains a PointCloud2 and metadata for its file storage location.

# The pointcloud data in ROS2 PointCloud2 format.
sensor_msgs/PointCloud2 pointcloud

# The relative file path where the pointcloud data is stored, typically a directory path.
# Example: "~/scans/trial/"
string file_path    # The directory path relative to the home directory (e.g., ~/ -> scans/trial/)

# The name of the file containing the pointcloud data.
# Example: "file_name.pcd"
string file_name    # The file name within the directory (e.g., scans/trial/ -> file_name)
