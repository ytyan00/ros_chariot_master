#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import ros_numpy
import numpy as np

def pointcloud2_to_open3d(pc2_msg):
    # Convert ROS PointCloud2 message to numpy array
    pc_data = ros_numpy.point_cloud2.pointcloud2_to_array(pc2_msg)
    
    # Extract points (x, y, z)
    points = np.array([[point[0], point[1], point[2]] for point in pc_data])

    # Extract color data
    if 'rgb' in pc_data.dtype.names:
        rgb_data = pc_data['rgb']
        
        # Convert RGB float32 to int and then to separate R, G, B values
        rgb_data = np.asarray(rgb_data, dtype=np.float32).view(np.uint32)
        r = ((rgb_data >> 16) & 255) / 255.0
        g = ((rgb_data >> 8) & 255) / 255.0
        b = (rgb_data & 255) / 255.0
        colors = np.stack([r, g, b], axis=-1)
    else:
        # Default to a color (white) if no RGB data is available
        colors = np.ones_like(points)

    # Create Open3D point cloud object
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.colors = o3d.utility.Vector3dVector(colors)
    #     # Flip it, otherwise the pointcloud will be upside down
    # cloud.transform([[1, 0, 0, 0],
    #                 [0, -1, 0, 0],
    #                 [0, 0, -1, 0],
    #                 [0, 0, 0, 1]])

    return cloud

def callback(pc2_msg):
    # Convert PointCloud2 to Open3D PointCloud
    print("Received data")
    cloud = pointcloud2_to_open3d(pc2_msg)
    
    if cloud is None or len(cloud.points) == 0:
        print("No data received")
    else:
        # Visualization or processing with Open3D
        # vis = o3d.visualization.Visualizer()
        vis = o3d.visualization.VisualizerWithEditing()

        vis.create_window()
        vis.add_geometry(cloud)
        
        # Set rendering options to use points
        render_option = vis.get_render_option()
        render_option.point_size = 1.0  # Set point size as needed, e.g., 1.0 for small points
        render_option.point_show_normal = False  # Make sure normal vectors are not displayed

        vis.run()
        vis.destroy_window()

if __name__ == "__main__":
    rospy.init_node('pointcloud2_to_open3d')
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback)
    print("Started")
    rospy.spin()
