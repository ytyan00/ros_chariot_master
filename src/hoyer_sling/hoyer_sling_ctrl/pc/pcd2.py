#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import ros_numpy
import numpy as np

class pcd2_target():
    def __init__(self,print_log=False) -> None:
        self.print_log = print_log
        self.cloud = None
        self.show = False # flag for valid pcd2 callback and to display
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.pointcloud2_callback)
        print("[INFO] pcd2_target() init")

    def pointcloud2_to_open3d(self,pc2_msg):
        print("making open3d")
        # Convert ROS PointCloud2 message to numpy array
        pc_data = ros_numpy.point_cloud2.pointcloud2_to_array(pc2_msg)
        
        # Extract points (x, y, z)
        points = np.array([[point[0], point[1], point[2]] for point in pc_data])
        print("finish Extract points (x, y, z)")
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
        print("finish to Extract color data")
        # Create Open3D point cloud object
        cloud = o3d.geometry.PointCloud()
        print("cloud init")
        cloud.points = o3d.utility.Vector3dVector(points)
        print("points inserted")
        cloud.colors = o3d.utility.Vector3dVector(colors)
        print("ready to return")
        return cloud
    
    def pointcloud2_callback(self,pc2_msg):
        # Convert PointCloud2 to Open3D PointCloud
        print("[INFO] Data Revieved")
        self.cloud = self.pointcloud2_to_open3d(pc2_msg)
        
        if self.cloud is None or len(self.cloud.points) == 0:
            print("No data received")
        else:
            self.show = True
            vis = o3d.visualization.VisualizerWithEditing()

            vis.create_window()
            vis.add_geometry(self.cloud)
            
            # Set rendering options to use points
            render_option = vis.get_render_option()
            render_option.point_size = 1.0  # Set point size as needed, e.g., 1.0 for small points
            render_option.point_show_normal = False  # Make sure normal vectors are not displayed

            vis.run()
            vis.destroy_window()

    
if __name__ == "__main__":
    rospy.init_node('get_pcd2_target')
    camera_pointer = pcd2_target(print_log=True)