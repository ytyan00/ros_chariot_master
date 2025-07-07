#!/usr/bin/env python3
import rospy
import numpy as np
import ros_numpy
import open3d as o3d
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge


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

    # Define the point picking function
    def pick_points(self,pcd):
        print("Press [shift] + left mouse button to pick a point.")
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window()
        vis.add_geometry(pcd)
        # Set rendering options to use points
        render_option = vis.get_render_option()
        render_option.point_size = 1.0  # Set point size as needed, e.g., 1.0 for small points
        render_option.point_show_normal = False  # Make sure normal vectors are not displayed
        vis.run()  # user picks points
        vis.destroy_window()
        picked_points = vis.get_picked_points()
        if len(picked_points) > 0:
            print(f"Selected points: {picked_points}")
            print(f"Coordinates of the first point: {np.asarray(pcd.points)[picked_points[0]]}")
        else:
            print("No points picked.")
        return picked_points
    
    def run(self):
        while not rospy.is_shutdown():
            if not self.show:
                continue
            picked_points = self.pick_points(self.cloud)
            # Get the depth scale from the RealSense camera
            # depth_scale = depth_frame.get_units()
            # List to store the true depths of picked points
            world_coordinates = []
            # Convert picked 3D points to 2D pixel coordinates and calculate true depths
            if len(picked_points) > 0:
                for picked_point_index in picked_points:
                    picked_point = np.asarray(self.cloud.points)[picked_point_index]

                    # Calculate pixel coordinates from 3D point using camera intrinsics
                    #pixel_x = int((picked_point[0] / picked_point[2]) * depth_intrinsics.fx + depth_intrinsics.ppx)
                    #pixel_y = int((picked_point[1] / picked_point[2]) * depth_intrinsics.fy + depth_intrinsics.ppy)

                    # Access the depth value at the pixel coordinates
                    #true_depth = depth_frame.get_distance(pixel_x, pixel_y)
                    x= 1 #= (pixel_x - depth_intrinsics.ppx) * true_depth / depth_intrinsics.fx
                    y = 1#= (pixel_y - depth_intrinsics.ppy) * true_depth / depth_intrinsics.fy
                    z =1# = true_depth
                    if x != 0 and y != 0 and z != 0:
                        world_coordinates.append((x,y,z))
                        print(f"x y z  at picked point {picked_point_index}: {x, y, z} meters")

                # Calculate the average depth
                if len(world_coordinates) > 0:        
                    average_coordinates = np.mean(world_coordinates, axis=0)  
                    print(f"Average depth of picked points: {average_coordinates} meters")
                    print("points number", len(world_coordinates))  
                    # z forward
                    # x leftward
                    # y up and down, dont know and dont care
                    return average_coordinates

if __name__ == "__main__":
    rospy.init_node('get_pcd2_target')
    camera_pointer = pcd2_target(print_log=True)
    camera_pointer.run()





# cloud = None
# show = None

# def get_distance():
#     global show, cloud
#     def pointcloud2_to_open3d(pc2_msg):
#         print("making open3d")
#         # Convert ROS PointCloud2 message to numpy array
#         pc_data = ros_numpy.point_cloud2.pointcloud2_to_array(pc2_msg)
        
#         # Extract points (x, y, z)
#         points = np.array([[point[0], point[1], point[2]] for point in pc_data])
#         print("finish Extract points (x, y, z)")
#         # Extract color data
#         if 'rgb' in pc_data.dtype.names:
#             rgb_data = pc_data['rgb']
            
#             # Convert RGB float32 to int and then to separate R, G, B values
#             rgb_data = np.asarray(rgb_data, dtype=np.float32).view(np.uint32)
#             r = ((rgb_data >> 16) & 255) / 255.0
#             g = ((rgb_data >> 8) & 255) / 255.0
#             b = (rgb_data & 255) / 255.0
#             colors = np.stack([r, g, b], axis=-1)
#         else:
#             # Default to a color (white) if no RGB data is available
#             colors = np.ones_like(points)
#         print("finish to Extract color data")
#         # Create Open3D point cloud object
#         cloud = o3d.geometry.PointCloud()
#         print("cloud init")
#         cloud.points = o3d.utility.Vector3dVector(points)
#         print("points inserted")
#         cloud.colors = o3d.utility.Vector3dVector(colors)
#         print("ready to return")
#         return cloud

#     def pointcloud2_callback(pc2_msg):
#         global cloud, show
#         # Convert PointCloud2 to Open3D PointCloud
#         print("Received data")
#         cloud = pointcloud2_to_open3d(pc2_msg)
        
#         if cloud is None or len(cloud.points) == 0:
#             print("No data received")
#         else:
#             show = True

#         print("here")
            
#     # Define the point picking function
#     def pick_points(pcd):
#         print("Press [shift] + left mouse button to pick a point.")
#         vis = o3d.visualization.VisualizerWithEditing()
#         vis.create_window()
#         vis.add_geometry(pcd)
#         # Set rendering options to use points
#         render_option = vis.get_render_option()
#         render_option.point_size = 1.0  # Set point size as needed, e.g., 1.0 for small points
#         render_option.point_show_normal = False  # Make sure normal vectors are not displayed
#         vis.run()  # user picks points
#         vis.destroy_window()
#         picked_points = vis.get_picked_points()
#         if len(picked_points) > 0:
#             print(f"Selected points: {picked_points}")
#             print(f"Coordinates of the first point: {np.asarray(pcd.points)[picked_points[0]]}")
#         else:
#             print("No points picked.")
#         return picked_points
    




#     # Initialize ROS node
#     # rospy.init_node('realsense_distance_node', anonymous=True)
    
#     # Define ROS subscribers

#     rospy.Subscriber("/camera/depth/color/points", PointCloud2, pointcloud2_callback)

#     while not rospy.is_shutdown():
#         if not show:
#             continue
#         # # Flip it, otherwise the pointcloud will be upside down
#         # cloud.transform([[1, 0, 0, 0],
#         #                [0, -1, 0, 0],
#         #                [0, 0, -1, 0],
#         #                [0, 0, 0, 1]])
#         # print("here")

#         picked_points = pick_points(cloud)
#         # Get the depth scale from the RealSense camera
#         # depth_scale = depth_frame.get_units()
#         # List to store the true depths of picked points
#         world_coordinates = []
#         # Convert picked 3D points to 2D pixel coordinates and calculate true depths
#         if len(picked_points) > 0:
#             for picked_point_index in picked_points:
#                 picked_point = np.asarray(cloud.points)[picked_point_index]

#                 # Calculate pixel coordinates from 3D point using camera intrinsics
#                 #pixel_x = int((picked_point[0] / picked_point[2]) * depth_intrinsics.fx + depth_intrinsics.ppx)
#                 #pixel_y = int((picked_point[1] / picked_point[2]) * depth_intrinsics.fy + depth_intrinsics.ppy)

#                 # Access the depth value at the pixel coordinates
#                 #true_depth = depth_frame.get_distance(pixel_x, pixel_y)
#                 x= 1 #= (pixel_x - depth_intrinsics.ppx) * true_depth / depth_intrinsics.fx
#                 y = 1#= (pixel_y - depth_intrinsics.ppy) * true_depth / depth_intrinsics.fy
#                 z =1# = true_depth
#                 if x != 0 and y != 0 and z != 0:
#                     world_coordinates.append((x,y,z))
#                     print(f"x y z  at picked point {picked_point_index}: {x, y, z} meters")

#             # Calculate the average depth
#             if len(world_coordinates) > 0:        
#                 average_coordinates = np.mean(world_coordinates, axis=0)  
#                 print(f"Average depth of picked points: {average_coordinates} meters")
#                 print("points number", len(world_coordinates))  
#                 # z forward
#                 # x leftward
#                 # y up and down, dont know and dont care
#                 return average_coordinates

# if __name__ == "__main__":
#     try:
#         get_distance()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         cv2.destroyAllWindows()  # Ensure all OpenCV windows are closed when the script ends
