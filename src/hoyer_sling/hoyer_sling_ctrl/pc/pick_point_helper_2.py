import numpy as np
import open3d as o3d
import cv2


class pick_point_gui():
    def __init__(self) -> None:
        # Create the visualization window once
        self.vis = o3d.visualization.VisualizerWithEditing()
        self.vis.create_window()
        # camera images init
        self.depth_image = None
        self.color_image = None
        self.depth_intrinsics = None


    # Helper function to convert images to Open3D images
    def numpy_to_o3d_image(self,np_image, is_color=False):
        if is_color:
            np_image = cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)
        return o3d.geometry.Image(np_image)

    # update frame
    def update_frame(self,depth_image,color_image,depth_intrinsics):
        # Convert ROS Image to numpy arrays
        self.depth_o3d = self.numpy_to_o3d_image(depth_image)
        self.color_o3d = self.numpy_to_o3d_image(color_image, is_color=True)
        self.depth_image = depth_image
        self.color_image = color_image
        self.depth_intrinsics = depth_intrinsics

    # Get the intrinsic parameters of the camera
    def get_camera_intrinsic_para(self):
        self.fx = self.depth_intrinsics.K[0]
        self.fy = self.depth_intrinsics.K[4]
        self.ppx = self.depth_intrinsics.K[2]
        self.ppy = self.depth_intrinsics.K[5]

        self.o3d_intrinsics = o3d.camera.PinholeCameraIntrinsic(
            self.depth_intrinsics.width,
            self.depth_intrinsics.height,
            self.fx,
            self.fy,
            self.ppx,
            self.ppy
        )

    #  ----- get distance functions with helpers -----
    # Define the point picking function
    def pick_points(self, vis, pcd):
        vis.clear_geometries()
        vis.add_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        vis.run()  # user picks points
        picked_points = vis.get_picked_points()
        if len(picked_points) > 0:
            print(f"Selected points: {picked_points}")
            print(f"Coordinates of the first point: {np.asarray(pcd.points)[picked_points[0]]}")
        else:
            print("No points picked.")
        return picked_points
    # Helper function to convert images to Open3D images
    def numpy_to_o3d_image(self, np_image, is_color=False):
        if is_color:
            np_image = cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)
        return o3d.geometry.Image(np_image)

    def get_distance(self):
        if self.depth_image is None or self.color_image is None or self.depth_intrinsics is None:
            return None
        # Create an RGBD image from the color and depth images
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            self.color_o3d,
            self.depth_o3d,
            depth_scale=1.0,  # Assuming depth is already in meters
            depth_trunc=5.0,  # Truncate depth values beyond 5 meters
            convert_rgb_to_intensity=False
        )

        # Create a point cloud from the RGBD image and the camera intrinsics
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            self.o3d_intrinsics
        )

        # Flip it, otherwise the pointcloud will be upside down
        pcd.transform([[1, 0, 0, 0],
                       [0, -1, 0, 0],
                       [0, 0, -1, 0],
                       [0, 0, 0, 1]])
        

        picked_points = self.pick_points(self.vis, pcd)
        
        # List to store the true depths of picked points
        world_coordinates = []
        
        # Convert picked 3D points to 2D pixel coordinates and calculate true depths
        if len(picked_points) > 0:
            for picked_point_index in picked_points:
                picked_point = np.asarray(pcd.points)[picked_point_index]

                # Calculate pixel coordinates from 3D point using camera intrinsics
                pixel_x = int((picked_point[0] / picked_point[2]) * self.fx + self.ppx)
                pixel_y = int((picked_point[1] / picked_point[2]) * self.fy + self.ppy)

                # Access the depth value at the pixel coordinates
                true_depth = self.depth_image[pixel_y, pixel_x] * self.depth_intrinsics.D[0]  # Use D[0] as the depth scale if available
                x = (pixel_x - self.ppx) * true_depth / self.fx
                y = (pixel_y - self.ppy) * true_depth / self.fy
                z = true_depth
                if x != 0 and y != 0 and z != 0:
                    world_coordinates.append((x, y, z))
                    print(f"x y z  at picked point {picked_point_index}: {x, y, z} meters")

            # Calculate the average depth
            if len(world_coordinates) > 0:
                average_coordinates = np.mean(world_coordinates, axis=0)
                print(f"Average depth of picked points: {average_coordinates} meters")
                print("points number", len(world_coordinates))
                return average_coordinates

        self.vis.destroy_window()
