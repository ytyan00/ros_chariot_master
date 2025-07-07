import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import cv2

def get_distance():
    # Define the point picking function
    def pick_points(pcd):
        print("Press [shift] + left mouse button to pick a point.")
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window()
        vis.add_geometry(pcd)
        vis.run()  # user picks points
        vis.destroy_window()
        picked_points = vis.get_picked_points()
        if len(picked_points) > 0:
            print(f"Selected points: {picked_points}")
            print(f"Coordinates of the first point: {np.asarray(pcd.points)[picked_points[0]]}")
        else:
            print("No points picked.")
        return picked_points
    # Helper function to convert images to Open3D images
    def numpy_to_o3d_image(np_image, is_color=False):
        if is_color:
            np_image = cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)
        return o3d.geometry.Image(np_image)

    # Start RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    # Get the intrinsic parameters of the camera
    profile = pipeline.get_active_profile()
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    depth_intrinsics = depth_profile.get_intrinsics()
    o3d_intrinsics = o3d.camera.PinholeCameraIntrinsic(
        depth_intrinsics.width,
        depth_intrinsics.height,
        depth_intrinsics.fx,
        depth_intrinsics.fy,
        depth_intrinsics.ppx,
        depth_intrinsics.ppy
    )
    
    while True:
        
        # Capture frames
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        print("capture")

        # Convert frames to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Convert numpy arrays to Open3D images
        depth_o3d = numpy_to_o3d_image(depth_image)
        color_o3d = numpy_to_o3d_image(color_image, is_color=True)

        # Create an RGBD image from the color and depth images
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d,
            depth_o3d,
            depth_scale=1.0 / depth_frame.get_units(),  # Use the depth scale from the RealSense camera
            depth_trunc=5.0,  # Truncate depth values beyond 3 meters
            convert_rgb_to_intensity=False
        )

        # Create a point cloud from the RGBD image and the camera intrinsics
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            o3d_intrinsics
        )

        # Flip it, otherwise the pointcloud will be upside down
        pcd.transform([[1, 0, 0, 0],
                    [0, -1, 0, 0],
                    [0, 0, -1, 0],
                    [0, 0, 0, 1]])

        picked_points = pick_points(pcd)
        # Get the depth scale from the RealSense camera
        depth_scale = depth_frame.get_units()
        # List to store the true depths of picked points
        picked_depths = []
        picked_x = []
        picked_y = []
        picked_z = []
        pixel_xs = []
        pixel_ys = []
        world_coordinates = []
        # Convert picked 3D points to 2D pixel coordinates and calculate true depths
        if len(picked_points) > 0:
            for picked_point_index in picked_points:
                picked_point = np.asarray(pcd.points)[picked_point_index]

                # Calculate pixel coordinates from 3D point using camera intrinsics
                pixel_x = int((picked_point[0] / picked_point[2]) * depth_intrinsics.fx + depth_intrinsics.ppx)
                pixel_y = int((picked_point[1] / picked_point[2]) * depth_intrinsics.fy + depth_intrinsics.ppy)

                # Access the depth value at the pixel coordinates
                true_depth = depth_frame.get_distance(pixel_x, pixel_y)
                x = (pixel_x - depth_intrinsics.ppx) * true_depth / depth_intrinsics.fx
                y = (pixel_y - depth_intrinsics.ppy) * true_depth / depth_intrinsics.fy
                z = true_depth
                if x != 0 and y != 0 and z != 0:
                    world_coordinates.append((x,y,z))
                    print(f"x y z  at picked point {picked_point_index}: {x, y, z} meters")

                # pixel_xs.append(pixel_x)
                # pixel_ys.append(pixel_y)
                
                # # Add the true depth to the list                # average_depth = np.mean(picked_depths)
                # print(f"Average depth of picked points: {average_depth} meters")
                # print(picked_x)
                # print(picked_y)
                # print(picked_z)
                # print(picked_depths)
                # print(pixel_xs)
                # print(pixel_ys)        
                # picked_depths.append(true_depth)
                # picked_z.append(picked_points[2])
                # picked_x.append(picked_points[0])
                # picked_y.append(picked_points[1])

                

            # Calculate the average depth
            if len(world_coordinates) > 0:
                # average_depth = np.mean(picked_depths)
                # print(f"Average depth of picked points: {average_depth} meters")
                # print(picked_x)
                # print(picked_y)
                # print(picked_z)
                # print(picked_depths)
                # print(pixel_xs)
                # print(pixel_ys)            
                average_coordinates = np.mean(world_coordinates, axis=0)  
                print(f"Average depth of picked points: {average_coordinates} meters")
                print("points number", len(world_coordinates))  
                # z forward
                # x leftward
                # y up and down, dont know and dont care
                return average_coordinates
                



