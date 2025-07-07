import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import cv2

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

# Create an Open3D Visualizer instance
vis = o3d.visualization.Visualizer()
vis.create_window()
pcd = o3d.geometry.PointCloud()

# Update loop
try:
    while True:
        # Capture frames from the RealSense device
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

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
            depth_scale=1.0 / depth_frame.get_units(),
            depth_trunc=3.0,
            convert_rgb_to_intensity=False
        )

        # Generate the point cloud
        temp_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            o3d_intrinsics
        )

        # Flip it, otherwise the pointcloud will be upside down
        temp_pcd.transform([[1, 0, 0, 0],
                            [0, -1, 0, 0],
                            [0, 0, -1, 0],
                            [0, 0, 0, 1]])

        # Update the point cloud
        pcd.points = temp_pcd.points
        pcd.colors = temp_pcd.colors

        # Update the visualizer
        vis.add_geometry(pcd)
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

except KeyboardInterrupt:
    # Stop the pipeline when interrupted
    pipeline.stop()
    vis.destroy_window()
