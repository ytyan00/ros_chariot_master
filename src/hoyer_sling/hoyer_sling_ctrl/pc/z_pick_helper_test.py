import rospy
import numpy as np
import ros_numpy
import open3d as o3d
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

def get_distance():
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

        return cloud






    # Define the point picking function
    def pick_points(vis, pcd):
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
    def numpy_to_o3d_image(np_image, is_color=False):
        print(f"Data type of np_image: {np_image.dtype}")
        print(f"Shape of np_image: {np_image.shape}")

        if is_color:
            # Expecting a 3-channel image for color
            if len(np_image.shape) == 3 and np_image.shape[2] == 3:
                np_image = cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)


            else:
                print("Warning: Color image does not have 3 channels!")
        else:
            # Ensure the depth image is in the expected single-channel format
            if len(np_image.shape) != 2:
                print("Error: Depth image should be a single-channel image.")
                return None
            
            # Normalize the depth image to the 0-1 range if necessary
            if np_image.max() > 0:
                np_image = np.array(np_image,dtype=np.float32)
            else:
                print("Warning: Depth image contains only zero values.")
                return None

        try:
            print(np_image)
            return o3d.geometry.Image(np_image)
        except RuntimeError as e:
            print(f"Open3D Image initialization error: {e}")
            return None




    # Initialize ROS node
    # rospy.init_node('realsense_distance_node', anonymous=True)
    
    # Define ROS subscribers
    bridge = CvBridge()
    depth_image = None
    color_image = None
    depth_intrinsics = None

    def depth_callback(msg):
        nonlocal depth_image
        # Convert ROS Image to OpenCV image
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def color_callback(msg):
        nonlocal color_image
        # Convert ROS Image to OpenCV image
        color_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("color",color_image)


    def camera_info_callback(msg):
        nonlocal depth_intrinsics
        depth_intrinsics = msg

    rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_callback)
    rospy.Subscriber('/camera/color/image_raw', Image, color_callback)
    rospy.Subscriber('/camera/depth/camera_info', CameraInfo, camera_info_callback)

    # Create the visualization window once
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()

    while not rospy.is_shutdown():
        if depth_image is None or color_image is None or depth_intrinsics is None:
            continue
        
        # Display the depth image
        # cv2.imshow('Depth Image', depth_image)
        cv2.waitKey(1)  # Add a small delay to allow the image to refresh

        # Ensure depth image is compatible with Open3D
        depth_o3d = numpy_to_o3d_image(depth_image)
        color_o3d = numpy_to_o3d_image(color_image, is_color=True)

        # Get intrinsics from CameraInfo message
        fx = depth_intrinsics.K[0]
        fy = depth_intrinsics.K[4]
        ppx = depth_intrinsics.K[2]
        ppy = depth_intrinsics.K[5]

        o3d_intrinsics = o3d.camera.PinholeCameraIntrinsic(
            depth_intrinsics.width,
            depth_intrinsics.height,
            fx,
            fy,
            ppx,
            ppy
        )

        # Create an RGBD image from the color and depth images
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d,
            depth_o3d,
            depth_scale=1.0,  # Assuming depth is already in meters
            depth_trunc=5.0,  # Truncate depth values beyond 5 meters
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

        picked_points = pick_points(vis, pcd)
        
        # List to store the true depths of picked points
        world_coordinates = []
        
        # Convert picked 3D points to 2D pixel coordinates and calculate true depths
        if len(picked_points) > 0:
            for picked_point_index in picked_points:
                picked_point = np.asarray(pcd.points)[picked_point_index]

                # Calculate pixel coordinates from 3D point using camera intrinsics
                pixel_x = int((picked_point[0] / picked_point[2]) * fx + ppx)
                pixel_y = int((picked_point[1] / picked_point[2]) * fy + ppy)

                # Access the depth value at the pixel coordinates
                true_depth = depth_image[pixel_y, pixel_x] * depth_intrinsics.D[0]  # Use D[0] as the depth scale if available
                x = (pixel_x - ppx) * true_depth / fx
                y = (pixel_y - ppy) * true_depth / fy
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

    vis.destroy_window()

if __name__ == "__main__":
    try:
        get_distance()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()  # Ensure all OpenCV windows are closed when the script ends
