import open3d as o3d
import rospy
import time
import ros_numpy
from PyQt5 import QtWidgets
from sensor_msgs.msg import PointCloud2

import lib_cloud_conversion_between_Open3D_and_ROS as ros_o3d

class CameraListener():
    def __init__(self):
        self.pc = None
        self.n = 0
        self.listener()

    def callback(self, points):
        self.pc = points
        self.n = self.n + 1
        # rospy.loginfo('new point')

    def listener(self):
        rospy.init_node('ros_open3d_visualizer', anonymous=True)
        rospy.Subscriber('/cam_1/depth/color/points', PointCloud2, self.callback)

class Viewer(QtWidgets.QWidget):
    def __init__(self, subscriber, parent=None):
        self.subscriber = subscriber
        rospy.loginfo('Initialization')

        self.vis = o3d.visualization.VisualizerWithEditing()
        self.vis.create_window("aaa")
        self.point_cloud = None

        # # Create a ViewControl object
        # self.view_control = self.vis.get_view_control()
        ctr = self.vis.get_view_control()
        print("Field of view (before changing) %.2f" % ctr.get_field_of_view())

        # # Set the camera position and orientation
        # self.view_control.set_lookat([0, 0, 0])  # Target point (where the camera is looking at)
        # self.view_control.set_up([0, 1, 0])     # Up direction
        # self.view_control.set_front([-1, 0, 0])  # Front direction

        # # You can also set other camera parameters, such as field of view (fov) and zoom
        # self.view_control.set_zoom(0.8)  # Adjust the zoom level

        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])

        # Add the coordinate frame to the Visualizer
        # self.vis.add_geometry(coordinate_frame)

        self.updater()

    def updater(self):
        rospy.loginfo('Start')
        while self.subscriber.pc is None:
            time.sleep(2)
        
        self.point_cloud = ros_o3d.convertCloudFromRosToOpen3d(self.subscriber.pc)
        self.point_cloud.voxel_down_sample(voxel_size=0.1)
        # self.point_cloud.farthest_point_down_sample(5000).to_legacy()
        self.vis.add_geometry(self.point_cloud)
        # self.vis.update_geometry(self.point_cloud)
        # o3d.visualization.draw_geometries_with_editing([self.point_cloud])
        # o3d.visualization.draw_geometries_with_vertex_selection([self.point_cloud],'aaa')
        
        while not rospy.is_shutdown():
            rospy.loginfo('new point')
            old = self.point_cloud
            self.point_cloud = ros_o3d.convertCloudFromRosToOpen3d(self.subscriber.pc)
            self.point_cloud.voxel_down_sample(voxel_size=0.1)
            # self.point_cloud.farthest_point_down_sample(5000).to_legacy()
            # o3d.visualization.draw_geometries_with_editing([self.point_cloud])
            # o3d.visualization.draw_geometries_with_vertex_selection([self.point_cloud],'aaa')
            self.vis.remove_geometry(old)
            self.vis.add_geometry(self.point_cloud)
            # self.vis.update_geometry(self.point_cloud)
            self.vis.poll_events()
            self.vis.update_renderer()
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)

if __name__ == '__main__':
    # rospy.init_node('ros_open3d_visualizer')
    app = QtWidgets.QApplication([])
    listener = CameraListener()
    updater = Viewer(listener)
    app.exec_()
