import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy 

from sensor_msgs.msg import Image, PointCloud2

from cv_bridge import CvBridge
import ros2_numpy as rnp

import cv2
import numpy as np

BEST_EFFORT_QOS_PROFILE = QoSProfile(reliability = QoSReliabilityPolicy.BEST_EFFORT,
                         history = QoSHistoryPolicy.KEEP_LAST,
                         durability = QoSDurabilityPolicy.VOLATILE,
                         depth = 5)

class DataCollectionNode(Node):
    
    def __init__(self):
        super().__init__("data_collection")

        self.bridge = CvBridge()

        self.left_color_subscriber = self.create_subscription(Image, '/zedsdk_left_color_image', self.left_color_callback, qos_profile=BEST_EFFORT_QOS_PROFILE)
        self.right_color_subscriber = self.create_subscription(Image, '/zedsdk_right_color_image', self.right_color_callback, qos_profile=BEST_EFFORT_QOS_PROFILE)
        self.point_subscriber = self.create_subscription(PointCloud2, '/velodyne_points', self.pointcloud_callback, qos_profile=BEST_EFFORT_QOS_PROFILE)

        self.collect_timer = self.create_timer(1, self.collect_callback)
        self.collection_instance = 0

        self.most_recent_left_color_img = None
        self.most_recent_right_color_img = None
        self.most_recent_pointcloud = None

    def collect_callback(self):
        left_exists = self.most_recent_left_color_img is not None
        right_exists = self.most_recent_right_color_img is not None
        point_exists = self.most_recent_pointcloud is not None
        if not (left_exists and right_exists and point_exists):
            return

        data_dict = {
            "left": self.most_recent_left_color_img,
            "right": self.most_recent_right_color_img,
            "point": self.most_recent_pointcloud,
        }

        # import pdb; pdb.set_trace()
        
        # import os
        # os.mkdir("~/Documents/np-data")
        print(f"{self.collection_instance}")
        np.savez(f"/home/chip/Documents/driverless/driverless_ws/tmp/instance-{self.collection_instance}.npz", **data_dict)
        # np.savez(f"~/Desktop/np-data/instance-{self.collection_instance}.npz", **data_dict)
        
        self.collection_instance += 1

    def left_color_callback(self, msg):
        img_arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        self.most_recent_left_color_img = img_arr

        cv2.imshow("left", img_arr)
        cv2.waitKey(1)

        return
    
    def right_color_callback(self, msg):
        img_arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        self.most_recent_right_color_img = img_arr

        cv2.imshow("right", img_arr)
        cv2.waitKey(1)

        return

    def pointcloud_callback(self, msg):
        # import pdb; pdb.set_trace()
        points = rnp.numpify(msg)

        points_arr = np.zeros((points.shape[0] * points.shape[1],6))

        # import pdb; pdb.set_trace()
        points_arr[:,0] = points['x'].reshape(-1)
        points_arr[:,1] = points['y'].reshape(-1)
        points_arr[:,2] = points['z'].reshape(-1)
        points_arr[:,3] = points['intensity'].reshape(-1)
        points_arr[:,4] = points['ring'].reshape(-1)
        points_arr[:,5] = points['time'].reshape(-1)

        self.most_recent_pointcloud = points_arr

        # import pdb; pdb.set_trace()

        return


def main(args=None):
    rclpy.init(args=args)

    collection_node = DataCollectionNode()

    rclpy.spin(collection_node)

    collection_node.destroy_node()
    rclpy.shutdown()

    pass

if __name__ == "__main__":
    main()