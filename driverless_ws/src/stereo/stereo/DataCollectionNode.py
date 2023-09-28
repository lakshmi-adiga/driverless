import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy 

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

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

        self.collect_timer = self.create_timer(0.1, self.collect_callback)
        self.collection_instance = 0

        self.most_recent_left_color_img = None
        self.most_recent_right_color_img = None

    def collect_callback(self):

        data_dict = {
            "left": self.most_recent_left_color_img,
            "right": self.most_recent_right_color_img,
        }
        
        # import os
        # os.mkdir("~/Documents/np-data")
        print(f"{self.collection_instance}")
        # np.savez(f"~/Desktop/np-data/instance-{self.collection_instance}.npz", **data_dict)
        
        self.collection_instance += 1

    def left_color_callback(self, msg):
        img_arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        self.most_recent_left_color_img = img_arr

        cv2.imshow("left", img_arr)
        cv2.waitKey(1)
    
    def right_color_callback(self, msg):
        img_arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        self.most_recent_right_color_img = img_arr

        cv2.imshow("right", img_arr)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    collection_node = DataCollectionNode()

    rclpy.spin(collection_node)

    collection_node.destroy_node()
    rclpy.shutdown()

    pass

if __name__ == "__main__":
    main()