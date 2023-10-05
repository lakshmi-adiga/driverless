import rclpy
from rclpy.node import Node
import torch
import numpy as np
import cv2
import open3d as o3d
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from stereo.predict import predict
from stereo.ZED import ZEDSDK
import stereo.visualization as lidar_vis

from eufs_msgs.msg import ConeArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, PointCloud2

from cv_bridge import CvBridge
import ros2_numpy as rnp

def np2points(cones):
    '''convert list of cones into a Point[] object'''
    arr = []
    for i in range(len(cones)):
        p = Point()
        p.x = float(cones[i][0])
        p.y = float(cones[i][1])
        p.z = float(cones[i][2])

        arr.append(p)
    return arr

RELIABLE_QOS_PROFILE = QoSProfile(reliability = QoSReliabilityPolicy.RELIABLE,
                         history = QoSHistoryPolicy.KEEP_LAST,
                         durability = QoSDurabilityPolicy.VOLATILE,
                         depth = 5)

BEST_EFFORT_QOS_PROFILE = QoSProfile(reliability = QoSReliabilityPolicy.BEST_EFFORT,
                         history = QoSHistoryPolicy.KEEP_LAST,
                         durability = QoSDurabilityPolicy.VOLATILE,
                         depth = 5)
STEREO_OUT = '/stereo_cones'
IMAGE_LEFT_OUT = '/zedsdk_left_color_image'
IMAGE_RIGHT_OUT = '/zedsdk_right_color_image'
DEPTH_OUT = '/zedsdk_depth_image'
POINT_OUT = '/zedsdk_point_cloud_image'

def detectCones(image):
    overallStart = time.time_ns()
    start = time.time_ns()
    # image = cv2.imread('cones.jpeg')
    # print("Read Image: " + str((time.time_ns() - start) / 1000000))
    # start = time.time_ns()
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    b_img = img_hsv[len(img_hsv)//2:]
    # h, s, v = cv2.split(img_hsv)
    # img_hsv = cv2.merge([h, s // 2, v])
    print("Convert Image: " + str((time.time_ns() - start) / 1000000))
    start = time.time_ns()
    # convert the hsv to a binary image by removing any pixels
    # that do not fall within the following HSV Min/Max values
    # REALmask = cv2.inRange(img_hsv, (20,100,0), (120, 255, 255))
    blue_mask = cv2.inRange(b_img, (90,100,50), (135, 255, 255))
    black_mask = cv2.inRange(img_hsv, (179, 255, 255), (179, 255, 255))
    print(blue_mask)
    # bh, bs, bv = cv2.split(blue_mask)
    yellow_mask = cv2.inRange(b_img, (10,100,100), (40, 255, 255))
    # yh, ys, yv = cv2.split(yellow_mask)
    blue_mask = cv2.vconcat([(black_mask[:len(black_mask)//2]), blue_mask])
    yellow_mask = cv2.vconcat([(black_mask[:len(black_mask) // 2]), yellow_mask])
    actual_mask = yellow_mask + blue_mask

    # cone_threshold = cv2.inRange(img_hsv, (22, 210, 205), (22, 255, 255))

    # cone_threshold = cv2.inRange(img_hsv, (0, 0, 0), (255, 100, 100))

    # cone_threshold = cv2.inRange(img_hsv, (100, 150, 50), (120, 200, 250))

    # find contours in the new binary image
    # cone_contours_yellow, _ = cv2.findContours(yellow_mask,
    # cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cone_contours, _ = cv2.findContours(actual_mask,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    print("Masking Image: " + str((time.time_ns() - start) / 1000000))
    start = time.time_ns()

    coneSize = 0

    # cone_contours_yellow = sorted(cone_contours_yellow, key = cv2.contourArea, reverse=True)
    cone_contours = sorted(cone_contours, key = cv2.contourArea, reverse=True)
    print("Sorting Contours: " + str((time.time_ns() - start) / 1000000))
    start = time.time_ns()
    # if contours have been detected, draw them
    coneRange = 0
    if len(cone_contours) >= 20: coneRange = 20
    if len(cone_contours) < 20: coneRange = len(cone_contours)
    for i in range(coneRange):
        # cv2.drawContours(image, cone_contours_yellow, -1, 255, 2)
        cv2.drawContours(image, cone_contours, -1, 255, 2)
        # record the largest contour
        # coneContour = max(cone_contours, key=cv2.contourArea)
        # coneContour_yellow = cone_contours_yellow[i]
        coneContour_blue = cone_contours[i]

        # get the unrotated bounding box that surrounds the contour
        # x,y,w,h = cv2.boundingRect(coneContour_yellow)
        # coneSize = w*h
        # # draw the unrotated bounding box
        # cv2.rectangle(image,(x,y),(x+w,y+h),(0,0,0),20)
        x,y,w,h = cv2.boundingRect(coneContour_blue)
        # # draw the unrotated bounding box
        cv2.rectangle(image,(x,y),(x+w,y+h),(0,0,0),20)
    print("Drawing Bounding Boxes: " + str((time.time_ns() - start) / 1000000))
    start = time.time_ns()


        # # record some custom data to send back to the robot

    shape = image.shape
    img_resize = cv2.resize(image, (int(shape[1] * 0.6), int(shape[0] * 0.6)))
    cv2.imshow("Saket Is Genius?????????????????????", img_resize)
    # cv2.imshow("blue??", actual_mask)
    print("Showing Image: " + str((time.time_ns() - start) / 1000000))
    start = time.time_ns()
    print("Final Image: " + str((time.time_ns() - overallStart) / 1000000))
    #return the largest contour for the LL crosshair, the modified image, and custom robot data


class StereoCamera(Node):

    def __init__(self):
        super().__init__('stereo_predictor')
        frame_rate = 20
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='src/stereo/stereo/model_params.pt')
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = self.model.to(self.device)

        self.prediction_publisher = self.create_publisher(msg_type=ConeArray,
                                                          topic=STEREO_OUT,
                                                          qos_profile=RELIABLE_QOS_PROFILE)
        
        self.left_publisher = self.create_publisher(msg_type=Image,
                                                     topic=IMAGE_LEFT_OUT,
                                                     qos_profile=BEST_EFFORT_QOS_PROFILE)
        self.right_publisher = self.create_publisher(msg_type=Image,
                                                     topic=IMAGE_RIGHT_OUT,
                                                     qos_profile=BEST_EFFORT_QOS_PROFILE)
        self.depth_publisher = self.create_publisher(msg_type=Image,
                                                     topic=DEPTH_OUT,
                                                     qos_profile=BEST_EFFORT_QOS_PROFILE)
        self.point_publisher = self.create_publisher(msg_type=Image,
                                                     topic=POINT_OUT,
                                                     qos_profile=BEST_EFFORT_QOS_PROFILE)

        
        self.left_subscriber = self.create_subscription(Image,
                                                        IMAGE_LEFT_OUT,
                                                        self.left_callback,
                                                        qos_profile=BEST_EFFORT_QOS_PROFILE)
        self.point_subscriber = self.create_subscription(Image,
                                                         POINT_OUT,
                                                         self.point_callback,
                                                         qos_profile=BEST_EFFORT_QOS_PROFILE)
        self.lidar_subscriber = self.create_subscription(PointCloud2,
                                                         "/lidar_points",
                                                         self.lidar_callback,
                                                         qos_profile=BEST_EFFORT_QOS_PROFILE)
        
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.bridge = CvBridge()

        self.recent_left = None
        self.recent_point = None
        self.iter = 0

        self.lidar_win = lidar_vis.init_visualizer_window()


        print(f"model-device: {self.device}")
        print("done-init-node")

    def lidar_callback(self, msg):
        lidar_arr = rnp.numpify(msg)

        arr = np.zeros((lidar_arr.shape[0], 3))
        arr[:,0] = lidar_arr['x']
        arr[:,1] = lidar_arr['y']
        arr[:,2] = lidar_arr['z']

        # self.lidar_win = None
        lidar_vis.update_visualizer_window(self.lidar_win, arr)

    def left_callback(self, msg):
        print("got new left image")
        self.recent_left = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    
    def point_callback(self, msg):
        self.recent_point = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC4")

    def timer_callback(self):
        # try displaying the image
        print("yellow")
        if self.recent_left is None or self.recent_point is None:
            print("urmom")
            return

        s = time.time()

        # left, right, depth, point = self.zed.grab_data()
        left = self.recent_left
        point = self.recent_point

        left_copy = np.zeros(left.shape, dtype=left.dtype)
        left_copy += left

        # detectCones(left_copy)

        blue_cones, yellow_cones, orange_cones = predict(self.model, left, point)

        # convert the data and check that it is the same going and backwards
        # have to extract out nan values that don't count to compare image values
        # left_enc = self.bridge.cv2_to_imgmsg(left, encoding="passthrough")
        # right_enc = self.bridge.cv2_to_imgmsg(right, encoding="passthrough")
        # depth_enc = self.bridge.cv2_to_imgmsg(depth, encoding="passthrough")
        # point_enc = self.bridge.cv2_to_imgmsg(point,encoding="32FC4")

        # publish the data
        # self.left_publisher.publish(left_enc)
        # self.right_publisher.publish(right_enc)
        # self.depth_publisher.publish(depth_enc)
        # self.point_publisher.publish(point_enc)

        # left_unenc = self.bridge.imgmsg_to_cv2(left_enc, desired_encoding="passthrough")
        # point_unenc = self.bridge.imgmsg_to_cv2(point_enc, desired_encoding="32FC4")
        # depth_unenc = self.bridge.imgmsg_to_cv2(depth_enc, desired_encoding="passthrough")

        # result = []
        # for i in range(len(blue_cones)):
        #     c = blue_cones[i]
        #     result.append([c[0], c[1], c[2]])
        # print(np.array(result))

        cone_msg = ConeArray()
        cone_msg.blue_cones = np2points(blue_cones)
        cone_msg.yellow_cones = np2points(yellow_cones)
        cone_msg.orange_cones = np2points(orange_cones)
        self.prediction_publisher.publish(cone_msg)

        t = time.time()
        print(f"Stereo {self.iter:>4}: {1000 * (t - s):.3f}ms")
        self.iter += 1

    def inference(self):
        # try displaying the image

        s = time.time()

        left, right, depth, point = self.zed.grab_data()
        blue_cones, yellow_cones, orange_cones = predict(self.model, left, point)

        # convert the data and check that it is the same going and backwards
        # have to extract out nan values that don't count to compare image values
        left_enc = self.bridge.cv2_to_imgmsg(left, encoding="passthrough")
        right_enc = self.bridge.cv2_to_imgmsg(right, encoding="passthrough")
        depth_enc = self.bridge.cv2_to_imgmsg(depth, encoding="passthrough")
        point_enc = self.bridge.cv2_to_imgmsg(point,encoding="32FC4")

        # publish the data
        # self.left_publisher.publish(left_enc)
        # self.right_publisher.publish(right_enc)
        # self.depth_publisher.publish(depth_enc)
        # self.point_publisher.publish(point_enc)


        left_unenc = self.bridge.imgmsg_to_cv2(left_enc, desired_encoding="passthrough")
        point_unenc = self.bridge.imgmsg_to_cv2(point_enc, desired_encoding="32FC4")
        depth_unenc = self.bridge.imgmsg_to_cv2(depth_enc, desired_encoding="passthrough")

        result = []
        for i in range(len(blue_cones)):
            c = blue_cones[i]
            result.append([c[0], c[1], c[2]])
        print(np.array(result))

        cone_msg = ConeArray()
        cone_msg.blue_cones = np2points(blue_cones)
        cone_msg.yellow_cones = np2points(yellow_cones)
        cone_msg.orange_cones = np2points(orange_cones)
        self.prediction_publisher.publish(cone_msg)

        t = time.time()
        print(f"Stereo: {1000 * (t - s):.3f}ms")

        

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = StereoCamera()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
