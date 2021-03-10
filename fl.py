import rospy
import sys, time
import numpy as np
import cv_bridge
from geometry_msgs.msg import Twist
import roslib
from sensor_msgs.msg import CompressedImage
import os
from imageai.Detection import ObjectDetection
import cv2


class Follow():

    def __init__(self):
        # Init node image_view_detect
        rospy.init_node("img_detect_following")
        os.system("clear")
        execution_path = os.getcwd()
        detector = ObjectDetection()
        detector.setModelTypeAsYOLOv3()
        custom_objects = detector.CustomObjects(person=True)
        detector.setModelPath(os.path.join(execution_path, "yolo.h5"))
        detector.loadModel(detection_speed="fastest")

        # publish into cmd vel
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.callback)
        self.vel = Twist()
        self.rate = rospy.Rate(10)

        self.img_root_size = [640, 480]
        self.frame = 1

        self.input_image_path = "./webCamImage.jpg"
        self.output_image_path = "./outputImage.jpg"
        self.min_prob = 20

        # time sleep ps
        self.rate_sleep = rospy.Rate(10)
        self.cv_bridge = cv_bridge.CvBridge()

        while not rospy.is_shutdown():
            # check, frame = vs.read()
            # img = msg.data
            # np_arr = np.fromstring(img, np.uint8)
            # img_np = cv2.imdecode(np_arr, 1)

            # cv2.imwrite(input_image_path, frame)
            cv2.imwrite(self.input_image_path, self.frame)
            detections = detector.detectCustomObjectsFromImage(custom_objects=custom_objects,
                                                                    input_image=os.path.join(execution_path,
                                                                                             self.input_image_path),
                                                                    output_image_path=os.path.join(execution_path,
                                                                                                   self.output_image_path),
                                                                    minimum_percentage_probability=self.min_prob)
            print("DA", detections)
            if len(detections) != 0:
                x1 = detections[0]["box_points"][0]
                y1 = detections[0]["box_points"][1]
                x2 = detections[0]["box_points"][2]
                y2 = detections[0]["box_points"][3]
                # print("(", x1, ", ", y1, ")")
                # print("(", x2, ", ", y2, ")")
                image_child = [x1, y1, x2, y2]
                self.listenAndMove(self.img_root_size, x1,y1,x2,y2)
                print("--------------------------------")
            else:
                print("Not found any person")
                self.stop()
                print("--------------------------------")

            img = cv2.imread(self.output_image_path)
            cv2.imshow('frame', img)
            # self.rate_sleep.sleep()
            key = cv2.waitKey(1)
            if key == ord("q"):
                self.stop()
                break
            self.rate.sleep()

        self.stop()

    def callback(self, data):
        # Handle Data
        #  self.frame = self.cv_bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
        img = data.data
        np_arr = np.fromstring(img, np.uint8)
        self.frame = cv2.imdecode(np_arr, 1)
        # cv2.imshow('cv_img', img_np)

    def listenAndMove(self, img_root_size, x1,y1,x2,y2):

        person_center = (x1 + x2) / 2
        if person_center > (img_root_size[0] / 2 + 40):
            print("go Right")
            self.vel.angular.z = -0.5
        elif person_center < (img_root_size[0] / 2 - 40):
            print("go Left")
            self.vel.angular.z = 0.5
        else:
            self.vel.angular.z = 0.0

        if y2 > img_root_size[1] or (x2 - x1) * (y2 - y1) > img_root_size[0]*img_root_size[1]*0.5:
            print("Stop")
            self.vel.linear.x = 0.0
        else:
            self.vel.linear.x = 0.5

        self.pub.publish(self.vel)

    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.pub.publish(self.vel)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Follow()

    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated")

# Caculator vector by img
