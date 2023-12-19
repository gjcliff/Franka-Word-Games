import rclpy
from rclpy.node import Node
from enum import Enum, auto

import numpy as np
import imutils
from imutils.perspective import four_point_transform

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import Bool

class State(Enum):
    START = (auto(),)  # start image modification
    STOPPED = (auto(),)  # stop image modification

def nothing(x):
    """Callback for trackbars"""
    pass

def kernel(x):
    """Callback for kernel trackbar"""
    if x % 2 == 0:
        x = x + 1 # ensure that the value is odd
    cv2.setTrackbarPos('Kernel', 'Parameters', x)

def kernel_cropped(x):
    """Callback for kernel_cropped trackbar"""
    if x % 2 == 0:
        x = x + 1 # ensure that the value is odd
    cv2.setTrackbarPos('Kernel_Cropped', 'Parameters', x)

class ImageModification(Node):
    def __init__(self):
        super().__init__("image_modification")

        # initialize CvBridge
        self.cv_bridge = CvBridge()

        # create subscriber to set state
        self.game_state = self.create_subscription(Bool, "/ocr_run", self.game_state_callback, qos_profile=10)
        # create subscriber to get image
        self.cap = self.create_subscription(Image, "camera/color/image_raw", self.image_modification, qos_profile=10)

        # create publisher to publish modified image
        self.modified_image_1_publish = self.create_publisher(Image, "modified_image_1", 10)
        self.modified_image_2_publish = self.create_publisher(Image, "modified_image_2", 10)

        # create trackbars to tune cv parameters
        cv2.namedWindow('Parameters')
        cv2.createTrackbar('Canny_T_min', 'Parameters', 0, 255, nothing)
        cv2.createTrackbar('Canny_T_max', 'Parameters', 0, 255, nothing)
        cv2.createTrackbar('Kernel', 'Parameters', 1, 31, kernel)
        cv2.createTrackbar('Kernel_Cropped', 'Parameters', 1, 31, kernel_cropped)
        cv2.createTrackbar('Dilate_Kernel', 'Parameters', 1, 30, nothing)

        # set default trackbar positions
        cv2.setTrackbarPos('Kernel', 'Parameters', 5)
        cv2.setTrackbarPos('Kernel_Cropped', 'Parameters', 5)
        cv2.setTrackbarPos('Dilate_Kernel', 'Parameters', 2)
        cv2.setTrackbarPos('Canny_T_min', 'Parameters', 50)
        cv2.setTrackbarPos('Canny_T_max', 'Parameters', 150)

        # define instance attributes
        self.state = State.STOPPED

    def game_state_callback(self, msg):
        if msg.data:
            self.state = State.START
            # self.get_logger().info("Starting")
        else:
            self.state = State.STOPPED
            # self.get_logger().info("Stopping")
            cv2.destroyWindow('Recognition')
            cv2.destroyWindow('image')

    def image_modification(self, msg):
        """Pre-process the image for OCR"""
        if self.state == State.START:
            # convert image to opencv format
            self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

            # fetch trackbar positions for tuning
            c_min = cv2.getTrackbarPos('Canny_T_min', 'Parameters')
            c_max = cv2.getTrackbarPos('Canny_T_max', 'Parameters')
            k_size = cv2.getTrackbarPos('Kernel', 'Parameters')
            k_size_2 = cv2.getTrackbarPos('Kernel_Cropped', 'Parameters')
            d_k_size = cv2.getTrackbarPos('Dilate_Kernel', 'Parameters')

            # resize the image
            resized_image = imutils.resize(self.frame, height=500)
            # cv2.imshow("resized_image", resized_image)

            # convert image to grayscale
            gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
            # cv2.imshow("gray", gray)

            # blur image
            blurred = cv2.GaussianBlur(gray, (k_size, k_size), 0)
            # cv2.imshow("blurred", blurred)

            # edge detection
            edged = cv2.Canny(blurred, c_min, c_max)
            # cv2.imshow("edged", edged)

            # find contours in the edge map, then sort them by their
            # size in descending order
            cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
            displayCnt = None

            # loop over the contours
            for c in cnts:
                # approximate the contour
                peri = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.01 * peri, True)
                # if the contour has four vertices identify it as the whiteboard
                if len(approx) == 4:
                    displayCnt = approx
                    cv2.drawContours(resized_image, [displayCnt], 0, (0, 255, 0), 2)
                    break

            # display captured frame with drawn contour
            cv2.imshow("image", resized_image)

            # extract the bounded whiteboard region and apply a perspective transform
            try:
                # apply 4 point transform on the contour in the grayscale image
                warped = four_point_transform(gray, displayCnt.reshape(4, 2))
                # cv2.imshow("warped", warped)

                # crop the borders of the image to remove inconsistencies
                height,width = warped.shape
                border = int(0.05*min(height,width))
                x1 = border
                y1 = border
                x2 = width - border
                y2 = height - border
                cropped = warped[y1:y2,x1:x2]

                # blur the cropped image
                cropped = cv2.GaussianBlur(cropped, (k_size_2, k_size_2), 0)

                # inv binarise the blurred image
                bin_thresh = cv2.getTrackbarPos('Bin_Thresh', 'Parameters')
                # ret3, binarised = cv2.threshold(cropped, bin_thresh, 255, cv2.THRESH_BINARY_INV)
                # ret3,binarised = cv2.threshold(warped,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
                binarised = cv2.adaptiveThreshold(cropped,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,7,2)
                # cv2.imshow("binarised", binarised)

                # dilate the image to widen letter strokes
                kernel = np.ones((d_k_size, d_k_size),np.uint8)
                dilation = cv2.dilate(binarised,kernel,iterations = 1)

                # invert the inv dilated image
                inverted_image = cv2.bitwise_not(dilation)
                # cv2.imshow("Letter_Recognition", inverted_image)
                
                # invert the inv binarised image
                binary_image = cv2.bitwise_not(binarised)
                # cv2.imshow("Word_Recognition", binary_image)

                # Create a named window that alllows resizing
                cv2.namedWindow('Recognition', cv2.WINDOW_NORMAL)

                # Resize the window to the specified height and width
                cv2.resizeWindow('Recognition', 400, 290)

                # display modified image
                # cv2.imshow("Recognition", imutils.resize(binary_image, height=200))
                cv2.imshow("Recognition", binary_image)

                # convert images to msg format and publish
                img_publish_1 = self.cv_bridge.cv2_to_imgmsg(binary_image)
                self.modified_image_1_publish.publish(img_publish_1)
                img_publish_2 = self.cv_bridge.cv2_to_imgmsg(inverted_image)
                self.modified_image_2_publish.publish(img_publish_2)

            except:
                pass

            cv2.waitKey(30)


def main(args=None):
    rclpy.init(args=args)
    node = ImageModification()
    rclpy.spin(node)
    rclpy.shutdown()