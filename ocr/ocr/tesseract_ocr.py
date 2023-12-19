import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import pytesseract

from sensor_msgs.msg import Image
from std_msgs.msg import String

class Tesseract_Ocr(Node):
    def __init__(self):
        super().__init__("tesseract_ocr")

        # initialize CvBridge
        self.cv_bridge = CvBridge()

        # create subscriber to get image
        self.cap = self.create_subscription(Image, "modified_image", self.image_reader, qos_profile=10)

        #declare and define parameters
        self.declare_parameter('ocr_frequency', 1)
        self.param_ocr_frequency = self.get_parameter('ocr_frequency').get_parameter_value().integer_value

        # create timer for calling the ocr function
        self.timer = self.create_timer(1.0/self.param_ocr_frequency, self.ocr_timer)

    def ocr_timer(self):
        """Call the ocr function"""
        self.ocr_func(self.frame)

    def ocr_func(self, frame):
        """Run OCR on the image frame"""
        config = ("--oem 3 --psm 10")
        result = pytesseract.image_to_string(frame,'eng',config=config)
        # if result[0] != None:
        #     self.guess_verification(result)
        print(result)

    def image_reader(self, msg):
        """Convert image to opencv format"""
        self.frame = self.cv_bridge.imgmsg_to_cv2(msg)
        cv2.imshow("read_image", self.frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Tesseract_Ocr()
    rclpy.spin(node)
    rclpy.shutdown()