import rclpy
from rclpy.node import Node
from enum import Enum, auto

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import string

from paddleocr import PaddleOCR

from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Bool

class State(Enum):
    START = (auto(),)  # start ocr
    STOPPED = (auto(),)  # stop ocr

class Paddle_Ocr(Node):
    def __init__(self):
        super().__init__("paddle_ocr")

        # initialize paddleocr class
        self.paddle_ocr = PaddleOCR(lang='en', use_gpu= False) # need to run only once to download and load model into memory

        # initialize CvBridge
        self.cv_bridge = CvBridge()

        # create subscriber to set state
        self.game_state = self.create_subscription(Bool, "/ocr_run", self.game_state_callback, qos_profile=10)

        # create subscriber to get image
        self.cap_1 = self.create_subscription(Image, "modified_image_1", self.image_reader_1, qos_profile=10)
        self.cap_2 = self.create_subscription(Image, "modified_image_2", self.image_reader_2, qos_profile=10)

        # create publisher to publish guesses
        self.guess_publish = self.create_publisher(String, "user_input", 10)

        #declare and define parameters
        self.declare_parameter('ocr_frequency', 0.5)
        self.param_ocr_frequency = self.get_parameter('ocr_frequency').get_parameter_value().double_value
        self.declare_parameter('ocr_threshold', 0.50)
        self.param_ocr_threshold = self.get_parameter('ocr_threshold').get_parameter_value().double_value

        # create timer for calling the ocr function
        self.timer = self.create_timer(1.0/self.param_ocr_frequency, self.ocr_timer)

        # Specify the size and type of the empty image
        width, height = 640, 480
        channels = 3
        # Initialize an empty image with the specified size and type
        empty_image = np.zeros((height, width, channels), dtype=np.uint8)

        # initialize empty images to prevent node from crashing if
        # it stops receiving frames
        self.frame_1 = empty_image
        self.frame_2 = empty_image

        # initialize alphabet dictionary
        self.alphabet_dict = {letter: 0 for letter in string.ascii_uppercase}

        self.guess_tracker = [] # queue verified word guesses
        self.guess_pub_tracker = [] # track published guesses

        # define instance attributes
        self.state = State.STOPPED

    def game_state_callback(self, msg):
        if msg.data:
            self.state = State.START
        else:
            self.state = State.STOPPED

    def ocr_timer(self):
        """Call the ocr function"""
        if self.state == State.START:
            self.ocr_func_letter(self.frame_1)
            self.ocr_func_word(self.frame_2)

    def ocr_func_letter(self, frame):
        """Run OCR on the single letter image frame"""
        result = self.paddle_ocr.ocr(frame, cls=False, det=False, rec=True)
        if result[0] != None:
            self.guess_verification_letter(result)
        print(result)

    def ocr_func_word(self, frame):
        """Run OCR on the six-letter word image frame"""
        result = self.paddle_ocr.ocr(frame, cls=False, det=False, rec=True)
        if result[0] != None:
            self.guess_verification_word(result)
        # print(result)

    def guess_verification_letter(self, result):
        """Confirm whether the guess is a single letter"""
        try:
            # check if the guess is a single letter
            if all(char.isalpha() for char in result[0][0][0]) and len(result[0][0][0])== 1:
                if result[0][0][1] > self.param_ocr_threshold: # check confidence
                    self.guess_tracking_letter(result)
                    print(result[0][0][0])
            # catch exception for "O"
            elif result[0][0][0] == '0': 
                print('O')
                if result[0][0][1] > self.param_ocr_threshold: # check confidence
                    self.guess_tracking_letter([[('O', result[0][0][1])]])
        except:
            pass
    def guess_verification_word(self, result):
        """Confirm whether the guess is a six-letter word"""
        try:
            # check if the guess is a single letter
            if all(char.isalpha() for char in result[0][0][0]) and len(result[0][0][0])== 5:
                if result[0][0][1] > 0.75: # check confidence
                    guess = result[0][0][0].upper()
                    self.guess_tracking_word(guess)
                    print(guess)
        except:
            pass

    def guess_tracking_letter(self, result):
        """Keep track of the previous guesses to confirm accuracy"""
        # add the guess confidence value to the alphabet dictionary
        self.alphabet_dict[result[0][0][0].upper()] += result[0][0][1]
        print(self.alphabet_dict[result[0][0][0].upper()])
        # pass the guess to the publisher if value exeeds threshold
        if self.alphabet_dict[result[0][0][0].upper()] > 1.6:
            self.guess_publisher(result[0][0][0].upper())
            # reinitialize alphabet dictionary
            self.alphabet_dict = {letter: 0 for letter in string.ascii_uppercase}
    
    def guess_tracking_word(self, guess):
        """Keep track of the previous guesses to confirm accuracy"""
        self.guess_tracker.append(guess)
        if len(self.guess_tracker) > 3:
            del self.guess_tracker[0]
        if len(self.guess_tracker) == 3:
            # check if 3 consecutive guesses are same
            if self.guess_tracker[0] == self.guess_tracker[1] and self.guess_tracker[1] == self.guess_tracker[2]:
                self.guess_tracker = []
                self.guess_publisher(guess)

    def guess_publisher(self, guess):
        """Publish the verified guess"""
        current_guess = String()
        publish = True
        # check if the guess has already been published
        if len(self.guess_pub_tracker) > 0:
            for item in self.guess_pub_tracker:
                if item == guess:
                    publish = False
        if publish:
            self.guess_pub_tracker.append(guess)
            current_guess.data = guess
            self.get_logger().info(f"Registering Guess: {guess}")
            self.guess_publish.publish(current_guess)
 
    def image_reader_1(self, msg):
        """Convert image to opencv format"""
        self.frame_1 = self.cv_bridge.imgmsg_to_cv2(msg)
        # cv2.imshow("read_image_1", self.frame_1)
        cv2.waitKey(1)
    
    def image_reader_2(self, msg):
        """Convert image to opencv format"""
        self.frame_2 = self.cv_bridge.imgmsg_to_cv2(msg)
        # cv2.imshow("read_image_2", self.frame_2)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Paddle_Ocr()
    rclpy.spin(node)
    rclpy.shutdown()