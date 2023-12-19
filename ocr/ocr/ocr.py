import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from paddleocr import PaddleOCR

from sensor_msgs.msg import Image
from std_msgs.msg import String


class Ocr(Node):
    def __init__(self):
        super().__init__("ocr")

        # initialize paddleocr class
        # need to run only once to download and load model into memory
        self.ocr = PaddleOCR(lang='en', use_gpu=False)
        # initialize CvBridge
        self.cv_bridge = CvBridge()

        # create publisher to publish guesses
        self.guess_publish = self.create_publisher(String, "user_input", 10)

        # create subscriber to get image
        self.cap = self.create_subscription(
            Image, "camera/color/image_raw", self.image_modification, qos_profile=10)

        # declare and define parameters
        self.declare_parameter('ocr_frequency', 1)
        self.param_ocr_frequency = self.get_parameter(
            'ocr_frequency').get_parameter_value().integer_value
        self.declare_parameter('ocr_threshold', 0.85)
        self.param_ocr_threshold = self.get_parameter(
            'ocr_threshold').get_parameter_value().double_value

        # define instance attributes
        self.frame = None
        self.guess_tracker = []  # queue verified guesses
        self.guess_pub_tracker = []  # track published guesses

        # create timer for calling the ocr function
        self.timer = self.create_timer(
            1.0/self.param_ocr_frequency, self.ocr_timer)

    def ocr_timer(self):
        """Call the ocr function"""
        self.ocr_func(self.frame)

    def ocr_func(self, frame):
        """Run OCR on the image frame"""
        result = self.ocr.ocr(frame, cls=False)
        if result[0] != None:
            self.guess_verification(result)
        print(result)

    def guess_verification(self, result):
        """Confirm whether the guess is a single letter or 6L word"""
        try:
            # check if the guess is a single letter according to instructions
            # check for single characters
            if result[0][0][1][0][-2] == ':' and result[0][0][1][0][-1].isalpha():
                if result[0][0][1][1] > self.param_ocr_threshold:  # check confidence
                    print(result[0][0][1][0][-1])
                    self.guess_tracking(result[0][0][1][0][-1])
            # catch exception for "O"
            elif result[0][0][1][0][-2] == ':' and result[0][0][1][0][-1] == '0':
                if result[0][0][1][1] > self.param_ocr_threshold:  # check confidence
                    self.guess_tracking('O')
            # check if guess is a 6L word with letters only
            elif len(result[0][0][1][0]) == 6 and result[0][0][1][0].isalpha():
                if result[0][0][1][1] > self.param_ocr_threshold:  # check confidence
                    self.guess_tracking(result[0][0][1][0])
        except:
            pass

    def guess_tracking(self, guess):
        """Keep track of the previous guesses to confirm accuracy"""
        self.guess_tracker.append(guess)
        if len(self.guess_tracker) > 3:
            del self.guess_tracker[0]
        if len(self.guess_tracker) == 3:
            if self.guess_tracker[0] == self.guess_tracker[1] and self.guess_tracker[1] == self.guess_tracker[2]:
                self.get_logger().info(f"Registering Guess: {guess}")
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
            self.guess_publish.publish(current_guess)

    def image_modification(self, msg):
        """Convert image to opencv format"""
        self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("image", self.frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = Ocr()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
