import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_srvs.srv import Empty
from std_msgs.msg import Bool, String
from matplotlib.font_manager import FontProperties
from matplotlib.textpath import TextToPath
# from brain_interfaces.msg import Cartesian
from brain_interfaces.srv import BoardTiles, MovePose, Cartesian, Box
from brain_interfaces.msg import LetterMsg
# from character_interfaces.alphabet import alphabet
from geometry_msgs.msg import Pose, Point, Quaternion

from enum import Enum, auto
import numpy as np


class State(Enum):
    INITIALIZE = auto(),
    CALIBRATE = auto(),
    APPROACHING = auto(),
    WAITING = auto(),
    WRITING = auto(),
    LETTER = auto()


class Brain(Node):

    def __init__(self):
        super().__init__("brain")

        self.timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.create_timer(0.01, self.timer_callback, self.timer_callback_group)

        # create publishers
        self.moveit_mp_pub = self.create_publisher(
            Pose, '/moveit_mp', 10)

        self.state_pub = self.create_publisher(
            String, '/brain_states', 10)  # maybe use this to publish the states

        self.ocr_pub = self.create_publisher(
            Bool, '/ocr_run', 10)

        # Callback groups
        self.cal_callback_group = MutuallyExclusiveCallbackGroup()
        self.tile_callback_group = MutuallyExclusiveCallbackGroup()
        self.mp_callback_group = MutuallyExclusiveCallbackGroup()
        self.cartesian_callback_group = MutuallyExclusiveCallbackGroup()
        self.kick_callback_group = MutuallyExclusiveCallbackGroup()
        self.make_board_callback_group = MutuallyExclusiveCallbackGroup()

        # Create clients
        self.board_service_client = self.create_client(
            BoardTiles, '/where_to_write', callback_group=self.tile_callback_group)  # create custom service type
        self.calibrate_service_client = self.create_client(
            Empty, 'calibrate', callback_group=self.cal_callback_group)  # create custom service type
        self.movepose_service_client = self.create_client(
            MovePose, '/moveit_mp', callback_group=self.mp_callback_group)  # create custom service type
        self.cartesian_mp_service_client = self.create_client(
            Cartesian, '/cartesian_mp', callback_group=self.cartesian_callback_group)  # create custom service type
        self.kickstart_service_client = self.create_client(
            Empty, '/kickstart_service', callback_group=self.kick_callback_group)
        # self.make_board_client = self.create_client(
        #     Box, '/make_board', callback_group=self.make_board_callback_group)

        while not self.calibrate_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Calibrate service not available, waiting...')
        while not self.board_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Where to Write service not available, waiting...')
        while not self.movepose_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Move It MP service not available, waiting...')
        while not self.cartesian_mp_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Carisiam mp  service not available, waiting...')
        while not self.kickstart_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Kickstart  service not available, waiting...')

        # Create subscription from hangman.py
        self.hangman = self.create_subscription(
            LetterMsg, '/writer', callback=self.hangman_callback, qos_profile=10)
        # self.home = self.create_subscription(
        #     Bool, '/RTH', callback=self.home_callback, qos_profile=10)
        # self.trajectory_status = self.create_subscription(
        #     String, '/execute_trajectory_status',callback=self.trajectory_status_callback, qos_profile=10)

        # define global variables

        self.home_position = Pose(
            position=Point(x=-0.5, y=0.0, z=0.4),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        )
        self.alphabet = {}
        self.board_scale = 1.0
        self.scale_factor = 0.001 * self.board_scale
        self.shape_list = []
        self.current_mp_pose = Pose()
        self.current_traj_poses = []
        self.current_shape_poses = []
        self.kick_future = None
        self.calibrate_future = None
        self.board_future = None

        self.state = State.INITIALIZE
        self.create_letters()

    def create_letters(self):
        """Create the dictionary of bubble letters"""

        letters = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ0|-/_'
        for i in range(0, len(letters)):
            letter = letters[i]
            if letter == '0':  # Head of man
                xvec = []
                yvec = []
                q = 25
                for t in range(0, q+1):
                    x = 35*np.cos(2*np.pi*t/q)
                    y = 35+35*np.sin(2*np.pi*t/q)
                    xvec.append(x*self.scale_factor * self.board_scale)
                    yvec.append(y*self.scale_factor * self.board_scale)
                point_dict = {letter: {'xlist': xvec, 'ylist': yvec}}
                self.alphabet.update(point_dict)
            elif letter == '|':  # Body of man
                xlist = [0.0, 0.0, 0.0]
                ylist = [0.1 * self.board_scale, 0.05 *
                         self.board_scale, 0.002 * self.board_scale]
                point_dict = {letter: {'xlist': xlist, 'ylist': ylist}}
                self.alphabet.update(point_dict)
            elif letter == '-':  # Arms of man
                xlist = [0.05 * self.board_scale, 0.1 *
                         self.board_scale, 0.15 * self.board_scale]
                ylist = [0.05 * self.board_scale, 0.05 *
                         self.board_scale, 0.05 * self.board_scale]
                point_dict = {letter: {'xlist': xlist, 'ylist': ylist}}
                self.alphabet.update(point_dict)
            elif letter == '/':  # Leg of man 1
                xlist = [0.1 * self.board_scale, 0.075 *
                         self.board_scale, 0.05 * self.board_scale]
                ylist = [0.1 * self.board_scale, 0.06 *
                         self.board_scale, 0.02 * self.board_scale]
                point_dict = {letter: {'xlist': xlist, 'ylist': ylist}}
                self.alphabet.update(point_dict)
            elif letter == '_':  # Leg of man 2
                xlist = [0.0 * self.board_scale, 0.025 *
                         self.board_scale, 0.05 * self.board_scale]
                ylist = [0.1 * self.board_scale, 0.06 *
                         self.board_scale, 0.02 * self.board_scale]
                point_dict = {letter: {'xlist': xlist, 'ylist': ylist}}
                self.alphabet.update(point_dict)
            else:  # All letters of alphabet
                fp = FontProperties(family="Liberation Sans Narrow", style="normal")
                verts, codes = TextToPath().get_text_path(fp, letters[i])
                xlist = []
                ylist = []
                for j in range(0, len(verts) - 1):
                    if verts[j][0] > 0: #Commented out because I want to keep the 0,0 for lifting off the board
                        xlist.append(
                            verts[j][0] * self.scale_factor * self.board_scale)
                        ylist.append(
                            verts[j][1] * self.scale_factor * self.board_scale)
                point_dict = {letter: {'xlist': xlist, 'ylist': ylist}}
                self.alphabet.update(point_dict)

    def process_letter_points(self, letter):
        """ Function to make it easier to prepare letters for board tile type"""
        xcoord = self.alphabet[letter]['xlist']
        ycoord = self.alphabet[letter]['ylist']
        board_x = []
        board_y = []
        board_bool = []
        for i in range(0, len(xcoord)):
            if not (0.0001 > xcoord[i] > -0.0001) or not (0.0001 > ycoord[i] > 0.0001):
                board_x.append(xcoord[i])
                board_y.append(ycoord[i])
                board_bool.append(True)
            elif i != len(xcoord):
                board_x.append(xcoord[i+1])
                board_y.append(ycoord[i+1])
                board_bool.append(False)
            else:
                board_x.append(xcoord[i])
                board_y.append(ycoord[i])
                board_bool.append(False)
        return board_x, board_y, board_bool

    # def trajectory_status_callback(self, msg: String):
    #     """Callback for the service to get execute the drawing on the board"""
    #     new_msg = msg
    #     if new_msg == 'done':
    #         # Remove the first instance in the shape list since it was just executed
    #         self.shape_list.pop(0)
    #         # Return to letter writing to see if more things need to be written
    #         self.state = State.LETTER
    #     # else:
    #         # self.get_logger().error("An error occured and the trajectory did not return done.")

    def hangman_callback(self, msg: LetterMsg):
        """Callback when feedback is given from hangman"""

        # establishes a global message variable for the duration of the letter state
        self.last_message = msg
        self.ocr_pub.publish(Bool(data=False))

        # Turns off the OCR pipeline
        self.ocr_pub.publish(Bool(data=False))

        # Turns off the OCR pipeline
        self.ocr_pub.publish(False)

        self.shape_list = []
        for i in range(0, len(self.last_message.positions)):
            tile_origin = BoardTiles.Request()
            tile_origin.mode = self.last_message.mode[i]
            tile_origin.position = self.last_message.positions[i]

            # get x, y, onboard values
            tile_origin.x, tile_origin.y, tile_origin.onboard = self.process_letter_points(
                self.last_message.letters[i])
            self.shape_list.append(tile_origin)

        # switches to calibrate state
        self.state = State.CALIBRATE

    def home_callback(self, msg: Bool):
        """Callback for whether or not the robot has returned to home after writing"""
        if msg == True:
            self.ocr_pub.publish(Bool(data=True))
            self.state = State.WRITING
        else:
            self.state = State.WAITING

    async def letter_writer(self, shape: BoardTiles.Request()):
        """Function to process the shape into trajectory service calls"""
        resp = await self.board_service_client.call_async(shape)
        pose1 = resp.initial_pose
        pose_list = resp.pose_list

        self.get_logger().info(f"Pose List for Dash: {pose1}")
        self.get_logger().info(f"Pose List for Dash: {pose_list}")

        request2 = MovePose.Request()
        request2.target_pose = pose1
        request2.use_force_control = False
        await self.movepose_service_client.call_async(request2)
        self.get_logger().info(f"one done")

        request2 = Cartesian.Request()
        request2.poses = [pose_list[0]]
        request2.velocity = 0.015
        request2.replan = False
        request2.use_force_control = [shape.onboard[0]]
        await self.cartesian_mp_service_client.call_async(request2)
        self.get_logger().info(f"second done")
        # draw remaining pose dashes with Cartesian mp
        request3 = Cartesian.Request()
        request3.poses = pose_list[1:]
        request3.velocity = 0.015
        request3.replan = True
        request3.use_force_control = shape.onboard[1:]
        self.get_logger().info(f"pose_list: {pose_list[1:]}")
        await self.cartesian_mp_service_client.call_async(request3)
        self.get_logger().info(f"all done")

        self.shape_list.pop(0)

    async def timer_callback(self):

        if self.state == State.INITIALIZE:

            # Initializes the kickstart feature then waits for completion
            await self.kickstart_service_client.call_async(request=Empty.Request())
            # Turns on the OCR because the play has ended and returns to WAITING
            goal_js = MovePose.Request()
            # goal_js.joint_names = ["panda_joint4", "panda_joint5", "panda_joint7"]
            # goal_js.joint_positions = [-2.61799, -1.04173, 2.11185]
            goal_js.target_pose.position = Point(
                x=0.545029890155533, y=0.05943234468738731, z=0.5893544164237723)
            goal_js.target_pose.orientation = Quaternion(
                x=-0.48576480709767544, y=-0.5175973920275, z=-0.4696623291331898, w=0.5249216975367619)
            goal_js.use_force_control = False
            ##################### moving to the position####################
            self.get_logger().info('before moved')
            await self.movepose_service_client.call_async(goal_js)
            self.ocr_pub.publish(Bool(data=True))
            self.state = State.WAITING

        elif self.state == State.CALIBRATE:
            # Starts calibration then moves to waiting
            # await self.calibrate_service_client.call_async(request=Empty.Request())
            self.state = State.LETTER

        # elif self.state == State.APPROACHING:
        #     # Calls the service for the approach pose then moves to writing state
        #     #TODO: use cartician move
        #     await self.movepose_service_client.call_async(self.current_mp_pose)
        #     self.state = State.WRITING

        elif self.state == State.LETTER:
            if self.shape_list:
                # moves to the approaching state if there are still things to be written
                await self.letter_writer(self.shape_list[0])

                # self.state = State.WAITING
            else:
                request4 = Cartesian.Request()
                request4.poses = [Pose(position=Point(x=0.0, y=-0.3, z=0.3), orientation=Quaternion(
                    x=0.7117299678289105, y=-0.5285053338340909, z=0.268057323473255, w=0.37718408812611504))]
                request4.velocity = 0.1
                request4.replan = False
                request4.use_force_control = [False]
                await self.cartesian_mp_service_client.call_async(request4)
                # Turns on the OCR because the play has ended and returns to WAITING
                goal_js = MovePose.Request()
                # goal_js.joint_names = ["panda_joint4", "panda_joint5", "panda_joint7"]
                # goal_js.joint_positions = [-2.61799, -1.04173, 2.11185]
                goal_js.target_pose.position = Point(
                    x=0.545029890155533, y=0.05943234468738731, z=0.5893544164237723)
                goal_js.target_pose.orientation = Quaternion(
                    x=-0.48576480709767544, y=-0.5175973920275, z=-0.4696623291331898, w=0.5249216975367619)
                goal_js.use_force_control = False
                ##################### moving to the position####################
                self.get_logger().info('before moved')
                await self.movepose_service_client.call_async(goal_js)
                self.ocr_pub.publish(Bool(data=True))
                self.state = State.WAITING

        elif self.state == State.WAITING:
            pass
            # waiting state for writing actions
            # if self.kick_future:
            # Turns on OCR when kickstart finishes and waits for hangman callback

            # self.kick_future = None
            # elif self.calibrate_future:
            #     # Listens for a return value from calibration to switch to LETTER

            #     self.calibrate_future = None
            # elif self.board_future:
            #     # Looks that board has returned values
            #     # Assigns poses for approach and cartesian then moves to APPROACHING
            #     self.current_mp_pose = self.board_future.initial_pose
            #     self.current_traj_poses = self.board_future.pose_list
            #     self.board_future = None
            #     self.state = State.APPROACHING
            # else:
            # If nothing has returned from client call, WAITING passes

        # elif self.state == State.WRITING:
        #     # waiting state for the Franka to complete the mp and cartesian trajectories
        #     # in 2 steps before it moves back to LETTER
        #     if self.movepose_future:
        #         #TODO: update for use_forece_control
        #         self.cartesian_mp_service_client.call_async(self.current_traj_poses)
        #         self.movepose_future = None
        #     else:
        #         pass
            # Node will only leave this state once trajectory_status returns 'done'


def main(args=None):
    """ The node's entry point """
    rclpy.init(args=args)
    brain = Brain()
    rclpy.spin(brain)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
