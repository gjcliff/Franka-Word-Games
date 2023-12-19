""" This node deals with the april tags and handels tf tree.
1. Publishes a static transform between camera and the robot (Done)
2. Calibrate service: takes the arm to a specified pose and looks at the april tags on the board and publishes a board to robot transform.
3. Letter pose service: gives the start pose of any letter wrt to the panda_link0 
"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Quaternion, Vector3, Pose
from geometry_msgs.msg import TransformStamped

from brain_interfaces.srv import BoardTiles, MovePose, UpdateTrajectory, Box

from path_planner.path_plan_execute import Path_Plan_Execute

from enum import Enum, auto

import modern_robotics as mr
import numpy as np
import transforms3d as tf


class State(Enum):
    CALIBRATE = auto()
    OTHER = auto()


class Grid:
    def __init__(self, xrange, yrange, cell_size):

        self.xrange = xrange
        self.yrange = yrange
        self.cell_size = cell_size
        self.xnum = int(((self.xrange[1] - self.xrange[0]) / self.cell_size))
        self.ynum = int(((self.yrange[1] - self.yrange[0]) / self.cell_size))
        self.grid = np.zeros((self.ynum, self.xnum))

    def grid_to_world(self, mode, position):
        if mode == 0:
            point = (0, position+2)

        if mode == 1:
            point = (2, position+2)

        if mode == 2:
            if position == 0:
                point = (2, 1)
            if position == 1:
                point = (1, 1)
            if position == 2:
                point = (1, 0)
            if position == 3:
                point = (0, 0)
            if position == 4:
                point = (0, 1)
        if mode == 3:
            point = (3, 0)

        point_y = (point[0])*self.cell_size + self.yrange[0]
        point_x = (point[1])*self.cell_size + self.xrange[0]
        if mode == 0 or mode == 1:
            # point_y += .2*position
            point_x += .1*position
        return [point_x, point_y]


class Tags(Node):

    def __init__(self):
        super().__init__("tags")
        self.freq = 100.0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.file_path_A = 'A.csv'
        self.file_path_B = 'B.csv'
        self.grid = Grid((0, .8), (0, .40), .1)
        self.path_planner = Path_Plan_Execute(self)
        self.state = State.OTHER
        self.execute_trajectory_status_callback_group = MutuallyExclusiveCallbackGroup()
        self.move_js_callback_group = MutuallyExclusiveCallbackGroup()
        self.make_board_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_callback_grp = MutuallyExclusiveCallbackGroup()
        self.calibrate_callback_grp = MutuallyExclusiveCallbackGroup()

        self.timer = self.create_timer(
            1 / self.freq, self.timer_callback, callback_group=self.timer_callback_grp)
        # creating services
        self.record_service = self.create_service(
            Empty, 'record_transform', self.record_callback)
        self.calibrate_service = self.create_service(
            Empty, 'calibrate', self.calibrate_callback, callback_group=self.calibrate_callback_grp)
        self.where_to_write = self.create_service(
            BoardTiles, 'where_to_write', self.where_to_write_callback)
        self.update_trajectory = self.create_service(
            UpdateTrajectory, 'update_trajectory', self.update_trajectory_callback)

        # create publishers
        self.state_publisher = self.create_publisher(String, 'cal_state', 10)

        # create subscribers
        # self.goal_reach_sub = self.create_subscription(
        #     String, 'execute_trajectory_status', self.goal_reach_sub_callback, 10, callback_group=self.execute_trajectory_status_callback_group)
        self.goal_state = "not"

        # create client
        self.move_js_client = self.create_client(
            MovePose, 'moveit_mp', callback_group=self.move_js_callback_group)

        # making static transform
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.make_transform()

        # making broadcast between board and robot
        self.broadcaster = TransformBroadcaster(self)
        self.robot_board = TransformStamped()
        self.robot_board.header.frame_id = "panda_link0"
        self.robot_board.child_frame_id = "board"
        self.robot_board.transform.translation.z = -0.9
        self.robot_board.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.robot_board)

        self.robot_board_write = TransformStamped()
        self.robot_board_write.header.frame_id = "panda_link0"
        self.robot_board_write.child_frame_id = "point"
        self.robot_board_write.header.stamp = self.get_clock().now().to_msg()

        # Transform to save the robot to board transform
        self.boardT = np.eye(4)

    # Create a new Future object.
        self.future = rclpy.task.Future()
        self.future_satate = rclpy.task.Future()
        # wait for services
        while not self.move_js_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Move Pose service not available, waiting again...')

        # while not self.make_board_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info(
        #         'Make Board service not available, waiting again...')

    # def goal_reach_sub_callback(self, msg):
    #     if msg == "done":
    #         self.future_satate.set_result("done")
    #     self.get_logger().info("subs")

    def make_transform(self):

        self.robot_to_camera = TransformStamped()
        self.robot_to_camera.header.stamp = self.get_clock().now().to_msg()
        self.robot_to_camera.header.frame_id = "panda_hand_tcp"
        self.robot_to_camera.child_frame_id = "camera_link"

        self.robot_to_camera.transform.translation = Vector3(
            x=0.03524146, y=-0.015, z=-0.043029)
        self.robot_to_camera.transform.rotation = Quaternion(
            x=7.07106765e-01, y=1.44018704e-04, z=7.07106768e-01, w=-1.44018703e-04)

        # self.robot_to_camera.transform.translation = Vector3(x=0.011807788327606367, y=-0.2697480532095115, z=0.03157999806748111)
        # self.robot_to_camera.transform.rotation = Quaternion(x=-0.10948317052954765, y=-0.28408415419824595, z=0.2376708393954742, w=0.9224002389447412)

        # self.get_logger().info(type(self))
        self.tf_static_broadcaster.sendTransform(self.robot_to_camera)

    def matrix_to_position_quaternion(self, matrix, point=0):
        translation = matrix[:3, 3]
        rotation_matrix = matrix[:3, :3]

        # Convert rotation matrix to quaternion using tf2
        quaternion = tf.quaternions.mat2quat(rotation_matrix)

        # Create Vector3 for position
        if point == 0:
            position = Vector3()
        elif point == 1:
            position = Point()
        position.x, position.y, position.z = translation

        # Create Quaternion for rotation
        rotation = Quaternion()
        rotation.w, rotation.x, rotation.y, rotation.z = quaternion

        return position, rotation

    def array_to_transform_matrix(self, translation, quaternion):
        # Normalize the quaternion
        quaternion /= np.linalg.norm(quaternion)
        quaternion = [quaternion[3], quaternion[0],
                      quaternion[1], quaternion[2]]

        # Create rotation matrix from quaternion
        rotation_matrix = tf.quaternions.quat2mat(quaternion)

        # Create the transformation matrix
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = translation

        return transform_matrix

    def mean_transformation_matrices(self, matrices):
        # Convert transformation matrices to exponential coordinates
        exp_coords = [mr.se3ToVec(mr.MatrixLog6(matrix))
                      for matrix in matrices]
        print(exp_coords)
        # Calculate the mean of exponential coordinates
        mean_exp_coord = np.mean(exp_coords, axis=0)

        # Convert the mean exponential coordinate back to a transformation matrix
        mean_matrix = mr.MatrixExp6(mr.VecTose3(mean_exp_coord))
        return mean_matrix

    async def calibrate_callback(self, request, response):
        self.state = State.CALIBRATE
        # ([], [])

        goal_js = MovePose.Request()
        # goal_js.joint_names = ["panda_joint4", "panda_joint5", "panda_joint7"]
        # goal_js.joint_positions = [-2.61799, -1.04173, 2.11185]
        goal_js.target_pose.position = Point(
            x=0.30744234834406486, y=-0.17674628233240325, z=0.5725350884705022)
        goal_js.target_pose.orientation = Quaternion(
            x=0.7117299678289105, y=-0.5285053338340909, z=0.268057323473255, w=0.37718408812611504)
        goal_js.use_force_control = False
        ##################### moving to the position####################
        self.get_logger().info('before moved')
        ans = await self.move_js_client.call_async(goal_js)
        self.goal_state = "done"
        self.get_logger().info('moved')
        # self.goal_state = await self.future_satate

        # 3when its done start doing calibrate sequence

        ansT, ansR = await self.future
        ansT1, ansT2 = ansT
        ansR1, ansR2 = ansR
        while ansT1[0] == 0.0 and ansT2[0]:
            ansT, ansR = await self.future
            ansT1, ansT2 = ansT
            ansR1, ansR2 = ansR
            # self.get_logger().info(f"{ansT, ansR}")
            self.get_logger().info('value set in service')
        # if ansT[0] != 0.0:
        Tt1b = np.array([[1, 0, 0, 0.05],
                        [0, 1, 0, 0.05],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        Tt2b = np.array([[1, 0, 0, 0.05],
                        [0, 1, 0, -0.05],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        # Ttb = np.array([0.063,0.063,0,1])
        Trt1 = self.array_to_transform_matrix(ansT1, ansR1)
        Trt2 = self.array_to_transform_matrix(ansT2, ansR2)
        Trb1 = Trt1 @ Tt1b
        Trb2 = Trt2 @ Tt2b
        Trb = self.mean_transformation_matrices([Trb1, Trb2])

        self.boardT = Trb1
        self.get_logger().info(f'Trb: \n{Trb1}')
        pos, rotation = self.matrix_to_position_quaternion(Trb1)
        self.get_logger().info(f'Trt: \n{Trt1}')
        self.get_logger().info(f'Trb: \n{Trb}')
        self.robot_board.transform.translation = pos
        self.robot_board.transform.rotation = rotation

        # pos, rotation = self.matrix_to_position_quaternion(Trb2)
        # self.get_logger().info(f'Trt: \n{Trt1}')
        # self.get_logger().info(f'Trb: \n{Trb}')

        # Tt1b_ = np.array([[1, 0, 0, 0.7],
        #                   [0, 1, 0, 0.5],
        #                   [0, 0, 1, -0.05],
        #                   [0, 0, 0, 1]])
        # Tt2b_ = np.array([[1, 0, 0, 0.7],
        #                   [0, 1, 0, 0.4],
        #                   [0, 0, 1, -0.05],
        #                   [0, 0, 0, 1]])
        # # Ttb = np.array([0.063,0.063,0,1])
        # Trt1 = self.array_to_transform_matrix(ansT1, ansR1)
        # Trt2 = self.array_to_transform_matrix(ansT2, ansR2)
        # Trb1_ = Trt1 @ Tt1b_
        # Trb2_ = Trt2 @ Tt2b_
        # Trb_ = self.mean_transformation_matrices([Trb1_, Trb2_])

        # p, r = self.matrix_to_position_quaternion(Trb_, 1)
        # board_pose = Pose()
        # board_pose.position = p
        # board_pose.orientation = r

        # board_request = Box.Request()
        # board_request.pose = board_pose
        # board_request.size = [2.0, 2.2, 0.02]
        # board_request.name = "board"
        # # await self.make_board_client.call_async(board_request)
        # self.robot_board_write.transform.translation = pos
        # self.robot_board_write.transform.rotation = rotation

        self.state = State.OTHER
        self.goal_state = "not"
        self.get_logger().info(f'Trb: {pos,rotation}')

        self.get_logger().info("calibrate")
        return response

    async def where_to_write_callback(self, request, response):
        self.get_logger().info("where_to_write1")
        response_a = []

        self.get_logger().info("where_to_write2")
        # ansT, ansR = self.get_transform('panda_link0', 'board')
        # ansT = self.robot_board.transform.translation
        # ansR = self.robot_board.transform.rotation
        # self.get_logger().info(f'board : {ansT, ansR}')
        # Trb = self.array_to_transform_matrix(ansT, ansR)
        Trb = self.boardT
        lx, ly = self.grid.grid_to_world(request.mode, request.position)
        Tbl = np.array([[1, 0, 0, lx * 0.667],
                        [0, 1, 0, ly * 0.667],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        Trl = Trb @ Tbl

        x, y = request.x[0], request.y[0]
        z = 0.12

        Tla = np.array([[-0.03948997,  0.99782373,  0.05280484, x],
                        [0.06784999,  0.05540183, -0.99615612,  y],
                        [-0.9969137, -0.03575537, -0.06989015,  z],
                        [0.,        0.,        0.,         1.]]
                       )
        pos = Pose()
        Tra = Trl @ Tla
        position, rotation = self.matrix_to_position_quaternion(Tra, 1)
        pos.position = position
        pos.orientation = rotation
        self.robot_board_write.transform.translation, self.robot_board_write.transform.rotation = self.matrix_to_position_quaternion(
            Tra)

        response.initial_pose = pos

        for i in range(len(request.x)):
            x, y = request.x[i], request.y[i]
            self.get_logger().info(f"sanity check:  {x,y}")
            self.get_logger().info(f'x,y : {x,y}')
            z = 0.004 if request.onboard[i] else 0.1

            # Tla = np.array([[0, 1, 0, x],
            #                 [0.5,  0.0 ,        -0.8660254, y],
            #                 [-0.8660254,  0.0  , -0.5, z],
            #                 [0, 0, 0, 1]])

            Tla = np.array([[-0.03948997,  0.99782373,  0.05280484, x],
                            [0.06784999,  0.05540183, -0.99615612,  y],
                            [-0.9969137, -0.03575537, -0.06989015,  z],
                            [0.,        0.,        0.,         1.]]
                           )
            Tra = Trl @ Tla
            position, rotation = self.matrix_to_position_quaternion(Tra, 1)
            pos = Pose()
            pos.position = position
            pos.orientation = rotation

            self.robot_board_write.transform.translation, self.robot_board_write.transform.rotation = self.matrix_to_position_quaternion(
                Tra)
            self.get_logger().info(f'pose is: {pos}')
            response_a.append(pos)
            self.get_logger().info(f'Now the list is: {response_a}')

        self.get_logger().info("where_to_write3")
        response.pose_list = response_a
        response.use_force_control = request.onboard
        # self.get_logger().info(f'{response.pose_list}')
        return response

    def update_trajectory_callback(self, request, response):

        ansT, ansR = self.get_transform("board", "panda_hand_tcp")

        # positive z is out of the board
        if request.into_board:
            z = ansT[2] + 0.0013
        else:
            z = ansT[2] - 0.002

        self.get_logger().info("reached update trajcetory callback")

        pose = request.input_pose
        self.get_logger().info(f"input pose: {pose}")

        # change the pose
        Trans_arr = [pose.position.x, pose.position.y, pose.position.z]
        Rot_arr = [pose.orientation.x, pose.orientation.y,
                   pose.orientation.z, pose.orientation.w]
        Tra = self.array_to_transform_matrix(Trans_arr, Rot_arr)
        Trb = self.boardT
        Tba = mr.TransInv(Trb)@Tra
        # update = np.array([[1, 0, 0, 0],
        #                    [0, 1, 0, 0],
        #                    [0, 0, 1, z],
        #                    [0, 0, 0, 1]])
        # new_Tba = update@Tba
        new_Tba = Tba
        new_Tba[2, 3] = z
        new_Tra = Trb @ new_Tba
        pos = Pose()
        position, rotation = self.matrix_to_position_quaternion(new_Tra, 1)
        pos.position = position
        pos.orientation = rotation
        # self.boardT[2][3] = z - 0.004
        self.get_logger().info(f"output pose: {pos}")

        response.output_pose = pos

        self.get_logger().info("exiting update trajectory callback")

        return response

    def record_callback(self, request, response):
        At, Aq = self.get_transform('panda_link0', 'panda_hand_tcp')
        # self.get_logger().info(f'Tba: {ans1}')
        Bt, Bq = self.get_transform('camera_link', 'tag56')
        # self.get_logger().info(f'Tca: {ans2}')

        A = self.array_to_transform_matrix(At, Aq)
        B = self.array_to_transform_matrix(Bt, Bq)
        self.write_matrix_to_file(self.file_path_A, A)
        self.write_matrix_to_file(self.file_path_B, B)

        return response

    # Function to read matrices from a file
    def read_matrices_from_file(self, file_path):
        matrices = []
        delimiter = '---'

        with open(file_path, 'r') as file:
            matrix_strings = file.read().split(delimiter)

            for matrix_string in matrix_strings:
                if matrix_string.strip():
                    matrix_lines = matrix_string.strip().split('\n')
                    matrix_values = [
                        list(map(float, line.strip().split(','))) for line in matrix_lines]
                    matrices.append(np.array(matrix_values))

        return matrices

    # Function to write matrices to a file
    def write_matrix_to_file(self, file_path, matrix):
        with open(file_path, 'a') as file:
            for row in matrix:
                file.write(','.join(map(str, row)) + '\n')
            file.write('---\n')

    # File path for your file

    def get_transform(self, parent_frame, child_frame):
        """
        Try catch block for Listning transforms between parent and child frame.

        Args:
        ----
            parent_frame (string): name of parent frame
            child_frame (string): name of child frame

        Returns
        -------
            brick_to_platform: the x,y,z of the translational transform

        """
        try:
            trans = self.buffer.lookup_transform(
                parent_frame, child_frame, rclpy.time.Time()
            )
            transl = trans.transform.translation
            rot = trans.transform.rotation
            brick_to_platform = [transl.x, transl.y, transl.z]
            rotation = [rot.x, rot.y, rot.z, rot.w]

            # print(brick_to_platform[2])
            return brick_to_platform, rotation

        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().info(f"Lookup exception: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f"Connectivity exception: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f"Extrapolation exception: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]

    async def timer_callback(self):
        # ans1T, ans1R = self.get_transform('board', 'panda_hand_tcp')
        # self.get_logger().info(f'hand: {ans1T[1]}')

        # self.get_logger().info("timmer function")
        if self.state == State.CALIBRATE and self.goal_state == "done":

            # self.get_logger().info('function done')
            ansT1, ansR1 = self.get_transform('panda_link0', 'tag11')
            ansT2, ansR2 = self.get_transform('panda_link0', 'tag12')
            # self.get_logger().info(f'{ansT, ansR}')
            self.future.set_result([[ansT1, ansT2], [ansR1, ansR2]])

        # ansTi, ansRi = self.get_transform('board', 'panda_hand_tcp')
        # pls = self.array_to_transform_matrix(ansTi, ansRi)
        # self.get_logger().info(f'{pls}')
        # if self.state == State.OTHER:
        self.robot_board.header.stamp = self.get_clock().now().to_msg()
        self.robot_board_write.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.robot_board)
        self.broadcaster.sendTransform(self.robot_board_write)


def Tags_entry(args=None):
    rclpy.init(args=args)
    node = Tags()
    rclpy.spin(node)
    rclpy.shutdown()
