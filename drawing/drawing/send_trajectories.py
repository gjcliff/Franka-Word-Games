import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.task import Future

from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String

from brain_interfaces.msg import EEForce

from brain_interfaces.srv import ExecuteJointTrajectories, Replan, UpdateTrajectory

from enum import Enum, auto

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros


class State(Enum):

    PUBLISH = auto()
    FORCE_CONTROL = auto()
    STOP = auto()


class Executor(Node):

    def __init__(self):

        super().__init__("Execute")

        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.joint_trajectories_callback_group = MutuallyExclusiveCallbackGroup()
        self.replan_callback_group = MutuallyExclusiveCallbackGroup()
        self.update_trajectory_callback_group = MutuallyExclusiveCallbackGroup()

        self.timer = self.create_timer(
            0.01, self.timer_callback, callback_group=self.timer_callback_group)

        # create publishers
        self.pub = self.create_publisher(
            JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)

        self.execute_trajectory_status_pub = self.create_publisher(
            String, '/execute_trajectory_status', 10)

        # create services
        self.joint_trajectories_service = self.create_service(
            ExecuteJointTrajectories, '/joint_trajectories', self.joint_trajectories_callback, callback_group=self.joint_trajectories_callback_group)

        # create clients
        self.replan_client = self.create_client(
            Replan, '/replan_path', callback_group=self.replan_callback_group)

        self.update_trajectory_client = self.create_client(UpdateTrajectory, 'update_trajectory',
                                                           callback_group=self.update_trajectory_callback_group)

        # create subscriptions
        self.force_sub = self.create_subscription(
            EEForce, '/ee_force', self.force_callback, 10)

        # these are used for computing the current location of the end-effector
        # using the tf tree.
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.joint_trajectories = []
        self.pose = None
        self.ee_force = 0
        self.upper_threshold = 3.0  # N
        self.lower_threshold = 1.0  # N
        self.state = None
        self.use_force_control = False
        self.use_control_loop = False
        self.force_error = 0.0  # N
        self.previous_force_error = 0.0  # N
        self.initial_trajectory_angle = 0.0  # rad
        self.output_angle = 0.0  # rad
        self.integral_force_error = 0.0

        self.distance = 0.01  # distance along quaternion to move
        self.replan = False

        self.future = Future()

        self.i = 1

    def get_transform(self, parent_frame, child_frame):
        """
        Try catch block for listening to transforms between parent and child frame.

        Args:
        ----
        parent_frame (string): name of parent frame
        child_frame (string): name of child frame

        Returns
        -------
        brick_to_platform: the x,y,z of the translational transform

        """
        try:
            pose = Pose()
            trans = self.buffer.lookup_transform(
                parent_frame, child_frame, rclpy.time.Time()
            )
            transl = trans.transform.translation
            rot = trans.transform.rotation
            pose.position = Point(x=transl.x, y=transl.y, z=transl.z)
            pose.orientation = Quaternion(x=rot.x, y=rot.y, z=rot.z, w=rot.w)

            # print(brick_to_platform[2])
            return pose

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

    def force_callback(self, msg):
        self.ee_force = msg.ee_force
        # self.use_force_control = msg.use_force_control

    async def joint_trajectories_callback(self, request, response):
        self.get_logger().info("message received!")
        self.i = 0

        self.joint_trajectories = request.joint_trajectories
        self.output_angle = self.joint_trajectories[0].points[0].positions[5]
        self.pose = request.current_pose
        self.replan = request.replan
        self.use_force_control = request.use_force_control
        if not self.use_force_control:
            self.use_control_loop = False
        self.get_logger().info(f"use force control: {self.use_force_control}")
        self.get_logger().info(f"replan: {self.replan}")
        self.get_logger().info(f"use control loop: {self.use_control_loop}")

        if request.state == "publish":
            self.state = State.PUBLISH
        else:
            self.state = State.STOP

        await self.future

        self.future = Future()

        return response

    async def replan_trajectory(self, into_the_board):
        self.get_logger().info("joint trajectories cleared")
        self.joint_trajectories.clear()

        # replan the trajectory!!
        self.get_logger().info(
            f"pose to be adjusted out of board: {self.pose}")
        update_trajectory_response = await self.update_trajectory_client.call_async(UpdateTrajectory.Request(input_pose=self.pose, into_board=into_the_board))

        self.pose = update_trajectory_response.output_pose

        replan_response = await self.replan_client.call_async(Replan.Request(pose=self.pose))

        self.joint_trajectories = replan_response.joint_trajectories
        self.output_angle = self.joint_trajectories[0].points[0].positions[5]

    async def timer_callback(self):
        # self.get_logger().info(f"ee_force: {self.ee_force}")

        if self.ee_force > self.upper_threshold and self.use_force_control and self.joint_trajectories:
            self.get_logger().info(
                f"upper_threshold: {self.upper_threshold}")
            self.get_logger().info(
                f"UPPER FORCE THRESHOLD EXCEEDED, EE_FORCE: {self.ee_force}")

            if self.replan:
                # clear the current trajectories first, as we don't want to execute them anymore
                await self.replan_trajectory(False)

                self.use_control_loop = True
                self.use_force_control = False
                self.initial_trajectory_angle = self.joint_trajectories[0].points[0].positions[5]

                # self.upper_threshold += 3  # essentially turning force control off, for now
                # self.lower_threshold = 1.0

            else:

                self.get_logger().info("joint trajectories cleared")
                self.get_logger().info("poses all done")
                self.joint_trajectories.clear()

        elif self.joint_trajectories and self.state == State.PUBLISH and self.i % 10 == 0:
            # self.get_logger().info(f"publishing!!!!!!!!!!!!!!!")

            Kp = 0.0028
            Ki = 0.000002
            Kd = 0.0009

            # self.get_logger().info(
            #     f"joint_Trajectory: {self.joint_trajectories[0].points[0]}")

            # self.pub.publish(self.joint_trajectories[0])
            # self.joint_trajectories.pop(0)

            self.get_logger().info(
                f"user control loop: {self.use_control_loop}")
            self.get_logger().info(
                f"initial angle: {self.initial_trajectory_angle}")

            if self.use_control_loop:
                self.get_logger().info(f"ee_force: {self.ee_force}")
                self.get_logger().info(
                    f"original joint pos: {self.output_angle}")
                force_error = 1.75 - self.ee_force
                self.integral_force_error += force_error * 0.1
                angle_adjustment = Kp * force_error + Ki * self.integral_force_error + \
                    Kd * (force_error - self.previous_force_error)
                self.output_angle += angle_adjustment
                self.joint_trajectories[0].points[0].positions[5] = self.output_angle
                self.previous_force_error = force_error
                # here i'm assuming joint angle 6 is basically the same
                # for all trjactories, which may or may not be true.

                difference_from_initial = self.output_angle - self.initial_trajectory_angle

                self.get_logger().info(
                    f"modified joint pos: {self.joint_trajectories[0].points[0].positions[5]}")

                if difference_from_initial > 0.09:
                    self.get_logger().info(f"tilted too far forward, replannign")
                    await self.replan_trajectory(False)
                    self.use_control_loop = False
                elif difference_from_initial < -0.09:
                    self.get_logger().info(f"tilted too far backward, replannign")
                    await self.replan_trajectory(True)
                    self.use_control_loop = False

                self.pub.publish(self.joint_trajectories[0])
                self.joint_trajectories.pop(0)

            else:
                self.pub.publish(self.joint_trajectories[0])
                self.joint_trajectories.pop(0)

        # if we've reached the goal, send a message to draw.py that says we're done.
        elif not self.joint_trajectories and self.state == State.PUBLISH:

            self.future.set_result("done")
            self.get_logger().info("done executing!!")

            self.state = State.STOP

        self.i += 1


def main(args=None):

    rclpy.init(args=args)

    executor = Executor()

    rclpy.spin(executor)


if __name__ == '__main__':
    main()
