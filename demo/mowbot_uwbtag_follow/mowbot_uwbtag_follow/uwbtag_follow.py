import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import tf2_ros
import tf2_geometry_msgs
import math
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration
from scipy.spatial.transform import Rotation as R
from rclpy.task import Future
from action_msgs.msg import GoalStatus


class TransformationHandler:
    def __init__(self, node, target_frame='map'):
        self.node = node
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        self.target_frame = target_frame

    def transform_point(self, point_stamped, timeout=Duration(seconds=1.0)):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                point_stamped.header.frame_id,
                now,
                timeout=timeout
            )
            point_in_target = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            return point_in_target
        
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.node.get_logger().error(f'Failed to transform point: {e}')
            return None

class GoalManager:
    def __init__(self, node: Node, action_name: str = 'navigate_to_pose'):
        self.node = node
        self.action_client = ActionClient(node, NavigateToPose, action_name)
        self.current_goal_handle = None  # Store the current goal handle
        
        # Waiting for the NavigateToPose action server with timeout
        self.node.get_logger().info(f'Waiting for {action_name} action server...')
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(f'{action_name} action server not available within the timeout.')
        else:
            self.node.get_logger().info(f'{action_name} action server available.')

    def send_goal(self, goal_pose: PoseStamped, feedback_callback=None) -> None:
        """Send a goal to the NavigateToPose action server."""
        if feedback_callback is None:
            feedback_callback = self.feedback_callback  # Use internal callback if not provided

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.node.get_logger().info(
            f'Sending goal to position x: {goal_pose.pose.position.x:.2f}, '
            f'y: {goal_pose.pose.position.y:.2f}'
        )

        # Send the goal asynchronously and handle the response
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=feedback_callback  # Use the provided feedback callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg) -> None:
        """Default feedback callback method."""
        distance_remaining = feedback_msg.feedback.distance_remaining
        current_position = feedback_msg.feedback.current_pose.pose.position

        # Log the feedback information
        self.node.get_logger().info(
            f'Feedback - Distance remaining: {distance_remaining:.2f} m, '
            f'Current position: x={current_position.x:.2f}, y={current_position.y:.2f}'
        )

    def goal_response_callback(self, future: Future) -> None:
        """Handle the response when the goal is accepted or rejected."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn('Goal was rejected by the action server.')
            return

        self.node.get_logger().info('Goal accepted by the action server.')
        self.current_goal_handle = goal_handle  # Store the goal handle for potential cancellation
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future) -> None:
        """Handle the result of the goal once completed."""
        try:
            result = future.result()
            status = result.status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.node.get_logger().info('Goal succeeded!')
            elif status == GoalStatus.STATUS_ABORTED:
                self.node.get_logger().warn('Goal was aborted by the action server.')
            elif status == GoalStatus.STATUS_CANCELED:
                self.node.get_logger().info('Goal was canceled before completion.')
            else:
                self.node.get_logger().info(f'Goal failed with status code: {status}')
        except Exception as e:
            self.node.get_logger().error(f'Exception in result callback: {e}')

    def cancel_goal(self) -> None:
        """Cancel the current goal if it exists."""
        if self.current_goal_handle is not None:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.node.get_logger().warn("No active goal to cancel.")

    def cancel_done_callback(self, future: Future) -> None:
        """Handle the result of the goal cancellation request."""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info('Goal successfully canceled.')
        else:
            self.node.get_logger().warn('Failed to cancel the goal or no goal was active.')

    @staticmethod
    def calculate_orientation(goal_x: float, goal_y: float) -> list:
        """Calculate the quaternion from a yaw angle towards (goal_x, goal_y)."""
        yaw = math.atan2(goal_y, goal_x)
        rotation = R.from_euler('z', yaw)
        quaternion = rotation.as_quat()  # [x, y, z, w]
        return quaternion


        
class UwbTagFollowNode(Node):

    world_frame = 'odom' #'map'
    robot_frame = 'base_link'

    goal_update_interval = 2.0  # seconds

    follow_distance_threshold = 2.0  # meters, distance to the tag to stop following


    STATE_IDLE = 0
    STATE_FOLLOWING = 1

    def __init__(self):
        super().__init__('uwbtag_follow')

        self.current_dist2D_to_tag = None
        self.current_state = self.STATE_IDLE
        self.prev_uwb_point = None

        # Initialize Goal Manager, TODO: retry to connect to the action server if it fails
        self.goal_manager = GoalManager(self, action_name='navigate_to_pose')

        # Initialize Transformation Handler
        self.transform_handler = TransformationHandler(node=self, target_frame=self.world_frame)

        # Initialize last goal time for throttling
        self.last_goal_time = self.get_clock().now()
        self.last_goal = None

        # Subscribe to uwb_tag_point topic
        self.subscription = self.create_subscription(
            PointStamped,
            '/uwb_tag_point/kf_spikes_filtered',
            self.tag_point_callback,
            10)
            


    def tag_point_callback(self, msg):

        # self.get_logger().info(f'Tag point: {msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f}')

        # check if prev pose not too far from current

        try:
            # Transform the point to the map frame
            point_in_map = self.transform_handler.transform_point(point_stamped=msg)
            if point_in_map is None:
                self.get_logger().error('Failed to transform point.')
                return
            # self.get_logger().info(f'Tag point in map: {point_in_map.point.x:.2f}, {point_in_map.point.y:.2f}, {point_in_map.point.z:.2f}')

            # Calculate distance to robot in base_link frame
            self.current_dist2D_to_tag = math.sqrt(
                msg.point.x ** 2 +
                msg.point.y ** 2
            )

            # Check if the tag is close enough to the robot
            if self.current_dist2D_to_tag < self.follow_distance_threshold:
                if self.current_state == self.STATE_FOLLOWING:
                    self.get_logger().info(f'Tag distance 2d: {self.current_dist2D_to_tag:.2f} m')
                    self.get_logger().info('Tag is close enough to the robot.')
                    self.get_logger().info('Stop following the tag.')
                    self.emergency_stop()
                    self.current_state = self.STATE_IDLE
                    
                return

            # if not close enough, process new goal

            if self.current_state == self.STATE_IDLE:
                self.current_state = self.STATE_FOLLOWING
                self.get_logger().info(f'Tag distance 2d: {self.current_dist2D_to_tag:.2f} m')
                self.get_logger().info('Start following the tag.')


            # Throttle goal updates
            current_time = self.get_clock().now()
            time_since_last_goal = (current_time - self.last_goal_time).nanoseconds / 1e9
            if time_since_last_goal < self.goal_update_interval:
                self.get_logger().debug('Goal update throttled.')
                return  # Skip sending a new goal
            
            self.last_goal_time = current_time

            # Calculate the goal position that is scaled based on the distance to the tag
            scaling_factor = (self.current_dist2D_to_tag - self.follow_distance_threshold) / self.current_dist2D_to_tag
            goal_x = scaling_factor * point_in_map.point.x
            goal_y = scaling_factor * point_in_map.point.y

            if self.last_goal is not None:
                if abs(goal_x - self.last_goal[0]) < 0.1 and abs(goal_y - self.last_goal[1]) < 0.1:
                    self.get_logger().info('Goal not changed significantly, skipping...')
                    return
            else:
                self.last_goal = (goal_x, goal_y)

            # Create a PoseStamped for the goal
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = self.world_frame
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = goal_x
            goal_pose.pose.position.y = goal_y
            goal_pose.pose.position.z = 0.0  # Assuming flat ground

            # Calculate orientation towards the goal using SciPy
            yaw = math.atan2(goal_y, goal_x)  # Adjusted for map frame
            quaternion = self.goal_manager.calculate_orientation(goal_x, goal_y)
            goal_pose.pose.orientation.x = quaternion[0]
            goal_pose.pose.orientation.y = quaternion[1]
            goal_pose.pose.orientation.z = quaternion[2]
            goal_pose.pose.orientation.w = quaternion[3]

            self.get_logger().info(f'Following tag at distance: {self.current_dist2D_to_tag:.2f} m')
            self.get_logger().info(f'Tag point in map:\
                    {point_in_map.point.x:.2f}, {point_in_map.point.y:.2f}, {point_in_map.point.z:.2f}')
            self.get_logger().info(f'Goal position: {goal_x:.2f}, {goal_y:.2f}')
            rpy = R.from_quat(quaternion).as_euler('xyz', degrees=True)
            self.get_logger().info(f'Goal orientation: {rpy[2]:.2f} deg')

            self.goal_manager.send_goal(goal_pose, feedback_callback=self.feedback_callback)

        except Exception as e:
            self.get_logger().error(f'Failed to process tag point: {e}')
    
    def feedback_callback(self, feedback_msg):
        distance_remaining = feedback_msg.feedback.distance_remaining
        current_position = feedback_msg.feedback.current_pose.pose.position

        # Log the feedback information
        self.get_logger().info(
            f'Feedback - Distance remaining: {distance_remaining:.2f} m, '
            f'Current position: x={current_position.x:.2f}, y={current_position.y:.2f}'
        )

    def emergency_stop(self):
        self.get_logger().warn('Emergency stop activated!')
        self.goal_manager.cancel_goal()



        




def main(args=None):
    rclpy.init(args=args)
    uwbtag_follow_node = UwbTagFollowNode()
    try:
        rclpy.spin(uwbtag_follow_node)
    except KeyboardInterrupt:
        pass

    uwbtag_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()