import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose

import rclpy
from rclpy.time import Duration


from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import gym
import time
from gym.envs.registration import register
import argparse

class BasicNavigator(Node):
    def __init__(self):
        super().__init__(node_name='basic_navigator')
        self.initial_pose = Pose()
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

        self.initial_pose_received = False
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                       'amcl_pose',
                                                       self._amclPoseCallback,
                                                       amcl_pose_qos)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)

    def setInitialPose(self, initial_pose):
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._setInitialPose()
    
    def goToPose(self, pose):
        # Sends a `NavToPose` action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                      str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)

        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                           str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelNav(self):
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.info('Goal succeeded!')
        return True

    def getFeedback(self):
        return self.feedback

    def getResult(self):
        return self.status

    def waitUntilNav2Active(self):
        self._waitForNodeToActivate('amcl')
        self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1)
        return

    def _amclPoseCallback(self, msg):
        self.initial_pose_received = True
        return

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        self.initial_pose_received = True
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return

def navigation_policy(waypoint_index, navigator):
    '''Hard coded motion policy
    
    Args:
        waypoint_index: Index of waypoint in the list
        naivgator: BasicNavigator object

    Returns:
        goal_pose: position to navigate to
    '''

    waypoints = [(0.0, 2.0), (-1.0, 2.0), (0.0, -2.0)]
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = waypoints[waypoint_index % 3][0]
    goal_pose.pose.position.y = waypoints[waypoint_index % 3][1]
    goal_pose.pose.orientation.z = 0.05
    goal_pose.pose.orientation.w = 1.0

    return goal_pose

if __name__ == '__main__':
    rclpy.init()

    # Setup multigrid environment. Using EasyCTF world.
    register(id='multigrid-nav2-demo',
             entry_point='multigrid_envs:EasyCTF')
    env = gym.make('multigrid-nav2-demo')
    _ = env.reset()
    navigator = BasicNavigator()

    # Set initial pose of the turtlebot
    initial_pose = Pose()
    initial_pose.position.x = 0.0
    initial_pose.position.y = -2.0
    initial_pose.position.z = 0.05
    initial_pose.orientation.z = 0.0
    initial_pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # gazebo grid to multigrid space factor. Each grid space in multigrid is 1/4 meter.
    multigrid_factor = 4
    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    for i in range(3):
        goal_pose = navigation_policy(i, navigator)
        navigator.goToPose(goal_pose)

        previous_x = None
        previous_y = None

        # Multigrid directions
        left = 1
        right = 2
        forward = 3
        # Store orientation of the robot 
        orientation = 0
        # env.step(right)
        while not navigator.isNavComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            # print(feedback.current_pose.pose.orientation.w/feedback.current_pose.pose.orientation.z)
            if previous_x is None:
                previous_x = feedback.current_pose.pose.position.x
                previous_y = feedback.current_pose.pose.position.y

            # print(feedback.current_pose.pose.position.x)
            env.render(mode='human', highlight=True)
            current_x = feedback.current_pose.pose.position.x
            current_y = feedback.current_pose.pose.position.y
            # print('Current x:', current_x)
            # print('Current y:', current_y)
            # Update x position in multigrid
            if abs(previous_x - current_x) >= 1/multigrid_factor:
                if previous_x > current_x:
                    # Move Right
                    env.step([right])
                    env.step([right])
                    env.step([forward])
                    env.step([left])
                    env.step([left])
                    # print('Down')
                elif previous_x < current_x:
                    # Move down and rotate back to Right
                    # env.step([left])
                    # env.step([left])
                    env.step([forward])
                    # env.step([right])
                    # env.step([right])
                    # print('Up')
                previous_x = current_x
            
            # Update y position in multigrid
            if abs(previous_y - current_y) >= 1/multigrid_factor:
                if previous_y < current_y:
                    # Turn the agent to the right, move forward, then turn back to facing up
                    # env.step([right])
                    env.step([left])
                    env.step([forward])
                    env.step([right])
                    # env.step([left])
                    # print('Left')
                elif previous_y > current_y:
                    # Turn agent to left, move forward, then back to facing up
                    env.step([right])
                    env.step([forward])
                    env.step([left])
                    # print('Right')
                previous_y = current_y

            if feedback and i % 5 == 0:
                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelNav()

        # Do something depending on the return code
        result = navigator.getResult()
        if result == GoalStatus.STATUS_SUCCEEDED:
            print('Goal succeeded!')
        elif result == GoalStatus.STATUS_CANCELED:
            print('Goal was canceled!')
        elif result == GoalStatus.STATUS_ABORTED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    exit(0)