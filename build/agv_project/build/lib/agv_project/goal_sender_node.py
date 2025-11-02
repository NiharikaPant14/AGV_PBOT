import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler
import sys
import time
import json
import os

# --- OBJECT DATABASE (Maps object names to coordinates) ---
OBJECT_DB = {
    "zone_a":     {"x": 4.0,   "y": -1.5, "yaw": 0.0},
    "zone_b":     {"x": -1.75, "y": 1.0,  "yaw": 1.5708},
    "zone_c":     {"x": -4.0,  "y": -3.0, "yaw": 0.01},
}

class NavigationCommander(Node):
    """
    ROS2 Node that acts as an Action Client to the Nav2 stack's NavigateToPose
    action server. 
    """

    def __init__(self, target_name=None):
        super().__init__('navigation_commander')
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose')
        
        self.target_name = target_name
        self.current_goal = self.lookup_goal(target_name)
        self._is_mission_finished = False
        self._goal_handle = None

        if self.current_goal:
             self.get_logger().info(f'Commander initialized. Target: {self.target_name} at X={self.current_goal["x"]:.2f}')
        else:
            self.get_logger().error('Could not find a valid goal.')


    def lookup_goal(self, name):
        """Looks up coordinates from the hardcoded dictionary."""
        name = name.lower()
        if name in OBJECT_DB:
            return OBJECT_DB[name]
        else:
            self.get_logger().error(f'Goal name "{name}" not found in object database.')
            self.get_logger().info(f'Available targets: {list(OBJECT_DB.keys())}')
            return None

    def send_goal(self):
        goal_data = self.current_goal
        x, y, yaw = goal_data['x'], goal_data['y'], goal_data['yaw']

        self.get_logger().info(f'Waiting for Nav2 action server...')
        
        # --- WAIT FOR SERVER ---
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 Action server is not available after 10s timeout.')
            self.get_logger().error('CRITICAL: Did you set the 2D Pose Estimate in RViz?')
            self.set_mission_finished() 
            return
        
        self.get_logger().info('Nav2 Action server is now available. Sending goal...')

        # 1. Create the Goal Request
        goal_msg = NavigateToPose.Goal()

        # Define the target pose (x, y, orientation)
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        # Convert yaw (Z rotation) to a quaternion for Nav2
        q = quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self.get_logger().info(f'Sending goal "{self.target_name}" to: X={x:.2f}, Y={y:.2f}, Yaw={yaw:.2f}...')

        # 2. Send the Goal and set up callbacks
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        
        if not self._goal_handle.accepted:
            self.get_logger().error('Goal was rejected by server.')
            self.set_mission_finished()
            return

        self.get_logger().info('Goal accepted! Robot is navigating...')
        
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        # Status code 4 is SUCCEEDED in ROS 2
        if result.status == 4:
            self.get_logger().info('✓✓✓ GOAL SUCCEEDED! Robot reached the target. ✓✓✓')
        else:
            self.get_logger().error(f'Goal failed with status: {result.status}')
        
        self.set_mission_finished()
        
    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'Distance remaining to {self.target_name}: {distance:.2f}m')
    
    def set_mission_finished(self):
        """Sets the flag to exit the manual spin loop."""
        self._is_mission_finished = True


def main(args=None):
    rclpy.init(args=args)
    
    # Check for arguments passed via the command line
    target_name = None
    if len(sys.argv) > 1:
        target_name = sys.argv[1]
    else:
        target_name = "zone_c"  # Default target
        
    commander = NavigationCommander(target_name)
    
    # Send goal only if a valid target was initialized
    if commander.current_goal:
        commander.send_goal()
    
    # CRITICAL: Keep spinning until mission is finished
    while rclpy.ok() and not commander._is_mission_finished:
        rclpy.spin_once(commander, timeout_sec=0.1)
        time.sleep(0.01)  # Small sleep to prevent CPU spinning

    # Clean shutdown
    commander.destroy_node()
    rclpy.shutdown()
    print("\n✓ Navigation Commander shutdown cleanly.")

if __name__ == '__main__':
    main()
