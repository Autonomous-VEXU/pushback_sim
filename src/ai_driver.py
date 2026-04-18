#!/usr/bin/env python3

import rclpy
import math
import numpy as np
from rclpy.node import Node
from vex_interfaces.msg import ActionState
from vex_interfaces.srv import IntakeBall, OutputBall
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker

from field_location import FieldLocation


class AIdriver(Node):
    def __init__(self):
        super().__init__('ai_driver')
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.marker_pub = self.create_publisher(Marker, '/ai_goal_marker', 10)
        
        self.create_subscription(ActionState, '/sai_output', self.interpret_action, 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.store_current_velocity, 10)

        # service clients
        # self.intake_ball = self.create_client(IntakeBall, '/robot_intake')
        # self.ball_action = self.create_client(OutputBall, '/score_ball')


    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        '''strategy AI only gives back yaw'''

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q

    def store_current_velocity(self, msg: TwistStamped):
        lin_x = msg.twist.linear.x
        lin_y = msg.twist.linear.y

        self.avg_vel = (lin_x + lin_y)/2

    def publish_goal_marker(self, x, y, robot_color):
        """Publish marker at raw output coordinates"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # Use a unique ID for each marker so they don't overwrite each other
        if not hasattr(self, '_marker_id_counter'):
            self._marker_id_counter = 0
        marker.id = self._marker_id_counter
        self._marker_id_counter += 1
        
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Set lifetime to a very large value (never fade)
        marker.lifetime.sec = 99999
        marker.lifetime.nanosec = 0
        
        if robot_color == 'red':
            marker.color.r = 1.0
            marker.color.a = 1.0
        else:
            marker.color.b = 1.0
            marker.color.a = 1.0
          
        self.marker_pub.publish(marker)

    
    def interpret_action(self, msg: ActionState):
        if msg.red_robot_action == 0:
            self.get_logger().info("driving to coordinate")

            self.publish_goal_marker(msg.red_robot.x, msg.red_robot.y, 'red')
            self.publish_goal_marker(msg.blue_robot.x, msg.blue_robot.y, 'blue')
            
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = msg.red_robot.x
            goal_pose.pose.position.y = msg.red_robot.y

            quat = AIdriver.quaternion_from_euler(0, 0, msg.red_robot.theta) 
            goal_pose.pose.orientation.x = quat[0]
            goal_pose.pose.orientation.y = quat[1]
            goal_pose.pose.orientation.z = quat[2]
            goal_pose.pose.orientation.w = quat[3]
            
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose
            self.nav_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        # elif msg.red_robot_action == 1 or msg.red_robot_action == 2:
        #     # call pickup ball service
        #     FieldLocation.check_collision()
            
        # elif msg.red_robot_action == 3:
        #     # score ball service at random height
        #     FieldLocation.scoring_callback(3)

    

    def _feedback_callback(self, feedback_msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = AIdriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()