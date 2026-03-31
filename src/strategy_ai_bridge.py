import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseArray
from vex_interfaces.msg import WorldState, BallArray, Loader, ControlZone, GoalState

class StrategyAIBridge(Node):
    def __init__(self):
        super().__init__('sai_bridge')

        # subscribe to score, robot pose, objects, goal states, etc
        self.create_subscription(Int32MultiArray, '/game_score', self.score_cb, 10)
        self.create_subscription(GoalState, '/goals', self.goal_ctrl_zone_cb, 10)
        self.create_subscription(PoseArray,'/otto_pose', self.robot_pose_cb)
        self.create_subscription(BallArray, '/robot_intake_status', self.intake_cb)

        # timer for controlling publishing rate
        self.create_timer(1.0, self.update_sai_world_state)

        # publisher for the world state on the /sai_input topic
        self.to_sai = self.create_publisher(WorldState, '/sai_input', 10)

        # globals for storing current state
        self.world_state = WorldState()
    
    def score_cb(self, msg): 
        self.world_state.score = msg
            
    def goal_ctrl_zone_cb(self, msg): 
        '''updates the goals and control zone part of the world state'''
        self.world_state.goals = msg


    def ctrl_zone_cb(self, msg): 
        pass

    def robot_pose_cb(self, msg): 
        pass

    def intake_cb(self, msg): 
        pass
    
    def update_sai_world_state(self): 
        # build header msg
        self.world_state.header.stamp = self.get_clock().now().to_msg()
        self.world_state.header.frame_id = 'map'

        # publish new world state
        self.to_sai.publish(self.world_state)
        self.get_logger().info("updated world state!")


def main(args=None):
    rclpy.init(args=args)
    node = StrategyAIBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()