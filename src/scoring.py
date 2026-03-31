import rclpy
from rclpy.node import Node
from pushback_sim.msg import GoalState
from std_msgs.msg import Int32MultiArray

class Scoring(Node):
    def __init__(self):
        super().__init__('scoring')

        # subscribe to all goal messages
        self.create_subscription(GoalState, '/goals', self.score_goals)

        # publish score and opponent score
        self.score = self.create_publisher(Int32MultiArray, '/current_score',)


def main(args=None):
    rclpy.init(args=args)
    node = Scoring()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()