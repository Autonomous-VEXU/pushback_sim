import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

class Opponent(Node):
    def __init__(self):
        super().__init__('opponent')

        # subscribe to strategy opponent topic
        self.create_subscription()

        self.create_client()

    def data_processing(self, msg):
        # msg is currently undefined due to not knowing the strategy AI output
        pass

    def move_to_pose(self, pose:Pose2D):
        pass

    def pickup_ball(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = Opponent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()