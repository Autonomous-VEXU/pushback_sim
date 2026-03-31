#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vex_interfaces.msg import GoalState  #type:ignore
from std_msgs.msg import Int64MultiArray

class Scoring(Node):
    '''
    VEX Push Back Scoring Breakdown (Blocks only):

    for each block in a goal - 3 pts
    majority top goal - 8 pts
    majority lower goal - 6 pts
    long goal control zone - 10 pts
    '''

    def __init__(self):
        super().__init__('scoring')

        # subscribe to all goal messages
        self.create_subscription(GoalState, '/goals', self.score_calculator, 10)

        # publish score and opponent score: [red_score, blue_score]
        self.score = self.create_publisher(Int64MultiArray, '/game_score', 10)  

        # globals for score tracking
        self.blue_score = None
        self.red_score = None

    def score_calculator(self, goals:GoalState):
        '''calculates the score of the game based on the number of blocks in each goal'''

        # reset scores before adding up blocks
        self.blue_score = 0
        self.red_score = 0

        goal_raw = [goals.long_b, goals.long_a, goals.center_low, goals.center_high]

        # adding up blocks in each goal (3 pts each)
        for goal in goal_raw:
            for ball in goal.object_array:
                if ball.color == 1:
                    self.red_score += 3
                elif ball.color == 2:
                    self.blue_score += 3
        
        # center low control bonus (6 pts)
        ctrl = goals.center_low_ctrl
        if ctrl == 1:
            self.red_score += 6
        elif ctrl == 2:
            self.blue_score += 6

        # center low control bonus (8 pts)
        ctrl = goals.center_high_ctrl
        if ctrl == 1:
            self.red_score += 8
        elif ctrl == 2:
            self.blue_score += 8
        
        # long goal A (10 pts)
        ctrl = goals.long_a_ctrl
        if ctrl == 1:
            self.red_score += 10
        elif ctrl == 2:
            self.blue_score += 10
        
        # long goal B (10 pts)
        ctrl = goals.long_b_ctrl
        if ctrl == 1:
            self.red_score += 10
        elif ctrl == 2:
            self.blue_score += 10

        score_array = Int64MultiArray()
        score_array.data = [self.red_score, self.blue_score]
        self.score.publish(score_array)

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