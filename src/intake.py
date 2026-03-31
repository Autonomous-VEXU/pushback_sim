#!/usr/bin/env python3

from rclpy.node import Node
import queue
from vex_interfaces.msg import BallArray
from vex_interfaces.srv import IntakeBall
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from ros_gz_interfaces.msg import Entity


class RobotIntake(Node):
    def __init__(self, matchload:bool):
        super().__init__('intake')

        # gazebo services
        self.remove_ball = self.create_client(DeleteEntity, '/world/pushback/remove')
        self.spawn_ball = self.create_client(SpawnEntity, '/world/pushback/create')

        # intaking a ball service
        self.intake_ball = self.create_service(IntakeBall, '/robot_intake', self.intake_ball)

        # publishing the current state of the intake
        self.robot_hopper_status = self.create_publisher(BallArray, '/robot_intake_status', 10)

        self.hopper_limit = 12
        self.matchload = matchload

        if self.matchload == True:
            self.robot_intake:queue = [1]
        else:
            self.robot_intake:queue = []

    def intake_ball(self, request, response): # service
        '''/robot_collisions callback function'''
        if len(self.robot_intake) < 12:
            self.remove_ball(request.ball_id)
            self.robot_intake.append(1)
            self.update_intake_gui() # actually build out later
        else:
            self.get_logger().info("Hopper full")

    def remove_ball(self, id:int):
        ball = Entity()
        ball.id = id # copy id of picked up ball to entity

        delete_req = DeleteEntity.Request()
        delete_req.entity = ball

        self.remove_ball.call_async(delete_req)

