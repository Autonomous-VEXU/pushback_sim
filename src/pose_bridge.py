#!/usr/bin/env python3

import json
import subprocess
import rclpy
from rclpy.node import Node
from pushback_sim.msg import BallArray, Ball

class PoseBridge(Node):
    def __init__(self):
        super().__init__('pose_bridge')
        self.auto_update = self.create_timer(0.5, self.update_locations)
        self.ball_locations = self.create_publisher(BallArray, '/object_locations', 10)

        # world name for pose topic
        self.world_name = 'pushback'

        # TODO: make this a argument or just automatically get it from a launch file idc 

    def echo_gz_topic(self):

        result = subprocess.run(
            ["gz", "topic", "-e", "-t", f"world/{self.world_name}/dynamic_pose/info", "-n", "1", "--json-output"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        if result.returncode != 0:
            self.get_logger().error(f"Command failed with return code {result.returncode}: {result.stderr}")
            return {"pose": []}
        try:
            # only get ONE json item
            lines = result.stdout.strip().split('\n')
            for line in lines:
                if line.strip():
                    try:
                        data = json.loads(line)
                        return data  # first valid json
                    except json.JSONDecodeError:
                        continue  
            
            # error handling
            self.get_logger().warning("No valid JSON found in output")
            return {"pose": []}
            
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}, stdout: '{result.stdout}', stderr: '{result.stderr}'")
            return {"pose": []}

    def update_locations(self):
        objects = BallArray()
        data = self.echo_gz_topic() # contains one header and the rest are poses/object positions

        non_object_names = ['link', 'Otto', 'wheel_a', 'wheel_b', 'wheel_c', 'wheel_d']

        for pose in data.get("pose", []):
            if pose.get("name") not in non_object_names:
                position = pose.get("position", {})
                if "x" in position and "y" in position and "z" in position:
                    ball = Ball()
                    ball.object_name = pose.get("name", "")
                    ball.id = pose.get("id", 0)
                    ball.location.x = position["x"]
                    ball.location.y = position["y"]
                    ball.location.z = position["z"]
                    objects.object_array.append(ball)
    
        self.ball_locations.publish(objects)

def main(args=None):
    rclpy.init(args=args)
    node = PoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

