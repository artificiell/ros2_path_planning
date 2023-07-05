#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros2_path_planning.srv import PlanTrajectory
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid

# Path planning client class
class PathPlanningClientNode(Node):
    def __init__(self):
        super().__init__('path_planning_client_node')

        # Setup ROS service client
        self.cli = self.create_client(PlanTrajectory, 'plan_trajectory')
        while not self.cli.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create service request object 
        self.req = PlanTrajectory.Request()

        # Setup timed callback 
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(2.0, self.send_request)

    # Send service request
    def send_request(self):

        # Set the robot position and target position
        self.req.robot_position = Pose()
        self.req.target_position = Pose()

        # Set the occupancy grid map
        self.req.grid_map = OccupancyGrid()

        # Send the request
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

    # Receive service response
    def get_response(self):
        try:
            response = self.future.result()
            self.get_logger().info(f'Received trajectory: {response.trajectory}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    # Check service response
    def back_from_the_future(self):
        return self.cli.service_is_ready()        
            
# Main function
def main(args = None):
    rclpy.init(args = args)

    # Create and run service client
    client = PathPlanningClientNode()
    client.send_request()
    while rclpy.ok():
        if client.back_from_the_future():
            client.get_response()
        rclpy.spin_once(client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
