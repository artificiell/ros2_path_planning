#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros2_path_planning.srv import PlanTrajectory

import numpy as np
from heapq import heappop, heappush

# Path planning service class
class PathPlanningService(Node):

    def __init__(self):
        super().__init__('path_planning_service')

        # Setup ROS service
        self.srv = self.create_service(PlanTrajectory, 'plan_trajectory', self.service_callback)

    # Callback function for handle a service call
    def service_callback(self, request, response):
        self.get_logger().info(f'Got positions: robot = {request.robot_position.position}, target = {request.target_position.position}')
        self.get_logger().info(f'Got map dimensions: width = {request.grid_map.info.width}, height = {request.grid_map.info.height}')

        # -----------------------------
        # Add your path planning algorithm here.
        # ----------------------------------------

        # Send resutling trajectory 
        return response

# Main function
def main(args = None):
    rclpy.init(args = args)

    # Create and run the service
    planner = PathPlanningService()
    rclpy.spin(planner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
