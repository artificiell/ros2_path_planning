# ros2_path_planning

# Path Planning Service

This is a ROS 2 service class that provides a path planning service for generating trajectories given the robot's position, target position, and an occupancy grid map. The service accepts a request containing the robot's position, target position, and the occupancy grid map, and responds with a trajectory.

## Installation

1. Clone the repository:
   ```
   git clone https://github.com/artificiell/ros2_path_planning.git
   ```

2. Build the package:
   ```
   cd ~/ros2_ws/
   colcon build --packages-select ros2_path_planning
   ```

## Usage

1. Run the path planning service node:
   ```
   ros2 run ros2_path_planning planner_service.py
   ```

2. Send a service request to the `plan_trajectory` service using your preferred method (e.g., ROS 2 service client or command-line tool).


## Service Definition

The path planning service expects a service request of type `PlanTrajectory` and returns a response of type `PlanTrajectory.Response`.

### Request

The request consists of the following fields:

- `robot_position` (geometry_msgs/Pose): The robot's current position.
- `target_position` (geometry_msgs/Pose): The desired target position.
- `grid_map` (nav_msgs/OccupancyGrid): The occupancy grid map representing the environment.

### Response

The response consists of the following field:

- `trajectory` (geometry_msgs/Pose[]): The generated trajectory as a list of poses.


## Customization

You can customize the path planning algorithm in the `service_callback` function of the `PathPlanningService` class. Implement your path planning algorithm based on the provided robot position, target position, and grid map. Modify the algorithm to generate the desired trajectory and update the `response` variable accordingly.


## License

This package is licensed under the MIT License. See the `LICENSE` file for more details.
A simple path planning ROS2 package.
