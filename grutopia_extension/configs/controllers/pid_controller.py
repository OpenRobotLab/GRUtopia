from typing import Any, Dict, List

import numpy as np
from omni.isaac.core.scenes import Scene
from omni.isaac.core.utils.rotations import quat_to_euler_angles
from omni.isaac.core.utils.types import ArticulationAction

from grutopia.core.robot.controller import BaseController
from grutopia.core.robot.robot import BaseRobot
from grutopia.core.robot.robot_model import ControllerModel


@BaseController.register('PIDSpeedController')
class PIDSpeedController(BaseController):
    """A PID-based speed controller for smoother target reaching."""

    def __init__(self, config: ControllerModel, robot: BaseRobot, scene: Scene) -> None:
        # Basic parameters
        self.goal_position = None
        self.last_threshold = None

        # Speed limits
        self.max_forward_speed = config.max_forward_speed if hasattr(config, 'max_forward_speed') else 2.0
        self.max_rotation_speed = config.max_rotation_speed if hasattr(config, 'max_rotation_speed') else 8.0
        self.threshold = config.threshold if hasattr(config, 'threshold') else 0.02

        # PID parameters
        self.Kp_linear = config.Kp_linear if hasattr(config, 'Kp_linear') else 1.5
        self.Ki_linear = config.Ki_linear if hasattr(config, 'Ki_linear') else 0.01
        self.Kd_linear = config.Kd_linear if hasattr(config, 'Kd_linear') else 0.01

        self.Kp_angular = config.Kp_angular if hasattr(config, 'Kp_angular') else 2.5
        self.Ki_angular = config.Ki_angular if hasattr(config, 'Ki_angular') else 0.0
        self.Kd_angular = config.Kd_angular if hasattr(config, 'Kd_angular') else 0.01

        # PID state variables
        self.linear_error_integral = 0.0
        self.angular_error_integral = 0.0
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0

        # Integral limit settings
        self.max_integral_angular = 1.0  # Maximum integral value
        self.integral_threshold = np.pi / 6  # Integral separation threshold

        super().__init__(config=config, robot=robot, scene=scene)

    @staticmethod
    def get_angle(start_position, start_orientation, goal_position):
        """Calculate the angle between current orientation and target direction."""
        normal_vec = np.array([0, 0, 1])
        robot_z_rot = quat_to_euler_angles(start_orientation)[-1]

        robot_vec = np.array([np.cos(robot_z_rot), np.sin(robot_z_rot), 0])
        robot_vec /= np.linalg.norm(robot_vec)

        target_vec = goal_position - start_position
        if np.linalg.norm(target_vec) == 0:
            return 0
        target_vec /= np.linalg.norm(target_vec)

        dot_prod = np.clip(np.dot(robot_vec, target_vec), -1.0, 1.0)
        cross_prod = np.cross(robot_vec, target_vec)

        angle = np.arccos(dot_prod)
        angle_sign = np.sign(np.dot(normal_vec, cross_prod))

        return angle * angle_sign

    def forward(
        self,
        start_position: np.ndarray,
        start_orientation: np.ndarray,
        goal_position: np.ndarray,
        dt: float = 4 / 200,
        forward_speed=None,
        rotation_speed=None,
        threshold=None,
    ) -> ArticulationAction:
        """Calculates forward and rotational speeds using PID control."""
        self.threshold = threshold
        # Ignore z-axis component
        start_position[-1] = 0
        goal_position[-1] = 0

        self.goal_position = goal_position

        # Calculate distance and angular errors
        dist_error = np.linalg.norm(start_position - goal_position)
        angle_error = self.get_angle(start_position, start_orientation, goal_position)

        # Stop advancing when facing a large error (over 90 degrees)
        if abs(angle_error) > np.pi / 3:
            forward_speed = 0
        else:
            # Linear velocity PID control
            self.linear_error_integral += dist_error * dt
            linear_error_derivative = (dist_error - self.prev_linear_error) / dt

            forward_speed = (
                self.Kp_linear * dist_error
                + self.Ki_linear * self.linear_error_integral
                + self.Kd_linear * linear_error_derivative
            )

        # Angular velocity PID control
        # Integral separation: no integration when error is too high
        if abs(angle_error) < self.integral_threshold:
            self.angular_error_integral += angle_error * dt
        else:
            self.angular_error_integral = 0

        # Integral limit
        self.angular_error_integral = np.clip(
            self.angular_error_integral, -self.max_integral_angular, self.max_integral_angular
        )

        # Reset integral if error changes direction
        if angle_error * self.prev_angular_error < 0:
            self.angular_error_integral = 0

        angular_error_derivative = (angle_error - self.prev_angular_error) / dt
        rotation_speed = (
            self.Kp_angular * angle_error
            + self.Ki_angular * self.angular_error_integral
            + self.Kd_angular * angular_error_derivative
        )

        # Update error records
        self.prev_linear_error = dist_error
        self.prev_angular_error = angle_error

        # Speed limiting and adjustment
        forward_speed = np.clip(forward_speed, -self.max_forward_speed, self.max_forward_speed)
        rotation_speed = np.clip(rotation_speed, -self.max_rotation_speed, self.max_rotation_speed)

        # Reduce rotation speed when the angular error is small (newly added)
        # angle_threshold = np.pi / 6  # 30 degrees
        # if abs(angle_error) < angle_threshold:
        #     rotation_speed *= (abs(angle_error) / angle_threshold) ** 3

        # Lower forward speed significantly when the angular error is large (using cubic for more noticeable effect)
        forward_speed *= (1 - (abs(angle_error) * 2 / np.pi)) ** 3

        # Slow down approaching the target (use cubic for smoother deceleration)
        # if dist_error < self.threshold * 2:
        #     forward_speed *= (dist_error / (self.threshold * 2))**3

        # Stop upon reaching the target point
        if dist_error < self.threshold:
            self.reset_pid_states()
            return self.sub_controllers[0].forward(
                forward_speed=0.0,
                rotation_speed=0.0,
            )

        return self.sub_controllers[0].forward(
            forward_speed=forward_speed,
            rotation_speed=rotation_speed,
        )

    def reset_pid_states(self):
        """Resets PID controller states."""
        self.linear_error_integral = 0.0
        self.angular_error_integral = 0.0
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0

    def action_to_control(self, action: List | np.ndarray) -> ArticulationAction:
        """Converts input actions to joint control signals."""
        assert len(action) == 1, 'action must contain 1 elements'
        start_position, start_orientation = self.robot.get_world_pose()
        return self.forward(
            start_position=start_position, start_orientation=start_orientation, goal_position=np.array(action[0])
        )

    def get_obs(self) -> Dict[str, Any]:
        """Gets controller status information."""
        if self.goal_position is None:
            return {}
        start_position = self.robot.get_world_pose()[0]
        start_position[-1] = 0
        dist_from_goal = np.linalg.norm(start_position - self.goal_position)
        finished = dist_from_goal < self.threshold
        return {'finished': finished, 'distance_to_goal': dist_from_goal}
