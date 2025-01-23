from typing import List, Optional

from grutopia.core.config.robot import ControllerModel


class HumanoidMoveBySpeedControllerModel(ControllerModel):
    """
    Represents a controller model for humanoid movement based on speed.

    This model facilitates the control of a humanoid robot's locomotion by specifying various speed parameters and configurations.
    It is designed to integrate with robotic systems, enabling the robot to move forward, rotate, or move laterally at designated speeds,
    with additional settings for fine-tuning behavior.

    Attributes:
        joint_names (Optional[List[str]]): The names of joints that the controller will manage. Defaults to None.
        policy_weights_path (Optional[str]): Path to the policy weights file, used for decision-making algorithms. Defaults to None.
    """

    joint_names: Optional[List[str]] = None
    policy_weights_path: Optional[str] = None
