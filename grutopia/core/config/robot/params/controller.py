from typing import List, Optional

from pydantic import BaseModel


class ControllerParams(BaseModel):
    """
    Controller config validator
    """
    name: str
    joint_names: Optional[List[str]]
    map_data_path: Optional[str]  # navi only, npy BOG (binary occupancy grid) file
    reference: Optional[str]  # ik only, world/robot/arm_base, default to world
    threshold: Optional[float]  # threshold to judge if action has been finished.

    # Planner controller
    planner: Optional[str]  # for planning policy.
    model: Optional[str]  # for planning policy model
    model_path: Optional[str]  # for planning policy, weight path of model
