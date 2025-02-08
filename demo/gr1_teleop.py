from grutopia.core.config import Config, SimConfig
from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.util.container import is_in_container
from grutopia_extension import import_extensions
from grutopia_extension.configs.objects import DynamicCubeCfg, UsdObjCfg
from grutopia_extension.configs.robots.gr1 import (
    GR1RobotCfg,
    camera_left_cfg,
    camera_right_cfg,
    teleop_cfg,
)
from grutopia_extension.configs.tasks import (
    SingleInferenceEpisodeCfg,
    SingleInferenceTaskCfg,
)

headless = False
webrtc = False

if is_in_container():
    headless = True
    webrtc = True

table_cfg = UsdObjCfg(
    name='table',
    prim_path='/World/table',
    scale=(0.001, 0.001, 0.001),
    position=(0.0, 0.0, 0.374),
    usd_path='GRUtopia/assets/objects/table/white_big/instance.usda',
)

cube_cfgs = [
    DynamicCubeCfg(
        name='target_cube_0',
        prim_path='/World/target_cube_0',
        position=(-0.28, 0, 0.8),
        scale=(0.2, 0.2, 0.05),
        color=(0.0, 0.5, 0.0),
    ),
    DynamicCubeCfg(
        name='target_cube_1',
        prim_path='/World/target_cube_1',
        position=(-0.24, -0.2, 0.8),
        scale=(0.05, 0.15, 0.08),
        color=(0.5, 0.0, 0.0),
    ),
    DynamicCubeCfg(
        name='target_cube_2',
        prim_path='/World/target_cube_2',
        position=(-0.24, 0.2, 0.8),
        scale=(0.05, 0.15, 0.08),
        color=(0.0, 0.0, 0.5),
    ),
]

config = Config(
    simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=False),
    task_config=SingleInferenceTaskCfg(
        episodes=[
            SingleInferenceEpisodeCfg(
                scene_asset_path='GRUtopia/assets/scenes/empty.usd',
                robots=[
                    GR1RobotCfg(
                        position=(-0.68, 0.0, 0.82),
                        controllers=[teleop_cfg],
                        sensors=[camera_left_cfg, camera_right_cfg],
                    )
                ],
                objects=[table_cfg] + cube_cfgs,
            ),
        ],
    ),
)

sim_runtime = SimulatorRuntime(config_class=config, headless=headless, webrtc=webrtc)

from typing import List, Tuple

from omni.isaac.core.prims import RigidPrim

from grutopia.core.util.joint import create_joint
from grutopia_extension.interactions.visionpro.visionpro import VuerTeleop

import_extensions()
# import custom extensions here.

env = Env(sim_runtime)

obs, _ = env.reset()

import numpy as np

actions = {}

teleop = VuerTeleop(
    cert_file='./GRUtopia/mkcert/cert.pem', key_file='./GRUtopia/mkcert/key.pem', resolution=(720, 1280)
)

cubes: List[RigidPrim] = []
original_poses: List[Tuple[np.ndarray, np.ndarray]] = []
for cube_cfg in cube_cfgs:
    cube: RigidPrim = env.runner.get_obj(cube_cfg.name)
    cubes.append(cube)
    original_poses.append(cube.get_world_pose())

cubes[0].set_mass(5.0)


table: RigidPrim = env.runner.get_obj(table_cfg.name)
table.set_mass(100.0)

i = 0
while env.simulation_app.is_running():
    i += 1

    # create a fixed joint to ensure stable vision.
    if i == 1:
        create_joint(
            prim_path='/fix_joint',
            body0='/World/env_0/robots/World/gr1/base_link',
            joint_type='FixedJoint',
            enabled=True,
        )

    # shapes:
    # - head_mat: (4, 4)
    # - left_wrist_mat: (4, 4)
    # - right_wrist_mat: (4, 4)
    # - left_hand_mat: (25, 3)
    # - right_hand_mat: (25, 3)
    head_mat, left_wrist_mat, right_wrist_mat, left_hand_mat, right_hand_mat, begin_move = teleop.step()

    actions[teleop_cfg.name] = (left_wrist_mat, right_wrist_mat, head_mat, left_hand_mat, right_hand_mat)

    obs, _, terminated, _, _ = env.step(action=actions)

    image_left: np.ndarray = obs['sensors']['camera_left']['rgba']
    image_right: np.ndarray = obs['sensors']['camera_right']['rgba']

    if image_left.shape[0] == 0:
        continue

    image_left = image_left[:, :, :3]
    image_right = image_right[:, :, :3]

    frame = np.hstack((image_left, image_right)).astype(np.uint8)
    np.copyto(teleop.img_array, frame)

    if i % 500 == 0:
        print(i)

teleop.cleanup()
env.simulation_app.close()
