from internutopia.core.config import Config, SimConfig
from internutopia.core.gym_env import Env
from internutopia.macros import gm
from internutopia_extension import import_extensions
from internutopia_extension.configs.metrics import RecordingMetricCfg
from internutopia_extension.configs.robots.mocap_controlled_franka import (
    MocapControlledFrankaRobotCfg,
    lh_controlled_camera_cfg,
    teleop_cfg,
)
from internutopia_extension.configs.tasks import ManipulationTaskCfg
from internutopia_extension.interactions.motion_capture import MocapInteraction

franka = MocapControlledFrankaRobotCfg(
    position=(-0.35, 0.0, 1.05),
    controllers=[
        teleop_cfg,
    ],
    sensors=[lh_controlled_camera_cfg],
)

config = Config(
    simulator=SimConfig(
        physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=False, headless=False, webrtc=False, native=True
    ),
    task_configs=[
        ManipulationTaskCfg(
            metrics=[
                RecordingMetricCfg(
                    robot_name='franka',
                    fields=['joint_action'],
                )
            ],
            scene_asset_path=gm.ASSET_PATH + '/scenes/demo_scenes/franka_mocap_teleop/table_scene.usd',
            robots=[franka],
            prompt='Prompt test 1',
            target='franka_manipulation_mocap_teleop',
            episode_idx=0,
            max_steps=10000,
        ),
    ],
)

import_extensions()

env = Env(config)
obs, _ = env.reset()

mocap_url = 'http://127.0.0.1:5001'
mocap_interaction = MocapInteraction(mocap_url)

while env.simulation_app.is_running():
    cur_mocap_info = mocap_interaction.step()
    arm_action = {teleop_cfg.name: [cur_mocap_info]}

    obs, _, _, _, _ = env.step(action=arm_action)

mocap_interaction.server_stop()
env.close()
