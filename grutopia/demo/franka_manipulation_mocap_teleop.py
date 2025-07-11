from grutopia.core.config import Config, SimConfig
from grutopia.core.gym_env import Env
from grutopia.macros import gm
from grutopia_extension import import_extensions
from grutopia_extension.configs.metrics import RecordingMetricCfg
from grutopia_extension.configs.robots.mocap_controlled_franka import (
    MocapControlledFrankaRobotCfg,
    lh_controlled_camera_cfg,
    teleop_cfg,
)
from grutopia_extension.configs.tasks import (
    ManipulationEpisodeCfg,
    ManipulationExtra,
    ManipulationTaskCfg,
    ManipulationTaskSetting,
)
from grutopia_extension.interactions.motion_capture import MocapInteraction

franka = MocapControlledFrankaRobotCfg(
    position=[-0.35, 0.0, 1.05],
    controllers=[
        teleop_cfg,
    ],
    sensors=[lh_controlled_camera_cfg],
)

config = Config(
    simulator=SimConfig(
        physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=False, headless=False, webrtc=False, native=True
    ),
    task_config=ManipulationTaskCfg(
        metrics=[
            RecordingMetricCfg(
                robot_name='franka',
                fields=['joint_action'],
            )
        ],
        episodes=[
            ManipulationEpisodeCfg(
                scene_asset_path=gm.ASSET_PATH + '/scenes/demo_scenes/franka_mocap_teleop/table_scene.usd',
                robots=[franka],
                extra=ManipulationExtra(
                    prompt='Prompt test 1',
                    target='franka_manipulation_mocap_teleop',
                    episode_idx=0,
                ),
            ),
        ],
        task_settings=ManipulationTaskSetting(max_step=10000),
    ),
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
