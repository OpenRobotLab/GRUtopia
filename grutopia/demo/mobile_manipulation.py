from grutopia.core.config import Config, SimConfig
from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia.macros import gm
from grutopia_extension import import_extensions
from grutopia_extension.agents.config import AgentCfg
from grutopia_extension.agents.util.agent import create_agent
from grutopia_extension.configs.metrics import MobileManipulationSuccessMetricCfg
from grutopia_extension.configs.robots.h1_with_hand import (
    H1WithHandRobotCfg,
    h1_with_hand_camera_cfg,
    h1_with_hand_tp_camera_cfg,
    joint_controller,
    move_along_path_cfg,
    move_by_speed_cfg,
    move_to_point_cfg,
    recover_cfg,
    right_arm_ik_controller_cfg,
    right_arm_joint_controller_cfg,
    rotate_cfg,
)
from grutopia_extension.configs.tasks import (
    MobileManipulationEpisodeCfg,
    MobileManipulationTaskCfg,
    MobileManipulationTaskSetting,
)

# AgentConfig
h1_1 = H1WithHandRobotCfg(
    position=(8.482455253601074, -0.8219017386436462, 1.05),
    controllers=[
        move_by_speed_cfg,
        joint_controller,
        recover_cfg,
        rotate_cfg,
        move_along_path_cfg,
        move_to_point_cfg,
        right_arm_joint_controller_cfg,
        right_arm_ik_controller_cfg,
    ],
    sensors=[
        h1_with_hand_camera_cfg.update(name='camera', resolution=(512, 512), enable=True),
        h1_with_hand_tp_camera_cfg.update(name='tp_camera', resolution=(512, 512), enable=True),
    ],
)

config = Config(
    simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, rendering_interval=100, use_fabric=False),
    task_config=MobileManipulationTaskCfg(
        metrics=[MobileManipulationSuccessMetricCfg()],
        metrics_save_path='GRUtopia/results/mm_result.json',
        task_settings=MobileManipulationTaskSetting(
            max_step=6000,
        ),
        episodes=[
            MobileManipulationEpisodeCfg(
                scene_asset_path=gm.ASSET_PATH
                + '/scenes/GRScenes-100/home_scenes/scenes/MV7J6NIKTKJZ2AABAAAAADA8_usd/start_result_interaction.usd',
                scene_scale=(0.01, 0.01, 0.01),
                robots=[h1_1],
                extra={
                    'candidates': ['bowl/SM_38_6JLDCIZVAZWGUPTUJQ888888'],
                    'conditions': [
                        {
                            'object_attribute': {
                                'appearance': None,
                                'category': 'nightstand',
                                'instance_id': 'nightstand/model_158effa8b26bf0fd5044704d5b91f1d5_0',
                                'relation': [[True, 'near', 'cabinet']],
                                'room': None,
                            },
                            'sample_relation': 'on',
                        },
                        {
                            'object_attribute': {
                                'appearance': None,
                                'category': 'pillow',
                                'instance_id': 'pillow/model_fd19fbe9bc625e175917e78b747bd35b_0',
                                'relation': [[True, 'near', 'cabinet']],
                                'room': None,
                            },
                            'sample_relation': 'nearby',
                        },
                    ],
                    'episode_idx': 0,
                    'instruction': 'Pick the bowl located near the window and place it on the nightstand near the pillow and the cabinet.',
                    'meta_path': gm.ASSET_PATH + 'benchmark/meta/MV7J6NIKTKJZ2AABAAAAADA8_usd',
                    'start_point': [4.983360290527344, 2.8359479904174805],
                    'target': 'bowl/model_a3dd96e3fed88481d94f5b8c727e3c6c_0',
                },
            ),
        ],
    ),
)

mm_agent_cfg = AgentCfg(
    type='MobileManipulationAgent',
    robot_name='h1',
)

sim_runtime = SimulatorRuntime(config_class=config, headless=True, webrtc=False, native=True)
print(sim_runtime.config)

import_extensions()
# import custom extensions here.

env = Env(sim_runtime)
obs, info = env.reset()
print(f'========INIT OBS{obs}=============')

# init agent
mm_agent = create_agent(agent_cfg=mm_agent_cfg, reset_info=info)

# init param
i = 0
task_finished = False

while env.simulation_app.is_running() and not env.finished():
    i += 1
    action = mm_agent.step(obs)

    if 'terminate' in action:
        obs, info = env.reset()
        if env.RESET_INFO_TASK_RUNTIME not in info:  # No more episode
            break

    obs, _, terminated, _, _ = env.step(action=action)
    task_finished = terminated

    if task_finished:
        obs, info = env.reset()
        if env.RESET_INFO_TASK_RUNTIME not in info:  # No more episode
            break

    if i % 1000 == 0:
        print(i)

env.close()
