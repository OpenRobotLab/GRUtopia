from grutopia.core.config import Config, SimConfig
from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia_extension import import_extensions
from grutopia_extension.agents.config import AgentCfg
from grutopia_extension.agents.util.agent import create_agent
from grutopia_extension.configs.metrics import (
    DebugMetricCfg,
    ECRMetricCfg,
    ResetTimeMetricCfg,
    SocialNavigationSuccessMetricCfg,
    SocialNavigationSuccessMetricConfig,
)
from grutopia_extension.configs.robots.humanoid import (
    HumanoidRobotCfg,
    humanoid_camera_cfg,
    humanoid_tp_camera_cfg,
    joint_controller,
    move_along_path_cfg,
    move_by_speed_cfg,
    move_to_point_cfg,
    recover_cfg,
    rotate_cfg,
)
from grutopia_extension.configs.tasks import (
    SocialNavigationEpisodeCfg,
    SocialNavigationTaskCfg,
    SocialNavigationTaskSetting,
)

# AgentConfig
h1_1 = HumanoidRobotCfg(
    position=(-13.947606086730957, 0.1635608822107315, 1.05),
    controllers=[move_by_speed_cfg, joint_controller, recover_cfg, rotate_cfg, move_along_path_cfg, move_to_point_cfg],
    sensors=[
        humanoid_camera_cfg.model_copy(update={'name': 'camera', 'resolution': (512, 512), 'enable': True}, deep=True),
        humanoid_tp_camera_cfg.model_copy(
            update={'name': 'tp_camera', 'resolution': (512, 512), 'enable': True}, deep=True
        ),
    ],
)

config = Config(
    simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, rendering_interval=100, use_fabric=False),
    task_config=SocialNavigationTaskCfg(
        metrics=[
            ECRMetricCfg(
                metric_config={
                    'azure_api_key_e_path': 'GRUtopia/grutopia_extension/agents/social_navigation_agent/modules/vlm/api_key/azure_api_key_e.txt',
                    'azure_api_key_path': 'GRUtopia/grutopia_extension/agents/social_navigation_agent/modules/vlm/api_key/azure_api_key.txt',
                    'captions_embeddings_path': 'evaluate_data/object_captions_embeddings.pkl',
                    'captions_path': 'evaluate_data/object_captions_score_sort.json',
                    'use_azure': True,
                }
            ),
            ResetTimeMetricCfg(),
            DebugMetricCfg(),
            SocialNavigationSuccessMetricCfg(
                metric_config=SocialNavigationSuccessMetricConfig(navigation_error_threshold=3)
            ),
        ],
        metrics_save_path='GRUtopia/results/sn_result.json',
        task_settings=SocialNavigationTaskSetting(
            max_step=6000,
        ),
        episodes=[
            SocialNavigationEpisodeCfg(
                scene_asset_path='evaluate_data/splits/MV7J6NIKTKJZ2AABAAAAADY8_usd/start_result_navigation.usd',
                scene_scale=(0.01, 0.01, 0.01),
                robots=[h1_1],
                extra={
                    'distance': 9.110975175263214,
                    'episode_idx': 0,
                    'model_mapping_path': 'evaluate_data/meta/MV7J6NIKTKJZ2AABAAAAADY8_usd/model_mapping.json',
                    'npc_scene_data_config': 'evaluate_data/meta/MV7J6NIKTKJZ2AABAAAAADY8_usd/object_dict_with_caption.json',
                    'object_dict_path': 'evaluate_data/meta/MV7J6NIKTKJZ2AABAAAAADY8_usd/object_dict.json',
                    'question': 'Can you help me locate the chest of drawers that is near a curtain?',
                    'start_point': [-13.947606086730957, 0.1635608822107315],
                    'target': 'chestofdrawers/model_7434427118680313ebf8e9cb10677aec_0',
                    'target_point': [-7.7889326380667345, 3.6017182993634376, 0.48993474030060125],
                },
            ),
        ],
    ),
)

sn_agent_cfg = AgentCfg(
    type='SocialNavigationAgent',
    robot_name='h1',
)
npc_agent_cfg = AgentCfg(
    type='NPCAgent',
    robot_name='npc',
    agent_config={
        'model_name': 'gpt-4o',
        'openai_api_key': 'GRUtopia/grutopia_extension/agents/social_navigation_agent/modules/vlm/api_key/azure_api_key.txt',
        'api_base_url': 'https://gpt-4o-pjm.openai.azure.com/',
    },
)

sim_runtime = SimulatorRuntime(config_class=config, headless=True, webrtc=False, native=True)

import_extensions()
# import custom extensions here.

env = Env(sim_runtime)
obs, info = env.reset()
print(f'========INIT OBS{obs}=============')

# init agent
npc_agent = create_agent(agent_cfg=npc_agent_cfg, reset_info=info)
sn_agent = create_agent(agent_cfg=sn_agent_cfg, reset_info=info)

# init param
i = 0
task_finished = False

while env.simulation_app.is_running() and not env.finished():
    i += 1
    npc_agent.step(
        obs
    )  # No action is returned since npc only communicates with sn_agent through internal shared buffer
    action = sn_agent.step(obs)

    if 'terminate' in action:
        env.reset()

    obs, _, terminated, _, _ = env.step(action=action)
    task_finished = terminated

    if task_finished:
        env.reset()

    if i % 1000 == 0:
        print(i)

env.close()
