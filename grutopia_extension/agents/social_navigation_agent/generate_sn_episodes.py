import argparse
import json
import os

from jinja2 import Template

python_template = """
from grutopia.core.config import Config, SimConfig
from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia.macros import gm
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
from grutopia_extension.configs.robots.h1 import (
    H1RobotCfg,
    h1_camera_cfg,
    h1_tp_camera_cfg,
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
h1_1 = H1RobotCfg(
    position=({{ start_point[0] }}, {{ start_point[1] }}, 1.05),
    controllers=[move_by_speed_cfg, joint_controller, recover_cfg, rotate_cfg, move_along_path_cfg, move_to_point_cfg],
    sensors=[
        h1_camera_cfg.model_copy(update={'name': 'camera', 'resolution': (512, 512), 'enable': True}, deep=True),
        h1_tp_camera_cfg.model_copy(
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
                    'azure_api_key_e_path': '{{ azure_api_key_e_path }}',
                    'azure_api_key_path': '{{ azure_api_key_path }}',
                    'captions_embeddings_path': gm.ASSET_PATH + 'benchmark/object_captions_embeddings.pkl',
                    'captions_path': gm.ASSET_PATH + 'benchmark/object_captions_score_sort.json',
                    'use_azure': True,
                }
            ),
            ResetTimeMetricCfg(),
            DebugMetricCfg(),
            SocialNavigationSuccessMetricCfg(
                metric_config=SocialNavigationSuccessMetricConfig(navigation_error_threshold=3)
            ),
        ],
        metrics_save_path='{{ metrics_save_path }}',
        task_settings=SocialNavigationTaskSetting(
            max_step={{ max_step }},
        ),
        episodes=[
            SocialNavigationEpisodeCfg(
                scene_asset_path=gm.ASSET_PATH
                + '/scenes/GRScenes-100/home_scenes/scenes/{{ scene_name }}.usd',
                scene_scale=(0.01, 0.01, 0.01),
                robots=[h1_1],
                extra={
                    'distance': {{ distance }},
                    'episode_idx': {{ episode_idx }},
                    'model_mapping_path': gm.ASSET_PATH
                    + 'benchmark/meta/{{ scene_name }}/model_mapping.json',
                    'npc_scene_data_config': gm.ASSET_PATH
                    + 'benchmark/meta/{{ scene_name }}/object_dict_with_caption.json',
                    'object_dict_path': gm.ASSET_PATH + 'benchmark/meta/{{ scene_name }}/object_dict.json',
                    'question': '{{ question }}',
                    'start_point': {{ start_point }},
                    'target': '{{ target }}',
                    'target_point': {{ target_point }},
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
        'openai_api_key': '{{ openai_api_key_path }}',
        'api_base_url': '{{ api_base_url }}',
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
"""


def parse_args():
    parser = argparse.ArgumentParser(description='Parse template variables')
    parser.add_argument(
        '--splits',
        type=str,
        default='validate',
        choices=['all', 'test', 'validate'],
        help='Splits to generate episodes for',
    )

    parser.add_argument('--asset_path', type=str, default='./', help='Path to the assets')

    parser.add_argument('--max_step', type=int, default=6000, help='Maximum steps of each episode')

    parser.add_argument(
        '--api_base_url', type=str, default='https://gpt-4o-pjm.openai.azure.com/', help='Base URL for the API'
    )
    parser.add_argument(
        '--openai_api_key_path',
        type=str,
        default='GRUtopia/grutopia_extension/agents/social_navigation_agent/modules/vlm/api_key/azure_api_key.txt',
        help='Path to the OpenAI API key',
    )
    parser.add_argument(
        '--azure_api_key_e_path',
        type=str,
        default='GRUtopia/grutopia_extension/agents/social_navigation_agent/modules/vlm/api_key/azure_api_key_e.txt',
        help='Path to the Azure API key E',
    )
    parser.add_argument(
        '--azure_api_key_path',
        type=str,
        default='GRUtopia/grutopia_extension/agents/social_navigation_agent/modules/vlm/api_key/azure_api_key.txt',
        help='Path to the Azure API key',
    )

    parser.add_argument('--metrics_save_path', type=str, default='GRUtopia/results/', help='Path to save metrics')
    parser.add_argument('--output_path', type=str, default='./sn_episodes', help='Path to save the generated episodes')

    return parser.parse_args()


def gen_episode(python_template: str, episode: dict, episode_idx: int, scene_name: str, name: str, args) -> str:
    question = episode['dialogue'][0]['dialog']
    current = scene_name + '_' + '_'.join(name.split('/')) + str(episode_idx) + '.py'
    template = Template(python_template)

    rendered_code = template.render(
        metrics_save_path=args.metrics_save_path,
        openai_api_key_path=args.openai_api_key_path,
        api_base_url=args.api_base_url,
        azure_api_key_path=args.azure_api_key_path,
        azure_api_key_e_path=args.azure_api_key_e_path,
        max_step=args.max_step,
        scene_name=scene_name,
        episode_idx=episode_idx,
        start_point=episode['start_point'],
        target_point=episode['target_point'],
        question=question,
        target=name,
        distance=episode['distance'],
    )

    # write the rendered code to a file
    with open(os.path.join(args.output_path, current), 'w') as f:
        f.write(rendered_code)

    print(f'Python script {os.path.join(args.output_path, current)} has been generated!')


if __name__ == '__main__':
    args = parse_args()

    if not os.path.exists(args.output_path):
        os.makedirs(args.output_path)

    episode_path = os.path.join(args.asset_path, 'benchmark/sn_episodes.json')

    with open(episode_path, 'r') as f:
        episodes_data = json.load(f)

    if args.splits == 'all':
        all_scenes_episodes = episodes_data['test'].update(episodes_data['validate'])
    else:
        all_scenes_episodes = episodes_data[args.splits]

    for scene_name, episodes in all_scenes_episodes.items():
        for name, object_episodes in episodes.items():
            for episode_idx, episode in enumerate(object_episodes):
                gen_episode(python_template, episode, episode_idx, scene_name, name, args)
