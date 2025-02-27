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
    position=({{ start_point[0] }}, {{ start_point[1] }}, 1.05),
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
        h1_with_hand_camera_cfg.model_copy(
            update={'name': 'camera', 'resolution': (512, 512), 'enable': True}, deep=True
        ),
        h1_with_hand_tp_camera_cfg.model_copy(
            update={'name': 'tp_camera', 'resolution': (512, 512), 'enable': True}, deep=True
        ),
    ],
)

config = Config(
    simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, rendering_interval=100, use_fabric=False),
    task_config=MobileManipulationTaskCfg(
        metrics=[MobileManipulationSuccessMetricCfg()],
        metrics_save_path='{{ metrics_save_path }}',
        task_settings=MobileManipulationTaskSetting(
            max_step={{ max_step }},
        ),
        episodes=[
            MobileManipulationEpisodeCfg(
                scene_asset_path=gm.ASSET_PATH
                + '/scenes/GRScenes-100/home_scenes/scenes/{{ scene_name }}/start_result_interaction.usd',
                scene_scale=(0.01, 0.01, 0.01),
                robots=[h1_1],
                extra={
                    'candidates': [
                        {% for candidate in candidates %}
                        '{{ candidate }}',{% endfor %}
                    ],
                    'conditions': [
                        {% for condition in conditions %}
                        {
                            'object_attribute': {
                                'appearance': {{ condition['object_attribute']['appearance'] }},
                                'category': '{{ condition['object_attribute']['category'] }}',
                                'instance_id': '{{ condition['object_attribute']['instance_id'] }}',
                                'relation': {{ condition['object_attribute']['relation'] }},
                                'room': {{ condition['object_attribute']['room'] }},
                            },
                            'sample_relation': '{{ condition['sample_relation'] }}',
                        },
                        {% endfor %}
                    ],
                    'episode_idx': {{ episode_idx }},
                    'instruction': '{{ instruction }}',
                    'meta_path': gm.ASSET_PATH + 'benchmark/meta/{{ scene_name }}',
                    'start_point': {{ start_point }},
                    'target': '{{ target }}',
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
"""


def parse_args():
    parser = argparse.ArgumentParser(
        description='Parse template variables to gen mobile manipulation benchmark episodes'
    )
    parser.add_argument(
        '--splits',
        type=str,
        default='validate',
        choices=['all', 'test', 'validate'],
        help='Splits to generate episodes for',
    )

    parser.add_argument('--asset_path', type=str, default='./', help='Path to the assets')

    parser.add_argument('--max_step', type=int, default=6000, help='Maximum steps of each episode')

    parser.add_argument('--metrics_save_path', type=str, default='GRUtopia/results/', help='Path to save metrics')
    parser.add_argument('--output_path', type=str, default='./mm_episodes', help='Path to save the generated episodes')

    return parser.parse_args()


def gen_episode(python_template: str, episode: dict, episode_idx: int, scene_name: str, name: str, args) -> str:
    current = scene_name + '_' + '_'.join(name.split('/')) + str(episode_idx) + '.py'
    with open(os.path.join('benchmark/meta', scene_name, 'paths.json'), 'r') as f:
        paths = json.load(f)
    start_point = paths[name][0]['start_point']
    template = Template(python_template)
    rendered_code = template.render(
        metrics_save_path=args.metrics_save_path,
        candidates=episode['candidates'],
        conditions=episode['condition'],
        max_step=args.max_step,
        scene_name=scene_name,
        episode_idx=episode_idx,
        start_point=start_point,
        instruction=episode['instruction'],
        target=name,
    )

    # write the rendered code to a file
    with open(os.path.join(args.output_path, current), 'w') as f:
        f.write(rendered_code)

    print(f'Python script {os.path.join(args.output_path, current)} has been generated!')


if __name__ == '__main__':
    args = parse_args()

    if not os.path.exists(args.output_path):
        os.makedirs(args.output_path)

    episode_path = os.path.join(args.asset_path, 'benchmark/mm_episodes.json')

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
