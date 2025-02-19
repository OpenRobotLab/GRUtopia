import json
import os
from typing import Any, Dict, List, Optional

import yaml
from pydantic import BaseModel

# Define some params
DATA_PATH = 'evaluate_data'
OUTPUT_FILE_PATH = 'GRUtopia/demo/configs'
AZURE_API_KEY_PATH = 'GRUtopia/grutopia_extension/agents/social_navigation_agent/modules/vlm/api_key/azure_api_key.txt'
AZURE_API_KEY_E_PATH = (
    'GRUtopia/grutopia_extension/agents/social_navigation_agent/modules/vlm/api_key/azure_api_key_e.txt'
)
ROBOT_TYPE = 'h1_with_hand'
ROBOT_SCALE = [1.0, 1.0, 1.0]
ROBOT_HEIGHT = 0 if ROBOT_TYPE == 'oracle' else 1.05
USE_AZURE = True
CONTINUE = True
metrics_save_path = 'GRUtopia/mm_results'
LAST_DATA = []
if CONTINUE and os.path.exists(metrics_save_path):
    # If the file exists, open and load the JSON content
    LAST_DATA = [f for f in os.listdir(metrics_save_path) if f.endswith('.json')]


# Define the EpisodeConfig class
class ControllerParams(BaseModel):
    """
    Controller config validator
    """

    name: str


class SensorParams(BaseModel):
    """
    Sensor config validator
    """

    name: str
    enable: Optional[bool] = True


class RobotUserConfig(BaseModel):
    name: str
    prim_path: str
    type: str
    position: List[float]
    scale: List[float]
    controller_params: Optional[List[ControllerParams]]
    sensor_params: Optional[List[SensorParams]] = None


class EpisodeConfig(BaseModel):
    scene_asset_path: Optional[str] = None
    scene_scale: Optional[List[float]] = [1.0, 1.0, 1.0]
    robots: Optional[List[RobotUserConfig]] = []
    extra: Optional[Dict[str, Any]] = {}


# Function to load episode data from the JSON file
def load_episode_data(
    base_path: str, scene_name: str, episode_file_path: str, path_file_path: str
) -> List[EpisodeConfig]:
    with open(episode_file_path, 'r') as f:
        episodes = json.load(f)
    with open(path_file_path, 'r') as file:
        paths = json.load(file)

    # Process each episode
    episode_configs = []
    i = 0
    for name, object_episodes in episodes.items():
        for episode_idx, episode in enumerate(object_episodes):
            instruction = episode['instruction']
            conditions = episode['condition']
            candidates = episode['candidates']
            start_point = paths[name][0]['start_point']
            current = scene_name + '_' + '_'.join(name.split('/')) + str(episode_idx) + '.json'
            if current in LAST_DATA:
                i += 1
                continue
            scene_asset_path = os.path.join(base_path, f'splits/{scene_name}/start_result_dialogue_new_all_mesh.usd')
            # scene_asset_path = os.path.join(base_path, f'splits/{scene_name}/start_result_dynamic.usd')
            scene_scale = [0.01, 0.01, 0.01]

            robots = [
                RobotUserConfig(
                    name='h1',
                    prim_path='/World/h1',
                    type='HumanoidRobot',
                    position=[*start_point[:2], ROBOT_HEIGHT],
                    scale=[1, 1, 1],
                    controller_params=[
                        ControllerParams(name='move_by_speed'),
                        ControllerParams(name='joint_controller'),
                        ControllerParams(name='move_to_point'),
                        ControllerParams(name='move_along_path'),
                        ControllerParams(name='rotate'),
                        ControllerParams(name='recover'),
                        ControllerParams(name='right_arm_joint_controller'),
                        ControllerParams(name='right_arm_ik_controller'),
                    ],
                    sensor_params=[
                        SensorParams(name='camera', enable=True),
                        SensorParams(name='tp_camera', enable=True),
                    ],
                )
            ]

            extra = {
                'episode_idx': episode_idx,
                'instruction': instruction,
                'candidates': candidates,
                'target': name,
                'start_point': start_point[:2],
                'conditions': conditions,
                'meta_path': os.path.join(base_path, f'meta/{scene_name}'),
                'use_azure': USE_AZURE,
                'azure_api_key_path': AZURE_API_KEY_PATH,
                'azure_api_key_e_path': AZURE_API_KEY_E_PATH,
            }

            episode_configs.append(
                EpisodeConfig(scene_asset_path=scene_asset_path, scene_scale=scene_scale, robots=robots, extra=extra)
            )
            # TORECOVER
    return episode_configs
    # return episode_configs, i


# Function to generate the YAML file
def generate_yaml_for_all_episodes(base_path: str, scene_name: str, i):
    all_episodes = []

    # Traverse the meta folder to find scenes with episodes_v4_mini.json
    # for scene_name in os.listdir(os.path.join(base_path, 'meta')):
    scene_path = os.path.join(base_path, 'meta', scene_name)
    episode_file_path = os.path.join(scene_path, 'interaction_instruction_mini_v1.json')
    path_file_path = os.path.join(scene_path, 'paths.json')

    # Check if the episode file exists
    if os.path.exists(episode_file_path):
        # TORECOVER
        episode_configs = load_episode_data(base_path, scene_name, episode_file_path, path_file_path)
        # episode_configs, num = load_episode_data(base_path, scene_name, episode_file_path, path_file_path)

        all_episodes.extend(episode_configs)
        # # TODELETE
        # if len(episode_configs) > 0:
        #     break

    # Write all episodes to a YAML file
    with open(f'{OUTPUT_FILE_PATH}/mm_episodes_config_{i}.yaml', 'w') as yaml_file:
        yaml.dump({'episodes': [episode.dict() for episode in all_episodes]}, yaml_file, default_flow_style=False)
    # # TODELETE
    # return num


if __name__ == '__main__':
    # Generate the YAML file
    generate_yaml_for_all_episodes(DATA_PATH)
    print(f'YAML file generated: {OUTPUT_FILE_PATH}')
