# Customize NPC with your algorithm

## Set configuration in task yaml file.

If you're using openai api, you can refer to [Web Demo](../get_started/webui.md).

If you are using other llm api service, you can set your api endpoint.

```yaml
npc:
- name: "npc_name"
  api_base_url: "api_endpoint"
    # other configurations
```

You can change the schema of NPC configuration in `grutopia.core.config.npc`.

## Customize your prompt and implement LLM caller

Our system message and in-context example are defined in `grutopia.npc.prompt`. And the LLM inference process are in `grutopia.npc.llm_caller`. You can customize them according to your own needs and algorithms.


## Reimplement `feed` method in `grutopia.npc.base`

Reimplement the `feed` method of `NPC` class in `grutopia.npc.base`.

In `feed` function, observation in `dict` type is processed and fed into the llm caller, new responses from llm caller are sent back to the robot.

Base `feed` method FYI.

```python
def feed(self, obs: dict):
    """feed npc with observation.

    Args:
        obs (dict): full observation of the world, with hierarchy of
            obs
            task_name:
                robot_name:
                position
                orientation
                controller_0
                controller_1
                ...
                sensor_0
                sensor_1
                ...
    """
    for task_obs in obs.values():
        for robot_obs in task_obs.values():
            chat = robot_obs.get('web_chat', None)
            if chat is not None and chat['chat_control']:
                # process observation
                position = robot_obs.get('position', None)
                orientation = robot_obs.get('orientation', None)
                bbox_label_data_from_camera = robot_obs['camera']['frame']['bounding_box_2d_tight']
                bbox = bbox_label_data['data']
                idToLabels = bbox_label_data['info']['idToLabels']
                # feed processed observation into llm caller
                # pick response from llm caller and send back to robot
                # details omitted

```

Then you can launch the web demo and chat with your NPC.
