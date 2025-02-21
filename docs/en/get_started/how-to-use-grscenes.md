# How to Use GRscenes

## Where to Download

GRScenes has been publicly released on [**OpenXLab**](https://openxlab.org.cn/datasets/OpenRobotLab/GRScenes), [**ModelScope**](https://www.modelscope.cn/datasets/Shanghai_AI_Laboratory/GRScenes/summary) and [**HuggingFace**](https://huggingface.co/datasets/OpenRobotLab/GRScenes).

## Instructions

The GRScenes dataset provides nearly 100 high-quality scenes, covering home and commercial scenes. The dataset directory structure is as follows.

### GRScenes-100 Directory Structure

```
GRScenes-100/commercial_scenes.zip --(unzip)--> commercial_scenes
└── ...
GRScenes-100/home_scenes.zip --(unzip)--> home_scenes
├── Materials
│   └── ... (material mdl files and texture pictures)
├── models
│   ├── layout
│   │   ├── articulated
│   │   │   └── ... ( window, door, etc.)
│   │   └── others
│   │       └── ... (ceiling, wall, ground, etc.)
│   └── object
│       ├── articulated
│       │   └── ... (microwave, refrigerator, etc.)
│       └── others
│           └── ... (bed, bottle, cup, etc.)
└── scenes
    ├── MV7J6NIKTKJZ2AABAAAAADA8_usd
    │   ├── Materials -> ../../Materials
    │   ├── models    -> ../../models
    │   ├── metadata.json (records the referenced model and material paths)
    │   └── start_result_xxx.usd (scene usd files)
    └── ... (other scene folders)
```

- **Materials** folder contains mdl files and texture pictures. The mdl files, which are Material Definition Language files commonly used by rendering engines such as NVIDIA Omniverse. These mdl files are used with texture pictures to define the physically based material properties such as color, reflectivity, and transparency that can be applied to 3D objects.

- **models** folder contains 3D object models, where layouts objects under `layout/` and interactive objects under `object/`. Subdirectories are further categorized according to the model semantic labels such as `door` and `oven`.

- **scenes** folder (e.g., `MV7J6NIKTKJZ2AABAAAAADA8_usd/`) contains the following files:
  - **Scene USD Files**

  	We provides three usd files.
  	- **raw scene**, named as `start_result_raw.usd`, which defines the layout of the scene.
  	- **navigation scene**, named as `start_result_navigation.usd`, which used for navigation tasks.
  	- **interaction scene**, named as `start_result_interaction.usd`, which used for manipulation tasks.

  - **metadata.json**

  	This file records the metadata information of the models and materials referenced in the raw scene.

  - **interactive_obj_list.json**

  	This file records the prim paths of the interactive objects in the interaction scene.

### Usage

Please ensure that the [prerequisites](../../../toolkits/grscenes_scripts/README.md#prerequisites) has been configured before using GRScenes-100.

**1. Use the scene asset for your custom task**

Currently, we have provided two types of scene assets for navigation and manipulation tasks respectively. Users can get these scene assets from the GRScenes dataset, the scene asset path is typically like `.../GRScenes-100/home_scenes/scenes/{scene_id}/start_result_xxx.usd`. Please refer to the [demo](../../../grutopia/demo/) to learn where to input the parameter `scene_asset_path`.

**2. Use the raw dataset**

The dataset contains raw models and raw scenes. We recommend using [OpenUSD python SDK](https://openusd.org/release/intro.html) to apply physics APIs such as the rigid body and collider to the models. We also provide an example [preprocess](../../../toolkits/grscenes_scripts/preprocess.py) script to learn the detailed workflow of the physics property bindings. Besides, here are some other [tool scripts](../../../toolkits/grscenes_scripts/) for GRScenes-100.
