# Explore GRscenes

## Download

You can refer to the chapter [Prepare Assets](./installation.md#prepare-assets) in the installation doc to download the GRscenes.

## Overview of dataset

The directory structure of dataset is as following:

```
GRScenes
├── benchmark
│   ├── meta.zip
│   ├── object_captions_embeddings.pkl
│   └── ...
├── metafile.yaml
├── objects
│   ├── table
│   └── ...
├── README.md
├── robots
│   ├── aliengo/
│   ├── franka/
│   ├── g1/
│   ├── gr1/
│   ├── h1/
│   └── ...
└── scenes
    ├── empty.usd
    ├── demo_scenes/
    └── GRScenes-100/
```

- benchmark: some metadata files required for running benchmark baselines.
- objects: USD files of standalone objects.
- robots: USD files of robots and model weights for controllers.
- scenes:
  - empty.usd: a minimum scene with ground plane.
  - demo_scenes: a directory containing scenes for demo scripts.
  - GRScenes-100: a directory containing nearly 100 high-quality scenes, covering home and commercial scenes. You can refer to [GRScenes-100](#grscenes-100) chapter for detailed information.

## GRScenes-100

Directory structure of GRScenes-100 is as following:

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

## Usage

1. Use the scene asset for your custom task

Currently, we have provided two types of scene assets for navigation and manipulation tasks respectively. Users can get these scene assets from the GRScenes dataset, the scene asset path is typically like `.../GRScenes-100/home_scenes/scenes/{scene_id}/start_result_xxx.usd`. Please refer to the [examples](https://github.com/OpenRobotLab/GRUtopia/tree/main/grutopia/demo) to learn how to specify scene through `scene_asset_path` field in config.

2. Use the raw dataset

The dataset contains raw models and raw scenes. We recommend using [OpenUSD python SDK](https://openusd.org/release/intro.html) to apply physics APIs such as the rigid body and collider to the models. We also provide an example [preprocess](https://github.com/OpenRobotLab/GRUtopia/blob/main/toolkits/grscenes_scripts/preprocess.py) script to learn the detailed workflow of the physics property bindings. Besides, here are some other [tool scripts](https://github.com/OpenRobotLab/GRUtopia/tree/main/toolkits/grscenes_scripts) for GRScenes-100.
