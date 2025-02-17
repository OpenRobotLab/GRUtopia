# How to Use GRscenes


## How to Download

GRScenes has been publicly released on OpenXLab, ModelScope, and HuggingFace. Users can download it from the respective platforms as needed. The dataset paths for GRScenes are as follows:

- **OpenXLab**: https://openxlab.org.cn/datasets/OpenRobotLab/GRScenes
- **ModelScope**: https://www.modelscope.cn/datasets/Shanghai_AI_Laboratory/GRScenes
- **HuggingFace**: https://huggingface.co/datasets/OpenRobotLab/GRScenes

## Usage Instructions

The GRScenes dataset provides nearly 100 high-quality scenes, thousands of materials, and tens of thousands of item assets. The dataset offers users raw item assets and raw scenes. Users can perform operations like binding physical properties to scenes using the example code in the provided scripts directory, or customize and process scenes according to their needs. Currently, the GRScenes dataset successfully supports benchmark testing and validation for Navigation and Manipulation scenarios.

The organizational structure of the GRScenes dataset is as follows: Users can find scene assets in the scenes directory, item assets by category in the models directory, and material files in the Materials directory. Each item and scene asset corresponds to a metadata.json file, which records the relative paths of referenced items and materials, allowing for the packaging and exporting of individual items or scene assets based on this metadata.json file.

```
.../homes_scenes
├── Materials
│   └── ... (material files)
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
    │   └── start_result_xxx.usd (initial stage usd)
    └── ... (other scene folders)
```
Additionally, the following utility scripts are provided with the dataset:
```
# The specific usages for these scripts can be reviewed by running python xxx.py --help
.../scripts
├── export_scenes.py
├── preprocess.py
```

- **preprocess.py**: This script is used for binding and processing physical properties (such as colliders and rigid bodies) for raw scenes, allowing for different types of binding for navigation and manipulation.

- **export_scenes.py**: This script is used to find specified one or more scenes associated with items and materials, allowing for individual packaging and exporting for sharing and validation.
