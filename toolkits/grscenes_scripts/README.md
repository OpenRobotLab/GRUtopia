# Introduction

This directory `toolkits/grscenes_scripts` includes some tool scripts for GRScenes usage.

Users can download the GRScenes dataset from [**OpenXLab**](https://openxlab.org.cn/datasets/OpenRobotLab/GRScenes), [**ModelScope**](https://www.modelscope.cn/datasets/Shanghai_AI_Laboratory/GRScenes/summary) and [**HuggingFace**](https://huggingface.co/datasets/OpenRobotLab/GRScenes).

## Prerequisites

### Configure MDL Material Search Path

In order to load scenes normally, we recommend to configure an environment variable named `MDL_SYSTEM_PATH` according to this [document](https://docs.omniverse.nvidia.com/materials-and-rendering/latest/mdl_search_path.html). Here are the steps:

```bash
# step 1. Find the Materials folder path in the downloaded GRScenes folder, such as `GRScenes-100/home_scenes/Materials`
# step 2. Configure the environment variable `MDL_SYSTEM_PATH` (saved to `~/.bashrc` is recommended).
echo 'export MDL_SYSTEM_PATH=$MDL_SYSTEM_PATH:</path/to/your_downloaded_materials_folder>' >> ~/.bashrc
source ~/.bashrc
```

### Install Dependencies

Our scripts depends on OpenUSD and IsaacSim Python SDK.

If the GRUtopia conda environment has been configured, users can skip this step, otherwise users need to install some dependencies as follows.

```bash
conda create -n <env_name> python=3.10
conda activate <env_name>
pip install usd-core==24.11
pip install isaacsim==4.2.0.2 isaacsim-extscache-physics==4.2.0.2 isaacsim-extscache-kit==4.2.0.2 isaacsim-extscache-kit-sdk==4.2.0.2 --extra-index-url https://pypi.nvidia.com
```

## Usage

The dataset folder GRScenes-100 includes 100 scenes, covering home and commercial scenes. Our scenes are in USD format, users can load these scenes through GRUtopia or Nvidia Isaac Sim.

After decompressing the scene zip package, users can find the scene usd file in a path like `.../GRScenes-100/home_scenes/scenes/{scene_id}/start_result_xxx.usd`. We already provided three types of scene usd files that can be utilized directly.

- **start_result_raw.usd** is a clean raw scene without binding any physics properties.
- **start_result_navigation.usd** is a scene specially processed for navigation tasks.
- **start_result_interaction.usd** is a scene specially processed for manipulation tasks.

We also provide our pre-process scripts. The following scripts can be used independently.

### **preprocess.py**: binding physics properties for raw scenes
Users can edit the properties for models and scenes in GUI. However, we recommend to do this through scripting APIs for an automatic and efficient workflow. The script **preprocess.py** implements pre-processing of scenes for navigation and manipulation tasks. Users can use this script directly or modify it in a customized need, then run the commands as follows.

```bash
## use `-i/--interaction` option to preprocess scenes for interaction.
python preprocess.py -i/--interaction -f/--files [</path/to/raw_scene_usd_file>...]
## use `-n/--navigation` option to preprocess scenes for navigation
python preprocess.py -n/--navigation -f/--files [</path/to/raw_scene_usd_file>...]
## besides, use `-d/--dirs` option to preprocess all scenes under the scenes folder such as `/home/$USER/home_scenes/scenes`
python preprocess.py -i/--interaction -n/--navigation -d/--dirs [</path/to/scene_root_folder>...]
```

### **warmup.py**: warming up the simulation process of the given scenes.
Users can play the scene directly in the GUI, or run this script to warm-up the scene in a headless way. It will save the cooking cache of the physics tasks to the local disk for an efficient play later.

```bash
## warm up the specific scenes
python warmup.py -f/--files [</path/to/scene_usd_file>...]
## warm up all scenes
python warmup.py -d/--dirs [</path/to/scene_root_folder>...]
```

### **play_scene.py**: loading and playing the given scene.
If users want to use the local mesh cache from the above warm-up process, users should run this script to play the scene. Because the location of the local cooking mesh cache is individual and likely different for isaacsim Python SDK and users' additional installed Isaac Sim.

```bash
python play_scene.py -f/--file </path/to/scene_usd_file>
```

### **export_scenes.py**: exporting the specified scenes with its related objects and material files.
Our GRScenes-100 is an overall folder containing all models, materials and scenes. If users want to export individual scenes, run this script.

```bash
python export_scenes.py -i/--input </path/to/source_scene_root_folder> -o/--output </path/to/target_scene_root_folder> -n/--names [<scene_id1>...]
```

### **get_metadata.py**: recording the metadata information of the models and materials referenced in given model instance or scene usd files.

```bash
python get_metadata.py -f/--files [</path/to/single_instance_or_scene_usd>...]
python get_metadata.py -d/--dirs [</path/to/instance_or_scene_root_folder>...]
```

### **extract_objaverse.py**: extracting model objects from objaverse.
This script can convert and detach the raw models and its materials of the objaverse dataset.
```bash
python extract_objaverse.py --usd_path </path/to/objavers_usd_file> --material_path </path/to/output_material_files_path>
```
