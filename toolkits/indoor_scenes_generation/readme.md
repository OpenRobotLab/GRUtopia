# Procedural Indoor Scene Generation with GRScenes-100 & Infinigen

This document provides guidance for generating indoor scenes using the [GRScenes-100 dataset](https://openxlab.org.cn/datasets/OpenRobotLab/GRScenes/tree/main/scenes/GRScenes-100) with [Infinigen](https://github.com/princeton-vl/infinigen), a procedural generation framework.

---

## 🚀 Workflow Setup

### 1. Install Infinigen

Refer to the official installation guide at [`infinigen/docs/Installation.md`](https://github.com/princeton-vl/infinigen/blob/main/docs/Installation.md). Below is our recommended method:

```bash
git clone https://github.com/princeton-vl/infinigen.git
cd infinigen
conda create --name infinigen python=3.11
conda activate infinigen
bash scripts/install/interactive_blender.sh
```

### 2. Apply GRScenes-100 Assets

#### 2.1 Replace Python Files
To utilize static assets from GRScenes-100, replace corresponding files in the `infinigen` with those from `toolkits/indoor_scenes_generation/infinigen`. Maintain identical directory structures.

> ⚠️ **Critical Notice**
> Ensure path consistency during replacement:
> - `toolkits/indoor_scenes_generation/infinigen/infinigen/...` → `infinigen/infinigen/...`
> - `toolkits/indoor_scenes_generation/infinigen/infinigen_examples/...` → `infinigen/infinigen_examples/...`

#### 2.2 Prepare Static Assets
Activate the InternUtopia environment and execute the following to generate asset sources in `infinigen/infinigen/assets/static_assets/source`:

```bash
python toolkits/indoor_scenes_generation/util/preparing_static_assets.py \
    --input_folder {{path_to_GRScenes-100}}/home_scenes/target_69_new/models/object \
    --output_folder {{path_to_infinigen}}/infinigen/assets/static_assets/source
```

### 3. Scene Generation

After completing previous steps, you can use `infinigen` to generate indoor scenes as described in the official document [`infinigen/docs/HelloRoom.md`](https://github.com/princeton-vl/infinigen/blob/main/docs/HelloRoom.md). The difference is that you will use the GRScenes-100 static assets.

> 💡 **Execution Note**
> Use Blender's bundled Python interpreter:  `infinigen/blender/4.2/python/bin/python3.11`

### 4. USD Export & Postprocessing

#### 4.1 Export to USDC Format
Convert generated `.blend` files to `.usdc` format following [`infinigen/docs/ExportingToExternalFileFormats.md`](https://github.com/princeton-vl/infinigen/blob/main/docs/ExportingToExternalFileFormats.md):

```bash
python -m infinigen.tools.export \
     --input_folder {PATH_TO_FOLDER_OF_BLENDFILES} \
     --output_folder outputs/my_export \
     -f usdc \
     -r 1024

```

#### 4.2 Scene Postprocessing

To better use the GRScenes-100 data, please activate InternUtopia environment and run the following script:

```bash
python toolkits/indoor_scenes_generation/util/data_post_processing.py \
    --input_path {{path_to_infinigen}}/outputs/my_export/export_scene.blend/export_scene.usdc \
    --source_assets {{path_to_infinigen}}/infinigen/assets/static_assets/source
```

You will find the `final_scene.usd` file in the `{{path_to_infinigen}}/outputs/my_export/export_scene.blend` directory. You can now use isaacsim to open the file and use it normally.
