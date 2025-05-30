[![demo](docs/en/_static/image/teaser.png "demo")](https://www.youtube.com/watch?v=fD0F1jIax5Y)
<div id="top" align="left">

[![arxiv](https://img.shields.io/badge/arXiv%202407.10943-red?logo=arxiv)](https://arxiv.org/abs/2407.10943)
[![pdf](https://img.shields.io/badge/Paper-06AC38?logo=pagekit)](https://github.com/grutopia/grutopia.github.io/releases/download/v0.1.0/GRUtopia.pdf)
[![github](https://img.shields.io/badge/Project-0065D3?logo=rocket&logoColor=white)](https://github.com/OpenRobotLab/GRUtopia)
[![doc](https://img.shields.io/badge/Document-FFA500?logo=readthedocs&logoColor=white)](https://grutopia.github.io)
[![video-en](https://img.shields.io/badge/YouTube-D33847?logo=youtube)](https://www.youtube.com/watch?v=fD0F1jIax5Y)
[![video-cn](https://img.shields.io/badge/Bilibili-00A1D6?logo=bilibili&logoColor=white)](https://www.bilibili.com/video/BV1JUbxeMEsL/?buvid=XU42709457560E0722A8AA591EE792A3DAE59&from_spmid=search.search-result.0.0&is_story_h5=false&mid=vxiHfNKVdk6fb8fduRusuX8FTQ%2FSZMtL1rElX6M3iMo%3D&p=1&plat_id=116&share_from=ugc&share_medium=android&share_plat=android&share_session_id=e78b4bb6-087b-4a72-817b-b06ef91167f3&share_source=COPY&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1720788955&unique_k=CeKgxGI&up_id=3546722198358311&vd_source=7f685cd616faf836ed7469749c100410)
[![PyPI Downloads](https://static.pepy.tech/badge/grutopia)](https://pepy.tech/projects/grutopia)
[![GitHub Issues](https://img.shields.io/github/issues/OpenRobotLab/GRUtopia)](https://github.com/OpenRobotLab/GRUtopia/issues)
<a href="https://cdn.vansin.top/taoyuan.jpg"><img src="https://img.shields.io/badge/WeChat-07C160?style=for-the-badge&logo=wechat&logoColor=white" height="20" style="display:inline"></a>
[![Discord](https://img.shields.io/discord/1373946774439591996?logo=discord)](https://discord.gg/TpGTg6uA)
</div>

# GRUtopia

## 🔥 News

- \[2025-02\] GRUtopia 2.0 released!
- \[2024-07\] We release the [paper](https://arxiv.org/abs/2407.10943) and demos of GRUtopia.

## 🚀 New Features in 2.0 release
- Gym compatible env implementation.
- Easy-to-use pythonic config system to use out-of-the-box [sensors](https://grutopia.github.io/usage/tutorials/how-to-use-sensor.html), [controllers](https://grutopia.github.io/usage/tutorials/how-to-use-controller.html), [robots](https://grutopia.github.io/usage/tutorials/how-to-use-robot.html) and [tasks](https://grutopia.github.io/usage/tutorials/how-to-use-task.html).
- Examples of driving [diverse robots](https://grutopia.github.io/usage/tutorials/how-to-use-robot.html) and the corresponding policies.
- [Benchmark and baseline](https://grutopia.github.io/usage/get_started/run-benchmark-baseline.html) for social navigation and mobile manipulation task.
- Teleportation tools with [Mocap](https://grutopia.github.io/usage/get_started/teleoperating-with-mocap.html) and [Apple VisionPro](https://grutopia.github.io/usage/get_started/teleoperating-with-visionpro.html).
- Physically accurate [interactive object assets](https://huggingface.co/datasets/OpenRobotLab/GRScenes) that are ready for simulation.
- Procedural [Indoor Scene Generation](https://github.com/OpenRobotLab/GRUtopia/tree/main/toolkits/indoor_scenes_generation) with [GRScenes-100](https://grutopia.github.io/usage/get_started/explore-grscenes.html).

## 📋 Contents
- [🏠 About](#-about) <!-- omit in toc -->
- [📚 Getting Started](#-getting-started)
- [🏙️ Assets](#️-assets)
- [📦 Benchmark & Method](#-benchmark--method)
- [👥 Support](#-support)
- [📝 TODO List](#-todo-list)
- [🔗 Citation](#-citation)
- [📄 License](#-license)
- [👏 Acknowledgements](#-acknowledgements)


## 🏠 About

<!-- ![Teaser](assets/teaser.jpg) -->

Recent works have been exploring the scaling laws in the field of Embodied AI. Given the prohibitive costs of collecting real-world data, we believe the <b>Simulation-to-Real (Sim2Real) paradigm</b> is a more feasible path for scaling the learning of embodied models.

We introduce project <b>GRUtopia</b> (aka. 桃源 in Chinese), a general-purpose research platform for embodied AGI.
It features several advancements:
* 🏙️ <b>GRScenes</b>, the scene dataset, includes 100k interactive finely annotated scenes. GRScenes covers 89 diverse scene categories, facilitating deployment of general robots across different scenarios.
* 🧑‍🤝‍🧑 <b>GRResidents</b>, a Large Language Model (LLM) driven Non-Player Character (NPC) system that enables social interaction, task generation, and task assignment, thus simulating <b>social scenarios</b> for embodied AI applications.
* 🤖 <b>GRBench</b>, a collection of embodied AI benchmarks for assessing various capabilities of solving embodied tasks.

We hope that this work can alleviate the scarcity of high-quality data in this field and provide a more comprehensive assessment of embodied AI research.





## 📚 Getting Started

### Prerequisites

- Ubuntu 20.04, 22.04
- [NVIDIA Omniverse Isaac Sim 4.2.0](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html)
  - Ubuntu 20.04/22.04 Operating System
  - NVIDIA GPU (RTX 2070 or higher)
  - NVIDIA GPU Driver (recommended version 535.216.01+)
  - Docker (Optional)
  - NVIDIA Container Toolkit (Optional)
- Conda
  - Python 3.10.16 (3.10.* should be ok)

### Installation

We provide the installation guide [here](https://grutopia.github.io/usage/get_started/installation.html). You can install locally or use docker and verify the installation easily.

### Documentation \& Tutorial

We provide detailed [docs](https://grutopia.github.io) for the basic usage of different modules supported in GRUtopia. Welcome to try and post your suggestions!

## 🏙️ Assets

> [!NOTE]
> 📝First of all you **MUST** complete the [User Agreement for GRScenes-100 Dataset Access](https://docs.google.com/forms/d/e/1FAIpQLSccX4pMb57eZbjXpH12Jz6WUBmCfeyc2t0s98k_u4Z-GD3Org/viewform?fbzx=8256642192244696391).

Then you can choose to download all assets (~80GB) or a minimum set (~500MB) to examine installation by running the following script with [GRUtopia](https://grutopia.github.io/usage/get_started/installation.html) installed:

```shell
$ python -m grutopia.download_assets
```
The default path to store the downloaded assets is `${PATH/TO/GRUTOPIA/ROOT}/grutopia/assets`. Users have two ways to configure the asset path:

1. Spcecify a custom path during download using `python -m grutopia.download_assets`.
2. Set it later by running `python -m grutopia.set_assets_path` and entering the preferred directory.



### GRScenes-100
If you want to separately download the `GRScenes-100` scene assets, you can manually download them from [OpenDataLab](https://openxlab.org.cn/datasets/OpenRobotLab/GRScenes/tree/main/scenes/GRScenes-100), [ModelScope](https://www.modelscope.cn/datasets/Shanghai_AI_Laboratory/GRScenes/files) and [HuggingFace](https://huggingface.co/datasets/OpenRobotLab/GRScenes/tree/main/scenes/GRScenes-100). Please refer to the [instructions](https://huggingface.co/datasets/OpenRobotLab/GRScenes#%F0%9F%93%9A-getting-started) for scene usage.

### Robots & Weights
If you want to separately download robots and policy weights, you can manually download the `robot` directory from from [OpenDataLab](https://openxlab.org.cn/datasets/OpenRobotLab/GRScenes/tree/main/robots), [ModelScope](https://www.modelscope.cn/datasets/Shanghai_AI_Laboratory/GRScenes/files) and [HuggingFace](https://huggingface.co/datasets/OpenRobotLab/GRScenes/tree/main/robots) and move it to the root of the asset path.

## 📦 Benchmark & Method
<p align="center">
  <img src="docs/en/_static/image/benchmark.png" align="center" width="100%">
</p>

 We preliminarily establish three benchmarks for evaluating the capabilities of embodied agents from different aspects: <b>Object Loco-Navigation</b>, <b>Social Loco-Navigation</b>, and <b>Loco-Manipulation</b>. Please refer to the [documentation](https://grutopia.github.io/usage/get_started/run-benchmark-baseline.html) for running the benchmarks.

## 👥 Support

Join our [WeChat](https://cdn.vansin.top/taoyuan.jpg) support group or [Discord](https://discord.gg/TpGTg6uA) for any help.

## 📝 TODO List

- \[x\] Release the paper with demos.
- \[x\] Release the platform with basic functions and demo scenes.
- \[x\] Release 100 curated scenes.
- \[x\] Polish APIs and related codes.
- \[x\] Full release and further updates.
- \[x\] Release the baseline methods and benchmark data.
- \[ \] Support multiple episodes.
- \[ \] Vectorized env and batch execution.
- \[ \] Training framework.

## 🔗 Citation

If you find our work helpful, please cite:

```bibtex
@inproceedings{grutopia,
    title={GRUtopia: Dream General Robots in a City at Scale},
    author={Wang, Hanqing and Chen, Jiahe and Huang, Wensi and Ben, Qingwei and Wang, Tai and Mi, Boyu and Huang, Tao and Zhao, Siheng and Chen, Yilun and Yang, Sizhe and Cao, Peizhou and Yu, Wenye and Ye, Zichao and Li, Jialun and Long, Junfeng and Wang, ZiRui and Wang, Huiling and Zhao, Ying and Tu, Zhongying and Qiao, Yu and Lin, Dahua and Pang Jiangmiao},
    year={2024},
    booktitle={arXiv},
}
```

</details>

## 📄 License

GRUtopia's simulation platform is [MIT licensed](LICENSE). The open-sourced GRScenes are under the <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License </a><a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/80x15.png" /></a>.

## 👏 Acknowledgements

- [OmniGibson](https://github.com/StanfordVL/OmniGibson): We refer to OmniGibson for designs of oracle actions.
- [RSL_RL](https://github.com/leggedrobotics/rsl_rl): We use `rsl_rl` library to train the control policies for legged robots.
- [ReferIt3D](https://github.com/referit3d/referit3d): We refer to the Sr3D's approach to extract spatial relationship.
- [Isaac Lab](https://github.com/isaac-sim/IsaacLab): We use some utilities from Orbit (Isaac Lab) for driving articulated joints in Isaac Sim.
- [Open-TeleVision](https://github.com/OpenTeleVision/TeleVision): We use Open-TeleVision to teleoperate with Apple VisionPro.
- [HaMeR](https://github.com/geopavlakos/hamer): We use HaMeR to recognize hand gesture in teleoperate with camera.
- [Infinigen](https://github.com/princeton-vl/infinigen): We use Infinigen to procedurally generate indoor scenes upon [GRScenes-100 dataset](https://openxlab.org.cn/datasets/OpenRobotLab/GRScenes/tree/main/scenes/GRScenes-100).
- [VLFM](https://github.com/bdaiinstitute/vlfm): We refer to VLFM to implement our benchmark baselines.
- [Grounding DINO](https://github.com/IDEA-Research/GroundingDINO): We use Grounding DINO in our benchmark baselines.
- [YOLOv7](https://github.com/WongKinYiu/yolov7): We use YOLOv7 in our benchmark baselines.
- [MobileSAM](https://github.com/ChaoningZhang/MobileSAM): We use MobileSAM in our benchmark baselines.
