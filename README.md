<br>
<p align="center">
<h1 align="center"><strong>GRUtopia: Dream General Robots in a City at Scale</strong></h1>
  <p align="center">
    <a href='https://github.com/OpenRobotLab' target='_blank'>OpenRobotLab</a>&emsp;
    <br>
    Shanghai AI Laboratory
    <br>
  </p>
</p>

<div id="top" align="center">

[![arXiv](https://img.shields.io/badge/arXiv-2407.10943-orange)](https://arxiv.org/abs/2407.10943)
[![](https://img.shields.io/badge/Paper-%F0%9F%93%96-yellow)](https://github.com/grutopia/grutopia.github.io/releases/download/v0.1.0/GRUtopia.pdf)
[![](https://img.shields.io/badge/Project-%F0%9F%9A%80-pink)](https://github.com/OpenRobotLab/GRUtopia)
[![](https://img.shields.io/badge/Doc-📘-green)](https://grutopia.github.io)
[![](https://img.shields.io/badge/Youtube-🎬-red)](https://www.youtube.com/watch?v=fD0F1jIax5Y)
[![](https://img.shields.io/badge/bilibili-📹-blue)](https://www.bilibili.com/video/BV1JUbxeMEsL/?buvid=XU42709457560E0722A8AA591EE792A3DAE59&from_spmid=search.search-result.0.0&is_story_h5=false&mid=vxiHfNKVdk6fb8fduRusuX8FTQ%2FSZMtL1rElX6M3iMo%3D&p=1&plat_id=116&share_from=ugc&share_medium=android&share_plat=android&share_session_id=e78b4bb6-087b-4a72-817b-b06ef91167f3&share_source=COPY&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1720788955&unique_k=CeKgxGI&up_id=3546722198358311&vd_source=7f685cd616faf836ed7469749c100410)

</div>

## 🤖 [Demo](https://www.youtube.com/watch?v=fD0F1jIax5Y)

[![demo](docs/en/_static/image/teaser.png "demo")](https://www.youtube.com/watch?v=fD0F1jIax5Y)

## 📋 Contents

- [🏠 About](#-about) <!-- omit in toc -->
- [🔥 News](#-news)
- [📚 Getting Started](#-getting-started)
- [📦 Model and Benchmark](#-model-and-benchmark)
- [📝 TODO List](#-todo-list)
- [🔗 Citation](#-citation)
- [📄 License](#-license)
- [👏 Acknowledgements](#-acknowledgements)

## 🏠 About

<!-- ![Teaser](assets/teaser.jpg) -->

Recent works have been exploring the scaling laws in the field of Embodied AI. Given the prohibitive costs of collecting real-world data, we believe the <b>Simulation-to-Real (Sim2Real) paradigm</b> is a more feasible path for scaling the learning of embodied models.

We introduce project <b>GRUtopia</b> (aka. 桃源 in Chinese), the first simulated interactive 3D society designed for various robots.
It features several advancements:
* 🏙️ <b>GRScenes</b>, the scene dataset, includes 100k interactive, finely annotated scenes, which can be freely combined into city-scale environments. In contrast to previous works mainly focusing on home, GRScenes covers 89 diverse scene categories, bridging the gap of <b>service-oriented environments</b> where general robots would initially be deployed.
* 🧑‍🤝‍🧑 <b>GRResidents</b>, a Large Language Model (LLM) driven Non-Player Character (NPC) system that is responsible for social interaction, task generation, and task assignment, thus simulating <b>social scenarios</b> for embodied AI applications.
* 🤖 <b>GRBench</b>, the benchmark, focuses on legged robots as primary agents and poses <b>moderately challenging</b> tasks involving Object Loco-Navigation, Social Loco-Navigation, and Loco-Manipulation.

We hope that this work can alleviate the scarcity of high-quality data in this field and provide a more comprehensive assessment of embodied AI research.

## 🔥 News

- \[2025-02\] GRUtopia 2.0 released!
- \[2024-07\] We release the [paper](https://arxiv.org/abs/2407.10943) and demos of GRUtopia.

## New Features in 2.0 release

- Use standard Gym Env.
- Use pythonic config system to support clearer and extendable configuration.
- Add diverse robots and corresponding policies
- Support teleportation with Mocap and VisionPro.
- Benchmark and baseline for social navigation and mobile manipulation task.
- Add more realistic and diverse scenes for simulation.
- Provide a large number of refined interactive object models in the scenes.
- Provide semantic annotations for a large number of objects in the scenes.
- Procedural Indoor Scene Generation with GRScenes-100.

## 📚 Getting Started

### Prerequisites

We test our codes under the following environment:

- Ubuntu 20.04, 22.04
- [NVIDIA Omniverse Isaac Sim 4.2.0](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html)
  - Ubuntu 20.04/22.04 Operating System
  - NVIDIA GPU (RTX 2070 or higher)
  - NVIDIA GPU Driver (recommended version 535.129.03)
  - Docker (Optional)
  - NVIDIA Container Toolkit (Optional)
- Conda
  - Python 3.10.16 (3.10.* should be ok)

### Installation

We provide the installation guide [here](docs/en/get_started/installation.md). You can install locally or use docker and verify the installation easily.

### Documentation \& Tutorial

We provide detailed [docs](docs/en) for the basic usage of different modules supported in GRUtopia. Welcome to try and post your suggestions!

## 📦 Model and Benchmark

### Benchmark Overview

<p align="center">
  <img src="docs/en/_static/image/benchmark.png" align="center" width="100%">
</p>
An embodied agent is expected to actively perceive its environment, engage in dialogue to clarify ambiguous human instructions, and interact with its surroundings to complete tasks. Here, we preliminarily establish three benchmarks for evaluating the capabilities of embodied agents from different aspects: <b>Object Loco-Navigation</b>, <b>Social Loco-Navigation</b>, and <b>Loco-Manipulation</b>. The target object in the instruction are subject to some constraints generated by the world knowledge manager. Navigation paths, dialogues, and actions are depicted in the figure.

For now, please see the [paper](https://arxiv.org/abs/2407.10943) for more details of our models and benchmarks. We are actively re-organizing the codes and will release them soon. Please stay tuned.

## 📝 TODO List

- \[x\] Release the paper with demos.
- \[x\] Release the platform with basic functions and demo scenes.
- \[x\] Release 100 curated scenes.
- \[x\] Release the baseline models and benchmark codes.
- \[x\] Polish APIs and related codes.
- \[x\] Full release and further updates.
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
