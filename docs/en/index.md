---
sidebar: false
myst:
  html_meta:
    "description lang=en": |
      Documentation for users who wish to build sphinx sites with
      pydata-sphinx-theme.
---

# Welcome to GRUtopia's documentation!

This document introduces installation steps, tutorials, and reference APIs of [GRUtopia](https://github.com/OpenRobotLab/GRUtopia).

GRUtopia, built on NVIDIA Isaac Sim, is an embodied AI research platform. It aims to reduce the burden of engineering implementation, tackle the data scarcity in the field of embodied intelligence, and offer simulations that are more reflective of real-world scenarios.

Here is a brief overview of GRUtopia:

![overview](../_static/image/overview.webp)

We provide some demostrations to show what GRUtopia can do after [installation](./usage/get_started/installation.md):

| Name | Description | Requirement | Reference |
| --- | --- | --- | --- |
| Robot simulation | Control a humanoid robot to explore a simulation world | A host with NVIDIA RTX GPU | [Wander with Keyboard](../usage/get_started/wander-with-keyboard.html) |
| Teleoperating with VisionPro | Teleoperate a humanoid robot with a VR headset (Apple VisionPro) | * A host with NVIDIA RTX GPU <br /> * [Apple VisionPro](https://www.apple.com/apple-vision-pro/) | [Teleoperate with VisionPro](../usage/get_started/teleoperating-with-visionpro.html) |
| Teleoperating with Mocap | Teleoperate a robot arm with camera-based motion capture system | * A host with NVIDIA RTX GPU <br /> * A camera | [Teleoperate with Mocap](../usage/get_started/teleoperating-with-mocap.html) |
| Layout Edit with Mocap | Interactively edit scene with camera-based motion capture system | * A host with NVIDIA RTX GPU <br /> * A camera | [Layout Edit with Mocap](../usage/get_started/layout_edit_with_mocap.html) |
| Benchmark | Run a social-navigation/mobile-manipulation benchmark with agent | A host with NVIDIA RTX GPU <br />(32GB+ VRAM and 64GB+ RAM) | [Run Benchmark Baseline](../usage/get_started/run-benchmark-baseline.html) |



```{toctree}
:maxdepth: 1

usage/index
api/index

```
