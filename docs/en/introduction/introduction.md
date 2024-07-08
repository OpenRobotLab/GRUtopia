# Introduction

Current embodied intelligence research urgently needs to overcome the problem of disconnection between high-level perceptual planning and low-level motion control. By constructing a highly realistic simulation environment, not only can it enhance the robot's perception and behavior planning capabilities but also promote the development of multi-module collaborative control strategies, ultimately steadily advancing towards the goal of general-purpose embodied robots. (Research Needs)

High-level studies are typically conducted on static datasets or simulation platforms, which often cannot provide environments with both visual realism and physical realism at the same time, limiting the transferability of research results to real-world application scenarios. At the same time, the development of large-model technology has opened up new paths for improving the perception and behavior planning abilities of robots, making the realization of the goal of universal robots no longer distant. (Industry Status)

To address these challenges, the OpenRobotLab team of Shanghai AI Lab proposes the GRUtopia Embodied Intelligence Simulation Platform. The platform features:

1. A large-scale scene dataset covering various application scenarios, capable of providing rich and realistic visual and physical environments for embodied research;
2. An API library and extensive toolkit containing mainstream robotic control algorithms, enabling plug-and-play functionality with just one line of code to achieve a realistic control process, reproducing all kinds of actual situations likely encountered during planning processes;
3. The toolkit also provides functions like algorithm migration and strategy training.

Additionally, there is a task generation system for embodied tasks driven by large models and an NPC interaction system, marking the first time automated embodied task generation and multimodal interactive NPCs have been achieved. This offers infinite training tasks for developing generalized agents and also serves as a foundation for studying embodied behavior interpretability and human-machine interactions. (Achievement Definition)
