# Benchmark

## Background
Currently GRUtopia supports benchmark for social navigation and mobile manipulation.
You can download the dataset for both social navigation and mobile manipulation tasks from [here](path/to/datasets).

## Social Navigation Benchmark

### Introduction for Social Navigation Dataset
Here is a brief introduction for some files in the datasets:
- episodes_mini.json: episodes for social navigation task
- model_mapping.json: name mapping from object name in scene to object's own name
- object_dict_with_caption.json: object captions and relations in scene

### Setup and Running Instructions
The follow steps shows how to run a demo for social navigation agent

#### 1. Install Environment Dependencies
To install the required Python packages, use the following command:
```
# make sure your cudatoolkit version matches you pytorch cuda version
cd grutopia_extension/agents/social_navigation_agent
pip install -r requirements.txt
```

#### 2. Install Submodules
```
cd grutopia_extension/agents/social_navigation_agent
mkdir images
# Please make sure your torch cuda version is equal with your cuda version
# GroundingDINO
git clone https://github.com/IDEA-Research/GroundingDINO.git

# YOLOv7
git clone https://github.com/WongKinYiu/yolov7.git
```

#### 3. Download Model Weights
```
mkdir data
cd data
```
Download the necessary model weights from the provided sources and place them in an accessible directory within the project, put them under data folder. These weights are essential for running the models.

- **MobileSAM**: Download mobile_sam.pt from the [MobileSAM GitHub Repository](https://github.com/ChaoningZhang/MobileSAM).
- **GroundingDINO**: Download groundingdino_swint_ogc.pth from the [GroundingDINO GitHub Repository](https://github.com/IDEA-Research/GroundingDINO).
- **YOLOv7**: Download yolov7-e6e.pt from the [YOLOv7 GitHub Repository](https://github.com/WongKinYiu/yolov7).

> Note: Ensure these files are saved in a directory where the code can access them.

#### 4. Set Api Key
Put your api key in ./modules/vlm/api_key
```
├── api_key.txt
├── azure_api_key.txt
```

#### 5. Run the Initialization Script
Launch required servers for the agent.
```
./scripts/launch_vlm_servers.sh
```

#### 6. Generate episodes config
```
# generate episodes config for social navigation task
python GRUtopia/grutopia_extension/agents/social_navigation_agent/create_sn_task_yaml.py
```
The results will be saved in /path/to/results given in the corresponding config creating files.

#### 7. Run a demo for agent
```
# run social navigation agent
# config yaml is in GRUtopia/demo/configs/h1_social_navigation.yaml
python GRUtopia/demo/h1_social_navigation.py
```
## Mobile Manipulation Benchmark

### Introduction for Mobile Manipulation Dataset
Here is a brief introduction for some files in the datasets:
- annotation.json: Polygons of room regions
- interaction_instruction_mini_v1.json: episodes for mobile manipulation task
- paths.json: ground truth path to specific object from start point

### Setup and Running Instructions
The follow steps shows how to run a demo for mobile manipulation agent

#### 1. Install Environment Dependencies
To install the required Python packages, use the following command:
```
# make sure your cudatoolkit version matches you pytorch cuda version
cd grutopia_extension/agents/mobile_manipulation_agent
pip install -r requirements.txt
```
#### 2. Install Submodules
```
cd grutopia_extension/agents/mobile_manipulation_agent
mkdir images
# Please make sure your torch cuda version is equal with your cuda version
# GroundingDINO
git clone https://github.com/IDEA-Research/GroundingDINO.git

# YOLOv7
git clone https://github.com/WongKinYiu/yolov7.git
```
#### 3. Download Model Weights
```
mkdir data
cd data
```
Download the necessary model weights from the provided sources and place them in an accessible directory within the project, put them under data folder. These weights are essential for running models.

- MobileSAM: Download mobile_sam.pt from the [MobileSAM GitHub Repository](https://github.com/ChaoningZhang/MobileSAM).
- GroundingDINO: Download groundingdino_swint_ogc.pth from the [GroundingDINO GitHub Repository](https://github.com/IDEA-Research/GroundingDINO).
- YOLOv7: Download yolov7-e6e.pt from the [YOLOv7 GitHub Repository](https://github.com/WongKinYiu/yolov7).

> Note: Ensure these files are saved in a directory where the code can access them.
#### 4. Set Api Key
Put your api key in ./modules/vlm/api_key
```
├── api_key.txt
├── azure_api_key.txt
```
#### 5. Run the Initialization Script
Launch required servers for the agent.
```
./scripts/launch_vlm_servers.sh
```
#### 6. Generate episodes config
```
# generate episodes config for mobile manipulation task
python GRUtopia/grutopia_extension/agents/mobile_manipulation_agent/create_mm_task_yaml.py
```
#### 7. Run a demo for agent
```
# run mobile manipulation agent
# config yaml is in GRUtopia/demo/configs/h1_mobile_manipulation.yaml
python GRUtopia/demo/h1_mobile_manipulation.py
```
