# Run Benchmark Baseline

## Start Baseline Agent

> These agents are currently in the directory `GRUtopia/grutopia_extension/agents/`. They may be removed from this repository later.

**System requirements:**

1. At least 2 pieces of NVIDIA's RTX-supported GPU on 1 server.
2. 64GB RAM
3. 16GB+ GRAM

> **Note**: We've tested our benchmarks on RTX 4090, and you can take it as a spec reference.

**Use the agent of the benchmark baseline:**

1. Start agent as follows.
2. Start the benchmark with run files.

### Start Baseline Agent For Social Navigation Benchmark

The follow steps shows how to run a demo for social navigation agent.

#### 1. Install Environment Dependencies

> **Create a new conda env for agent FIRST !!!!!**

Then install the required Python packages, use the following command:

```
# make sure your cudatoolkit version matches you pytorch cuda version
cd grutopia_extension/agents/social_navigation_agent
pip install -r requirements.txt
```

#### 2. Install Submodules

```shell
cd grutopia_extension/agents/social_navigation_agent
mkdir images
# Please make sure your torch cuda version is equal with your cuda version
# GroundingDINO
git clone https://github.com/IDEA-Research/GroundingDINO.git

# YOLOv7
git clone https://github.com/WongKinYiu/yolov7.git
```

#### 3. Download Model Weights

```shell
mkdir data
cd data
```

Download the necessary model weights from the provided sources and place them in an accessible directory within the
project, put them under data folder. These weights are essential for running the models.

- **MobileSAM**: Download mobile_sam.pt from
  the [MobileSAM GitHub Repository](https://github.com/ChaoningZhang/MobileSAM).
- **GroundingDINO**: Download groundingdino_swint_ogc.pth from
  the [GroundingDINO GitHub Repository](https://github.com/IDEA-Research/GroundingDINO).
- **YOLOv7**: Download yolov7-e6e.pt from the [YOLOv7 GitHub Repository](https://github.com/WongKinYiu/yolov7).

> Note: Ensure these files are saved in a directory where the code can access them.

#### 4. Set Api Key

Put your api key in `./modules/vlm/api_key`.

```
├── api_key.txt
├── azure_api_key.txt
```

#### 5. Run the Initialization Script

Launch required servers for the agent.

```shell
./scripts/launch_vlm_servers.sh
```

#### 6. Generate episodes config

```
# generate episodes config for social navigation task
python GRUtopia/grutopia_extension/agents/social_navigation_agent/generate_sn_episodes.py
```

The results will be saved in /path/to/results given in the corresponding config creating files.

### Start Baseline Agent For Mobile Manipulation Benchmark

The follow steps shows how to run a demo for mobile manipulation agent.

#### 1. Install Environment Dependencies

> **Create a new conda env for agent FIRST !!!!!**

Then install the required Python packages, use the following command:

```
# make sure your cudatoolkit version matches you pytorch cuda version
cd grutopia_extension/agents/mobile_manipulation_agent
pip install -r requirements.txt
```

#### 2. Install Submodules
8
```shell
cd grutopia_extension/agents/mobile_manipulation_agent
mkdir images
# Please make sure your torch cuda version is equal with your cuda version
# GroundingDINO
git clone https://github.com/IDEA-Research/GroundingDINO.git

# YOLOv7
git clone https://github.com/WongKinYiu/yolov7.git
```

#### 3. Download Model Weights

```shell
mkdir data
cd data
```

Download the necessary model weights from the provided sources and place them in an accessible directory within the
project, put them under data folder. These weights are essential for running models.

- MobileSAM: Download mobile_sam.pt from the [MobileSAM GitHub Repository](https://github.com/ChaoningZhang/MobileSAM).
- GroundingDINO: Download groundingdino_swint_ogc.pth from
  the [GroundingDINO GitHub Repository](https://github.com/IDEA-Research/GroundingDINO).
- YOLOv7: Download yolov7-e6e.pt from the [YOLOv7 GitHub Repository](https://github.com/WongKinYiu/yolov7).

> Note: Ensure these files are saved in a directory where the code can access them.

#### 4. Set Api Key

Put your api key in `./modules/vlm/api_key`.

```
├── api_key.txt
├── azure_api_key.txt
```

#### 5. Run the Initialization Script

Launch required servers for the agent.

```shell
./scripts/launch_vlm_servers.sh
```

#### 6. Generate episodes config

```
# generate episodes config for mobile manipulation task
python GRUtopia/grutopia_extension/agents/mobile_manipulation_agent/generate_mm_episodes.py
```

## Run a demo for agent

> Please use the conda env of grutopia to run the following demos rather than the agents' conda env.

It takes too long to run the entire benchmark. You can try demos first.

### For social navigation Benchmark

```
# run Social Navigation agent
python GRUtopia/grutopia/demo/h1_social_navigation.py
```

### For Mobile Manipulation Benchmark

```
# run mobile manipulation agent
python GRUtopia/grutopia/demo/mobile_manipulation.py
```

> To run the entire benchmark, you can run the executable files generated by generate_sn_episodes.py/generate_mm_episodes.py one by one
>
> Manual distribution is also supported :)
>
> Automatic distributed benchmark is coming soon...
