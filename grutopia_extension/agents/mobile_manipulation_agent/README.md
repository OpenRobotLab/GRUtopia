# Project Setup and Run Instructions

## 1 Install Environment Dependencies

Create a new conda/venv env (for agent), and then install the dependencies within the env:

```shell
# make sure your cuda toolkit version matches your pytorch cuda version
# PWD: grutopia_extension/agents/mobile_manipulation_agent
pip install -r requirements.txt
```

Tmux is required for running the agent entrypoint script. You can install it by:

```shell
sudo apt update
sudo apt install tmux
```

## 2 Install Submodules

```shell
# PWD: grutopia_extension/agents/mobile_manipulation_agent
mkdir images
# Please make sure your torch cuda version is equal with your cuda version
# GroundingDINO
git clone https://github.com/IDEA-Research/GroundingDINO.git

# YOLOv7
git clone https://github.com/WongKinYiu/yolov7.git
```

The directory structure of the submodules are as follows:

```
grutopia_extension
├── agents
│   ├── mobile_manipulation_agent
│   │   ├── GroundingDINO                            # GroundingDINO repo
│   │   ├── images                                   # Empty directory
│   │   └── yolov7                                   # yolov7 repo
```

## 3 Download Model Weights

```shell
# PWD: grutopia_extension/agents/mobile_manipulation_agent
mkdir data
cd data
```

Download the necessary model weights from the provided sources and place them in an accessible directory within the
project, put them under data folder. These weights are essential for running models.

- MobileSAM: Download mobile_sam.pt from the [MobileSAM GitHub Repository](https://github.com/ChaoningZhang/MobileSAM/blob/master/weights/mobile_sam.pt).
- GroundingDINO: Download groundingdino_swint_ogc.pth from
  the [GroundingDINO GitHub Repository](https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha2/groundingdino_swinb_cogcoor.pth).
- YOLOv7: Download yolov7-e6e.pt from the [YOLOv7 GitHub Repository](https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7-e6e.pt).

**NOTE**: Ensure these files are saved in a directory where the code can access them.

```
grutopia_extension
├── agents
│   ├── mobile_manipulation_agent
│   │   ├── data
│   │   │   ├── groundingdino_swint_ogc.pth
│   │   │   ├── mobile_sam.pt
│   │   │   └── yolov7-e6e.pt
```

## 4 Configure API Key

Put your api key under `./modules/vlm/api_key`.

```
grutopia_extension
├── agents
│   ├── mobile_manipulation_agent
│   │   ├── modules
│   │   │   └── vlm
│   │   │   │   └── api_key
│   │   │   │   │   ├── azure_api_key.txt
│   │   │   │   │   └── azure_api_key_e.txt
```

As mentioned above:
- `azure_api_key.txt`: The API key for the Azure OpenAI `gpt-4o` model.
- `azure_api_key_e.txt`: The API key for the Azure OpenAI `text-embedding-3-large` embedding model.

You can learn how to get the keys through this [link](https://learn.microsoft.com/zh-cn/azure/ai-services/openai/concepts/models?tabs=global-standard%2Cstandard-chat-completions).


## 5 Run the Agent Process

Launch the agent process

```shell
# PWD: grutopia_extension/agents/mobile_manipulation_agent
./scripts/launch_vlm_servers.sh
```

For complete instructions of how to run the benchmark with the agent, please refer to the [RUN BENCHMARK BASELINE](https://grutopia.github.io/get_started/run-benchmark-baseline.html).
