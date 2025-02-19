# Project Setup and Run Instructions

This guide will help you set up the necessary environment, install dependencies and model weights, and run the project.

## 1. Install Environment Dependencies
To install the required Python packages, use the following command:

```bash
# make sure your cudatoolkit version matches you pytorch cuda version
pip install -r requirements.txt
```
## 2. Install Submodules
```bash
cd mobile_manipulation_agent
mkdir images
# Please make sure your torch cuda version is equal with your cuda version
# GroundingDINO
git clone https://github.com/IDEA-Research/GroundingDINO.git

# YOLOv7
git clone https://github.com/WongKinYiu/yolov7.git
```

## 3. Download Model Weights
```bash
mkdir data
cd data
```
Download the necessary model weights from the provided sources and place them in an accessible directory within the project, put them under data folder. These weights are essential for running the models.

- **MobileSAM**: Download `mobile_sam.pt` from the [MobileSAM GitHub Repository](https://github.com/ChaoningZhang/MobileSAM).
- **GroundingDINO**: Download `groundingdino_swint_ogc.pth` from the [GroundingDINO GitHub Repository](https://github.com/IDEA-Research/GroundingDINO).
- **YOLOv7**: Download `yolov7-e6e.pt` from the [YOLOv7 GitHub Repository](https://github.com/WongKinYiu/yolov7).

> **Note**: Ensure these files are saved in a directory where the code can access them.

## 4. Run the Initialization Script

```bash
./scripts/launch_vlm_servers.sh
```
## 5. Put your api key in ./modules/vlm/api_key

```bash
├── api_key.txt
├── azure_api_key.txt
```
