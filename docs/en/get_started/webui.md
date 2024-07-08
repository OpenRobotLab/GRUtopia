# Interact with NPC through WebUI

> This tutorial guides you to interact with NPC through WebUI.

> Currently WebUI is only supported with docker. Make sure you have built the docker image following the instruction of [installation with docker](./installation.md#install-with-docker-linux).

## Start WebUI process

Start docker container and start WebUI process within the container.

```bash
cd PATH/TO/GRUTOPIA/ROOT

$ export CACHE_ROOT=$HOME/docker  # set cache root path
$ export WEBUI_HOST=127.0.0.1  # set WebUI listen address, default to 127.0.0.1

$ docker run --name grutopia -it --rm --gpus all --network host \
  -e "ACCEPT_EULA=Y" \
  -e "PRIVACY_CONSENT=Y" \
  -e "WEBUI_HOST=${WEBUI_HOST}" \
  -v ${PWD}:/isaac-sim/GRUtopia \
  -v ${PWD}/test/.test_scripts:/isaac-sim/run_scripts \
  -v ${CACHE_ROOT}/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ${CACHE_ROOT}/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ${CACHE_ROOT}/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ${CACHE_ROOT}/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ${CACHE_ROOT}/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ${CACHE_ROOT}/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ${CACHE_ROOT}/isaac-sim/data:/root/.local/share/ov/data:rw \
  -v ${CACHE_ROOT}/isaac-sim/documents:/root/Documents:rw \
  grutopia:0.0.1

# run inside container
$ ./webui_start.sh  # start WebUI process
```

Now you can access WebUI from `http://${WEBUI_HOST}:8080`.

## Start simulation

GPT-4o is used as npc by default so an openai api key is required.

Run inside container:

```bash
# run inside container
$ sed -i 's/YOUR_OPENAI_API_KEY/{YOUR_OPENAI_API_KEY}/g' GRUtopia/demo/config/h1_npc.yaml  # set openai api key
$ python GRUtopia/demo/h1_npc.py  # start simulation
```

Now the simulation is available through WebRTC in the WebUI page.

You can control the h1 robot with keyboard command:

- W: Move Forward
- S: Move Backward
- A: Move Left
- D: Move Right
- Q: Turn Left
- E: Turn Right

And you can talk to npc as agent in the chatbox. The left side of the screen will display Isaac Sim's window, where you can switch to the robot's camera view. The right side features the chat window, where you can interact with the NPC. Ensure your questions are related to the scene, the robot's view, or its position, as unrelated queries might not yield useful responses. Replies will appear in the chat window within a few seconds. During this time, you can continue moving the robot or ask additional questions, which will be answered sequentially.

Note that the NPC might not always provide accurate answers due to design limitations.

Occasionally, unexpected responses from the LLM or code errors may cause issues. Check the error logs or contact us for support in resolving these problems.
