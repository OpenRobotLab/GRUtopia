FROM nvcr.io/nvidia/isaac-sim:2023.1.1

RUN apt update
RUN apt install -y nginx vim git

COPY ./webui/fe/html /usr/share/nginx/html
RUN chmod -R 755 /usr/share/nginx/html

COPY ./webui/fe/isaac/index.html /isaac-sim/extscache/omni.services.streamclient.webrtc-1.3.8/web/index.html
COPY . /isaac-sim/GRUtopia
WORKDIR /isaac-sim/GRUtopia

RUN mv ./webui/fe/nginx/default /etc/nginx/sites-available/default


# for isaac-sim:2023.1.1 webrtc error (0x800B1000)
RUN sed  -i "s/\"omni.kit.livestream.native\"/#\"omni.kit.livestream.native\"/g" /isaac-sim/apps/omni.isaac.sim.python.kit
RUN sed  -i "s/\"omni.kit.streamsdk.plugins\"/#\"omni.kit.streamsdk.plugins\"/g" /isaac-sim/apps/omni.isaac.sim.python.kit


RUN bash -c "cd ../ && \
    ./python.sh -m venv .venv && source .venv/bin/activate && \
    chmod +x ./GRUtopia/requirements/docker_install_req.sh && \
    cp ./GRUtopia/requirements/docker_install_req.sh . && \
    bash ./docker_install_req.sh && \
    chmod +x ./GRUtopia/webui_start.sh && \
    cp ./GRUtopia/webui_start.sh . && \
    sed 's/^\$python_exe/#\$python_exe/g' ./python.sh > python.env.init && \
    echo 'source /isaac-sim/.venv/bin/activate' >> /root/.bashrc && \
    echo ' . /isaac-sim/python.env.init' >> /root/.bashrc && \
    echo 'set +e' >> /root/.bashrc && \
    echo 'export MDL_SYSTEM_PATH=/isaac-sim/materials/' >> /root/.bashrc"

WORKDIR /isaac-sim

ENTRYPOINT ["/bin/bash"]
