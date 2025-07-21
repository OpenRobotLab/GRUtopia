FROM nvcr.io/nvidia/isaac-sim:4.5.0

RUN apt-get update
RUN apt-get install -y --allow-downgrades perl-base=5.34.0-3ubuntu1 netbase
RUN apt-get install -y vim git

COPY . /isaac-sim/InternUtopia
WORKDIR /isaac-sim/InternUtopia


RUN bash -c "cd ../ && \
    ./python.sh -m venv .venv && source .venv/bin/activate && \
    pip install --upgrade pip -i https://pypi.tuna.tsinghua.edu.cn/simple && \
    export PYTHONPATH=/isaac-sim/.venv/lib/python3.10/site-packages && \
    chmod +x ./InternUtopia/requirements/docker_install_req.sh && \
    cp ./InternUtopia/requirements/docker_install_req.sh . && \
    bash ./docker_install_req.sh && \
    sed 's/^\$python_exe/#\$python_exe/g' ./python.sh > python.env.init && \
    echo 'source /isaac-sim/.venv/bin/activate' >> /root/.bashrc && \
    echo ' . /isaac-sim/python.env.init' >> /root/.bashrc && \
    echo 'set +e' >> /root/.bashrc && \
    echo 'export MDL_SYSTEM_PATH=/isaac-sim/materials/' >> /root/.bashrc"
WORKDIR /isaac-sim
ENTRYPOINT ["/bin/bash"]
