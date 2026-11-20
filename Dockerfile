FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    python3-numpy \
    libgl1 \
    libglx-mesa0 \
    python3-colcon-common-extensions \
    git \
    nano \
    wget \
    curl \
    libcap-dev \
    geographiclib-tools \
    libgeographiclib-dev \
    ros-jazzy-mavros \
    ros-jazzy-mavros-msgs \
    ros-jazzy-mavros-extras \
    ros-jazzy-py-trees \
    ros-jazzy-py-trees-ros \
    python3-opencv \
    ros-jazzy-vision-opencv \
    ros-jazzy-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

RUN pip3 install --no-cache-dir \
    pymavlink \
    pyserial \
    defusedxml \
    pynmeagps && \
    pip3 install --no-cache-dir --no-deps MAVProxy && \
    pip3 install --no-cache-dir \
    "numpy<2" \
    ultralytics \
    lgpio \
    adafruit-blinka \
    adafruit-circuitpython-pca9685 \
    adafruit-circuitpython-motor

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /workspace

# Shell setup in /etc/bash.bashrc (not /root/.bashrc) so it applies to every
# user: root in the prod/compose containers and `ubuntu` in the Dev Container.
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc && \
    echo "alias cb='colcon build --symlink-install --parallel-workers 2'" >> /etc/bash.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /etc/bash.bashrc

RUN git config --system --add safe.directory '*'

# The Dev Container runs as the image's `ubuntu` user (UID 1000, like the usual
# host user) so files it creates in the /workspace bind mount (.git, build/, ...)
# are not owned by root on the host. Give it camera + serial (Pixhawk) access.
RUN usermod -aG video,dialout ubuntu

CMD ["bash"]
