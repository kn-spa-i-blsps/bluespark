FROM ros:humble-ros-base
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1-mesa-glx \
    python3-colcon-common-extensions \
    git \
    nano \
    wget \
    curl \
    libcap-dev \
    geographiclib-tools \
    libgeographic-dev \
    ros-humble-mavros \
    ros-humble-mavros-msgs \
    ros-humble-mavros-extras \
    ros-humble-py-trees \
    ros-humble-py-trees-ros \
    python3-opencv \
    ros-humble-vision-opencv \
    ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*


RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

# Heavy requirements that do not change often.
RUN pip3 install --upgrade pip && \
    pip3 install MAVProxy opencv-python ultralytics numpy

RUN pip3 install \
    lgpio \
    adafruit-blinka \
    adafruit-circuitpython-pca9685 \
    adafruit-circuitpython-motor

# Lighter or more susceptible to change requirements.
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt

WORKDIR /workspace

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "alias cb='colcon build --symlink-install --parallel-workers 2'" >> /root/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /root/.bashrc

# Turn off some git warnings.
RUN git config --global --add safe.directory '*'

CMD ["bash"]
