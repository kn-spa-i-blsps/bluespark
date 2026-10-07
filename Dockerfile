FROM ros:jazzy-ros-base
ENV DEBIAN_FRONTEND=noninteractive

# System dependencies & ROS 2 Jazzy packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1 \
    libglx-mesa0 \
    python3-colcon-common-extensions \
    git \
    nano \
    wget \
    curl \
    libcap-dev \
    geographiclib-tools \
    libgeographic-dev \
    ros-jazzy-mavros \
    ros-jazzy-mavros-msgs \
    ros-jazzy-mavros-extras \
    ros-jazzy-py-trees \
    ros-jazzy-py-trees-ros \
    ros-jazzy-vision-opencv \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets required by MAVROS
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

# Core Python & Vision ML dependencies (PEP 668 override for 24.04)
RUN pip3 install --no-cache-dir --upgrade pip --break-system-packages && \
    pip3 install --no-cache-dir --break-system-packages \
    MAVProxy \
    opencv-python \
    ultralytics \
    numpy

# Hardware interfaces (for deployment on physical Pi)
RUN pip3 install --no-cache-dir --break-system-packages \
    lgpio \
    adafruit-blinka \
    adafruit-circuitpython-pca9685 \
    adafruit-circuitpython-motor

COPY requirements.txt /tmp/requirements.txt
RUN if [ -s /tmp/requirements.txt ]; then pip3 install --no-cache-dir --break-system-packages -r /tmp/requirements.txt; fi

WORKDIR /workspace

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "alias cb='colcon build --symlink-install --parallel-workers 2'" >> /root/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /root/.bashrc

RUN git config --global --add safe.directory '*'

CMD ["bash"]
