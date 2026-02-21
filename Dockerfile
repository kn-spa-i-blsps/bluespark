FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1-mesa-glx \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# to prevent crashing building with colcon
RUN echo "alias cb='colcon build --symlink-install --parallel-workers 2'" >> /root/.bashrc

RUN echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /root/.bashrc