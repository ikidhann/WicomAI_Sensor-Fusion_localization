FROM koide3/gtsam_docker:jazzy

RUN apt-fast update \
  && apt-fast install -y --no-install-recommends \
  libomp-dev libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev \
  && apt-fast clean \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /root
RUN git clone https://github.com/ceres-solver/ceres-solver \
  && cd ceres-solver \
  && git checkout e47a42c2957951c9fafcca9995d9927e15557069 \
  && mkdir build \
  && cd build \
  && cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DUSE_CUDA=OFF \
  && make -j$(nproc) \
  && make install \
  && rm -rf /root/ceres-solver

COPY direct_visual_lidar_calibration /root/ros2_ws/src/

WORKDIR /root/ros2_ws

RUN apt-fast update && \
  rosdep install -i --from-paths src --simulate | \
  sed '1d' | sed 's/apt-get install//' | sed 's/ //g' > /tmp/depends && \
  xargs apt-fast install --no-install-recommends -y < /tmp/depends && \
  apt-fast clean && \
  rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c ". /opt/ros/jazzy/setup.bash; colcon build"

RUN echo "#!/bin/bash" >> /ros_entrypoint.sh \
  && echo "set -e" >> /ros_entrypoint.sh \
  && echo "source /opt/ros/jazzy/setup.bash" >> /ros_entrypoint.sh \
  && echo "source /root/ros2_ws/install/setup.bash" >> /ros_entrypoint.sh \
  && echo "export PYTHONPATH=/opt/venv/lib/python3.12/site-packages:\${PYTHONPATH:-}" >> /ros_entrypoint.sh \
  && echo "export PYTHONPATH=\${PYTHONPATH}:/root/SuperGlue" >> /ros_entrypoint.sh \
  && echo 'exec "$@"' >> /ros_entrypoint.sh \
  && chmod a+x /ros_entrypoint.sh

WORKDIR /root/ros2_ws/src/direct_visual_lidar_calibration

RUN apt-fast update \
  && apt-fast install -y --no-install-recommends \
     python3-pip \
     python3-numpy \
     python3-matplotlib \
     python3-opencv \
     python3-venv \
  && apt-fast clean \
  && rm -rf /var/lib/apt/lists/* \
  && python3 -m venv --system-site-packages /opt/venv \
  && /bin/bash -c "source /opt/venv/bin/activate && \
     pip install --no-cache-dir torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121"

RUN git clone https://github.com/magicleap/SuperGluePretrainedNetwork /root/SuperGlue

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
