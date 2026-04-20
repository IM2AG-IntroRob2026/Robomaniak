FROM alpine AS extract
WORKDIR /tmp
COPY src /tmp/src
RUN find src -type f ! -name "package.xml" -delete

FROM alpine AS downloader
ENV ONNX_VERSION=1.23.2
ENV SHERPA_ONNX_VERSION=1.12.35
RUN apk add --no-cache wget tar bzip2
WORKDIR /deps
RUN wget https://github.com/microsoft/onnxruntime/releases/download/v${ONNX_VERSION}/onnxruntime-linux-x64-${ONNX_VERSION}.tgz \
    && tar -xzf onnxruntime-linux-x64-${ONNX_VERSION}.tgz
RUN wget https://github.com/k2-fsa/sherpa-onnx/releases/download/v${SHERPA_ONNX_VERSION}/sherpa-onnx-v${SHERPA_ONNX_VERSION}-linux-x64-shared-no-tts.tar.bz2 \
    && tar -xjf sherpa-onnx-v${SHERPA_ONNX_VERSION}-linux-x64-shared-no-tts.tar.bz2

FROM osrf/ros:iron-desktop

RUN apt-get update && apt-get install -y \
    ccache \
    python3-colcon-common-extensions \
    ros-iron-rmw-cyclonedds-cpp \
    libevdev-dev \
    portaudio19-dev \
    libportaudio2 \
    git \
    ros-iron-behaviortree-cpp-v3 \
    ros-iron-vision-msgs \
    ros-iron-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN mkdir -p /usr/local/include/sherpa-onnx/c-api
COPY --from=downloader /deps/onnxruntime-linux-x64-1.23.2/include/* /usr/local/include/
COPY --from=downloader /deps/onnxruntime-linux-x64-1.23.2/lib/* /usr/local/lib/
COPY --from=downloader /deps/sherpa-onnx-v1.12.35-linux-x64-shared-no-tts/include/sherpa-onnx/c-api/* /usr/local/include/sherpa-onnx/c-api/
COPY --from=downloader /deps/sherpa-onnx-v1.12.35-linux-x64-shared-no-tts/lib/* /usr/local/lib/
RUN ldconfig

WORKDIR /root/create3_ws/src
RUN git clone -b 2.1.0 https://github.com/iRobotEducation/irobot_create_msgs.git
WORKDIR /root/create3_ws
RUN . /opt/ros/iron/setup.sh && colcon build

WORKDIR /ros2_ws
COPY --from=extract /tmp/src /ros2_ws/src

RUN . /opt/ros/iron/setup.sh && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro iron --skip-keys "behaviortree_cpp" --skip-keys "irobot_create_msgs" --skip-keys "vision_msgs" && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc \
    && echo "source /root/create3_ws/install/setup.bash" >> ~/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

RUN echo "source /root/create3_ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]