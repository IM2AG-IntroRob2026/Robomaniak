FROM alpine AS extract
WORKDIR /tmp
COPY src /tmp/src
RUN find src -type f ! -name "package.xml" -delete

FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y \
    ccache \
    python3-colcon-common-extensions \
    wget \
    tar \
    && rm -rf /var/lib/apt/lists/*

ENV ONNX_VERSION=1.24.4
RUN wget https://github.com/microsoft/onnxruntime/releases/download/v${ONNX_VERSION}/onnxruntime-linux-x64-${ONNX_VERSION}.tgz \
    && tar -xzf onnxruntime-linux-x64-${ONNX_VERSION}.tgz \
    && cp -r onnxruntime-linux-x64-${ONNX_VERSION}/include/* /usr/local/include/ \
    && cp -r onnxruntime-linux-x64-${ONNX_VERSION}/lib/* /usr/local/lib/ \
    && rm -rf onnxruntime-linux-x64-${ONNX_VERSION}* \
    && ldconfig

WORKDIR /ros2_ws

COPY --from=extract /tmp/src /ros2_ws/src

RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro jazzy && \
    rm -rf /var/lib/apt/lists/*