FROM alpine AS extract
WORKDIR /tmp
COPY src /tmp/src
RUN find src -type f ! -name "package.xml" -delete

FROM alpine AS downloader
ENV ONNX_VERSION=1.24.4
RUN apk add --no-cache wget tar
WORKDIR /deps
RUN wget https://github.com/microsoft/onnxruntime/releases/download/v${ONNX_VERSION}/onnxruntime-linux-x64-${ONNX_VERSION}.tgz \
    && tar -xzf onnxruntime-linux-x64-${ONNX_VERSION}.tgz

FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y \
    ccache \
    python3-colcon-common-extensions \
    libevdev-dev \
    && rm -rf /var/lib/apt/lists/*

COPY --from=downloader /deps/onnxruntime-linux-x64-1.24.4/include/* /usr/local/include/
COPY --from=downloader /deps/onnxruntime-linux-x64-1.24.4/lib/* /usr/local/lib/
RUN ldconfig

WORKDIR /ros2_ws

COPY --from=extract /tmp/src /ros2_ws/src

RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro jazzy && \
    rm -rf /var/lib/apt/lists/*
