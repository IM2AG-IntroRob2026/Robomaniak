## onnxruntime

https://github.com/microsoft/onnxruntime/releases

```bash
sudo cp -r <ORT_VARIANT>/include/* /usr/local/include/
sudo cp -r <ORT_VARIANT>/lib/* /usr/local/lib/
sudo ldconfig
```

## YOLOv8

```bash
python3 -m venv yolo_env
source yolo_env/bin/activate
pip install ultralytics

python3 - <<'EOF'
from ultralytics import YOLO
model = YOLO("yolov8n.pt")
model.export(format="onnx", opset=12, imgsz=640, simplify=True)
print("Export terminé : yolov8n.onnx")
EOF

deactivate

cp yolov8n.onnx ~/ros2_ws/src/robot_vision/models/
```

## Flux vidéo

```bash
ros2 run robot_vision video_node --ros-args -p stream_url:="http://10.202.48.94:8080/video" -p publish_rate_hz:=30
```

```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

## Détection

```bash
`ros2 run robot_vision detection_node --ros-args -p model_path:="$(ros2 pkg prefix robot_vision)/share/robot_vision/models/models/yolov8s.onnx" -p confidence_threshold:=0.5 -p use_gpu:=false --remap cmd_lightring:=/Robot3/cmd_lightring
````

## Suivi de cible

```bash
```ros2 run robot_vision follow_node --ros-args -p strategy:="most_centered" -p angular_dead_zone:=0.2 -p allow_reverse:=false -p target_bbox_height:=1000.0 -p lock_lost_frames:=10 -p linear_dead_zone:=50.0 -p allow_reverse:=true --remap /ir_intensity:=/Robot3/ir_intensity --remap /hazard_detection:=/Robot3/hazard_detection
``````

## BT Manager

```bash
`ros2 run robot_vision bt_manager_node --ros-args -p bt_tick_hz:=50.0 --remap /cmd_vel:=/Robot3/cmd_vel
````

```bash
`ros2 run robot_vision teleop_node
````

## Listen node

```bash
MODEL_DIR=$(ros2 pkg prefix robot_vision)/share/robot_vision/models && \
     ros2 run robot_vision listen_node --ros-args \
       -p encoder_path:=\"${MODEL_DIR}/kws_encoder.onnx\" \
       -p decoder_path:=\"${MODEL_DIR}/kws_decoder.onnx\" \
       -p joiner_path:=\"${MODEL_DIR}/kws_joiner.onnx\" \
       -p tokens_path:=\"${MODEL_DIR}/kws_tokens.txt\" \
       --log-level listen_node:=debug
```