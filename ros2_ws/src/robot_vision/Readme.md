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
ros2 run robot_vision detection_node --ros-args -p
  model_path:="$(ros2 pkg prefix robot_vision)/share/robot_vision/models/yolov8s.onnx"
  -p confidence_threshold:=0.5 -p use_gpu:=false -p debug_output_dir:="/tmp/robot_debug"
  -p debug_max_saves:=100 --remap cmd_lightring:=/Robot3/cmd_lightring
```