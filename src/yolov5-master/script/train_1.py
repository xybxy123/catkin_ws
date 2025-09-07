from ultralytics import YOLO

# 直接使用库内置的 YOLOv5s 配置（无需手动指定本地 YAML 路径）
model = YOLO("yolov5s.yaml")  # 库会自动加载兼容的内置配置
model = YOLO("yolov5s.pt")    # 加载预训练权重（同样使用库自动下载的兼容版本）


# Train the model
results = model.train(
    data="/home/xybxy/catkin_ws/src/yolov5-master/script/date/zipper/test/rough/coco128.yaml",
    epochs=100,
    imgsz=640,
    batch=4,  # 关键：降低批次大小，从16→4（根据显存情况可尝试6或8）
    device=0,  # 显式指定使用GPU 0，避免自动分配导致的显存波动
    project="/home/xybxy/catkin_ws/src/yolov5-master/script",  # 自定义根目录
    name="zipper_train_exp"               # 自定义实验名称
)
