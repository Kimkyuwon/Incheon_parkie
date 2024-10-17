from ultralytics import YOLO

# Load a model
model = YOLO("../../enter_car_ws/weights/ver8n.pt") 

# Export the model 
model.export(format='onnx',imgsz=(480,640),batch=2,int8= True,simplify=True, dynamic=False,device=0)

# make engine process
# step1 : python3 pt_onnx_.py -> onnx file generated , engine file creation failed due to unknown issue
# step2 : cd build && ./onnx_engine ->engine file generated




