# 인천 국제 공항 실외 측방형 2차년도 - Wheel Detection

#### make_engine
engine 파일 만드는 폴더
step1: cd ./make_engine && mkdir build && cd build
step2: cmake .. && make -j64
step3 : python3 ../src/pt_onnx.py -> onnx file generated ,생성된 engine은 삭제 int32용 별도 생성
step4: ./onnx_engine ->engine파일 생성
step5: ./wheel_detector ->engine정상작동 확인,result_img_0,result_img_1 생성됨

#### enter_car_ws 
zed2i센서 2개 사용, ros2기반 wheel_detection streaming 
step0: cd {상위path}/enter_car_ws $$ gedit enter_car_ws/settings/camera.ini ->사용할 zed2개 serial number로 수정, 외측방형의 포크바가 차량을 바라봤을 때 오른쪽이 front, 왼쪽이 rear
step1: colcon build && source ./install/setup.bash
step2: ros2 run pkg_sensor_handler node_zed_camera_handler ->카메라 실행
steo3: ros2 run pkg_enter_car node_wheel_detection ->wheel_detection 실행