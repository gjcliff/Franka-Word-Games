# Hangman using OCR
## Quickstart
1. Install PaddlePaddle using `python -m pip install paddlepaddle-gpu -i https://pypi.tuna.tsinghua.edu.cn/simple`
2. Install paddleocr using `pip install "paddleocr>=2.0.1" # Recommend to use version 2.0.1+`
3. Install Imutils using `pip install imutils`
4. Run the Realsense node using `ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30`
5. Run the image modification node using `ros2 run ocr image_modification`
6. Run the OCR node using `ros2 run ocr paddle_ocr`