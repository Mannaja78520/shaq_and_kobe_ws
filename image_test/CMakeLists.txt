cmake_minimum_required(VERSION 3.10)
project(OnnxWebcamDetection)

set(CMAKE_CXX_STANDARD 17)

# Path to ONNX Runtime and OpenCV (adjust these if needed)
# Or use environment variables instead if preferred
# include_directories(/path/to/onnxruntime/include)
# link_directories(/path/to/onnxruntime/lib)
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)

add_executable(test_hoop_detect_onnx test_hoop_detect_onnx.cpp)

target_link_libraries(test_hoop_detect_onnx ${OpenCV_LIBS} onnxruntime)
