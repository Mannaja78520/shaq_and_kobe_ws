cmake_minimum_required(VERSION 3.10)
project(HoopDetection)

set(CMAKE_CXX_STANDARD 17)

# ---------------------
# OpenCV
# ---------------------
find_package(OpenCV REQUIRED)

# ---------------------
# ONNX Runtime
# ---------------------
find_package(PkgConfig REQUIRED)

# ---------------------
# TensorFlow Lite
# ---------------------
# Customize this path to where libtensorflow-lite.a is located
set(TFLITE_DIR "$ENV{HOME}/tensorflow")

# Path to your libtensorflow-lite.a file
# set(TFLITE_LIB_PATH "${TFLITE_DIR}/tensorflow/lite/tools/make/gen/linux_x86_64/lib/libtensorflow-lite.a")


include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${TFLITE_DIR}
  ${TFLITE_DIR}/tensorflow/lite
  /usr/local/include      
)

link_directories(
  ${ONNXRUNTIME_LIBRARY_DIRS}
  /usr/local/lib         
)

# ---------------------
# Executables
# ---------------------
add_executable(test_hoop_detect_onnx test_hoop_detect_onnx.cpp)
add_executable(test_hoop_detect_tflite test_hoop_detect_tflite.cpp)

# ---------------------
# Link Libraries
# ---------------------
target_link_libraries(test_hoop_detect_onnx
  ${OpenCV_LIBS}
  onnxruntime
)

target_link_libraries(test_hoop_detect_tflite
  ${OpenCV_LIBS}
  # /home/mannaja/tensorflow/tensorflow/lite/build/libtensorflow-lite.a
  ${TFLITE_LIB_PATH}
  tensorflowlite
  pthread
  dl
  flatbuffers
)