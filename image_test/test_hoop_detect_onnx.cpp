#include <iostream>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <chrono>

using namespace std;
using namespace cv;
using namespace Ort;

const int input_width = 320;
const int input_height = 320;
const float confidence_threshold = 0.5f;

int main() {
    Env env(ORT_LOGGING_LEVEL_WARNING, "OnnxRuntime");
    SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    Session session(env, "../trainvschair.onnx", session_options);

    Ort::AllocatorWithDefaultOptions allocator;

    // Get input name
    auto input_name_alloc = session.GetInputNameAllocated(0, allocator);
    const char* input_name = input_name_alloc.get();  // used in Run()

    // Get output names
    size_t num_outputs = session.GetOutputCount();
    std::vector<std::string> output_names_str;
    std::vector<const char*> output_names;
    for (size_t i = 0; i < num_outputs; ++i) {
        auto out_name_alloc = session.GetOutputNameAllocated(i, allocator);
        output_names_str.emplace_back(out_name_alloc.get());
        output_names.push_back(output_names_str.back().c_str());
    }

    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Error: Could not open camera." << endl;
        return -1;
    }

    while (true) {
        Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        Mat resized;
        resize(frame, resized, Size(input_width, input_height));
        resized.convertTo(resized, CV_32FC3, 1.0f / 255.0f);

        // Prepare input tensor
        vector<float> input_tensor_values(input_width * input_height * 3);
        int idx = 0;
        for (int c = 0; c < 3; ++c) {
            for (int i = 0; i < input_height; ++i) {
                for (int j = 0; j < input_width; ++j) {
                    input_tensor_values[idx++] = resized.at<Vec3f>(i, j)[c];
                }
            }
        }

        array<int64_t, 4> input_shape = {1, 3, input_height, input_width};

        auto memory_info = MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        Value input_tensor = Value::CreateTensor<float>(memory_info, input_tensor_values.data(), input_tensor_values.size(), input_shape.data(), input_shape.size());

        // Run inference
        auto start = chrono::high_resolution_clock::now();
        std::vector<Value> output_tensors = session.Run(RunOptions{nullptr},
                                                        &input_name,
                                                        &input_tensor,
                                                        1,
                                                        output_names.data(),
                                                        output_names.size());
        auto end = chrono::high_resolution_clock::now();
        double inference_time = chrono::duration<double, milli>(end - start).count();

        // Assuming output[0] shape = (1, 5, N)
        float* output_data = output_tensors[0].GetTensorMutableData<float>();
        auto output_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
        size_t num_detections = output_shape[2];

        vector<float> best_det;
        float max_conf = 0.0f;

        for (size_t i = 0; i < num_detections; ++i) {
            float x = output_data[i * 5 + 0];
            float y = output_data[i * 5 + 1];
            float w = output_data[i * 5 + 2];
            float h = output_data[i * 5 + 3];
            float conf = output_data[i * 5 + 4];

            if (conf > confidence_threshold && conf > max_conf) {
                best_det = {x, y, w, h, conf};
                max_conf = conf;
            }
        }

        if (!best_det.empty()) {
            float x = best_det[0];
            float y = best_det[1];
            float w = best_det[2];
            float h = best_det[3];
            float conf = best_det[4];

            int x1 = static_cast<int>((x - w / 2.0f) * input_width);
            int y1 = static_cast<int>((y - h / 2.0f) * input_height);
            int x2 = static_cast<int>((x + w / 2.0f) * input_width);
            int y2 = static_cast<int>((y + h / 2.0f) * input_height);

            rectangle(resized, Point(x1, y1), Point(x2, y2), Scalar(0, 255, 255), 2);
            putText(resized, format("hoop: %.2f", conf), Point(x1, y1 - 5),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
        }

        putText(resized, format("Inference: %.1f ms", inference_time), Point(10, 25),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);

        imshow("Webcam Detections", resized);
        if (waitKey(10) == 'q') break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
