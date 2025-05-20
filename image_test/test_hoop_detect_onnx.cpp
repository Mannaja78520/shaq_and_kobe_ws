#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <iostream>
#include <chrono>
#include <vector>
#include <algorithm>
#include <iomanip>

int main() {
    try {
        // 1. Initialize ONNX Runtime
        Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ONNX_DETECTION");
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(1);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

        // 2. Load model
        Ort::Session session(env, "../trainvschair.onnx", session_options);

        // 3. Get model info
        Ort::AllocatorWithDefaultOptions allocator;
        
        // Input info
        std::string input_name = session.GetInputNameAllocated(0, allocator).get();
        std::vector<const char*> input_names = {input_name.c_str()};
        auto input_shape = session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        
        // Output info
        size_t num_outputs = session.GetOutputCount();
        std::vector<std::string> output_names_str;
        std::vector<const char*> output_names;
        for (size_t i = 0; i < num_outputs; i++) {
            output_names_str.push_back(session.GetOutputNameAllocated(i, allocator).get());
            output_names.push_back(output_names_str.back().c_str());
        }

        const int input_height = input_shape[2];
        const int input_width = input_shape[3];

        // 4. Open camera
        cv::VideoCapture cap(0);
        if (!cap.isOpened()) {
            std::cerr << "ERROR: Failed to open camera" << std::endl;
            return -1;
        }

        cv::Mat frame, image_resized;
        while (true) {
            cap >> frame;
            if (frame.empty()) break;

            // 5. Preprocessing - MUST MATCH PYTHON EXACTLY
            cv::resize(frame, image_resized, cv::Size(input_width, input_height));
            
            // Convert to CHW and normalize - CRITICAL STEP
            std::vector<float> input_data(3 * input_height * input_width);
            for (int c = 0; c < 3; c++) {
                for (int h = 0; h < input_height; h++) {
                    for (int w = 0; w < input_width; w++) {
                        // IMPORTANT: Verify if your model expects BGR or RGB
                        // For BGR (OpenCV default): [c] index
                        // For RGB: [2 - c] index
                        input_data[c * input_height * input_width + h * input_width + w] = 
                            // image_resized.at<cv::Vec3b>(h, w)[c] / 255.0f; // BGR
                        image_resized.at<cv::Vec3b>(h, w)[2 - c] / 255.0f; // RGB
                    }
                }
            }

            // 6. Create input tensor
            std::vector<int64_t> input_tensor_shape = {1, 3, input_height, input_width};
            Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
                OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
            Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                memory_info, input_data.data(), input_data.size(),
                input_tensor_shape.data(), input_tensor_shape.size());

            // 7. Run inference
            auto start = std::chrono::high_resolution_clock::now();
            auto outputs = session.Run(Ort::RunOptions{nullptr}, 
                                    input_names.data(), &input_tensor, 1,
                                    output_names.data(), output_names.size());
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - start);

            // 8. Process outputs - verify your output format!
            float* raw_detections = outputs[0].GetTensorMutableData<float>();
            auto output_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
            
            // // DEBUG: Print output shape and first few values
            // std::cout << "Output shape: [";
            // for (auto dim : output_shape) std::cout << dim << " ";
            // std::cout << "]" << std::endl;

            // Correct interpretation of output shape
            // output_shape[0] = batch size (1)
            // output_shape[1] = number of features per detection (e.g., 5 for x,y,w,h,conf)
            // output_shape[2] = number of detections/anchors (e.g., 1344)
            
            const int features_per_detection = output_shape[1]; // Should be 5
            const int num_predictions = output_shape[2];        // Should be 1344

            // std::cout << "First 5 output values: ";
            // for (int i = 0; i < std::min(5, features_per_detection * num_predictions); i++) {
            //     std::cout << raw_detections[i] << " ";
            // }
            // std::cout << std::endl;

            // 9. Post-processing - ADJUST BASED ON YOUR MODEL'S OUTPUT FORMAT
            std::vector<std::vector<float>> valid_detections;
            for (int i = 0; i < num_predictions; i++) {
                // Access detection features correctly
                // If output is [features_per_detection, num_predictions]
                // For example, if output is [5, 1344]
                // det[0] is x for current detection, det[1] is y, etc.
                float x = raw_detections[0 * num_predictions + i];
                float y = raw_detections[1 * num_predictions + i];
                float w = raw_detections[2 * num_predictions + i];
                float h = raw_detections[3 * num_predictions + i];
                float conf = raw_detections[4 * num_predictions + i]; // Confidence is typically at index 4

                if (conf > 0.35f) {
                    valid_detections.push_back({x, y, w, h, conf});
                }
            }

            // 10. Visualization
            if (!valid_detections.empty()) {
                auto best = *std::max_element(valid_detections.begin(), valid_detections.end(),
                    [](const std::vector<float>& a, const std::vector<float>& b) {
                        return a[4] < b[4];
                    });

                int x1 = static_cast<int>((best[0] - best[2]/2) * input_width);
                int y1 = static_cast<int>((best[1] - best[3]/2) * input_height);
                int x2 = static_cast<int>((best[0] + best[2]/2) * input_width);
                int y2 = static_cast<int>((best[1] + best[3]/2) * input_height);

                cv::rectangle(image_resized, cv::Point(x1, y1), cv::Point(x2, y2), 
                 cv::Scalar(255, 255, 30), 2);
                
                // Original: std::cout << "%.2f", best[4] << std::endl;
                std::cout << std::fixed << std::setprecision(2) << "hoop: " << best[4] << std::endl;
                
                char label[50];
                snprintf(label, sizeof(label), "hoop: %.2f", best[4]);
                cv::putText(image_resized, label, cv::Point(x1, y1-5),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 30), 1);
            }

            // Display inference time
            char time_text[50];
            snprintf(time_text, sizeof(time_text), "Inference: %.1f ms", static_cast<double>(duration.count()));
            cv::putText(image_resized, time_text, cv::Point(10, 25),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

            cv::imshow("Detection", image_resized);
            if (cv::waitKey(10) == 'q') break;
        }

        cap.release();
        cv::destroyAllWindows();
    } 
    catch (const Ort::Exception& e) {
        std::cerr << "ONNX Runtime Error: " << e.what() << std::endl;
        return -1;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}