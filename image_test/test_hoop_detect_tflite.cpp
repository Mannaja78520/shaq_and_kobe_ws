#include <opencv2/opencv.hpp>
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include <iostream>
#include <chrono>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <memory>

int main() {
    try {
        // 1. Load TFLite model
        std::unique_ptr<tflite::FlatBufferModel> model = 
            tflite::FlatBufferModel::BuildFromFile("trainvschair_float16.tflite");
        if (!model) {
            std::cerr << "ERROR: Failed to load TFLite model" << std::endl;
            return -1;
        }

        // 2. Create interpreter
        tflite::ops::builtin::BuiltinOpResolver resolver;
        std::unique_ptr<tflite::Interpreter> interpreter;
        tflite::InterpreterBuilder(*model, resolver)(&interpreter);
        if (!interpreter) {
            std::cerr << "ERROR: Failed to create interpreter" << std::endl;
            return -1;
        }

        // 3. Allocate tensors
        if (interpreter->AllocateTensors() != kTfLiteOk) {
            std::cerr << "ERROR: Failed to allocate tensors" << std::endl;
            return -1;
        }

        // 4. Get input tensor details
        int input_tensor_idx = interpreter->inputs()[0];
        TfLiteTensor* input_tensor = interpreter->tensor(input_tensor_idx);
        const int input_height = input_tensor->dims->data[2];
        const int input_width = input_tensor->dims->data[3];

        // 5. Open camera
        cv::VideoCapture cap(0);
        if (!cap.isOpened()) {
            std::cerr << "ERROR: Failed to open camera" << std::endl;
            return -1;
        }

        cv::Mat frame, image_resized;
        while (true) {
            cap >> frame;
            if (frame.empty()) break;

            // 6. Preprocessing - MUST MATCH PYTHON EXACTLY
            cv::resize(frame, image_resized, cv::Size(input_width, input_height));
            
            // Convert to CHW and normalize - CRITICAL STEP
            float* input_data = interpreter->typed_input_tensor<float>(0);
            for (int c = 0; c < 3; c++) {
                for (int h = 0; h < input_height; h++) {
                    for (int w = 0; w < input_width; w++) {
                        // IMPORTANT: Verify if your model expects BGR or RGB
                        // For BGR (OpenCV default): [c] index
                        // For RGB: [2 - c] index
                        input_data[c * input_height * input_width + h * input_width + w] = 
                            image_resized.at<cv::Vec3b>(h, w)[2 - c] / 255.0f; // RGB
                    }
                }
            }

            // 7. Run inference
            auto start = std::chrono::high_resolution_clock::now();
            if (interpreter->Invoke() != kTfLiteOk) {
                std::cerr << "ERROR: Failed to invoke interpreter" << std::endl;
                return -1;
            }
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - start);

            // 8. Get output tensor
            int output_tensor_idx = interpreter->outputs()[0];
            TfLiteTensor* output_tensor = interpreter->tensor(output_tensor_idx);
            float* raw_detections = output_tensor->data.f;
            
            // Get output shape
            const int features_per_detection = output_tensor->dims->data[1]; // Should be 5
            const int num_predictions = output_tensor->dims->data[2];        // Should be 1344

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
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}