#include <opencv2/opencv.hpp>
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include <iostream>
#include <memory>

int main() {
    std::string model_path = "trainvschair_float32.tflite";
    std::string image_path = "hoop.png";

    // Load image
    cv::Mat image = cv::imread(image_path);
    if (image.empty()) {
        std::cerr << "Failed to load image: " << image_path << std::endl;
        return -1;
    }

    // Load model
    auto model = tflite::FlatBufferModel::BuildFromFile(model_path.c_str());
    if (!model) {
        std::cerr << "Failed to load model: " << model_path << std::endl;
        return -1;
    }

    tflite::ops::builtin::BuiltinOpResolver resolver;
    std::unique_ptr<tflite::Interpreter> interpreter;

    if (tflite::InterpreterBuilder(*model, resolver)(&interpreter) != kTfLiteOk || !interpreter) {
        std::cerr << "Failed to build interpreter." << std::endl;
        return -1;
    }

    if (interpreter->AllocateTensors() != kTfLiteOk) {
        std::cerr << "Failed to allocate tensors." << std::endl;
        return -1;
    }

    int input = interpreter->inputs()[0];
    TfLiteIntArray* dims = interpreter->tensor(input)->dims;
    int height = dims->data[1];
    int width = dims->data[2];
    int channels = dims->data[3];

    // Preprocess image
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(width, height));
    resized.convertTo(resized, CV_32FC3, 1.0 / 255.0);
    cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);

    // Copy to input tensor
    float* input_tensor = interpreter->typed_input_tensor<float>(0);
    std::memcpy(input_tensor, resized.data, sizeof(float) * height * width * channels);

    // Run inference
    if (interpreter->Invoke() != kTfLiteOk) {
        std::cerr << "Failed to run inference." << std::endl;
        return -1;
    }

    // Get output
    float* output = interpreter->typed_output_tensor<float>(0);
    float x = output[0];
    float y = output[1];
    float w = output[2];
    float h = output[3];
    float confidence = output[4];

    std::cout << "Confidence: " << confidence << std::endl;

    if (confidence > 0.2) {
        // Draw bounding box
        int x_pixel = static_cast<int>(x);
        int y_pixel = static_cast<int>(y);
        int box_w = static_cast<int>(w);
        int box_h = static_cast<int>(h);

        int x1 = x_pixel - box_w / 2;
        int y1 = y_pixel - box_h / 2;
        int x2 = x_pixel + box_w / 2;
        int y2 = y_pixel + box_h / 2;

        cv::rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), {0, 255, 0}, 2);
        cv::putText(image, "Hoop", cv::Point(x1, y1 - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 0, 0}, 1);
    }

    // Show image
    cv::imshow("Detected Hoop", image);
    cv::waitKey(0);

    return 0;
}
