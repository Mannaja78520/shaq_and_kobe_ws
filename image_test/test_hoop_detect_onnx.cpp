#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
#include <vector>
#include <string>

// Helper function to get class names (fill with your classes)
std::vector<std::string> getClassNames() {
    return { "class0", "class1", "class2" /* add your class names here */ };
}

int main() {
    // Load ONNX model
    std::string modelPath = "trainvschair.onnx";
    cv::dnn::Net net = cv::dnn::readNetFromONNX(modelPath);
    if (net.empty()) {
        std::cerr << "Failed to load ONNX model\n";
        return -1;
    }

    // Load image
    cv::Mat image = cv::imread("hoop.png");
    if (image.empty()) {
        std::cerr << "Image not found or invalid path!\n";
        return -1;
    }

    // Resize image to input size (640x480)
    cv::Size inputSize(640, 480);
    cv::Mat imageResized;
    cv::resize(image, imageResized, inputSize);

    cv::Mat imageTransposed;
    cv::transpose(imageResized, imageTransposed);

    // Prepare input blob
    cv::Mat blob = cv::dnn::blobFromImage(imageResized, 1.0/255.0, inputSize, cv::Scalar(), true, false);

    // Set input blob
    net.setInput(blob);

    // Run forward pass
    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    // Assuming YOLOv8 ONNX output format: [num_detections, 85] (for COCO classes: 80 classes + 5)
    // 85 = 4 box coords + 1 objectness + 80 class scores
    // You may need to adjust parsing depending on your model output

    float confThreshold = 0.25f;
    float nmsThreshold = 0.45f;

    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    // Retrieve output data (assume outputs[0] contains detections)
    cv::Mat& detections = outputs[0];

    int rows = detections.rows;

    for (int i = 0; i < rows; ++i) {
        float* data = (float*)detections.ptr(i);
        float confidence = data[4]; // objectness score

        if (confidence >= confThreshold) {
            // Get class scores and find max class id
            float* scores = data + 5;
            cv::Mat scoresMat(1, (int)(detections.cols - 5), CV_32FC1, scores);
            cv::Point classIdPoint;
            double maxClassScore;
            minMaxLoc(scoresMat, 0, &maxClassScore, 0, &classIdPoint);

            if (maxClassScore > confThreshold) {
                int centerX = (int)(data[0] * inputSize.width);
                int centerY = (int)(data[1] * inputSize.height);
                int width = (int)(data[2] * inputSize.width);
                int height = (int)(data[3] * inputSize.height);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)maxClassScore);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
    }

    // Perform Non-Maximum Suppression to eliminate redundant overlapping boxes
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);

    // Get class names
    std::vector<std::string> classNames = getClassNames();

    // Draw boxes and labels on the image
    for (int idx : indices) {
        cv::Rect box = boxes[idx];
        int clsId = classIds[idx];
        float conf = confidences[idx];

        cv::rectangle(imageResized, box, cv::Scalar(0, 255, 0), 2);

        std::string label = classNames.size() > clsId ? classNames[clsId] : std::to_string(clsId);
        label += ": " + cv::format("%.2f", conf);

        int baseline = 0;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, &baseline);
        int top = std::max(box.y, labelSize.height);

        cv::rectangle(imageResized, cv::Point(box.x, top - labelSize.height),
                      cv::Point(box.x + labelSize.width, top + baseline),
                      cv::Scalar(255, 255, 255), cv::FILLED);
        cv::putText(imageResized, label, cv::Point(box.x, top),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
    }

    // Show image with detections
    cv::imshow("YOLO_Onnx_Detection", imageResized);
    cv::waitKey(0);

    return 0;
}
