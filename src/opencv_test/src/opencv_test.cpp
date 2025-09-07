#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
#include <vector>
#include <string>

int main() {
    // 1. 加载模型
    std::string modelFile = "/home/xybxy/catkin_ws/src/yolov5-master/script/zipper_train_exp2/weights/best.onnx";
    cv::dnn::Net net = cv::dnn::readNetFromONNX(modelFile);
    if (net.empty()) {
        std::cerr << "Failed to load network!" << std::endl;
        return -1;
    }

    // 2. 打开摄像头（使用索引为2的摄像头）
    cv::VideoCapture cap(2);
    if (!cap.isOpened()) {
        std::cerr << "无法打开摄像头！" << std::endl;
        return -1;
    }

    // 3. 配置参数
    float confThreshold = 0.5;  // 置信度阈值
    std::vector<std::string> classNames = {"bad_zipper"};  // 仅保留一个类别

    // 处理视频流
    cv::Mat img;
    while (true) {
        // 读取摄像头帧
        cap >> img;
        if (img.empty()) {
            std::cerr << "无法获取图像帧！" << std::endl;
            break;
        }
        
        // 保存原图副本用于绘制
        cv::Mat img_with_detections = img.clone();

        // 4. 图像预处理
        cv::Mat blob = cv::dnn::blobFromImage(
            img, 
            1.0 / 255.0,  // 缩放至0-1
            cv::Size(640, 640),  // 模型输入尺寸
            cv::Scalar(0, 0, 0),  // 均值
            true,  // 交换RB通道
            false  // 不裁剪
        );

        // 5. 模型推理
        net.setInput(blob);
        cv::Mat output = net.forward();  // 输出形状：[1, N, 7]

        // 6. 解析输出（重塑为2D数组便于处理）
        cv::Mat detections = output.reshape(1, output.size[1]);  // 形状变为[N, 7]

        // 图像尺寸信息
        int imgWidth = img.cols;
        int imgHeight = img.rows;
        float scaleW = (float)imgWidth / 640;  // 宽度缩放比例
        float scaleH = (float)imgHeight / 640; // 高度缩放比例

        // 7. 遍历检测框并绘制
        for (int i = 0; i < detections.rows; i++) {
            const float* data = detections.ptr<float>(i);
            
            // 解析检测框属性
            float x = data[0] * scaleW;     // 映射到原图的中心x
            float y = data[1] * scaleH;     // 映射到原图的中心y
            float w = data[2] * scaleW;     // 映射到原图的宽度
            float h = data[3] * scaleH;     // 映射到原图的高度
            float objConf = data[4];        // 目标存在置信度
            float classConf = data[5];      // 仅使用第一个类别置信度

            // 过滤低置信度检测框
            if (objConf > confThreshold) {
                // 确定类别（只有一个类别）
                int classId = 0;  // 固定为0，对应bad_zipper
                
                // 计算边界框坐标
                int left = static_cast<int>(x - w / 2);
                int top = static_cast<int>(y - h / 2);
                int right = static_cast<int>(x + w / 2);
                int bottom = static_cast<int>(y + h / 2);

                // 确保边界框在图像范围内
                left = std::max(0, std::min(left, imgWidth - 1));
                top = std::max(0, std::min(top, imgHeight - 1));
                right = std::max(0, std::min(right, imgWidth - 1));
                bottom = std::max(0, std::min(bottom, imgHeight - 1));

                // 8. 绘制边界框
                cv::rectangle(img_with_detections, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 255, 0), 2);

                // 9. 绘制类别标签和置信度
                std::string label = classNames[classId] + " : " + 
                                   std::to_string(static_cast<int>(classConf * 100)) + "%";
                int baseLine;
                cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                cv::rectangle(img_with_detections,
                              cv::Point(left, top - labelSize.height),
                              cv::Point(left + labelSize.width, top + baseLine),
                              cv::Scalar(0, 255, 0), cv::FILLED);
                cv::putText(img_with_detections, label, cv::Point(left, top),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            }
        }

        // 10. 显示检测结果
        cv::namedWindow("Bad Zipper Detection", cv::WINDOW_NORMAL);
        cv::imshow("Bad Zipper Detection", img_with_detections);
        
        // 按下ESC键退出程序
        char c = (char)cv::waitKey(1);
        if (c == 27) {
            break;
        }
    }

    // 11. 释放资源
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
