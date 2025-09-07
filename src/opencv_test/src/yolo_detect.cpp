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

    // 2. 读取图像
    cv::Mat img = cv::imread("/home/xybxy/catkin_ws/src/opencv_test/src/_cgi-bin_mmwebwx-bin_webwxgetmsgimg__&MsgID=325046517185808638&skey=@crypt_b2e5889f_df08cefebbbc030ad541c82e476591c7&mmweb_appid=wx_webfilehelper.jpeg");
    if (img.empty()) {
        std::cerr << "Failed to read image!" << std::endl;
        return -1;
    }
    // 保存原图副本用于绘制（避免修改原图）
    cv::Mat img_with_detections = img.clone();

    // 3. 图像预处理
    cv::Mat blob = cv::dnn::blobFromImage(
        img, 
        1.0 / 255.0,  // 缩放至0-1（根据模型训练时的预处理调整）
        cv::Size(640, 640),  // 模型输入尺寸
        cv::Scalar(0, 0, 0),  // 均值（YOLO系列通常用0）
        true,  // 交换RB通道（OpenCV默认BGR，模型通常用RGB）
        false  // 不裁剪
    );

    // 4. 模型推理
    net.setInput(blob);
    cv::Mat output = net.forward();  // 输出形状：[1, N, 7]（3D）

    // 5. 解析输出（重塑为2D数组便于处理）
    cv::Mat detections = output.reshape(1, output.size[1]);  // 形状变为[N, 7]

    // 6. 配置参数
    float confThreshold = 0.5;  // 置信度阈值（过滤低置信度检测框）
    std::vector<std::string> classNames = {"bad_zipper"};  // 类别名称（根据模型实际类别修改）
    int imgWidth = img.cols;    // 原图宽度
    int imgHeight = img.rows;   // 原图高度
    float scaleW = (float)imgWidth / 640;  // 宽度缩放比例（将640x640的检测框映射回原图）
    float scaleH = (float)imgHeight / 640; // 高度缩放比例

    // 7. 遍历检测框并绘制
    for (int i = 0; i < detections.rows; i++) {
        const float* data = detections.ptr<float>(i);
        
        // 解析检测框属性（x,y为中心坐标，w,h为宽高，均基于640x640输入）
        float x = data[0] * scaleW;     // 映射到原图的中心x
        float y = data[1] * scaleH;     // 映射到原图的中心y
        float w = data[2] * scaleW;     // 映射到原图的宽度
        float h = data[3] * scaleH;     // 映射到原图的高度
        float objConf = data[4];        // 目标存在置信度
        float class1Conf = data[5];     // 类别1置信度
        float class2Conf = data[6];     // 类别2置信度

        // 过滤低置信度检测框
        if (objConf > confThreshold) {
            // 确定最大置信度的类别
            float maxClassConf = std::max(class1Conf, class2Conf);
            int classId = (maxClassConf == class1Conf) ? 0 : 1;  // 类别ID

            // 计算边界框左上角和右下角坐标（OpenCV绘图需要）
            int left = static_cast<int>(x - w / 2);  // 左上角x
            int top = static_cast<int>(y - h / 2);   // 左上角y
            int right = static_cast<int>(x + w / 2); // 右下角x
            int bottom = static_cast<int>(y + h / 2); // 右下角y

            // 确保边界框在图像范围内（避免超出图像边界）
            left = std::max(0, std::min(left, imgWidth - 1));
            top = std::max(0, std::min(top, imgHeight - 1));
            right = std::max(0, std::min(right, imgWidth - 1));
            bottom = std::max(0, std::min(bottom, imgHeight - 1));

            // 8. 绘制边界框（绿色，线宽2）
            cv::rectangle(img_with_detections, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 255, 0), 2);

            // 9. 绘制类别标签和置信度
            std::string label = classNames[classId] + " : " + 
                               std::to_string(static_cast<int>(maxClassConf * 100)) + "%";
            // 计算标签文本尺寸
            int baseLine;
            cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            // 绘制标签背景（绿色填充）
            cv::rectangle(img_with_detections,
                          cv::Point(left, top - labelSize.height),
                          cv::Point(left + labelSize.width, top + baseLine),
                          cv::Scalar(0, 255, 0), cv::FILLED);
            // 绘制标签文本（黑色）
            cv::putText(img_with_detections, label, cv::Point(left, top),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        }
    }

    // 10. 显示检测结果
    cv::namedWindow("Detection Result", cv::WINDOW_NORMAL);  // 创建窗口
    cv::imshow("Detection Result", img_with_detections);     // 显示图像
    cv::waitKey(0);  // 等待按键（0表示无限等待）

    // 11. 释放资源
    cv::destroyAllWindows();
    return 0;
}
