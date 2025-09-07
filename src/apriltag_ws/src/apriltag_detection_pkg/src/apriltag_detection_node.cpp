#include <iostream>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

// 从旋转矩阵计算 RPY 角
cv::Vec3d rotationMatrixToRPY(const cv::Mat& R) {
    double sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
    bool singular = sy < 1e-6;

    double x, y, z;
    if (!singular) {
        x = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
        x = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return cv::Vec3d(x, y, z);
}

int main() {
    // 打开摄像头
    cv::VideoCapture cap(2);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera." << std::endl;
        return 1;
    }

    // 创建AprilTag检测器
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_add_family(td, tf);

    // 相机内参，需要根据实际相机进行校准
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);

    cv::Mat frame, grayFrame;
    while (true) {
        // 从摄像头读取一帧图像
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Failed to capture frame." << std::endl;
            break;
        }

        // 将彩色图像转换为灰度图像
        cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);

        // 将OpenCV图像转换为AprilTag格式
        image_u8_t img = {
                .width = grayFrame.cols,
                .height = grayFrame.rows,
                .stride = grayFrame.cols,
                .buf = grayFrame.data
        };

        // 检测AprilTag
        zarray_t *detections = apriltag_detector_detect(td, &img);

        // 处理检测结果
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            // 打印检测到的标签ID和位置
            std::cout << "Detected tag ID: " << det->id << std::endl;
            for (int j = 0; j < 4; j++) {
                std::cout << "Corner " << j << ": (" << det->p[j][0] << ", " << det->p[j][1] << ")" << std::endl;
            }

            // 在彩色图像上绘制标签的边界
            for (int j = 0; j < 4; j++) {
                cv::Point2f p1(det->p[j][0], det->p[j][1]);
                cv::Point2f p2(det->p[(j + 1) % 4][0], det->p[(j + 1) % 4][1]);
                cv::line(frame, p1, p2, cv::Scalar(0, 255, 0), 2);
            }

            // 在彩色图像上显示标签ID
            cv::putText(frame, std::to_string(det->id), cv::Point2f(det->c[0], det->c[1]), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

            // 定义AprilTag在世界坐标系中的三维坐标
            std::vector<cv::Point3f> objectPoints = {
                    cv::Point3f(-0.05, -0.05, 0),
                    cv::Point3f(0.05, -0.05, 0),
                    cv::Point3f(0.05, 0.05, 0),
                    cv::Point3f(-0.05, 0.05, 0)
            };
            std::vector<cv::Point2f> imagePoints = {
                    cv::Point2f(det->p[0][0], det->p[0][1]),
                    cv::Point2f(det->p[1][0], det->p[1][1]),
                    cv::Point2f(det->p[2][0], det->p[2][1]),
                    cv::Point2f(det->p[3][0], det->p[3][1])
            };

            // 求解相机的旋转向量和平移向量
            cv::Mat rvec, tvec;
            cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

            // 将旋转向量转换为旋转矩阵
            cv::Mat R;
            cv::Rodrigues(rvec, R);

            // 计算RPY角
            cv::Vec3d rpy = rotationMatrixToRPY(R);

            // 打印RPY角
            std::cout << "翻滚: " << rpy[0] * 180 / CV_PI << " 角度" << std::endl;
            std::cout << "俯仰: " << rpy[1] * 180 / CV_PI << " 角度" << std::endl;
            std::cout << "偏航: " << rpy[2] * 180 / CV_PI << " 角度" << std::endl;

            // 计算AprilTag到相机的距离
            double distance = std::abs(tvec.at<double>(2));
            std::cout << "AprilTag到相机的距离: " << distance << " 米" << std::endl;

            // 将AprilTag的位置转换到相机坐标系
            cv::Mat objectPointsMat(3, 1, CV_64F);
            objectPointsMat.at<double>(0, 0) = objectPoints[0].x;
            objectPointsMat.at<double>(1, 0) = objectPoints[0].y;
            objectPointsMat.at<double>(2, 0) = objectPoints[0].z;

            cv::Mat cameraCoords = R * objectPointsMat + tvec;

            std::cout << "AprilTag在相机坐标系下的位置: " << std::endl;
            std::cout << "点 0: (" << cameraCoords.at<double>(0, 0) << ", " << cameraCoords.at<double>(1, 0) << ", " << cameraCoords.at<double>(2, 0) << ")" << std::endl;
        }

        // 显示可视化后的图像
        cv::imshow("AprilTag Detection", frame);

        // 按下 'q' 键退出循环
        if (cv::waitKey(1) == 'q') {
            break;
        }

        // 清理检测结果
        apriltag_detections_destroy(detections);
    }

    // 释放摄像头资源
    cap.release();

    // 销毁AprilTag检测器和标签家族
    tag36h11_destroy(tf);
    apriltag_detector_destroy(td);

    // 关闭所有OpenCV窗口
    cv::destroyAllWindows();

    return 0;
}