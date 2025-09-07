#include<opencv2/opencv.hpp>
#include<iostream>

using namespace std;
using namespace cv;

// 修正函数声明：使用正确的参数格式
double calculate_color_area(Mat frame_hsv, Scalar low_range, Scalar high_range);

int main(int argc, char **argv){
    VideoCapture cap;
    cap.open(2);

    Mat frame, frame_HSV, frame_threshold;
    int frameCount = 0;
    namedWindow("original",WINDOW_NORMAL);
    namedWindow("detected",WINDOW_NORMAL);

    double t_detect = 0;

    while(1){
        cap>>frame;
        imshow("original",frame);

        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        
        t_detect = calculate_color_area(frame_HSV,Scalar(0,0,0),Scalar(180,255,255));
        
        cout<<t_detect<<endl;
        
        char key = waitKey(30);
        if(key == 's' || key == 'S'){
            string filename = "frame_" + to_string(frameCount++) + ".jpg";
            if (imwrite(filename, frame)) {
                cout << "已保存帧: " << filename << endl;
            } else {
                cerr << "ERROR: 保存帧失败" << endl;
            }
        }
        if (key == 'q' || key == 'Q' || key == 27) {
            break;
        } 
    }

    cap.release();
    destroyAllWindows();
    return 0;
}

double calculate_color_area(Mat frame_hsv, Scalar low_range, Scalar high_range) {
    Mat frame_threshold;
    // 创建颜色掩码
    inRange(frame_hsv, low_range, high_range, frame_threshold);

    imshow("detected",frame_threshold);
    
    // 计算白色像素数量
    int white_pixels = countNonZero(frame_threshold);
    
    // 计算总像素数量
    int total_pixels = frame_threshold.total();
    
    // 计算面积占比（百分比）
    double area_percentage = (double)white_pixels / total_pixels * 100.0;
    
    return area_percentage;
}
