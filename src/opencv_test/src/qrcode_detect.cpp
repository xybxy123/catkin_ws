//检测二维码
#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>

using namespace std;
using namespace cv;

int main()
{
    VideoCapture cap;
    cap.open(0);
    
    // 检查摄像头是否成功打开
    if(!cap.isOpened()){
        cout << "Error opening video stream" << endl;
        return -1;
    }

    Mat frame;
    Mat img_gray;
    Mat img_binary;

    QRCodeDetector qrcode_detector;
    vector<Point> points;  // 存储二维码角点
    vector<string> urlList;   // 存储解码结果

    namedWindow("original img",WINDOW_NORMAL);
    namedWindow("img_gray",WINDOW_NORMAL);
    namedWindow("result image",WINDOW_NORMAL);

    while(1)
    {
        cap>>frame;
        if(frame.empty()) break; // 检查帧是否为空
        
        imshow("original img",frame);  // 修正窗口名称

        cvtColor(frame, img_gray, COLOR_BGR2GRAY);
        imshow("img_gray",img_gray);

        threshold(img_gray, img_binary, 0, 255, THRESH_BINARY | THRESH_OTSU);
        imshow("img_binary", img_binary);

        string decode_info = qrcode_detector.detectAndDecode(img_binary, points);

    if (!decode_info.empty()) {
        cout << "Decoded QR Code:" << decode_info << endl;

        if (!points.empty()) {
            for (size_t i = 0; i < points.size(); i++) {
                line(frame, points[i], points[(i + 1) % points.size()], Scalar(0, 255, 0), 2);
            }
        }
	}
     else {
        cout << "No QR Code" << endl;
    }
        imshow("result image", frame);

        // 按 'q' 键退出循环
        char key = waitKey(30);
        if(key == 'q' || key == 27)  // 支持ESC键退出
        {
            break;
        }
    }

    // 释放资源
    cap.release();
    destroyAllWindows();

    return 0;
}
