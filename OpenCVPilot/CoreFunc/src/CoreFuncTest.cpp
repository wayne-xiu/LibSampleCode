#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

using namespace cv;

using std::cout;
using std::endl;

void Mat_basics() {
    cout << "Mat object explicit creation:" << endl;
    Mat m1(2, 2, CV_8UC3, Scalar(0, 0, 255));
    cout << "m1 = " << m1 << endl << endl;

    // has issues
    // cout << "Use C/C++ array and initalize via constructor:" << endl;
    // int sz[3] = {2, 2, 2};
    // Mat m2(3, sz, CV_8UC1, Scalar::all(0));
    // cout << "m2 = " << m2 << endl << endl;

    cout << "MATLAB style initializer:" << endl;
    Mat m3 = Mat::eye(4, 4, CV_64F);
    cout << "m3 = " << m3 << endl << endl;
    Mat m4 = Mat::ones(3, 3, CV_32F);
    cout << "m4 = " << m4 << endl << endl;
    Mat m5 = Mat::zeros(2, 2, CV_8UC1);
    cout << "m5 = " << m5 << endl << endl;

    cout << "Initializer list for small matrices:" << endl;
    Mat m6 = (Mat_<double>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
    cout << "m6 = " << m6 << endl << endl;
    m6 = (Mat_<double>({0, 1, 2, 3, 4, 5, 6, 7, 8})).reshape(3, 3);  // row major
    cout << "m6 = " << m6 << endl;
    // TODO
    cout << "m6 size: " << m6.size() << ", " << m6.rows << "x" /*<< m6.cols()*/ << endl << endl;

    cout << "clone from existing Mat object" << endl;
    Mat m7 = m6.row(1).clone();
    cout << "RowClone = " << m7 << endl << endl;
    Mat m8 = m6.col(0).clone();
    cout << "ColumnClone = " << m8 << endl << endl;

    cout << "other common types" << endl;
    Point3f p3f(2, 6, 7);
    cout << "3D point: " << p3f << endl << endl;
}

void Mat_image_basics() {
    cv::Mat image1(240, 320, CV_8U, 100);
    cv::imshow("Image", image1);
    cv::waitKey(0);

    // assign a new image
    image1.create(200, 200, CV_8U);
    image1 = 200;
    cv::imshow("Image", image1);
    cv::waitKey(1000);

    // create a RED image; channels order BGR
    cv::Mat image2(240, 320, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::imshow("Image2", image2);
    cv::waitKey(500);

    // all images point to the same data
    cv::Mat image3 = cv::imread("../../Media/Lenna.png");
    cv::Mat image4(image3);
    image1 = image3;
    image3.copyTo(image2);
    cv::Mat image5 = image3.clone();
    cv::flip(image3, image3, 1);
    // check which images are affected
    cv::imshow("Image 3", image3);
    cv::imshow("Image 1", image1);
    cv::imshow("Image 2", image2);
    cv::imshow("Image 4", image4);
    cv::imshow("Image 5", image5);
    cv::waitKey(0);
    // image 1, 4 are referred to 3; while image 2, 5 are deep copy


    // load image as grayscale
    image1 = imread("../../Media/Lenna.png", IMREAD_GRAYSCALE);
    cv::imshow("Gray scale", image1);
    cv::waitKey(500);
    image1.convertTo(image2, CV_32F, 1/255.0, 0.0);
    cv::imshow("ImageG", image2);
    cv::waitKey(600);

    // basic info - notice the order of .size() output
    image1.create(100, 200, CV_8U);
    std::cout << "Image1 info: " << image1.rows << ", " << image1.cols << ", " << image1.channels() << std::endl;
    std::cout << "Image1 size: " << image1.size() << std::endl;
}


void image_traversal() {
    
}

int main() {

    // Mat_basics();
    Mat_image_basics();

    return 0;
}