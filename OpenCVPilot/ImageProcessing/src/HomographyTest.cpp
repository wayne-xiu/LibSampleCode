#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main( int argc, char** argv)
{
    // Read source image.
    Mat im_src = imread("../../Media/book2.png");
    // Four corners of the book in source image
    vector<Point2f> pts_src;
    pts_src.push_back(Point2d(141, 131));
    pts_src.push_back(Point2f(480, 159));
    pts_src.push_back(Point2f(493, 630));
    pts_src.push_back(Point2f(64, 601));

    // Read destination image.
    Mat im_dst = imread("../../Media/book1.png");
    // Four corners of the book in destination image.
    vector<Point2f> pts_dst;
    pts_dst.push_back(Point2f(318, 256));
    pts_dst.push_back(Point2f(534, 372));
    pts_dst.push_back(Point2f(316, 670));
    pts_dst.push_back(Point2f(73, 473));

    // Calculate Homography
    Mat h = findHomography(pts_src, pts_dst);
    std::cout << h << std::endl;

    // Output image
    Mat im_out;
    // Warp source image to destination based on homography
    warpPerspective(im_src, im_out, h, im_dst.size());

    std::vector<cv::Point2f> transformed_points;
    perspectiveTransform(pts_src, transformed_points, h);
    for (auto& p: transformed_points) {
        cout << p << endl;
        // cout << p.x << ", " << p.y << endl;
    }

    // Display images
    imshow("Source Image", im_src);
    imshow("Destination Image", im_dst);
    imshow("Warped Source Image", im_out);

    waitKey(0);
}
