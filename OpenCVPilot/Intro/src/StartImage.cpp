#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <string>

using namespace cv;

void check_installation()
{
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;
}

void load_image(const std::string &filename = std::string())
{
    std::string imagefile = filename;
    if (imagefile.empty())
    {
        imagefile = "../../Media/Lenna.png";
    }
    Mat image;
    image = imread(imagefile, 1);
    if (!image.data)
    {
        printf("No image data \n");
        return;
    }

    cv::Mat flip_image;
    cv::flip(image, flip_image, 1);

    namedWindow("Display Image", WINDOW_AUTOSIZE);
    imshow("Display Image", image);
    namedWindow("Flipped image");
    imshow("Flipped image", flip_image);

    waitKey(0);
}

void load_write_image()
{
    std::string image_path = samples::findFile("starry_night.jpg");
    Mat img = imread(image_path, IMREAD_COLOR);

    if (img.empty())
    {
        std::cout << "Could not read the image: " << image_path << std::endl;
    }
    imshow("Display window", img);
    int k = waitKey(0); // wait for a keystroke
    if (k == 's')
    {
        imwrite("../../Media/starry_night.png", img);
    }
}

int main(int argc, char **argv)
{
    check_installation();
    load_image();
    load_write_image();

    return 0;
}