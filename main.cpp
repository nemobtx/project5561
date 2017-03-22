#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

int main() {
  std::cout << "Hello, World!" << std::endl;
  Mat a = Mat::zeros(640, 480, CV_8U);
  imshow("test", a);
  waitKey();
  return 0;
}