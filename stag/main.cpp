#include "Stag.h"
#include "opencv2/opencv.hpp"

int main() {
  cv::Mat image = cv::imread("1.png");
  cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);

  Stag stag(15, 7, true);

  stag.detectMarkers(image);
  stag.logResults("");
  return 0;
}