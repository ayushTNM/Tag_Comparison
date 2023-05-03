#include <chrono>
#include "opencv2/opencv.hpp"
#include <opencv2/aruco.hpp>

class detector {
    public:
    cv::Ptr<cv::aruco::Dictionary> dictionary_aruco = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    int detect(cv::Mat &image, double markerLength, std::vector<cv::Vec3d> &tvecs, std::vector<cv::Vec3d> &pyrs, std::vector<int> &ids,
            cv::Mat cameraMatrix, cv::Mat distCoeffs);
};