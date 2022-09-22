#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <boost/program_options.hpp>
#include "aruco_samples_utility.hpp"


#include <opencv2/aruco.hpp>
// #include <opencv2/aruco_detector.hpp>
#include <opencv2/aruco/dictionary.hpp>

extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/apriltag_pose.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag16h5.h"
#include "apriltag/tagCircle21h7.h"
#include "apriltag/tagCircle49h12.h"
#include "apriltag/tagCustom48h12.h"
#include "apriltag/tagStandard41h12.h"
#include "apriltag/tagStandard52h13.h"
#include "apriltag/common/image_u8x3.h"
}

#include "stag/src/Stag.h"

#include "runetag.hpp"
#include "auxrenderer.hpp"
#include "ellipsefitter.hpp"

cv::Mat cameraMatrix, distCoeffs;
cv::Ptr<cv::aruco::Dictionary> dictionary_aruco = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  // You can read camera parameters from tutorial_camera_params.yml
