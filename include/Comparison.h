#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <boost/program_options.hpp>
#include "aruco_samples_utility.hpp"

#include <sstream>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/zlib.hpp>


// #include <opencv2/aruco_detector.hpp>

// #include <bits/stdc++.h>

// extern "C" {
// #include "apriltag/apriltag.h"
// #include "apriltag/apriltag_pose.h"
// #include "apriltag/tag36h11.h"
// #include "apriltag/tag25h9.h"
// #include "apriltag/tag16h5.h"
// #include "apriltag/tagCircle21h7.h"
// #include "apriltag/tagCircle49h12.h"
// #include "apriltag/tagCustom48h12.h"
// #include "apriltag/tagStandard41h12.h"
// #include "apriltag/tagStandard52h13.h"
// #include "apriltag/common/image_u8x3.h"
// }

// #include "stag/src/Stag.h"

// #include "stag2/src/Stag2.h"


// #include "runetag.hpp"
// #include "auxrenderer.hpp"
// #include "ellipsefitter.hpp"
#include <thread>
#include "WebcamStream.h"

// #include "serial/serial.h"
#include "gcode.h"
// #include "base64.h"

cv::Mat cameraMatrix, distCoeffs;
grbl controller;
// cv::Vec3f loc = cv::Vec3f(0,0,0),maxLoc;
double dist = 0;
// serial::Serial controller;

  // You can read camera parameters from tutorial_camera_params.yml
