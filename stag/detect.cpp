#include "src/Stag.h"
#include "../include/detect.h"

int detector::detect(cv::Mat &image, double markerLength, vector<cv::Vec3d> &tvecs, vector<cv::Vec3d> &pyrs, std::vector<int> &ids,
            cv::Mat cameraMatrix, cv::Mat distCoeffs) {
   tvecs.clear();
   pyrs.clear();
   ids.clear();
   cv::Mat RR(3, 3, CV_64F);
   cv::Mat RQ(3, 3, CV_64F);
   cv::Mat gray;
   cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
   std::vector<std::vector<cv::Point2f>> corners;

   std::vector<cv::Point2f> cnrs;
   Stag stag(11, 7, true);
   // std::cout << "test" << std::endl;
   std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
   stag.detectMarkers(gray);
   // std::cout << "test" << std::endl;
   for (int p = 0; p < stag.markers.size(); p++)
   {
   cv::Mat(stag.markers[p].corners).copyTo(cnrs);
   corners.push_back(cnrs);
   ids.push_back(stag.markers[p].id);
   }
   if (ids.size() > 0)
   {
      vector<cv::Vec3d> rvecs;
      cv::aruco::drawDetectedMarkers(image, corners, ids);
      cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
      std::cout << rvecs.size() << std::endl;
      for (int i = 0; i < rvecs.size(); i++)
      {

         cv::drawFrameAxes(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
         cv::Mat RotM;
         cv::Rodrigues(rvecs[i], RotM);
         // cv::Vec3d pyr = rotationMatrixToEulerAngles(RotM);
         cv::Vec3d pyr = cv::RQDecomp3x3(RotM,RR,RQ);
         pyr[0] = -(180-pyr[0]);
         if (pyr[0] < -180)
         pyr[0]+=360;
         pyrs.push_back(pyr);
      }
   }
   std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
   return (std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count());
}