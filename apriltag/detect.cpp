extern "C"
{
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

#include "../include/detect.h"

int detector::detect(cv::Mat &image, double markerLength, std::vector<cv::Vec3d> &tvecs, std::vector<cv::Vec3d> &pyrs, std::vector<int> &ids,
                     cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
   tvecs.clear();
   pyrs.clear();
   ids.clear();
   cv::Mat RR(3, 3, CV_64F);
   cv::Mat RQ(3, 3, CV_64F);
   cv::Mat gray;
   cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
   std::vector<std::vector<cv::Point2f>> corners;
   std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
   apriltag_family_t *tf_april = tag36h11_create();

   apriltag_detector_t *td = apriltag_detector_create();
   apriltag_detector_add_family(td, tf_april);
   td->quad_decimate = 2;

   image_u8_t im = {.width = gray.cols,
                    .height = gray.rows,
                    .stride = gray.cols,
                    .buf = gray.data};

   zarray_t *detections = apriltag_detector_detect(td, &im);

   for (int i = 0; i < zarray_size(detections); i++)
   {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);
      apriltag_pose_t pose;
      apriltag_detection_info_t info = {det, markerLength, cameraMatrix.at<double>(0, 0), cameraMatrix.at<double>(1, 1), cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2)};
      double pose_error = estimate_tag_pose(&info, &pose);

      cv::Mat R(pose.R->nrows, pose.R->ncols, CV_64F, pose.R->data);
      cv::Vec3d tvec = {pose.t->data[0], pose.t->data[1], pose.t->data[2]};

      // Draw
      std::vector<cv::Point2f> cnrs;
      for (int p = 3; p >= 0; p--)
      {
         cnrs.push_back(cv::Point2f(det->p[p][0], det->p[p][1]));
      }
      corners.push_back(cnrs);
      ids.push_back(det->id);
      cv::aruco::drawDetectedMarkers(image, corners, ids);
      cv::drawFrameAxes(image, cameraMatrix, distCoeffs, R, tvec, 0.1);

      tvecs.push_back(tvec);
      // cv::Vec3d pyr = rotationMatrixToEulerAngles(R);

      cv::Vec3d pyr = cv::RQDecomp3x3(R, RR, RQ);

      pyrs.push_back(pyr);
   }
   apriltag_detections_destroy(detections);
   apriltag_detector_destroy(td);
   tag36h11_destroy(tf_april);
   std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
   return (std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count());
}