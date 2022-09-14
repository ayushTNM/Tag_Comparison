#include <iostream>
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
#include "apriltag/tag36h11.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag16h5.h"
#include "apriltag/tagCircle21h7.h"
#include "apriltag/tagCircle49h12.h"
#include "apriltag/tagCustom48h12.h"
#include "apriltag/tagStandard41h12.h"
#include "apriltag/tagStandard52h13.h"
}

#include "stag/src/Stag.h"

#include "runetag.hpp"
#include "auxrenderer.hpp"
#include "ellipsefitter.hpp"
// using namespace cv;
// using namespace std;

// int main( int argc, char** argv )
// {
//   cout << "OpenCV version : " << CV_VERSION << endl;
//   cout << "Major version : " << CV_MAJOR_VERSION << endl;
//   cout << "Minor version : " << CV_MINOR_VERSION << endl;
//   cout << "Subminor version : " << CV_SUBMINOR_VERSION << endl;

//   if ( CV_MAJOR_VERSION < 3)
//   {
//       // Old OpenCV 2 code goes here.
//   } else
//   {
//       // New OpenCV 3 code goes here.
//   }
// }
cv::Ptr<cv::aruco::Dictionary> dictionary_aruco = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

// Camera Calibration (Charuco)
static inline void createBoard()
{
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary_aruco);
    cv::Mat boardImage;
    board->draw(cv::Size(500, 600), boardImage, 10, 1);
    cv::imwrite("BoardImage.jpg", boardImage);
}


static inline void detectCharucoBoardWithCalibrationPose()
{
    cv::VideoCapture inputVideo;
    inputVideo.open(0);
    cv::Mat cameraMatrix, distCoeffs;
    std::string filename = "calib.txt";
    bool readOk = readCameraParameters(filename, cameraMatrix, distCoeffs);
    if (!readOk) {
        std::cerr << "Invalid camera file" << std::endl;
    } else {
        // cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary_aruco);
        cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
        while (inputVideo.grab()) {
            cv::Mat image;
            cv::Mat imageCopy;
            inputVideo.retrieve(image);
            image.copyTo(imageCopy);
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f> > markerCorners;
            cv::aruco::detectMarkers(image, dictionary_aruco, markerCorners, markerIds, params);
            // if at least one marker detected
            if (markerIds.size() > 0) {
                cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
                std::vector<cv::Point2f> charucoCorners;
                std::vector<int> charucoIds;
                cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
                // if at least one charuco corner detected
                if (charucoIds.size() > 0) {
                    cv::Scalar color = cv::Scalar(255, 0, 0);
                    cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
                    cv::Vec3d rvec, tvec;
                    bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                    // if charuco pose is valid
                    if (valid)
                        cv::drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1f);
                }
            }
            cv::imshow("out", imageCopy);
            char key = (char)cv::waitKey(30);
            if (key == 27)
                break;
        }
    }
}

void detect (std::string marker_type, cv::Mat &image, std::vector<std::vector<cv::Point2f>> &corners, std::vector<int> &ids,
             cv::Mat cameraMatrix, cv::Mat distCoeffs) {
  corners.clear();
  ids.clear();
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
  if (marker_type == "aruco") {
    cv::aruco::detectMarkers(image, dictionary_aruco, corners, ids);
    return;
  }
  else if (marker_type == "april") {
    apriltag_family_t *tf_april = tag36h11_create();
    
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td,tf_april);
    td->quad_decimate = 2;
    
    image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

    zarray_t *detections = apriltag_detector_detect(td, &im);
    // detections->data
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        std::vector <cv::Point2f> cnrs;
        for (int p = 0; p < 4;p++) {
          cnrs.push_back(cv::Point2f(det->p[p][0],det->p[p][1]));
        }
        corners.push_back(cnrs);
        ids.push_back(det->id);
    }
    apriltag_detections_destroy(detections);
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf_april);
    return;
  }
  else if (marker_type == "stag") {
    std::vector <cv::Point2f> cnrs;
    Stag stag(11, 7, true);
    stag.detectMarkers(gray);
    
    for (int p = 0; p < stag.markers.size();p++){
      cv::Mat(stag.markers[p].corners).copyTo(cnrs);
      corners.push_back(cnrs);
      ids.push_back(stag.markers[p].id);
    }
  }
  else if (marker_type == "rune") {
    unsigned short min_ellipse_contour_points=10;
    unsigned short max_ellipse_contour_points=10000;
    float min_ellipse_area=100.0f;
    float max_ellipse_area=10000.0f;
    float min_roundness=0.3f;
    float max_mse = 0.3f;
    float size_compensation=-1.5f;
    cv::runetag::MarkerDetector* pDetector=0;
    
    pDetector = new cv::runetag::MarkerDetector( cameraMatrix );    

    try 
    {
        std::cout << " " << pDetector->addModelsFromFile( "/home/ayush/Downloads/Tag Comparison/RUNEtag/RUNETagGenerator/build/TestMarker.txt" ) <<  " loaded" << std::endl;
    } catch( cv::runetag::DigitalMarkerModel::MarkerModelLoadException& ex ) 
    {
        std::cout << std::endl << ex.what() << std::endl;
        return;
    }
    // cv::Mat intrinsics;
    

    std::vector< cv::RotatedRect > foundEllipses;
    cv::runetag::EllipseDetector ellipseDetector( min_ellipse_contour_points, 
        max_ellipse_contour_points, 
        min_ellipse_area,
        max_ellipse_area, 
        min_roundness, 
        max_mse, 
        size_compensation);    
    std::cout << "> Detecting ellipses" << std::endl;
    ellipseDetector.detectEllipses( image, foundEllipses );
    std::cout << "  " << foundEllipses.size() << " found." << std::endl << std::endl;

    std::cout << "> Detecting RUNE tags" << std::endl;
    std::vector< cv::runetag::MarkerDetected > tags_found;
    pDetector->dbgimage = image.clone();
    pDetector->detectMarkers( foundEllipses, tags_found);
    std::cout << "  " << tags_found.size() << " found." << std::endl << std::endl;
    // cv::Mat dbgout = input_image.clone();
    std::cout << "> Rendering tags" << std::endl;
    // std::cout << "   -> " << tagsimg_filename << std::endl << std::endl;
    std::cout << "> Estimating tags poses" << std::endl;
    std::map< int, cv::runetag::Pose > poses;
    
    for( size_t i=0; i<tags_found.size(); ++i )
    {
        bool poseok;
        
        std::cout << "  Tag IDX:" << tags_found[i].associatedModel()->getIDX();

		unsigned int flags=0;
		// if(pnpransac)
		// 	flags |= cv::runetag::FLAG_REPROJ_ERROR;
		// if(pnprefine)
			flags |= cv::runetag::FLAG_REFINE;

		cv::runetag::Pose RT = cv::runetag::findPose( tags_found[i], cameraMatrix, distCoeffs, &poseok, cv::SOLVEPNP_ITERATIVE, flags);

        if( poseok )
        {
            poses[i]=RT;
            std::cout << " OK! R(rodriguez):  [" << RT.R.at<double>(0,0) << " ; " << RT.R.at<double>(1,0) << " ; " << RT.R.at<double>(2,0) << "]" << std::endl;        
            std::cout << "                     T:  [" << RT.t.at<double>(0,0) << " ; " << RT.t.at<double>(1,0) << " ; " << RT.t.at<double>(2,0) << "]" << std::endl;        
        } else
        {
            std::cout << " Error." << std::endl;
        }

    }

    for( size_t i=0; i<tags_found.size(); ++i )
    {
      // cv::runetag::AuxRenderer::drawDetectedMarker(image,tags_found[i],cameraMatrix);
      if( poses.find(i)!=poses.end()) 
            {
                cv::runetag::AuxRenderer::drawDetectedMarker3DCylinder2(image,tags_found[i],poses[i],cameraMatrix,distCoeffs);
            }      
    }
  }
}

int main(void) {
  cv::VideoCapture inputVideo;
  // Timer clock;
  inputVideo.open(0);
  cv::Mat cameraMatrix, distCoeffs;
  // You can read camera parameters from tutorial_camera_params.yml
  // readCameraParameters(filename, cameraMatrix, distCoeffs); // This function is located in detect_markers.cpp
  float frames = 0;
  float detFrames = 0;
  // detectCharucoBoardWithCalibrationPose();
  while (inputVideo.grab()) {
    cv::Mat image, imageCopy;
    inputVideo.retrieve(image);
    image.copyTo(imageCopy);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    // std::cout << "test" << std::endl;
    cv::Mat cameraMatrix, distCoeffs;
    std::string filename = "calib.txt";
    readCameraParameters(filename, cameraMatrix, distCoeffs);
    detect("rune", image, corners, ids, cameraMatrix, distCoeffs);
    frames++;
  // std::cout << "test" << std::endl;
    // if at least one marker detected
    if (ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
        detFrames++;
        // std::vector<cv::Vec3d> rvecs, tvecs;
        // cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
        // // draw axis for each marker
        // for(int i=0; i<ids.size(); i++)
        //     cv::drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count() << "[ms]" << std::endl;
    cv::imshow("out", image);
    std::cout << "accuracy " << (detFrames/frames) << std::endl;
    
    char key = (char) cv::waitKey(1);
    if (key == 27)
        break;
  }
}