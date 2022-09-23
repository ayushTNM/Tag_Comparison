#include <iostream>
#include "Comparison.h"


cv::Vec3d rotationMatrixToEulerAngles(cv::Mat &R)
{

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float roll, pitch, yaw;
    if (!singular)
    {
        yaw = atan2(R.at<double>(1,0) , R.at<double>(0,0));
        pitch = atan2(-R.at<double>(2,0),sy);
        roll = atan2(R.at<double>(2,1), R.at<double>(2,2));
    }
    else
    {
        yaw = atan2(-R.at<double>(1,2), R.at<double>(1,1));;
        pitch = atan2(-R.at<double>(2,0),sy);
        roll = 0;
    }
    return cv::Vec3d(yaw * (180/CV_PI), pitch * (180/CV_PI), roll * (180/CV_PI));

}


void detect (std::string marker_type, cv::Mat &image, double markerLength,
             vector<cv::Vec3d> &tvecs, vector<cv::Vec3d> &yprs) {

  tvecs.clear();
  yprs.clear();
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;

  if (marker_type == "aruco") {
    cv::aruco::detectMarkers(image, dictionary_aruco, corners, ids);
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
    
    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);
        apriltag_pose_t pose;
        apriltag_detection_info_t info = {det,markerLength,cameraMatrix.at<double>(0,0),cameraMatrix.at<double>(1,1),cameraMatrix.at<double>(0,2),cameraMatrix.at<double>(1,2)};
        double pose_error = estimate_tag_pose(&info,&pose);
        
        cv::Mat R(pose.R->nrows,pose.R->ncols,CV_64F,pose.R->data);
        cv::Vec3d tvec = {pose.t->data[0],pose.t->data[1],pose.t->data[2]};

        //Draw
        std::vector <cv::Point2f> cnrs;
        for (int p = 3; p >= 0;p--) {
          cnrs.push_back(cv::Point2f(det->p[p][0],det->p[p][1]));
        }
        corners.push_back(cnrs);
        ids.push_back(det->id);
        cv::aruco::drawDetectedMarkers(image,corners,ids);
        cv::drawFrameAxes(image, cameraMatrix, distCoeffs, R, tvec, 0.1);


        tvecs.push_back(tvec);
        cv::Vec3d ypr = rotationMatrixToEulerAngles(R);
        
        yprs.push_back(ypr);

        
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
    // std::map< int, cv::runetag::Pose > poses;
    
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
            // poses.push_back(RT);
            cv::Vec3d tvec = {RT.t.at<double>(0)/1000,RT.t.at<double>(1)/1000,RT.t.at<double>(2)/1000};
            cv::Mat RotM;
            cv::Rodrigues(RT.R,RotM);
            cv::Vec3d ypr = rotationMatrixToEulerAngles(RotM);
            tvecs.push_back(tvec);
            yprs.push_back(ypr);     
        } else
        {
            std::cout << " Error." << std::endl;
        }

    }
    return;
  }
  if (ids.size() > 0) {
    vector<cv::Vec3d> rvecs;
    cv::aruco::drawDetectedMarkers(image,corners,ids);
    cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
    std::cout << rvecs.size() << std::endl;
    for (int i = 0; i < rvecs.size();i++) {
      
      cv::drawFrameAxes(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
      cv::Mat RotM;
      cv::Rodrigues(rvecs[i],RotM);
      cv::Vec3d ypr = rotationMatrixToEulerAngles(RotM);
      yprs.push_back(ypr);
    }
  }
}

void save(string type, vector<string> out) {
  int count =0;
  // string filepath = "results/"+type;
  while (std::ifstream(type + ".txt"))
  {
      type += std::to_string(count);
      count++;
      // return false;
  }
  std::ofstream output_file(type + ".txt");
  std::ostream_iterator<std::string> output_iterator(output_file, "\n");
  std::copy(out.begin(), out.end(), output_iterator);
}


int main(void) {
  cv::VideoCapture inputVideo;
  string type = "april";
  double size = 0.033; //m
  // Timer clock;
  inputVideo.open(0);
  std::string filename = "calib.txt";
  readCameraParameters(filename, cameraMatrix, distCoeffs); // This function is located in detect_markers.cpp
  float frames = 0;
  float detFrames = 0;
  // detectCharucoBoardWithCalibrationPose();
  vector<string> data;
  while (inputVideo.grab()) {
    cv::Mat image, imageCopy;
    inputVideo.retrieve(image);
    image.copyTo(imageCopy);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::vector<cv::Vec3d> tvecs, yprs;
    detect(type, image, size, tvecs,yprs);
    
    frames++;
    if (yprs.size() > 0) {
      for(int i=0; i<yprs.size(); i++) {
      
        std::cout << "x " << tvecs[i][0] * 100<< " , y " << tvecs[i][1] * 100 << " , z " << tvecs[i][2] * 100 << std::endl;
        std::cout << "yaw " << yprs[i][0] << " , pitch " << yprs[i][1] << " , roll " << yprs[i][2] << std::endl;
        std::cout << std::endl;
        data.push_back("("+std::to_string( tvecs[i][0] * 100)+","+std::to_string( tvecs[i][1] * 100)+","+std::to_string( tvecs[i][2] * 100)+","+
                        std::to_string(yprs[i][0])+","+std::to_string(yprs[i][1])+","+std::to_string(yprs[i][0])+","+")"+" (null,null,null,null,null,null)");
      }
    }
    else {
      data.push_back("(null,null,null,null,null,null) (null,null,null,null,null,null)");
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count() << "[ms]" << std::endl;
    cv::imshow("out", image);
    // std::cout << "accuracy " << (detFrames/frames) << std::endl;
    
    char key = (char) cv::waitKey(1);
    if (key == 27)
        break;
  }
  save(type,data);
}