#include <iostream>
// #include <string.h>

#include "Comparison.h"
#include "detect.h"

#ifdef MARKER_TYPE
    #define marker_type MARKER_TYPE
#endif

// extern int detect(cv::Mat &image, double markerLength, std::vector<cv::Vec3d> &tvecs, std::vector<cv::Vec3d> &pyrs, std::vector<int> &ids,
//             // cv::Mat cameraMatrix, cv::Mat distCoeffs);

cv::Vec3f REAL_PYR = cv::Vec3f(0,0,0); // Yaw, Roll, Pitch
int SPEED = 15000;
std::string MARKER_SYSTEM = marker_type;
float CAM_DIST = 60;
int ID;
double MARKERLENGTH = 0.0589; // m


// cv::Vec3d rotationMatrixToEulerAngles(cv::Mat &R)
// {

//   float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

//   bool singular = sy < 1e-6; // If

//   float roll, pitch, yaw;
//   if (!singular)
//   {
//     yaw = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
//     pitch = atan2(-R.at<double>(2, 0), sy);
//     roll = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
//   }
//   else
//   {
//     yaw = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
//     pitch = atan2(-R.at<double>(2, 0), sy);
//     roll = 0;
//   }
//   return cv::Vec3d(yaw * (180 / CV_PI), pitch * (180 / CV_PI), roll * (180 / CV_PI));
// }

cv::Vec3f angleToAx(float angle) {
  angle = (angle*CV_PI)/180;
  return cv::Vec3f(cos(CV_PI-angle), sin(angle),0);
}

cv::Mat AxAngToRotMat(cv::Vec3f ax, float angle) {
  angle = (angle*CV_PI)/180;
  float c = cos(angle);
  float s = sin(angle);
  float t = 1- c;
  float x = ax[0],y = ax[1],z = ax[2];
  return (cv::Mat_<double>(3,3) <<  t*x*x + c,    t*x*y - z*s,  t*x*z + y*s,
                                    t*x*y + z*s,  t*y*y + c,    t*y*z - x*s,
                                    t*x*z - y*s,  t*y*z + x*s,  t*z*z + c);
}



// int detect(std::string marker_type, cv::Mat &image, double markerLength,
//             vector<cv::Vec3d> &tvecs, vector<cv::Vec3d> &pyrs, std::vector<int> &ids)
// {

//   tvecs.clear();
//   pyrs.clear();
//   ids.clear();
//   cv::Mat RR(3, 3, CV_64F);
//   cv::Mat RQ(3, 3, CV_64F);
//   cv::Mat gray;
//   cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
//   std::vector<std::vector<cv::Point2f>> corners;
//   // std::vector<int> ids;
//   std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
//   if (marker_type == "ArUco")
//   {
    // cv::aruco::detectMarkers(image, dictionary_aruco, corners, ids);
//   }
//   else if (marker_type == "AprilTag")
//   {
//     apriltag_family_t *tf_april = tag36h11_create();

//     apriltag_detector_t *td = apriltag_detector_create();
//     apriltag_detector_add_family(td, tf_april);
//     td->quad_decimate = 2;

//     image_u8_t im = {.width = gray.cols,
//                      .height = gray.rows,
//                      .stride = gray.cols,
//                      .buf = gray.data};

//     zarray_t *detections = apriltag_detector_detect(td, &im);

//     for (int i = 0; i < zarray_size(detections); i++)
//     {
//       apriltag_detection_t *det;
//       zarray_get(detections, i, &det);
//       apriltag_pose_t pose;
//       apriltag_detection_info_t info = {det, markerLength, cameraMatrix.at<double>(0, 0), cameraMatrix.at<double>(1, 1), cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2)};
//       double pose_error = estimate_tag_pose(&info, &pose);

//       cv::Mat R(pose.R->nrows, pose.R->ncols, CV_64F, pose.R->data);
//       cv::Vec3d tvec = {pose.t->data[0], pose.t->data[1], pose.t->data[2]};

//       // Draw
//       std::vector<cv::Point2f> cnrs;
//       for (int p = 3; p >= 0; p--)
//       {
//         cnrs.push_back(cv::Point2f(det->p[p][0], det->p[p][1]));
//       }
//       corners.push_back(cnrs);
//       ids.push_back(det->id);
//       cv::aruco::drawDetectedMarkers(image, corners, ids);
//       cv::drawFrameAxes(image, cameraMatrix, distCoeffs, R, tvec, 0.1);

//       tvecs.push_back(tvec);
//       // cv::Vec3d pyr = rotationMatrixToEulerAngles(R);
      

//       cv::Vec3d pyr = cv::RQDecomp3x3(R,RR,RQ);

//       pyrs.push_back(pyr);
//     }
//     apriltag_detections_destroy(detections);
//     apriltag_detector_destroy(td);
//     tag36h11_destroy(tf_april);
//     goto end;
//   }
//   else if (marker_type == "STag")
//   {
//     std::vector<cv::Point2f> cnrs;
//     Stag stag(11, 7, true);
//     stag.detectMarkers(gray);

//     for (int p = 0; p < stag.markers.size(); p++)
//     {
//       cv::Mat(stag.markers[p].corners).copyTo(cnrs);
//       corners.push_back(cnrs);
//       ids.push_back(stag.markers[p].id);
//     }
//   }
//   else if (marker_type == "STag2")
//   {
//     std::vector<cv::Point2f> cnrs;
//     Stag2 stag2(11, 7, true);
//     // std::cout << "test" << std::endl;
//     stag2.detectMarkers(gray);
//     // std::cout << "test" << std::endl;
//     for (int p = 0; p < stag2.markers.size(); p++)
//     {
//       cv::Mat(stag2.markers[p].corners).copyTo(cnrs);
//       corners.push_back(cnrs);
//       ids.push_back(stag2.markers[p].id);
//     }
//   }
//   else if (marker_type == "RUNE-Tag")
//   {
//     unsigned short min_ellipse_contour_points = 5;
//     unsigned short max_ellipse_contour_points = 1000;
//     float min_ellipse_area = 1.0f;
//     float max_ellipse_area = 1000.0f;
//     float min_roundness = 0.1f;
//     float max_mse = 0.3f;
//     float size_compensation = -1.5f;
//     cv::runetag::MarkerDetector *pDetector = 0;

//     pDetector = new cv::runetag::MarkerDetector(cameraMatrix);

//     try
//     {
//       std::cout << " " << pDetector->addModelsFromFile("/home/ayush/Downloads/Tag Comparison/RUNEtag/RUNETagGenerator/build/runetag 8/RUNETag_8.txt") << " loaded" << std::endl;
//     }
//     catch (cv::runetag::DigitalMarkerModel::MarkerModelLoadException &ex)
//     {
//       std::cout << std::endl
//                 << ex.what() << std::endl;
//       goto end;
//     }
//     std::vector<cv::RotatedRect> foundEllipses;
//     cv::runetag::EllipseDetector ellipseDetector(min_ellipse_contour_points,
//                                                  max_ellipse_contour_points,
//                                                  min_ellipse_area,
//                                                  max_ellipse_area,
//                                                  min_roundness,
//                                                  max_mse,
//                                                  size_compensation);
//     std::cout << "> Detecting ellipses" << std::endl;
//     ellipseDetector.detectEllipses(image, foundEllipses);
//     std::cout << "  " << foundEllipses.size() << " found." << std::endl
//               << std::endl;

//     std::cout << "> Detecting RUNE tags" << std::endl;
//     std::vector<cv::runetag::MarkerDetected> tags_found;
//     pDetector->dbgimage = image.clone();
//     pDetector->detectMarkers(foundEllipses, tags_found);
//     std::cout << "  " << tags_found.size() << " found." << std::endl
//               << std::endl;
//     // cv::Mat dbgout = input_image.clone();
//     // cv::Mat dbgout = input_image.clone();
//     std::cout << "> Rendering tags" << std::endl;
//     // std::cout << "   -> " << tagsimg_filename << std::endl << std::endl;

//     // std::cout << "   -> " << tagsimg_filename << std::endl << std::endl;
//     std::cout << "> Estimating tags poses" << std::endl;
//     std::map< int, cv::runetag::Pose > poses;

//     for (size_t i = 0; i < tags_found.size(); ++i)
//     {
//       bool poseok;

//       std::cout << "  Tag IDX:" << tags_found[i].associatedModel()->getIDX();
//       // tags_found[i].associateModel().world_size = (MARKERLENGTH*1000)/2;

//       unsigned int flags = 0;
//       // if(pnpransac)
//       // 	flags |= cv::runetag::FLAG_REPROJ_ERROR;
//       // if(pnprefine)
//       flags |= cv::runetag::FLAG_REFINE;

//       cv::runetag::Pose RT = cv::runetag::findPose(tags_found[i], cameraMatrix, distCoeffs, &poseok, cv::SOLVEPNP_ITERATIVE, flags);

//       if (poseok)
//       {
//         poses[i] =RT;
//         cv::Vec3d tvec = {RT.t.at<double>(0) / 1000, RT.t.at<double>(1) / 1000, RT.t.at<double>(2) / 1000};
//         cv::Mat RotM;
//         cv::Rodrigues(RT.R, RotM);
//         // cv::Vec3d pyr = rotationMatrixToEulerAngles(RotM);
//         cv::Vec3d pyr = cv::RQDecomp3x3(RotM,RR,RQ);
        
//         tvecs.push_back(tvec);
//         pyrs.push_back(pyr);
//         ids.push_back(tags_found[i].associatedModel()->getIDX());
//       }
//       else
//       {
//         std::cout << " Error." << std::endl;
//       }
//     }
//     for( size_t i=0; i<tags_found.size(); ++i )
//     {
//         if( poses.find(i)!=poses.end()) 
//         {
//             cv::runetag::AuxRenderer::drawDetectedMarker3DCylinder2(image,tags_found[i],poses[i],cameraMatrix,distCoeffs);
//         }            
//     }
//     goto end;
//   }
//   if (ids.size() > 0)
//   {
//     vector<cv::Vec3d> rvecs;
//     cv::aruco::drawDetectedMarkers(image, corners, ids);
//     cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
//     std::cout << rvecs.size() << std::endl;
//     for (int i = 0; i < rvecs.size(); i++)
//     {

//       cv::drawFrameAxes(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
//       cv::Mat RotM;
//       cv::Rodrigues(rvecs[i], RotM);
//       // cv::Vec3d pyr = rotationMatrixToEulerAngles(RotM);
//       cv::Vec3d pyr = cv::RQDecomp3x3(RotM,RR,RQ);
//       pyr[0] = -(180-pyr[0]);
//       if (pyr[0] < -180)
//         pyr[0]+=360;
//       pyrs.push_back(pyr);
//     }
//   }
//   end:
//   std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//   return (std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count());
// }



// std::string compressImg2base64(cv::mat image)
// {
//   std::stringstream compressed;
//   std::stringstream original;
//   original << data;
//   boost::iostreams::filtering_streambuf<boost::iostreams::input> out;
//   out.push(boost::iostreams::zlib_compressor());
//   out.push(original);
//   boost::iostreams::copy(out, compressed);

//   /**need to encode here **/
//   std::string compressed_encoded = base64_encode(reinterpret_cast<const unsigned char*>(compressed.c_str()), compressed.length());

//   return compressed_encoded;
// }

void save(std::vector<cv::Mat> missclass_images, cv::Mat example_image, cv::Vec2f rw_rotations, std::vector<std::string> out, std::string occlusion_conf)
{
  int count = 1;
  // out[0] = "[" + out[0];
  // out[out.size()-1][out[out.size()-1].size()-1] = ']';
  out.insert(out.begin(), "frame,index,id,est_loc,real_loc,est_pyr,real_pyr,det_time_ns");
  std::string prefix = "./results/"+occlusion_conf+"/" + MARKER_SYSTEM + "/";
  if (!cv::utils::fs::exists(prefix) || !cv::utils::fs::isDirectory(prefix))
  {
    cv::utils::fs::createDirectories(prefix);

  }
  std::string basefilename = MARKER_SYSTEM + "_r" + std::to_string((int)rw_rotations[0])+ "p" + std::to_string((int)rw_rotations[1]);
  std::string filename = basefilename;
  std::string filepath = prefix + filename;
  while (std::ifstream(filepath + ".csv"))
  {
    filename = basefilename;
    filename += "("+std::to_string(count)+")";
    filepath = prefix + filename;
    count++;
    // return false;
  }
  std::ofstream output_file(filepath + ".csv");
  std::ostream_iterator<std::string> output_iterator(output_file, "\n");
  std::copy(out.begin(), out.end(), output_iterator);
  cv::imwrite(filepath+".jpg",example_image);

  if (missclass_images.size() > 0) {
    if (!cv::utils::fs::exists(prefix+"missclassification images") || !cv::utils::fs::isDirectory(prefix+"missclassification images"))
    {
      cv::utils::fs::createDirectory(prefix+"missclassification images");

    }
    cv::utils::fs::createDirectory(prefix+"missclassification images/" + filename);
    for (int i = 0; i < missclass_images.size();i++) {
      cv::imwrite(prefix+"missclassification images/" + filename + "/"+std::to_string(i+1)+".jpg",missclass_images[i]);
    }
  }
}

// template <typename T>
std::string vec2str(cv::Mat vect, char delimiter=',') {
  std::string asStr;
  asStr << vect;
  asStr.erase(std::remove(asStr.begin(), asStr.end(), '\n'), asStr.end());
  std::replace( asStr.begin(), asStr.end(), ';', delimiter);
  asStr.pop_back();
  asStr.erase(0,1);
  return asStr;
}

void print_stats(std::vector<cv::Vec3d> tvecs, std::vector<cv::Vec3d> pyrs) {
  cv::Vec3f det_loc;
  cv::Vec3f real_loc;
  // float cam_dist = 55.0;

  

  if (tvecs.size() > 0)
  {
    for (int i = 0; i < tvecs.size(); i++)
    {
      det_loc = tvecs[i]*100;
      real_loc = cv::Vec3f(controller.loc[0],0,controller.loc[1]+CAM_DIST);
      float distDiff = pow(pow(det_loc[0] - real_loc[0],2) + pow(det_loc[2] - real_loc[2],2),.5);

      std::cout << "x " << det_loc[0] << "(" << real_loc[0] << ")" << " ,y " << det_loc[1]
                << " ,z " << det_loc[2] << "(" << real_loc[2] << ")" << "\nDistDiff: " << distDiff
                << std::endl;
      std::cout << "pitch " << pyrs[i][0] << "("<< REAL_PYR[0] << ") , yaw " << pyrs[i][1] << "("<< REAL_PYR[1] << ") , roll " << pyrs[i][2] << "("<< REAL_PYR[2] << ")" << std::endl;
      std::cout << std::endl;
    }
  }
}

void store(std::vector<cv::Vec3d> tvecs, std::vector<cv::Vec3d> pyrs,std::vector<int> ids, std::vector<std::string> &data, int det_time,std::vector<cv::Mat> &misclass_images, int &frame) {
  cv::Vec3f det_loc;
  cv::Vec3f real_loc;
  // float cam_dist = 55.0;
  // std::cout << real_loc.rows * real_loc.cols << std::endl;
  // for (int i = 0; i < real_loc.rows * real_loc.cols; i++)
  // exit(0);
  // controller.update();
  std::string stats = "";
  // std::string stats = "[";
  std::vector<int> compression_params;
  // compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
  // compression_params.push_back(30);
  // compression_params.push_back(cv::IMWRITE_JPEG_PROGRESSIVE);
  // compression_params.push_back(1);
  // compression_params.push_back(cv::IMWRITE_JPEG_OPTIMIZE);
  // compression_params.push_back(1);
  // compression_params.push_back(cv::IMWRITE_JPEG_LUMA_QUALITY);
  // compression_params.push_back(30);
  frame++;
  // frame, index, id, est_loc, est_pyr, real_loc, real_pyr, det_time_ns

  real_loc = cv::Vec3f(controller.loc[0],0,controller.loc[1]+CAM_DIST);
  if (ids.size() != 0 &&  std::equal(ids.begin() + 1, ids.end(), ids.begin()) && ids[0] == ID)
    misclass_images.pop_back();
  if (ids.size() == 0) {
    // stats += "{\"real_loc\" : [" + vec2str(cv::Mat(real_loc)) +"], \"real_pyr\" : ["+vec2str(cv::Mat(REAL_PYR))+
    //           // "], \"det_time_ns\" : " + std::to_string(det_time) + 
    //           "]}, ";
    data.push_back(std::to_string(frame) + ",0,,,\"("+vec2str(cv::Mat(real_loc))+")\",,\"("+vec2str(cv::Mat(REAL_PYR))+")\","
          // + std::to_string(det_time)
    );
  }
  else for (int i = 0; i < ids.size(); i++)
  {
    det_loc = tvecs[i]*100;
    float distDiff = pow(pow(det_loc[0] - real_loc[0],2) + pow(det_loc[2] - real_loc[1],2),.5);

    // std::cout << "x " << det_loc[0] << "(" << real_loc[0] << ")" << " ,y " << det_loc[1]
    //           << " ,z " << det_loc[2] << "(" << real_loc[1] << ")" << "\nDistDiff: " << distDiff
    //           << std::endl;
    // std::cout << "yaw " << pyrs[i][0] << " , pitch " << pyrs[i][2] << " , roll " << pyrs[i][1] << std::endl;
    // std::cout << std::endl;
    // stats += "{\"id\" : "+std::to_string(ids[i])+", \"est_loc\" : [" + vec2str(cv::Mat(det_loc)) + 
    //           "], \"est_pyr\" : [" + vec2str(cv::Mat(pyrs[i])) + 
    //           "], \"real_loc\" : [" + vec2str(cv::Mat(real_loc)) +"], \"real_pyr\" : ["+vec2str(cv::Mat(REAL_PYR))+
    //           "], \"det_time_ns\" : " + std::to_string(det_time) + "}, ";
    data.push_back(std::to_string(frame) + ","+std::to_string(i)+","+std::to_string(ids[i])+",\"("+vec2str(cv::Mat(det_loc))+")\",\"("+vec2str(cv::Mat(real_loc))+")\",\"("+vec2str(cv::Mat(pyrs[i]))+")\",\"("+vec2str(cv::Mat(REAL_PYR))+")\"," + std::to_string(det_time));
  }
  // stats.pop_back();
  // stats.pop_back();
  // stats+="],";
  // data.push_back(stats);
}

void move_and_log(cv::VideoCapture inputVideo, std::string type, cv::Vec2f location, float speed)
{
  float frames = 0;
  float detFrames = 0;
  // detectCharucoBoardWithCalibrationPose();
  int fps = 20;
  std::vector<std::string> data;
  cv::Mat image;
  std::vector<cv::Vec3d> tvecs, pyrs;
  // std::vector<std::string> locStats;
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  // std::chrono::steady_clock::time_point beginFPS = std::chrono::steady_clock::now();
  inputVideo.grab();
  frames++;
  inputVideo.retrieve(image);
  // detect(type, image, size, tvecs, pyrs, ids);
  // store(tvecs,pyrs,data);
  // std::chrono::steady_clock::time_point endFPS = std::chrono::steady_clock::now();
  // double fps = 1/((double)std::chrono::duration_cast<std::chrono::milliseconds> (endFPS - beginFPS).count()/1000);
  controller.move(location, speed, false);
  // controller.update();
  while (true)
  {
    // while (frames < time*fps) {

    // double fps = inputVideo.get(cv::CAP_PROP_FPS);
    // std::cout << "fps " << fps << std::endl;
    inputVideo.grab();
    inputVideo.retrieve(image);
    // image.copyTo(imageCopy);

    // detect(type, image, size, tvecs, pyrs);
    // loc = parse_loc();
    // cv::Vec2f newloc = controller.getLoc();
    // newloc[0]+=0.001;
    // newloc[1]+=0.001;
    // endFPS = std::chrono::steady_clock::now();
    // fps = 1/((double)std::chrono::duration_cast<std::chrono::milliseconds> (endFPS - beginFPS).count()/1000);
    // std::cout << "fps " << fps << std::endl;
    // beginFPS = std::chrono::steady_clock::now();

    frames++;
    // store(tvecs,pyrs,data);

    // }
    // std::cout << std::endl;
    cv::imshow("out", image);
    if (controller.state == "<Idle")
    {
      break;
    }

    char key = (char)cv::waitKey(1);
    if (key == 27) {
      controller.close();
      break;
    }
  }
  std::cout << "Size: " << data.size()  << " total: " << frames << std::endl;
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cout << "Time = " << std::chrono::duration_cast<std::chrono::seconds> (end - begin).count() << "[s]" << std::endl;
  // save(type,data);
}

// void unlock_controller() {
//   controller.read();
//   sleep(1);
//   // controller.waitReadable();
//   // controller.readlines();
//   controller.write("$X\n");
//   while(controller.read(2) != "ok") {}
// }

// void center_controller() {
//   controller.write("G21G91G1X177F10000\nG90G21\n");
//   wait_controller();
//   loc[0]+=177;
// }

// void parse_grbl() {
//   controller.write("$$\n");
//   controller.waitReadable();
//   vector<string> data = controller.readlines();
//   for (int i =0; i <  data.size(); i++) {
//     vector<string> splitted = split(data[i],' ');
//     splitted = split(data[i],'=');
//     if (splitted[0] == "$130") {
//       maxLoc[0] = std::stod(splitted[1]);
//     }
//     if (splitted[0] == "$131") {
//       maxLoc[1] = std::stod(splitted[1]);
//     }

//   }
//   // std::cout << maxLoc << std::endl;
// }

void flush(cv::VideoCapture& camera)
{
    int delay = 0;
    // QElapsedTimer timer;

    int framesWithDelayCount = 0;

    while (framesWithDelayCount <= 1)   
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        camera.grab();        
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();              

        delay = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();        

        if(delay > 0)
        {
            framesWithDelayCount++;
        }
    }
}

int main(void)
{
  std::vector<cv::Mat> misclass_images;
  controller.connect("/dev/ttyACM0", 115200);
  // sleep(1);
  // std::cout << std::endl;
  // unlock_controller();
  // std:: cout << controller.readline() << std::endl;
  // sleep(0.5);
  // result = controller.readlines();
  // for (int i =0;i < result.size();i++) {
  //   std::cout << result[i] << std::endl;
  // }
  // std::cout << std::endl;

  controller.home();

  // Center controller
  // controller.move(cv::Vec2f(17.7330, 0), 10000);
  controller.setTimeout(20);
  // controller.center();

  // setup moves
  // controller.move(controller.maxLoc, 10000);


  // parse_grbl();
  // controller.waitReadable();
  // vector<string> result = controller.readlines();
  // for (int i =0;i < result.size();i++) {
  // }
  // std::cout << std::endl;
  // center_controller();
  // std::cout << "Give Distance: ";
  // std::cin >> dist;
  // controller.write("$$\n");
  // // sleep(1);
  // vector<string> test =controller.readlines();
  // for (int i =0; i < test.size();i++) {
  //   std::cout << test[i] << std::endl;
  // }
  // vector<serial::PortInfo> devices_found = serial::list_ports();

  // vector<serial::PortInfo>::iterator iter = devices_found.begin();

  // while( iter != devices_found.end() )
  // {
  // 	serial::PortInfo device = *iter++;

  // 	printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
  //    device.hardware_id.c_str() );
  // }
  // cv::VideoCapture inputVideo(2);
  WebcamStream ws(0,cv::CAP_ANY);
  
  // inputVideo.set(cv::CAP_PROP_BUFFERSIZE, 0);
  cv::Mat image,example_image;
  // int speed = 15000;

  // Timer clock;
  // inputVideo.open(0);
  std::string filename = "calib.txt";
  readCameraParameters(filename, cameraMatrix, distCoeffs); // This function is located in detect_markers.cpp
  // cv::Vec2f moveLoc(controller.maxLoc[0],0);
  cv::Vec2f moveLoc(1,0);
  cv::Vec2f moveLoc2(0, 1);
  std::vector<cv::Vec3d> tvecs, pyrs;
  std::vector<cv::String> data = std::vector<cv::String>();
  std::vector<int> ids;

  cv::Vec3f moveRot(0, 0, 0);
  // double speed = 5000; // mm/min
  ws.start();
  image = *ws.read();
  int frame = 0;
  // cv::Mat orig_img = image;
  // flush(inputVideo);
  // inputVideo >> image;
  cv::Mat image_temp;

  controller.center();
  detector det;
  cv::Mat bg = cv::imread("results/STag2/STag2_r30p60.jpg");
  while (true) {
    // inputVideo >> image;
    // inputVideo.grab();
    // inputVideo.retrieve(image);
    image_temp = image.clone(); 
    // cv::addWeighted(bg,0.4,image.clone(),0.6,0,image_temp);
    det.detect(image_temp,MARKERLENGTH,tvecs,pyrs,ids,cameraMatrix,distCoeffs);
    print_stats(tvecs,pyrs);
    cv::imshow("out", image_temp);
    char key = (char)cv::waitKey(1);
    if (key == 27) {
      // controller.close();
      break;
    }
  }
  example_image = image.clone();
  // controller.move(-moveLoc,speed,true);

  // int holder_pitch;
  std:: cout << "Give id: ";
  std::cin >> ID;
  cv::Vec2f rw_rotations;
  std:: cout << "Give marker orientation (in deg,): ";
  std::cin >> rw_rotations[0];
  cv::Vec3f rot_ax = angleToAx(rw_rotations[0]);
  cv::Mat rot = AxAngToRotMat(cv::Vec3f(0,0,1), rw_rotations[0]);
  std:: cout << "Give real pitch (of holder in deg.): ";
  std::cin >> rw_rotations[1];
  rot *= AxAngToRotMat(rot_ax, rw_rotations[1]);
  cv::Mat RQ(3,3,CV_64F);
  cv::Mat RR(3,3,CV_64F);
  // rot1*=rot2;
  REAL_PYR = cv::RQDecomp3x3(rot,RR,RQ);// - cv::RQDecomp3x3(rot2,RR,RQ);
  std::string occlustion_config;
  // int occlusion_inp;
  std::cin.ignore();
  std::cout << "Give occlusion config(0:none,1:top right,2: bottom left,3: half): ";
  std::getline (std::cin, occlustion_config);
  if (occlustion_config == "0")
    occlustion_config = "No occlusion";
  else if (occlustion_config == "1")
    occlustion_config = "TopRight occlusion";
  else if (occlustion_config == "2")
    occlustion_config = "BottomLeft occlusion";
  else if (occlustion_config == "3")
    occlustion_config = "Half occlusion";
  // while (true) {
  //   // inputVideo >> image;
  //   // inputVideo.grab();
  //   // inputVideo.retrieve(image);
  //   detect(MARKER_SYSTEM,image,0.0589,tvecs,pyrs,ids);
  //   print_stats(tvecs,pyrs);
  //   cv::imshow("out", image);
  //   char key = (char)cv::waitKey(1);
  //   if (key == 27) {
  //     // controller.close();
  //     break;
  //   }
  // }
  controller.home();
  image_temp = image.clone();
  float det_time = det.detect(image_temp,MARKERLENGTH,tvecs,pyrs,ids,cameraMatrix,distCoeffs);

  print_stats(tvecs,pyrs);
  misclass_images.push_back(image_temp.clone());
  store(tvecs,pyrs, ids, data, det_time,misclass_images,frame);
  cv::imshow("out", image_temp);
  // std:: cout << "Give real pitch: ";
  // std::cin >> REAL_PYR[2];

  // move_and_log(inputVideo, "april", moveLoc, speed);
  // move_and_log(inputVideo, "april", moveLoc2, speed);
  // move_and_log(inputVideo, "april", -moveLoc, speed);
  // move_and_log(inputVideo, "aruco", moveLoc, speed);
  // move_and_log(inputVideo, "april", moveLoc, speed);
    while (controller.loc[1] < controller.maxLoc[1]-1) {
  while (((int)round(controller.loc[1]) % 2 == 0 && controller.loc[0] < (int)controller.maxLoc[0]-1) ||
          ((int)round(controller.loc[1]) % 2 == 1 && controller.loc[0] > controller.minLoc[0])) {
      if ((int)round(controller.loc[1]) % 2 == 0)
        controller.move(moveLoc,SPEED);
      else controller.move(-moveLoc,SPEED);
    // while (controller.state != "<Idle") {
      // controller.update();
      // flush(inputVideo);
      // controller.update();
      // inputVideo.grab();
      // inputVideo.retrieve(image);
      // sleep(0.5);image.png
      // inputVideo >> image;
      // image = *ws.read();
      image_temp = image.clone();
      det.detect(image_temp,MARKERLENGTH,tvecs,pyrs,ids,cameraMatrix,distCoeffs);
      print_stats(tvecs,pyrs);
      misclass_images.push_back(image_temp.clone());
      store(tvecs, pyrs, ids, data, det_time,misclass_images,frame);
      cv::imshow("out", image_temp);
      char key = (char)cv::waitKey(1);
      if (key == 27) {
        controller.close();
        goto savestats;
      }
    }
    std::cout << "Size: " << data.size() << std::endl;
    // goto end;
    controller.move(moveLoc2,SPEED);
    // flush(inputVideo);
    // sleep(0.5);
    // controller.update();
    // inputVideo >> image;
    // image = *ws.read();
    image_temp = image;
    det_time = det.detect(image_temp,MARKERLENGTH,tvecs,pyrs,ids,cameraMatrix,distCoeffs);
    print_stats(tvecs,pyrs);
    misclass_images.push_back(image_temp.clone());
    store(tvecs,pyrs, ids, data, det_time,misclass_images,frame);
    cv::imshow("out", image_temp);
    char key = (char)cv::waitKey(1);
    if (key == 27) {
      // controller.close();
      break;
    }
  }
  while (((int)round(controller.loc[1]) % 2 == 0 && controller.loc[0] < controller.maxLoc[0]-1) ||
          ((int)round(controller.loc[1]) % 2 == 1 && controller.loc[0] > controller.minLoc[0])) {
    if ((int)round(controller.loc[1]) % 2 == 0)
      controller.move(moveLoc,SPEED);
    else controller.move(-moveLoc,SPEED);
  // while (controller.state != "<Idle") {
    // controller.update();
    // flush(inputVideo);
    // controller.update();
    // inputVideo.grab();
    // inputVideo.retrieve(image);
    // sleep(0.5);image.png
    // inputVideo >> image;
    // image = *ws.read();
    image_temp = image.clone();
    det_time = det.detect(image_temp,MARKERLENGTH,tvecs,pyrs,ids,cameraMatrix,distCoeffs);
    misclass_images.push_back(image_temp.clone());
    store(tvecs, pyrs, ids, data, det_time,misclass_images,frame);
    cv::imshow("out", image_temp);
    char key = (char)cv::waitKey(1);
    if (key == 27) {
      // controller.close();
      break;
    }
  }
  savestats:
  std::cout << "Size: " << data.size() << std::endl;
  save(misclass_images,example_image, rw_rotations, data, occlustion_config);
  // move_and_log(inputVideo, "april", -moveLoc2, SPEED);

  // move(moveLoc,moveLoc,10000,20);
  // move(moveLoc2,moveLoc,10000,20);
  // move(moveLoc,moveLoc,10000,20);
  // move(moveLoc2,moveLoc,10000,20);
  // move_and_log();

  // while (inputVideo.grab()) {
  //   cv::Mat image, imageCopy;
  //   inputVideo.retrieve(image);
  //   image.copyTo(imageCopy);

  //   std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  //   std::vector<cv::Vec3d> tvecs, pyrs;
  //   detect(type, image, size, tvecs,pyrs);

  //   frames++;
  //   if (pyrs.size() > 0) {
  //     for(int i=0; i<pyrs.size(); i++) {

  //       std::cout << "x " << tvecs[i][0] * 100<< " , y " << tvecs[i][1] * 100 << " , z " << tvecs[i][2] * 100 << std::endl;
  //       std::cout << "yaw " << pyrs[i][0] << " , pitch " << pyrs[i][1] << " , roll " << pyrs[i][2] << std::endl;
  //       std::cout << std::endl;
  //       data.push_back("("+std::to_string( tvecs[i][0] * 100)+","+std::to_string( tvecs[i][1] * 100)+","+std::to_string( tvecs[i][2] * 100)+","+
  //                       std::to_string(pyrs[i][0])+","+std::to_string(pyrs[i][1])+","+std::to_string(pyrs[i][0])+","+")"+" (null,null,null,null,null,null)");
  //     }
  //   }
  //   else {
  //     data.push_back("(null,null,null,null,null,null) (null,null,null,null,null,null)");
  //   }
  //   std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  //   // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count() << "[ms]" << std::endl;
  //   cv::imshow("out", image);
  //   // std::cout << "accuracy " << (detFrames/frames) << std::endl;

  //   char key = (char) cv::waitKey(1);
  //   if (key == 27)
  //       break;
  // }
  controller.close();
  ws.stop();
}