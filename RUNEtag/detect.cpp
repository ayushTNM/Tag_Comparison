#include "runetag.hpp"
#include "auxrenderer.hpp"
#include "ellipsefitter.hpp"
#include "markerpose.hpp"
#include "coding.h"

#include "../include/detect.h"

int detector::detect(cv::Mat &image, double markerLength, std::vector<cv::Vec3d> &tvecs, std::vector<cv::Vec3d> &pyrs, std::vector<int> &ids,
            cv::Mat cameraMatrix, cv::Mat distCoeffs) {
   tvecs.clear();
   pyrs.clear();
   ids.clear();
   cv::Mat RR(3, 3, CV_64F);
   cv::Mat RQ(3, 3, CV_64F);
   cv::Mat gray;
   cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
   std::vector<std::vector<cv::Point2f>> corners;

   unsigned short min_ellipse_contour_points = 5;
   unsigned short max_ellipse_contour_points = 1000;
   float min_ellipse_area = 1.0f;
   float max_ellipse_area = 1000.0f;
   float min_roundness = 0.1f;
   float max_mse = 0.3f;
   float size_compensation = -1.5f;
   cv::runetag::MarkerDetector *pDetector = 0;
   std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
   pDetector = new cv::runetag::MarkerDetector(cameraMatrix);

   try
   {
   std::cout << " " << pDetector->addModelsFromFile("/home/ayush/Downloads/Tag Comparison/RUNEtag/RUNETagGenerator/build/runetag 8/RUNETag_8.txt") << " loaded" << std::endl;
   }
   catch (cv::runetag::DigitalMarkerModel::MarkerModelLoadException &ex)
   {
   std::cout << std::endl
               << ex.what() << std::endl;
   }
   std::vector<cv::RotatedRect> foundEllipses;
   cv::runetag::EllipseDetector ellipseDetector(min_ellipse_contour_points,
                                                max_ellipse_contour_points,
                                                min_ellipse_area,
                                                max_ellipse_area,
                                                min_roundness,
                                                max_mse,
                                                size_compensation);
   std::cout << "> Detecting ellipses" << std::endl;
   ellipseDetector.detectEllipses(image, foundEllipses);
   std::cout << "  " << foundEllipses.size() << " found." << std::endl
            << std::endl;

   std::cout << "> Detecting RUNE tags" << std::endl;
   std::vector<cv::runetag::MarkerDetected> tags_found;
   pDetector->dbgimage = image.clone();
   pDetector->detectMarkers(foundEllipses, tags_found);
   std::cout << "  " << tags_found.size() << " found." << std::endl
            << std::endl;
   // cv::Mat dbgout = input_image.clone();
   // cv::Mat dbgout = input_image.clone();
   std::cout << "> Rendering tags" << std::endl;
   // std::cout << "   -> " << tagsimg_filename << std::endl << std::endl;

   // std::cout << "   -> " << tagsimg_filename << std::endl << std::endl;
   std::cout << "> Estimating tags poses" << std::endl;
   std::map< int, cv::runetag::Pose > poses;

   for (size_t i = 0; i < tags_found.size(); ++i)
   {
   bool poseok;

   std::cout << "  Tag IDX:" << tags_found[i].associatedModel()->getIDX();
   // tags_found[i].associateModel().world_size = (MARKERLENGTH*1000)/2;

   unsigned int flags = 0;
   // if(pnpransac)
   // 	flags |= cv::runetag::FLAG_REPROJ_ERROR;
   // if(pnprefine)
   flags |= cv::runetag::FLAG_REFINE;

   cv::runetag::Pose RT = cv::runetag::findPose(tags_found[i], cameraMatrix, distCoeffs, &poseok, cv::SOLVEPNP_ITERATIVE, flags);

   if (poseok)
   {
      poses[i] =RT;
      cv::Vec3d tvec = {RT.t.at<double>(0) / 1000, RT.t.at<double>(1) / 1000, RT.t.at<double>(2) / 1000};
      cv::Mat RotM;
      cv::Rodrigues(RT.R, RotM);
      // cv::Vec3d pyr = rotationMatrixToEulerAngles(RotM);
      cv::Vec3d pyr = cv::RQDecomp3x3(RotM,RR,RQ);
      
      tvecs.push_back(tvec);
      pyrs.push_back(pyr);
      ids.push_back(tags_found[i].associatedModel()->getIDX());
   }
   else
   {
      std::cout << " Error." << std::endl;
   }
   }
   for( size_t i=0; i<tags_found.size(); ++i )
   {
      if( poses.find(i)!=poses.end()) 
      {
         cv::runetag::AuxRenderer::drawDetectedMarker3DCylinder2(image,tags_found[i],poses[i],cameraMatrix,distCoeffs);
      }            
   }
   std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
   return (std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count());
}