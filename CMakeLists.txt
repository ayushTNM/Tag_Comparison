cmake_minimum_required(VERSION 2.8)
project( Comparison )
find_package( OpenCV REQUIRED )
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/stag/lib)

IF( NOT CMAKE_BUILD_TYPE )
	SET( CMAKE_BUILD_TYPE "Release" )
ENDIF()

file(GLOB SRC_FILE1 "stag/src/*.c*")
file(GLOB SRC_FILE2 "stag/src/ED/*.c*")
add_library(
    LibStag SHARED
    ${SRC_FILE1}
    ${SRC_FILE2}
)

ADD_SUBDIRECTORY( RUNEtag/RUNETag/WinNTL )

SET( RUNETAG_SRCS 
RUNEtag/RUNETag/auxmath.cpp
		RUNEtag/RUNETag/auxrenderer.cpp
		RUNEtag/RUNETag/ellipsefitter.cpp
		RUNEtag/RUNETag/ellipsepoint.cpp
		RUNEtag/RUNETag/fpscounter.cpp
		RUNEtag/RUNETag/hirestimer.cpp
		RUNEtag/RUNETag/markerdetector.cpp
		RUNEtag/RUNETag/markerpose.cpp
		RUNEtag/RUNETag/slotfitter.cpp
		RUNEtag/RUNETag/ellipserefine.cpp
		RUNEtag/RUNETag/ellipsedetector.cpp
		RUNEtag/RUNETag/digitalmarkermodel.cpp
		RUNEtag/RUNETag/slot.cpp
		RUNEtag/RUNETag/coding.cpp
		)

SET( RUNETAG_INCL 
	  "RUNEtag/RUNETag/include/auxmath.hpp"
	  "RUNEtag/RUNETag/include/auxrenderer.hpp"
	  "RUNEtag/RUNETag/include/digitalmarkermodel.hpp"
	  "RUNEtag/RUNETag/include/ellipsefitter.hpp"
	  "RUNEtag/RUNETag/include/ellipsepoint.hpp"
	  "RUNEtag/RUNETag/include/fpscounter.hpp"
	  "RUNEtag/RUNETag/include/hirestimer.hpp"
	  "RUNEtag/RUNETag/include/imgtextstream.hpp"
	  "RUNEtag/RUNETag/include/markerdetected.hpp"
	  "RUNEtag/RUNETag/include/markerdetector.hpp"
	  "RUNEtag/RUNETag/include/markerpose.hpp"
	  "RUNEtag/RUNETag/include/slot.hpp"
	  "RUNEtag/RUNETag/include/slotfitter.hpp"
	  "RUNEtag/RUNETag/include/ellipserefine.hpp"
	  "RUNEtag/RUNETag/include/ellipsedetector.hpp"
	  "RUNEtag/RUNETag/include/runetag.hpp"
	  "RUNEtag/RUNETag/include/coding.h")

ADD_LIBRARY( libRUNETag STATIC ${RUNETAG_SRCS} ${RUNETAG_INCL} )
include_directories( "${PROJECT_SOURCE_DIR}/RUNEtag/RUNETag/include" ${OpenCV_INCLUDE_DIRS} "RUNEtag/RUNETag/WinNTL/include" stag/src/ stag/src/ED/)
add_executable( Comparison Comparison.cpp Comparison.h)
target_link_libraries( Comparison apriltag LibStag libRUNETag WinNTL ${OpenCV_LIBS})

add_executable( Calib calibrate_camera_charuco.cpp )
target_link_libraries( Calib ${OpenCV_LIBS})