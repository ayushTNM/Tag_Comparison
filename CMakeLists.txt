cmake_minimum_required(VERSION 3.1)
project( Comparison )
find_package( OpenCV REQUIRED )

IF( NOT CMAKE_BUILD_TYPE )
SET( CMAKE_BUILD_TYPE "Release" )
ENDIF()


set(MARKER_TYPE
	"STag2"
)


file(GLOB STAG_SRC_FILE1 "stag/src/*.c*")
file(GLOB STAG_SRC_FILE2 "stag/src/ED/*.c*")

file(GLOB STAG2_SRC_FILE1 "stag2/src/*.c*")
# file(GLOB STAG2_INCL "stag2/src/*.h")
# file(GLOB STAG2_SRC_FILE1 "stag2/src/ED/*.c*")

# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/stag2/lib)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})


set(serial_SRCS
    serial/src/serial.cc
	serial/src/impl/unix.cc
	serial/src/impl/list_ports/list_ports_linux.cc
	gcode.cpp
)

set(serial_INCL
    serial/include/serial/serial.h
    serial/include/serial/v8stdint.h
	"include/gcode.h"
)

ADD_LIBRARY( serial ${serial_SRCS} ${serial_INCL})

set(util_SRCS
	WebcamStream.cpp
	base64.cpp
)

set(util_INCL
	"include/WebcamStream.h"
	"include/base64.h"
)

ADD_LIBRARY( utils ${util_SRCS} ${util_INCL})
# ADD_SUBDIRECTORY(serial/src/impl)
# ADD_SUBDIRECTORY(serial/include/impl)
if (${MARKER_TYPE} STREQUAL "STag")
	add_library(
		Stag SHARED
		${STAG_SRC_FILE1}
		${STAG_SRC_FILE2}
		"stag/detect.cpp"
	)

	set(INCLUDE_DIRS 
		stag/src/ 
		stag/src/ED/)

	set(LINK_LIBS
		Stag)

elseif(${MARKER_TYPE} STREQUAL "STag2")
	add_library(
		Stag2 SHARED
		${STAG2_SRC_FILE1}
		${STAG_SRC_FILE2}
		"stag2/detect.cpp"
		# "include/detect.h"
		# ${STAG2_INCL}
	)
	set(INCLUDE_DIRS 
		stag2/src/ 
		stag/src/ED/
		)
	set(LINK_LIBS
		Stag2)
elseif(${MARKER_TYPE} STREQUAL "AprilTag")
	aux_source_directory(apriltag/common COMMON_SRC)
	file(GLOB TAG_FILES apriltag/tag*.c)
	ADD_LIBRARY( April STATIC 
	"apriltag/detect.cpp"
	"apriltag/apriltag.c"
	"apriltag/apriltag_pose.c" 
	apriltag/apriltag_quad_thresh.c
	${COMMON_SRC}
	${TAG_FILES})
	set(INCLUDE_DIRS  
		apriltag
		aprilta/common)
	set(LINK_LIBS
		April)
elseif(${MARKER_TYPE} STREQUAL "ArUco")
	ADD_LIBRARY( Aruco STATIC 
		"Aruco/detect.cpp" )
	set(INCLUDE_DIRS  )
	set(LINK_LIBS
		Aruco)
else(${MARKER_TYPE} STREQUAL "RUNETag")
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
	ADD_LIBRARY( RUNETag STATIC 
				${RUNETAG_SRCS} 
				${RUNETAG_INCL}
				"RUNEtag/detect.cpp" )
	set(INCLUDE_DIRS 
		"RUNEtag/RUNETag/WinNTL/include"
		"RUNEtag/RUNETag/include"
		)
	set(LINK_LIBS
		RUNETag
		WinNTL)

endif()
include_directories(include ${OpenCV_INCLUDE_DIRS} serial/include ${INCLUDE_DIRS})
# include_directories( "${PROJECT_SOURCE_DIR}/RUNEtag/RUNETag/include" ${OpenCV_INCLUDE_DIRS} "RUNEtag/RUNETag/WinNTL/include" stag/src/ stag/src/ED/ stag2/src/ED/ stag2/src/ serial/include)

add_executable( Comparison Comparison.cpp)
# if (MY_VAR)
# endif()
target_link_libraries( Comparison serial utils ${LINK_LIBS} ${OpenCV_LIBS})
# target_compile_definitions(Comparison PRIVATE MARKER_TYPE=${MARKER_TYPE})
ADD_DEFINITIONS( -DMARKER_TYPE=\"${MARKER_TYPE}\")

add_executable( Calib calibrate_camera_charuco.cpp )
target_link_libraries( Calib ${OpenCV_LIBS})