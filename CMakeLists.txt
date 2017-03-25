cmake_minimum_required(VERSION 3.4)
project(project5561)

set(OpenCV_DIR "/home/mars/soft/opencv/")
find_package( OpenCV 3.2.0 REQUIRED )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_CXX_STANDARD 11)


#nclude_directories("/usr/local/include")
#link_directories("/usr/local/lib")
#set(OpenCV_library 
#"opencv_gpu;opencv_contrib;opencv_legacy;opencv_objdetect;opencv_calib3d;opencv_features2d;opencv_video;opencv_highgui;opencv_ml;opencv_imgproc;opencv_flann;opencv_core")

#set(SOURCE_FILES main.cpp)
#add_executable(cvtest ${SOURCE_FILES})
#target_link_libraries(cvtest ${OpenCV_LIBS})
set(SOURCE_FILES findOrb.cpp)
add_executable(orbTest ${SOURCE_FILES})
target_link_libraries(orbTest ${OpenCV_LIBS})