cmake_minimum_required(VERSION 3.4)
project(project5561)

set(OpenCV_LIBS -L${CMAKE_CURRENT_SOURCE_DIR}/../3rd-party/opencv-cv/linux/lib/
        -lopencv_contrib
        -lopencv_nonfree
        -lopencv_videostab
        -lopencv_photo
        -lopencv_objdetect
        -lopencv_video
        -lopencv_ml
        -lopencv_calib3d
        -lopencv_features2d
        -lopencv_highgui
        -lopencv_imgproc
        -lopencv_core
        )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_CXX_STANDARD 11)


include_directories("include")
link_directories("lib")

include_directories("opencv_mars/include")
link_directories("opencv_mars/lib")

#set(SOURCE_FILES main.cpp)
#add_executable(cvtest ${SOURCE_FILES})
#target_link_libraries(cvtest ${OpenCV_LIBS})

set(SOURCE_FILES findOrb.cpp)
add_executable(orbTest ${SOURCE_FILES})
target_link_libraries(orbTest ${OpenCV_LIBS})

set(SOURCE_FILES_EYEMARS EyeMARSTest.cpp)
add_executable(eyeMARSTest ${SOURCE_FILES_EYEMARS})
target_link_libraries(eyeMARSTest -lEyeMARS ${OpenCV_LIBS})

