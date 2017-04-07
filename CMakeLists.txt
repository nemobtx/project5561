cmake_minimum_required(VERSION 3.4)
project(project5561)

set(OpenCV_DIR "/home/mars/soft/opencv/")
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_CXX_STANDARD 11)


include_directories("include")
link_directories("lib")

#set(SOURCE_FILES main.cpp)
#add_executable(cvtest ${SOURCE_FILES})
#target_link_libraries(cvtest ${OpenCV_LIBS})

set(SOURCE_FILES findOrb.cpp)
add_executable(orbTest ${SOURCE_FILES})
target_link_libraries(orbTest ${OpenCV_LIBS})

set(SOURCE_FILES_EYEMARS EyeMARSTest.cpp)
add_executable(eyeMARSTest ${SOURCE_FILES_EYEMARS})
target_link_libraries(eyeMARSTest -lEyeMARS ${OpenCV_LIBS})

