cmake_minimum_required(VERSION 3.4)
project(project5561)

find_package( OpenCV REQUIRED )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_CXX_STANDARD 11)

#set(SOURCE_FILES main.cpp)
#add_executable(cvtest ${SOURCE_FILES})
#target_link_libraries(cvtest ${OpenCV_LIBS})
set(SOURCE_FILES findOrb.cpp)
add_executable(orbTest ${SOURCE_FILES})
target_link_libraries(orbTest ${OpenCV_LIBS})