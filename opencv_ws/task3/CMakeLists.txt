cmake_minimum_required(VERSION 3.16)
project(task3_contour LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include: ${OpenCV_INCLUDE_DIRS}")

add_executable(contour_detect src/contour_detect.cpp)
target_include_directories(contour_detect PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(contour_detect PRIVATE ${OpenCV_LIBS})
