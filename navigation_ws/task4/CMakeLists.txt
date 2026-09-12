cmake_minimum_required(VERSION 3.16)
project(task4_astar LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED)

add_executable(task4_astar
    src/main.cpp
)

target_include_directories(task4_astar PRIVATE src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(task4_astar PRIVATE ${OpenCV_LIBS})
