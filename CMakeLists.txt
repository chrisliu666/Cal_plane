
#SET(CMAKE_CXX_COMPILER "/usr/bin/g++")


#SET(CMAKE_BUILD_TYPE "Debug")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
#SET(CUDA_USE_STATIC_CUDA_RUNTIME OFF)




cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(CalculateRotAngle)
SET(CMAKE_BUILD_TYPE Debug)

set(OpenCV_DIR "/home/Downloads/opencv-2.4.13.6/build")
set(OpenCV_INCLUDE_DIRS "/home/Downloads/opencv-2.4.13.6/include")
set(OpenCV_LIBRARIES "/home/Downloads/opencv-2.4.13.6/build/lib")

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

find_package(OpenCV 2.4 REQUIRED PATHS "/home/Downloads/opencv-2.4.13.6/build")
find_package(PCL 1.2 REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
include_directories("/usr/include/eigen3" ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${OpenCV_INCLUDE_DIRS} "/usr/include/eigen3" ${PCL_INCLUDE_DIRS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
add_executable (cal cal.cpp)
target_link_libraries(cal ${OpenCV_LIBS})
target_link_libraries (cal ${PCL_LIBRARIES})



