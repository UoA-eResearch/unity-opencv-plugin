cmake_minimum_required(VERSION 4.2)

project(DanceVisPlugin)

# 1. Point CMake to the folder containing 'OpenCVConfig.cmake'
#    (Adjust this path if your extraction location is different)
set(OpenCV_DIR "C:/dev/libs/opencv/opencv-4.12.0/build/x64/vc16/lib")

# 2. actually load the package
find_package(OpenCV REQUIRED)

# 3. Define the Plugin
add_library(DanceVisPlugin SHARED source/DanceVisPlugin.cpp)

# 4. Link OpenCV libraries to your plugin
target_link_libraries(DanceVisPlugin ${OpenCV_LIBS})