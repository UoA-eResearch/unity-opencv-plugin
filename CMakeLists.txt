cmake_minimum_required(VERSION 4.2)

set(PROJECT_NAME "OpenCvPlugin")

project(${PROJECT_NAME}
    VERSION 1.0
    DESCRIPTION "A plugin that uses OpenCV for computer vision tasks"
    LANGUAGES CXX
)

# 1. Point CMake to the folder containing 'OpenCVConfig.cmake'
#    (Adjust this path if your extraction location is different)
set(OpenCV_DIR "C:/dev/libs/opencv/opencv-4.12.0/build/x64/vc16/lib")

# 2. actually load the package
find_package(OpenCV REQUIRED)

# 3. Define the Plugin
add_library(${PROJECT_NAME} SHARED 
    source/OpenCvPlugin.cpp
    source/OpenCvPlugin.h
    source/pnp/PnPContext.cpp
    source/pnp/PnPContext.h
)

# 4. Allow #include paths relative to source/
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/source)

# 5. Link OpenCV libraries to your plugin
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})