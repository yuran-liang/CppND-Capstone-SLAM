cmake_minimum_required(VERSION 3.10)

# Project name
project(LidarSLAM)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)

# Find PCL
find_package(PCL 1.10 REQUIRED)

# Recent macOS no longer ships the AGL framework, but Qt6 (pulled in via VTK) still adds it
# to WrapOpenGL::WrapOpenGL, so drop it to avoid a linker error
if(APPLE AND TARGET WrapOpenGL::WrapOpenGL)
    get_target_property(_wrap_opengl_libs WrapOpenGL::WrapOpenGL INTERFACE_LINK_LIBRARIES)
    list(FILTER _wrap_opengl_libs EXCLUDE REGEX "AGL")
    set_target_properties(WrapOpenGL::WrapOpenGL PROPERTIES INTERFACE_LINK_LIBRARIES "${_wrap_opengl_libs}")
endif()

# Include directories
include_directories(
    ${PCL_INCLUDE_DIRS}
    include  # This assumes you have a folder named 'include' with your header files
)

# Link directories
link_directories(${PCL_LIBRARY_DIRS})

# Add definitions
add_definitions(${PCL_DEFINITIONS})

# Find Ceres library
find_package(Ceres 2.1 REQUIRED)

# Add Ceres include directories 
include_directories(${CERES_INCLUDE_DIRS})

# Add executable
add_executable(LidarSLAM 
src/main.cpp 
src/PointCloudReader.cpp 
src/PointCloudVisualizer.cpp 
src/FeatureExtraction.cpp 
src/LaserOdom.cpp 
src/OdoRefinement.cpp)

# Link with PCL libraries
target_link_libraries(LidarSLAM 
${PCL_LIBRARIES} 
${CERES_LIBRARIES})

# Viewer only, without pointcloud processing
add_executable(pcViewer 
src/pc_viewer_main.cpp 
src/PointCloudReader.cpp 
src/PointCloudVisualizer.cpp)
target_link_libraries(pcViewer ${PCL_LIBRARIES})
