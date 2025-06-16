cmake_minimum_required(VERSION 3.5)
project(pcl_sphere)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
    
add_executable(
    pcl_sphere
    pcl_sphere.cpp
)
target_link_libraries(
    pcl_sphere
    PRIVATE ${PCL_LIBRARIES} ${OpenCV_LIBS}
)
target_include_directories(
    pcl_sphere PRIVATE
    ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)
