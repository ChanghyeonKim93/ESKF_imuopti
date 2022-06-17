cmake_minimum_required(VERSION 3.5.1)
project(eskf)
set(CMAKE_BUILD_TYPE Debug) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -pg -O2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 
find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  
  std_msgs
  sensor_msgs
  geometry_msgs
)
catkin_package(
  DEPENDS Eigen
  CATKIN_DEPENDS roscpp
  INCLUDE_DIRS include
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

# For 'cvplot'
set(CVPLOT_INCLUDE_DIR /home/larrkchlaptop/Downloads/cv-plot-1.2.2/CvPlot/inc)
set(CVPLOT_SRC /home/larrkchlaptop/Downloads/cv-plot-1.2.2/CvPlot/src/cvplot.cpp)
include_directories(${CVPLOT_INCLUDE_DIR}) 


add_executable(eskf_node src/main.cpp)
target_link_libraries(eskf_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(txtdraw_node src/main_txtdraw.cpp ${CVPLOT_SRC})
target_link_libraries(txtdraw_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})