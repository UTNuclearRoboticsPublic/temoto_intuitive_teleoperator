# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(spacenav_pro)

# Load catkin and all dependencies required for this package
set(CATKIN_DEPS roscpp geometry_msgs sensor_msgs)
find_package(catkin REQUIRED ${CATKIN_DEPS})
catkin_package(DEPENDS ${CATKIN_DEPS})

include_directories(${catkin_INCLUDE_DIRS})
add_executable(spacenav_pro src/spacenav_pro.cpp)
target_link_libraries (spacenav_pro spnav X11 ${catkin_LIBRARIES})

# Install targets
install(TARGETS spacenav_pro
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
