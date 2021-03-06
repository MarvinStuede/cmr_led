#cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.7.2)

project(cmr_led)


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(basepath)

set(ROS_DEPENDENCIES roscpp rospy)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  roslib
  cmr_msgs
  cmr_os
)

catkin_package(
    INCLUDE_DIRS
    include
    LIBRARIES
    CATKIN_DEPENDS ${ROS_DEPENDENCIES} roslib
    )


find_library(LEDMATRIX_LIBRARY librgbmatrix.a matrix/lib)


include_directories(include/rpi-rgb-led-matrix/include)


include_directories(
  include
  matrix/include
  ${catkin_INCLUDE_DIRS}
  #${OpenCV_INCLUDE_DIRS}
)


add_executable(led_panel
    src/led_panel.cpp
    include/cmr_led/led_panel.h
    )

add_library(img_loader
  include/cmr_led/img_loader.h
  include/cmr_led/gifdec.c
  include/cmr_led/gifdec.h
)
target_link_libraries(img_loader
  ${catkin_LIBRARIES}
)


target_link_libraries(led_panel
    ${catkin_LIBRARIES}
	${LEDMATRIX_LIBRARY}
	img_loader
	-lrt -lm -lpthread
	)


set(EXECUTABLE_PATH "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}")

# Run node as root to avoid timing/ghosting issues
add_custom_command(TARGET led_panel POST_BUILD
	 COMMAND sudo setcap 'cap_sys_nice=eip' led_panel
         COMMAND sudo chown root:root led_panel
	 COMMAND sudo chmod a+rx led_panel
         COMMAND sudo chmod u+s led_panel
	 WORKING_DIRECTORY ${EXECUTABLE_PATH}
)

