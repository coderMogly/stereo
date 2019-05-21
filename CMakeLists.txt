cmake_minimum_required(VERSION 2.8.3)
project(stereo_loc)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
    COMPONENTS
        message_filters
        cmake_modules
        roscpp
        rospy
        tf2_ros
	mathlib
)
find_package(PCL 1.2 REQUIRED)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})



catkin_package(
    CATKIN_DEPENDS
        roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# stereo_loc
add_library(stereo_loc
  src/stereo_loc.cpp
)
target_link_libraries(stereo_loc
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
	${PCL_LIBRARIES}
    )
add_dependencies(stereo_loc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(stereo_loc_node
  src/stereo_loc_node.cpp
)
target_link_libraries(stereo_loc_node ${mathlib_LIBRARIES})
add_dependencies(stereo_loc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stereo_loc_node stereo_loc)
set_target_properties(stereo_loc_node PROPERTIES OUTPUT_NAME stereo_loc)

install(
    TARGETS
        stereo_loc
        stereo_loc_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
