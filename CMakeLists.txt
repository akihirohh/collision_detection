cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 11)

# Find Packages
find_package( PCL 1.8 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )

# octomap
FIND_PACKAGE( octomap REQUIRED )
INCLUDE_DIRECTORIES( ${OCTOMAP_INCLUDE_DIRS} )

# fcl
FIND_PACKAGE( fcl REQUIRED )
INCLUDE_DIRECTORIES( ${FCL_INCLUDE_DIRS} )

# ccd
FIND_PACKAGE( ccd REQUIRED )
INCLUDE_DIRECTORIES( ${CCD_INCLUDE_DIRS} )

add_library(test_fcl_utility test_fcl_utility.cpp)
target_link_libraries(test_fcl_utility fcl)


add_executable (collision_detection collision_detection.cpp)
target_link_libraries (collision_detection
    test_fcl_utility
    ${PCL_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    ${FCL_LIBRARIES}
    ${CCD_LIBRARIES})