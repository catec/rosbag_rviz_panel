cmake_minimum_required(VERSION 3.10)
project(rosbag_rviz_panel DESCRIPTION "Interactive rosbag player - RViz Panel"
                            LANGUAGES   CXX C
)

#####################################
##     General configuration       ##
#####################################
include(cmake_stuff/versions.cmake          REQUIRED)
include(cmake_stuff/options.cmake           REQUIRED)
include(cmake_stuff/compilerOptions.cmake   REQUIRED)
include(cmake_stuff/findDependencies.cmake  REQUIRED)

#####################################
##        Create library           ##
#####################################
file(GLOB_RECURSE SRCS "src/*.cpp")
file(GLOB_RECURSE HDRS "include/*.h")

file(GLOB_RECURSE UIS   "src/*.ui")
QT5_WRAP_UI(UIS_H ${UIS})

add_library(${PROJECT_NAME} SHARED ${SRCS} ${HDRS} ${UIS_H})
target_include_directories(${PROJECT_NAME} PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME} PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES} Qt5::Widgets)
target_compile_options(${PROJECT_NAME} PUBLIC "-Wno-register") # Avoid OGRE deprecaton warnings under C++17

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

#####################################
##   General info & Build files    ##
#####################################
include(cmake_stuff/buildFiles.cmake       REQUIRED)
include(cmake_stuff/printInfo.cmake        REQUIRED)