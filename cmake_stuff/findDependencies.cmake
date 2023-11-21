## Find includes in corresponding build directories
IF(NOT DEFINED CMAKE_PREFIX_PATH)
   IF(NOT "$ENV{CMAKE_PREFIX_PATH}" STREQUAL "")
      IF(NOT WIN32)
        STRING(REPLACE ":" ";" CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
      ELSE()
        SET(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
      ENDIF()
   ENDIF()
ENDIF()

IF(NOT "/usr/local" IN_LIST CMAKE_INSTALL_PREFIX)
   LIST(APPEND CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF()
   
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_INSTALL_PREFIX}/lib/cmake/)
   
MESSAGE(STATUS "Using CMAKE_PREFIX_PATH - ${CMAKE_PREFIX_PATH}")

#######################
## Configuring ROS   ##
#######################
find_package(catkin REQUIRED 
                    COMPONENTS roscpp pluginlib rviz rosbag)
  
catkin_package(
   INCLUDE_DIRS   include
   LIBRARIES      ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp pluginlib rviz rosbag
   DEPENDS        
)

#######################
## Configuring Qt5   ##
#######################
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)