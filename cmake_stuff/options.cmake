INCLUDE(GenerateExportHeader)

#------------------------------------------------------
#   Build type
#------------------------------------------------------
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs" FORCE)
IF(DEFINED CMAKE_BUILD_TYPE)
   SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES})
ENDIF()

IF(NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE "Debug")
ENDIF()

IF(NOT ${CMAKE_BUILD_TYPE} STREQUAL "Debug" AND
   NOT ${CMAKE_BUILD_TYPE} STREQUAL "Release")
   MESSAGE(FATAL_ERROR "Only Release and Debug build types are allowed.")
ENDIF()

# ----------------------------------------------------------------------------
#   PROJECT CONFIGURATION
#   force some variables that could be defined in the command line to be written to cache
# ----------------------------------------------------------------------------
OPTION(WARNINGS_ARE_ERRORS "Treat warnings as errors"                                 OFF)
OPTION(WARNINGS_ANSI_ISO   "Issue all the mandatory diagnostics Listed in C standard" ON)
OPTION(WARNINGS_EFFCPP     "Issue all the warnings listed in the book of Scot Meyers" OFF)

OPTION(BUILD_SHARED_LIBS   "Build shared libraries"                                   ON)
OPTION(BUILD_UTILS         "Build applications using the different modules"           ON)