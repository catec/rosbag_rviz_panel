
# ----------------------------------------------------------------------------
#   Create config.h file
# ----------------------------------------------------------------------------
CONFIGURE_FILE("${PROJECT_SOURCE_DIR}/cmake_stuff/config.h.in"
   "${PROJECT_BINARY_DIR}/config.h" @ONLY)
