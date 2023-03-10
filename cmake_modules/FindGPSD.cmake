# - Find GPSD
# Find the native GPSD includes and library

FIND_PATH(GPSD_INCLUDE_DIR libgpsmm.h gps.h)

SET(GPSD_NAMES ${GPSD_NAMES} gps)
FIND_LIBRARY(GPSD_LIBRARY NAMES ${GPSD_NAMES} )

# handle the QUIETLY and REQUIRED arguments and set JPEG_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GPSD DEFAULT_MSG GPSD_LIBRARY GPSD_INCLUDE_DIR)

IF(GPSD_FOUND)
  SET(GPSD_LIBRARIES ${GPSD_LIBRARY})
  message(STATUS "Found libgps: ${GPSD_LIBRARIES}")
ENDIF(GPSD_FOUND)


