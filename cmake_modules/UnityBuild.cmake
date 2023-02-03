#
# Copyright (c) 2009-2012 Christoph Heindl
# Copyright (c) 2015 Csaba Kertész (csaba.kertesz@gmail.com)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    * Neither the name of the <organization> nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
#

MACRO (COMMIT_UNITY_FILE UNITY_FILE FILE_CONTENT)
  SET(DIRTY FALSE)
  # Check if the build file exists
  SET(OLD_FILE_CONTENT "")
  IF (NOT EXISTS ${${UNITY_FILE}} AND NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${${UNITY_FILE}})
    SET(DIRTY TRUE)
  ELSE ()
    # Check the file content
    FILE(STRINGS ${${UNITY_FILE}} OLD_FILE_CONTENT)
    STRING(REPLACE ";" "" OLD_FILE_CONTENT "${OLD_FILE_CONTENT}")
    STRING(REPLACE "\n" "" NEW_CONTENT "${${FILE_CONTENT}}")
    STRING(COMPARE EQUAL "${OLD_FILE_CONTENT}" "${NEW_CONTENT}" EQUAL_CHECK)
    IF (NOT EQUAL_CHECK EQUAL 1)
      SET(DIRTY TRUE)
    ENDIF ()
  ENDIF ()
  IF (DIRTY MATCHES TRUE)
    MESSAGE(STATUS "Write Unity Build file: " ${${UNITY_FILE}})
    FILE(WRITE ${${UNITY_FILE}} "${${FILE_CONTENT}}")
  ENDIF ()
  # Create a dummy copy of the unity file to trigger CMake reconfigure if it is deleted.
  SET(UNITY_FILE_PATH "")
  SET(UNITY_FILE_NAME "")
  GET_FILENAME_COMPONENT(UNITY_FILE_PATH ${${UNITY_FILE}} PATH)
  GET_FILENAME_COMPONENT(UNITY_FILE_NAME ${${UNITY_FILE}} NAME)
  CONFIGURE_FILE(${${UNITY_FILE}} ${UNITY_FILE_PATH}/CMakeFiles/${UNITY_FILE_NAME}.dummy)
ENDMACRO ()

MACRO (ENABLE_UNITY_BUILD TARGET_NAME SOURCE_VARIABLE_NAME UNIT_SIZE EXTENSION)
  # Limit is zero based conversion of unit_size
  MATH(EXPR LIMIT ${UNIT_SIZE}-1)
  SET(FILES ${SOURCE_VARIABLE_NAME})
  # Effectivly ignore the source files from the build, but keep track them for changes.
  SET_SOURCE_FILES_PROPERTIES(${${FILES}} PROPERTIES HEADER_FILE_ONLY true)
  # Counts the number of source files up to the threshold
  SET(COUNTER ${LIMIT})
  # Have one or more unity build files
  SET(FILE_NUMBER 0)
  SET(BUILD_FILE "")
  SET(BUILD_FILE_CONTENT "")
  SET(UNITY_BUILD_FILES "")
  SET(_DEPS "")

  FOREACH (SOURCE_FILE ${${FILES}})
    IF (COUNTER EQUAL LIMIT)
      SET(_DEPS "")
      # Write the actual Unity Build file
      IF (NOT ${BUILD_FILE} STREQUAL "" AND NOT ${BUILD_FILE_CONTENT} STREQUAL "")
        COMMIT_UNITY_FILE(BUILD_FILE BUILD_FILE_CONTENT)
      ENDIF ()
      SET(UNITY_BUILD_FILES ${UNITY_BUILD_FILES} ${BUILD_FILE})
      # Set the variables for the current Unity Build file
      SET(BUILD_FILE ${CMAKE_CURRENT_BINARY_DIR}/unitybuild_${FILE_NUMBER}_${TARGET_NAME}.${EXTENSION})
      SET(BUILD_FILE_CONTENT "// Unity Build file generated by CMake\n")
      MATH(EXPR FILE_NUMBER ${FILE_NUMBER}+1)
      SET(COUNTER 0)
    ENDIF ()
    # Add source path to the file name if it is not there yet.
    SET(FINAL_SOURCE_FILE "")
    SET(SOURCE_PATH "")
    GET_FILENAME_COMPONENT(SOURCE_PATH ${SOURCE_FILE} PATH)
    IF (SOURCE_PATH STREQUAL "" OR NOT EXISTS ${SOURCE_FILE})
      SET(FINAL_SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_FILE})
    ELSE ()
      SET(FINAL_SOURCE_FILE ${SOURCE_FILE})
    ENDIF ()
    # Treat only the existing files or moc_*.cpp files
    STRING(FIND ${SOURCE_FILE} "moc_" MOC_POS)
    IF (EXISTS ${FINAL_SOURCE_FILE} OR MOC_POS GREATER -1)
      # Add md5 hash of the source file (except moc files) to the build file content
      IF (MOC_POS LESS 0)
        SET(MD5_HASH "")
        FILE(MD5 ${FINAL_SOURCE_FILE} MD5_HASH)
        SET(BUILD_FILE_CONTENT "${BUILD_FILE_CONTENT}// md5: ${MD5_HASH}\n")
      ENDIF ()
      # Add the source file to the build file content
      IF (MOC_POS GREATER -1)
        SET(BUILD_FILE_CONTENT "${BUILD_FILE_CONTENT}#include <${SOURCE_FILE}>\n")
      ELSE ()
        SET(BUILD_FILE_CONTENT "${BUILD_FILE_CONTENT}#include <${FINAL_SOURCE_FILE}>\n")
      ENDIF ()
      # Add the source dependencies to the Unity Build file
      GET_SOURCE_FILE_PROPERTY(_FILE_DEPS ${SOURCE_FILE} OBJECT_DEPENDS)

      IF (_FILE_DEPS)
        SET(_DEPS ${_DEPS} ${_FILE_DEPS})
        SET_SOURCE_FILES_PROPERTIES(${BUILD_FILE} PROPERTIES OBJECT_DEPENDS "${_DEPS}")
      ENDIF()
      # Keep counting up to the threshold. Increment counter.
      MATH(EXPR COUNTER ${COUNTER}+1)
    ENDIF ()
  ENDFOREACH ()
  # Write out the last Unity Build file
  IF (NOT ${BUILD_FILE} STREQUAL "" AND NOT ${BUILD_FILE_CONTENT} STREQUAL "")
    COMMIT_UNITY_FILE(BUILD_FILE BUILD_FILE_CONTENT)
  ENDIF ()
  SET(UNITY_BUILD_FILES ${UNITY_BUILD_FILES} ${BUILD_FILE})
  SET(${SOURCE_VARIABLE_NAME} ${${SOURCE_VARIABLE_NAME}} ${UNITY_BUILD_FILES})
ENDMACRO ()

MACRO (UNITY_GENERATE_MOC TARGET_NAME SOURCES HEADERS)
  SET(NEW_SOURCES "")
  FOREACH (HEADER_FILE ${${HEADERS}})
    IF (NOT EXISTS ${HEADER_FILE})
      MESSAGE(FATAL_ERROR "Header file does not exist (mocing): ${HEADER_FILE}")
    ENDIF ()
    FILE(READ ${HEADER_FILE} FILE_CONTENT)
    STRING(FIND "${FILE_CONTENT}" "Q_OBJECT" QOBJECT_POS)
    STRING(FIND "${FILE_CONTENT}" "Q_SLOTS" QSLOTS_POS)
    STRING(FIND "${FILE_CONTENT}" "Q_SIGNALS" QSIGNALS_POS)
    STRING(FIND "${FILE_CONTENT}" "QObject" OBJECT_POS)
    STRING(FIND "${FILE_CONTENT}" "slots" SLOTS_POS)
    STRING(FIND "${FILE_CONTENT}" "signals" SIGNALS_POS)
    IF (QOBJECT_POS GREATER 0 OR OBJECT_POS GREATER 0 OR QSLOTS_POS GREATER 0 OR Q_SIGNALS GREATER 0 OR
        SLOTS_POS GREATER 0 OR SIGNALS GREATER 0)
      # Generate the moc filename
      GET_FILENAME_COMPONENT(HEADER_BASENAME ${HEADER_FILE} NAME_WE)
      SET(MOC_FILENAME "moc_${HEADER_BASENAME}.cpp")
      SET(NEW_SOURCES ${NEW_SOURCES} ; "${CMAKE_CURRENT_BINARY_DIR}/${MOC_FILENAME}")
      ADD_CUSTOM_COMMAND(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${MOC_FILENAME}"
                         DEPENDS ${HEADER_FILE}
                         COMMAND ${QT_MOC_EXECUTABLE} ${HEADER_FILE} -o "${CMAKE_CURRENT_BINARY_DIR}/${MOC_FILENAME}")
    ENDIF ()
  ENDFOREACH ()
  IF (NEW_SOURCES)
    SET_SOURCE_FILES_PROPERTIES(${NEW_SOURCES} PROPERTIES GENERATED TRUE)
    SET(${SOURCES} ${${SOURCES}} ; ${NEW_SOURCES})
  ENDIF ()
ENDMACRO ()
