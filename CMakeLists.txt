cmake_minimum_required(VERSION 2.8.0)

project(cvx)

###########################################33333

# Set PROJECT_NAME_UPPERCASE and PROJECT_NAME_LOWERCASE variables
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPERCASE)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWERCASE)

# Version variables
set(MAJOR_VERSION 0)
set(MINOR_VERSION 1)
set(PATCH_VERSION 0)
set(PROJECT_VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION})

# INSTALL_LIB_DIR
set(INSTALL_LIB_DIR lib
    CACHE PATH "Relative instalation path for libraries")

# INSTALL_BIN_DIR
set(INSTALL_BIN_DIR bin
    CACHE PATH "Relative instalation path for executables")

# INSTALL_INCLUDE_DIR
set(INSTALL_INCLUDE_DIR include/cvx
    CACHE PATH "Relative instalation path for header files")

# INSTALL_CMAKE_DIR
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/cmake/${PROJECT_NAME})
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR}
    CACHE PATH "Relative instalation path for CMake files")

# Convert relative path to absolute path (needed later on)
foreach(substring LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${substring}_DIR)
  if(NOT IS_ABSOLUTE ${${var}})
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
  message(STATUS "${var}: "  "${${var}}")
endforeach()


# The export set for all the targets
set(PROJECT_EXPORT ${PROJECT_NAME}EXPORT)

# Path of the CNake files generated
set(PROJECT_CMAKE_FILES ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY})

# The RPATH to be used when installing
set(CMAKE_INSTALL_RPATH ${INSTALL_LIB_DIR})

#######################################################################33

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMake/ ${CMAKE_MODULE_PATH} )

find_package(OpenMP)

if (OPENMP_FOUND)
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
  if( CMAKE_CXX_COMPILER_VERSION VERSION_LESS "4.9")
    find_package(Boost 1.49 REQUIRED COMPONENTS system regex)
    add_definitions(-DUSE_BOOST_REGEX)
  endif()
endif()

FIND_PACKAGE(OpenCV 3 REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(ZLIB REQUIRED)

find_package(F2C REQUIRED)
find_package(LAPACK)

add_definitions(-std=c++11)

include_directories(
	${CMAKE_SOURCE_DIR}/include/
	${EIGEN3_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
)

IF ( CMAKE_COMPILER_IS_GNUCXX )
	SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wreturn-type" )
ENDIF( CMAKE_COMPILER_IS_GNUCXX )

option(BUILD_SHARED_LIBS "Build ${_PN} as a shared library." ON)

if(BUILD_SHARED_LIBS)
  set(LIBRARY_TYPE SHARED)
else()
  set(LIBRARY_TYPE STATIC)
endif()

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release'.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)

  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Create 'version.h'
configure_file(${CMAKE_SOURCE_DIR}/CMake/version.h.in
  "${CMAKE_CURRENT_BINARY_DIR}/version.h" @ONLY)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/version.h DESTINATION "${INSTALL_INCLUDE_DIR}" )

SET ( CVX_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include/cvx/ )

ADD_SUBDIRECTORY(src/libs/utils/)
ADD_SUBDIRECTORY(test)

include(${CMAKE_SOURCE_DIR}/CMake/Install.cmake)

