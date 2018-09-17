cmake_minimum_required(VERSION 2.8.0)

project(cvx)

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

SET ( CVX_UTIL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include/cvx/util/ )

ADD_SUBDIRECTORY(src/libs/utils/)
ADD_SUBDIRECTORY(test)


