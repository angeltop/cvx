cmake_minimum_required(VERSION 2.8.0)

project(cvx)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMake/ ${CMAKE_MODULE_PATH} )

include(Setup)
include(CMakeDependentOption)

find_package(OpenMP)

if (OPENMP_FOUND)
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
  if( CMAKE_CXX_COMPILER_VERSION VERSION_LESS "4.9")
    find_package(Boost 1.49 REQUIRED COMPONENTS system regex)
    add_definitions(-DHAS_BOOST_REGEX)
  endif()
endif()


add_definitions(-std=c++11)

IF ( CMAKE_COMPILER_IS_GNUCXX )
	SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wreturn-type" )
ENDIF( CMAKE_COMPILER_IS_GNUCXX )

SET ( CVX_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include/cvx/ )


# build modules
set(MODULES_SRC_DIR "${CMAKE_SOURCE_DIR}/src/modules")
file(GLOB modules RELATIVE ${MODULES_SRC_DIR} "${MODULES_SRC_DIR}/*")

set(BUILD_MODULE_UTIL_CONDITION "TRUE")

foreach(module ${modules})
    string(TOUPPER ${module} module_uc)
    if ( EXISTS ${MODULES_SRC_DIR}/${module}/CMakeLists.txt )
        cmake_dependent_option(BUILD_MODULE_${module_uc} "Build module ${module}" ON BUILD_MODULE_${module_uc}_CONDITION OFF)
        if ( BUILD_MODULE_${module_uc} )
            add_subdirectory(${MODULES_SRC_DIR}/${module})
        else()
            message(WARNING "module ${module} not configured, skipping...")
        endif()
    endif()
endforeach(module)


#ADD_SUBDIRECTORY(src/modules/utils)
ADD_SUBDIRECTORY(test)

include(${CMAKE_SOURCE_DIR}/CMake/Install.cmake)

