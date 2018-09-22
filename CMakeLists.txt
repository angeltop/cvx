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


# build modules
set(MODULES_SRC_DIR "${CMAKE_SOURCE_DIR}/modules")
file(GLOB modules RELATIVE ${MODULES_SRC_DIR} "${MODULES_SRC_DIR}/*")

set(BUILD_MODULE_UTIL_CONDITION "TRUE")
set(BUILD_MODULE_UTIL_DEFAULT "ON")

set(BUILD_MODULE_VIZ_CONDITION "BUILD_MODULE_UTIL")
set(BUILD_MODULE_VIZ_DEFAULT "OFF")

set(BUILD_MODULE_CALIB_CONDITION "BUILD_MODULE_UTIL")
set(BUILD_MODULE_CALIB_DEFAULT "OFF")

# collect module information

foreach(module ${modules})
    string(TOUPPER ${module} module_uc)
    if ( EXISTS ${MODULES_SRC_DIR}/${module}/CMakeLists.txt )
        cmake_dependent_option(BUILD_MODULE_${module_uc} "Build module ${module}" ${BUILD_MODULE_${module_uc}_DEFAULT} BUILD_MODULE_${module_uc}_CONDITION OFF)
        if ( BUILD_MODULE_${module_uc} )
            set(MODULE_${module_uc}_LOCATION ${MODULES_SRC_DIR}/${module})
            list(APPEND CVX_INCLUDE_DIRS ${MODULES_SRC_DIR}/${module}/include)
            list(APPEND BUILD_MODULES ${module_uc})
        else()
            message(WARNING "module ${module} not configured, skipping...")
        endif()
    endif()
endforeach(module)

foreach(module ${BUILD_MODULES})
    add_subdirectory(${MODULE_${module}_LOCATION})
endforeach(module)


include(${CMAKE_SOURCE_DIR}/CMake/Install.cmake)

