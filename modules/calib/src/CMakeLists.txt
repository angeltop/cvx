find_package(OpenCV 3 REQUIRED COMPONENTS core highgui video imgproc)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Glog REQUIRED)
add_definitions(${Glog_DEFINITIONS})
include_directories(
        ${CVX_INCLUDE_DIRS}
        3rdparty
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIR}
)

set ( LIBRARY_FOLDER "calib")
get_filename_component(LIBRARY_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../include/cvx/${LIBRARY_FOLDER}/" ABSOLUTE)

set ( LIB_SOURCES

    3rdparty/RPP/RPP.cpp
    3rdparty/RPP/Rpoly.cpp

    pattern.cpp
    calibration.cpp
    pose.cpp
    homography.cpp
    handeye.cpp
)

set ( LIB_HEADERS
    ${LIBRARY_INCLUDE_DIR}/pattern.hpp
    ${LIBRARY_INCLUDE_DIR}/calibration.hpp
    ${LIBRARY_INCLUDE_DIR}/pose.hpp
    ${LIBRARY_INCLUDE_DIR}/homography.hpp
    ${LIBRARY_INCLUDE_DIR}/handeye.hpp
)

add_subdirectory(3rdparty/apriltag)

add_library(cvx_calib ${LIBRARY_TYPE} ${LIB_SOURCES} ${LIB_HEADERS})

TARGET_LINK_LIBRARIES(cvx_calib cvx_util apriltag ${OpenCV_LIBRARIES} ${CERES_LIBRARY} )

add_executable(make_calib_pattern "make_pattern.cpp")
target_link_libraries(make_calib_pattern cvx_calib cvx_util ${GLOG_LIBRARIES})

# Install library
install(TARGETS cvx_calib make_calib_pattern
  EXPORT ${PROJECT_EXPORT}
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT stlib
  COMPONENT dev)

# Install headers
install(DIRECTORY "${LIBRARY_INCLUDE_DIR}" DESTINATION "${INSTALL_INCLUDE_DIR}/cvx" )
