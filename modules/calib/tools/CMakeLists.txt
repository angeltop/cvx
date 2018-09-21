include_directories(
    ${CVX_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(camera_intrinsics camera_intrinsics.cpp)
target_link_libraries(camera_intrinsics  cvx_calib ${OpenCV_LIBRARIES})

add_executable(camera_ba camera_ba.cpp)
target_link_libraries(camera_ba  cvx_calib ${OpenCV_LIBRARIES} ${CERES_LIBS} ${GLOG_LIBRARIES})

# Install library
install(TARGETS camera_intrinsics camera_ba
  EXPORT ${PROJECT_EXPORT}
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT stlib
  COMPONENT dev)
