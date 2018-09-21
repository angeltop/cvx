include_directories(
    ${CVX_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(test_homography test_homography.cpp)
target_link_libraries(test_homography  cvx_calib)

add_executable(test_calib test_calib.cpp)
target_link_libraries(test_calib  cvx_calib)


