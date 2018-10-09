#add_executable(test_linefit test_linefit.cpp)
#target_link_libraries(test_linefit cvx_util ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(test_triangulate test_triangulate.cpp)
#target_link_libraries(test_triangulate cvx_util ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(test_polyscanner test_polyscanner.cpp)
#target_link_libraries(test_polyscanner cvx_util ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(test_solvers test_solvers.cpp)
#target_link_libraries(test_solvers cvx_util ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
find_package(Eigen3 REQUIRED)
include_directories(
    ../include
        ${EIGEN3_INCLUDE_DIR}
 )
add_executable(test_strings test_strings.cpp)
target_link_libraries(test_strings cvx_util)

add_executable(test_path test_path.cpp)
target_link_libraries(test_path cvx_util)

add_executable(test_argparse test_argparse.cpp)
target_link_libraries(test_argparse cvx_util)

add_executable(test_appsettings test_appsettings.cpp)
target_link_libraries(test_appsettings cvx_util)

#add_executable(test_xml test_xml.cpp)
#target_link_libraries(test_xml cvx_util)

add_executable(test_lm test_lm.cpp)
target_link_libraries(test_lm  cvx_util )

add_executable(test_logger test_logger.cpp)
target_link_libraries(test_logger  cvx_util )

add_executable(test_geometry test_geometry.cpp)
target_link_libraries(test_geometry cvx_util )


