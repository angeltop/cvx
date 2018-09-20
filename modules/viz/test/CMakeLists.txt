include_directories(../include ${EIGEN3_INCLUDE_DIR})

add_executable(test_renderer test_renderer.cpp)
target_link_libraries(test_renderer cvx_viz ${OPENGL_LIBRARIES} ${GLFW3_LIBRARY} ${GLEW_LIBRARIES} ${catkin_LIBRARIES})

add_executable(test_robot test_robot.cpp)
target_link_libraries(test_robot cvx_viz ${OPENGL_LIBRARIES} ${GLFW3_LIBRARY} ${GLEW_LIBRARIES} ${catkin_LIBRARIES})

add_executable(test_offscreen test_offscreen.cpp)
target_link_libraries(test_offscreen cvx_viz ${OPENGL_LIBRARIES} ${GLFW3_LIBRARY} ${GLEW_LIBRARIES} ${catkin_LIBRARIES})

add_executable(test_shapes test_shapes.cpp)
target_link_libraries(test_shapes cvx_viz ${OPENGL_LIBRARIES} ${GLFW3_LIBRARY} ${GLEW_LIBRARIES} ${catkin_LIBRARIES})

add_executable(test_point_cloud test_point_cloud.cpp)
target_link_libraries(test_point_cloud cvx_viz ${OPENGL_LIBRARIES} ${GLFW3_LIBRARY} ${GLEW_LIBRARIES} ${catkin_LIBRARIES})
