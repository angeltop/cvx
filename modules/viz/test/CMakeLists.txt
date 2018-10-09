find_package(OpenCV 3 REQUIRED)

include_directories(
    ${CVX_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(test_renderer test_renderer.cpp)
target_link_libraries(test_renderer cvx_viz ${OPENGL_LIBRARIES} ${GLFW3_LIBRARY} ${GLEW_LIBRARIES})

add_executable(test_robot test_robot.cpp)
target_link_libraries(test_robot cvx_viz ${OPENGL_LIBRARIES} ${GLFW3_LIBRARY} ${GLEW_LIBRARIES})

add_executable(test_offscreen test_offscreen.cpp)
target_link_libraries(test_offscreen cvx_viz ${OPENGL_LIBRARIES} ${GLFW3_LIBRARY} ${GLEW_LIBRARIES})

add_executable(test_shapes test_shapes.cpp)
target_link_libraries(test_shapes cvx_viz ${OPENGL_LIBRARIES} ${GLFW3_LIBRARY} ${GLEW_LIBRARIES})

add_executable(test_point_cloud test_point_cloud.cpp)
target_link_libraries(test_point_cloud cvx_viz ${OPENGL_LIBRARIES} ${GLFW3_LIBRARY} ${GLEW_LIBRARIES})



# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)

# Find the QtWidgets library
find_package(Qt5Widgets 5.5 CONFIG REQUIRED)

# Tell CMake to create the helloworld executable
add_executable(test_qt test_qt.cpp qt_glwidget.cpp qt_glwidget.hpp)
# Use the Widgets module from Qt 5
target_link_libraries(test_qt cvx_viz Qt5::Widgets ${GLEW_LIBRARIES})
