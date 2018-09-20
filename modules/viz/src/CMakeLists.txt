FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Assimp)
FIND_PACKAGE(PkgConfig REQUIRED)
FIND_PACKAGE(ZLIB REQUIRED)
FIND_PACKAGE(GLEW REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(GLFW3 3.2 REQUIRED)
FIND_PACKAGE(OpenCV 3 REQUIRED)


set ( LIBRARY_FOLDER "viz")
get_filename_component(LIBRARY_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../include/cvx/${LIBRARY_FOLDER}/" ABSOLUTE)
message(${LIBRARY_INCLUDE_DIR})
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    3rdparty/
    ${EIGEN3_INCLUDE_DIR}
    ${GLFW3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIR}
)

set(LIB_SOURCES
    renderer/ogl_shaders.cpp
    renderer/renderer.cpp
    renderer/renderer_impl.cpp

    scene/scene.cpp
    scene/assimp_loader.cpp
    scene/camera.cpp
    scene/mesh.cpp
    scene/material.cpp

    gui/trackball.cpp
    gui/glfw_window.cpp
    gui/offscreen.cpp

    3rdparty/pugixml/pugixml.cpp
    3rdparty/pugixml/pugixml.hpp

    robot/robot_scene.cpp
    robot/urdf_loader.cpp
    robot/urdf_loader.hpp

    ${LIBRARY_INCLUDE_DIR}/scene/node.hpp
    ${LIBRARY_INCLUDE_DIR}/scene/drawable.hpp
    ${LIBRARY_INCLUDE_DIR}/scene/geometry.hpp
    ${LIBRARY_INCLUDE_DIR}/scene/mesh.hpp
    ${LIBRARY_INCLUDE_DIR}/scene/camera.hpp
    ${LIBRARY_INCLUDE_DIR}/scene/light.hpp
    ${LIBRARY_INCLUDE_DIR}/scene/material.hpp

    ${LIBRARY_INCLUDE_DIR}/gui/trackball.hpp
    ${LIBRARY_INCLUDE_DIR}/gui/glfw_window.hpp
    ${LIBRARY_INCLUDE_DIR}//gui/offscreen.hpp

    ${LIBRARY_INCLUDE_DIR}/robot/robot_scene.hpp

    ${LIBRARY_INCLUDE_DIR}/renderer/renderer.hpp
)

add_library(cvx_viz ${LIBRARY_TYPE} ${LIB_SOURCES})
target_link_libraries(cvx_viz cvx_util ${ASSIMP_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} ${GLFW3_LIBRARY} ${OpenCV_LIBRARIES} )

# Install library
install(TARGETS cvx_viz
  EXPORT ${PROJECT_EXPORT}
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT stlib
  COMPONENT dev)


# Install headers
install(DIRECTORY "${LIBRARY_INCLUDE_DIR}" DESTINATION "${INSTALL_INCLUDE_DIR}" )


