FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Assimp REQUIRED)

FIND_PACKAGE(ZLIB REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(glfw3 3 REQUIRED)
FIND_PACKAGE(OpenCV 3 REQUIRED)

FIND_PACKAGE(Fontconfig REQUIRED)
FIND_PACKAGE(Freetype REQUIRED)
FIND_PACKAGE(HarfBuzz REQUIRED)


set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5Widgets 5.5 CONFIG REQUIRED)

set ( LIBRARY_FOLDER "viz")
get_filename_component(LIBRARY_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../include/cvx/${LIBRARY_FOLDER}/" ABSOLUTE)

include_directories(
    ${CVX_INCLUDE_DIRS}
    3rdparty/
    ${EIGEN3_INCLUDE_DIR}
    ${GLFW3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIR}
    ${FREETYPE_INCLUDE_DIRS}
    ${HARFBUZZ_INCLUDE_DIRS}
    ${FONTCONFIG_INCLUDE_DIR}
)

set(LIB_SOURCES
    renderer/ogl_shaders.cpp
    renderer/renderer.cpp
    renderer/renderer_impl.cpp
    renderer/font_manager.cpp
    renderer/font_manager.hpp
    renderer/glyph_cache.cpp
    renderer/glyph_cache.hpp
    renderer/text_item.cpp
    renderer/text_item.hpp
    renderer/mesh_data.cpp
    renderer/mesh_data.hpp
    renderer/text.cpp
    renderer/gl3w.c

    scene/scene.cpp
    scene/assimp_loader.cpp
    scene/camera.cpp
    scene/mesh.cpp
    scene/material.cpp
    scene/geometry.cpp
    scene/intersect.cpp
    scene/intersect.hpp
    scene/octree.hpp
    scene/octree.cpp
    scene/marker.cpp

    gui/trackball.cpp
    gui/glfw_window.cpp
    gui/offscreen.cpp

    3rdparty/pugixml/pugixml.cpp
    3rdparty/pugixml/pugixml.hpp

    robot/robot_scene.cpp
    robot/urdf_loader.cpp
    robot/urdf_loader.hpp

    image/widget.cpp
    image/view.cpp
    image/tools.cpp
    image/tools_p.hpp

    ${LIBRARY_INCLUDE_DIR}/scene/node.hpp
    ${LIBRARY_INCLUDE_DIR}/scene/drawable.hpp
    ${LIBRARY_INCLUDE_DIR}/scene/geometry.hpp
    ${LIBRARY_INCLUDE_DIR}/scene/mesh.hpp
    ${LIBRARY_INCLUDE_DIR}/scene/camera.hpp
    ${LIBRARY_INCLUDE_DIR}/scene/light.hpp
    ${LIBRARY_INCLUDE_DIR}/scene/material.hpp
    ${LIBRARY_INCLUDE_DIR}/scene/marker.hpp

    ${LIBRARY_INCLUDE_DIR}/gui/trackball.hpp
    ${LIBRARY_INCLUDE_DIR}/gui/glfw_window.hpp
    ${LIBRARY_INCLUDE_DIR}//gui/offscreen.hpp

    ${LIBRARY_INCLUDE_DIR}/robot/robot_scene.hpp

    ${LIBRARY_INCLUDE_DIR}/renderer/renderer.hpp
    ${LIBRARY_INCLUDE_DIR}/renderer/text.hpp


    ${LIBRARY_INCLUDE_DIR}/image/widget.hpp
    ${LIBRARY_INCLUDE_DIR}/image/tools.hpp
    ${LIBRARY_INCLUDE_DIR}/image/view.hpp
)

QT5_ADD_RESOURCES(RCC_SRCS "image/mdi.qrc")

add_library(cvx_viz ${LIBRARY_TYPE} ${LIB_SOURCES} ${RCC_SRCS})

target_link_libraries(cvx_viz cvx_util
    Qt5::Widgets
    ${ASSIMP_LIBRARIES}
    ${OPENGL_LIBRARIES}
    glfw
    ${GLFW_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${HARFBUZZ_LIBRARIES}
    ${FONTCONFIG_LIBRARIES}
    ${FREETYPE_LIBRARIES})

add_executable(iview
    image/iview/main_window.cpp
    image/iview/main_window.hpp
    image/iview/iview.cpp
)

target_link_libraries(iview cvx_viz cvx_util
    Qt5::Widgets
    ${OpenCV_LIBRARIES}
)

# Install library
install(TARGETS cvx_viz iview
  EXPORT ${PROJECT_EXPORT}
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT stlib
  COMPONENT dev)


# Install headers
install(DIRECTORY "${LIBRARY_INCLUDE_DIR}" DESTINATION "${INSTALL_INCLUDE_DIR}/cvx" )


