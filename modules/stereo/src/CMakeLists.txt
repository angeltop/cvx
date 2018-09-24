find_package(OpenCV 3 REQUIRED COMPONENTS core highgui video imgproc)
find_package(Eigen3 REQUIRED)

include_directories(
        ${CVX_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)

set ( LIBRARY_FOLDER "stereo")
get_filename_component(LIBRARY_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../include/cvx/${LIBRARY_FOLDER}/" ABSOLUTE)

set ( LIB_SOURCES

    gaussian.cpp
)

set ( LIB_HEADERS
#@    ${LIBRARY_INCLUDE_DIR}/pattern.hpp

)


add_library(cvx_stereo ${LIBRARY_TYPE} ${LIB_SOURCES} ${LIB_HEADERS})
TARGET_LINK_LIBRARIES(cvx_stereo cvx_util ${OpenCV_LIBRARIES} )

add_executable(sgm sgm.cpp)
TARGET_LINK_LIBRARIES(sgm cvx_util cvx_stereo ${OpenCV_LIBRARIES} )



# Install library
install(TARGETS cvx_stereo
  EXPORT ${PROJECT_EXPORT}
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT stlib
  COMPONENT dev)

# Install headers
install(DIRECTORY "${LIBRARY_INCLUDE_DIR}" DESTINATION "${INSTALL_INCLUDE_DIR}" )
