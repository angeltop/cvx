
if ( NOT LAPACK_FOUND )
    message("Lapack not found. levmar will be build without lapack")
else ( NOT LAPACK_FOUND)
    add_definitions(-DHAVE_LAPACK)
endif ( NOT LAPACK_FOUND )

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# levmar library source files
add_library(levmar SHARED
  lm.c Axb.c misc.c lmlec.c lmbc.c lmblec.c lmbleic.c
  levmar.h misc.h compiler.h
)

#link_libraries(levmar ${LAPACK_LIBRARIES} ${F2C_LIBRARIES})
