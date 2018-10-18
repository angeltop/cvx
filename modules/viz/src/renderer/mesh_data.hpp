#ifndef __CVX_VIZ_MESH_DATA_HPP__
#define __CVX_VIZ_MESH_DATA_HPP__

#include <cvx/viz/scene/mesh.hpp>

#include "GL/gl3w.h"

namespace cvx { namespace viz { namespace impl {

class MeshData {
public:

    MeshData() ;
    MeshData(const Mesh &mesh) ;

    static const int max_textures_ = 4 ;

    GLuint pos_ = 0, normals_ = 0, colors_ = 0, tex_coords_[max_textures_] = {0}, tf_ = 0, index_ = 0;
    GLuint vao_ ;
    GLuint elem_count_, indices_  ;

    ~MeshData() ;

} ;


}}}

#endif
