#include "mesh_data.hpp"
#include <cvx/util/geometry/point_list.hpp>

using namespace cvx::util ;

namespace cvx { namespace viz { namespace impl {


#define POSITION_LOCATION    0
#define NORMALS_LOCATION    1
#define COLORS_LOCATION    2
#define BONE_ID_LOCATION    3
#define BONE_WEIGHT_LOCATION    4
#define UV_LOCATION 5

MeshData::MeshData(const Mesh &mesh)
{
    // Create the VAO

    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    const PointList3f &vertices = mesh.vertices().data() ;
    const PointList3f &normals = mesh.normals().data() ;
    const PointList3f &colors = mesh.colors().data() ;

    elem_count_ = mesh.vertices().data().size() ;
    indices_ = mesh.vertices().indices().size() ;

    glGenBuffers(1, &pos_);
    glBindBuffer(GL_ARRAY_BUFFER, pos_);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat) * 3, &vertices[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(POSITION_LOCATION);
    glVertexAttribPointer(POSITION_LOCATION, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    if ( !normals.empty() ) {
        glGenBuffers(1, &normals_);
        glBindBuffer(GL_ARRAY_BUFFER, normals_);
        glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(GLfloat) * 3, (GLfloat *)normals.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(NORMALS_LOCATION);
        glVertexAttribPointer(NORMALS_LOCATION, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    }

    if ( !colors.empty() ) {
        glGenBuffers(1, &colors_);
        glBindBuffer(GL_ARRAY_BUFFER, colors_);
        glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLfloat) * 3, (GLfloat *)colors.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(COLORS_LOCATION);
        glVertexAttribPointer(COLORS_LOCATION, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    }

    for( uint t = 0 ; t<MAX_TEXTURES ; t++ ) {
        if ( !mesh.texCoords(t).data().empty() ) {
            glGenBuffers(1, &tex_coords_[t]);
            glBindBuffer(GL_ARRAY_BUFFER, tex_coords_[t]);
            glBufferData(GL_ARRAY_BUFFER, mesh.texCoords(t).data().size() * sizeof(GLfloat) * 2, (GLfloat *)mesh.texCoords(t).data().data(), GL_STATIC_DRAW);
            glEnableVertexAttribArray(UV_LOCATION + t);
            glVertexAttribPointer(UV_LOCATION + t, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
        }
    }

#if 0
    glGenBuffers(1, &tf_);
    glBindBuffer(GL_ARRAY_BUFFER, tf_);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat) * 3, 0, GL_STATIC_READ);
#endif

    if ( mesh.vertices().hasIndices() ) {
        glGenBuffers(1, &index_);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.vertices().indices().size() * sizeof(uint32_t),
                     mesh.vertices().indices().data(), GL_STATIC_DRAW);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

MeshData::~MeshData()
{
     glDeleteVertexArrays(1, &vao_) ;
}


}}}

namespace cvx {  namespace  viz {

void Mesh::makeMeshData() {
    MeshPtr fmesh = Mesh::flatten(shared_from_this());
    data_.reset(new impl::MeshData(*fmesh)) ;

}

void BoxGeometry::makeMeshData() {
    MeshPtr mesh = Mesh::createSolidCube(halfExtents()) ;
    MeshPtr fmesh = Mesh::flatten(mesh) ;
    data_.reset(new impl::MeshData(*fmesh)) ;
}

}}
