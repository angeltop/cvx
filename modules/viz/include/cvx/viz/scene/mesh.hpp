#ifndef __CVX_VIZ_MESH_HPP__
#define __CVX_VIZ_MESH_HPP__

#include <cvx/viz/scene/scene_fwd.hpp>
#include <cvx/viz/scene/geometry.hpp>
#include <cvx/util/geometry/point_list.hpp>

#include <string>
#include <vector>
#include <memory>
#include <map>

#include <Eigen/Core>

namespace cvx { namespace viz {

#define MAX_TEXTURES 4

template<typename T, typename Allocator = std::allocator<T>>
class VertexBuffer {
public:
    VertexBuffer() = default ;
    VertexBuffer(std::initializer_list<T> &vdata, std::initializer_list<T> &indices = {}): data_(data), indices_(indices) {}

    std::vector<T, Allocator> &data() { return data_ ; }
    std::vector<uint32_t> &indices() { return indices_ ; }

    const std::vector<T, Allocator> &data() const { return data_ ; }
    const std::vector<uint32_t> &indices() const { return indices_ ; }

    bool hasIndices() const { return !indices_.empty() ; }

protected:

    std::vector<T, Allocator> data_ ;
    std::vector<uint32_t> indices_ ;
};

class Mesh: public Geometry {
public:

    enum PrimitiveType { Triangles, Lines, Points } ;

    Mesh(PrimitiveType t): ptype_(t) {}

    using vb3_t = VertexBuffer<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> ;
    using vb2_t = VertexBuffer<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> ;

    vb3_t &vertices() { return vertices_ ; }
    vb3_t &normals() { return normals_ ; }
    vb3_t &colors() { return colors_ ; }
    vb2_t &texCoords(uint t) {
        assert(t<MAX_TEXTURES) ;
        return tex_coords_[t] ;
    }

    const vb3_t &vertices() const { return vertices_ ; }
    const vb3_t &normals() const { return normals_ ; }
    const vb3_t &colors() const { return colors_ ; }
    const vb2_t &texCoords(uint t) const {
        assert(t<MAX_TEXTURES) ;
        return tex_coords_[t] ;
    }

    PrimitiveType ptype() const { return ptype_ ; }

    // it is a simple triangle mesh with per-pertex attributes
    bool isSimpleIndexed() const ;

    // primitive shape factories

    static MeshPtr createWireCube(const Eigen::Vector3f &hs) ;
    static MeshPtr createSolidCube(const Eigen::Vector3f &hs) ;

    static MeshPtr createWireSphere(float radius, size_t slices, size_t stacks) ;
    static MeshPtr createSolidSphere(float radius, size_t slices, size_t stacks) ;

    // the base of the cone is on (0, 0, 0) aligned with the z-axis and pointing towards positive z

    static MeshPtr createWireCone(float radius, float height, size_t slices, size_t stacks) ;
    static MeshPtr createSolidCone(float radius, float height, size_t slices, size_t stacks) ;

    static MeshPtr createWireCylinder(float radius, float height, size_t slices, size_t stacks) ;
    static MeshPtr createSolidCylinder(float radius, float height, size_t slices, size_t stacks) ;

    static MeshPtr makePointCloud(const cvx::util::EPointList3f &pts) ;
    static MeshPtr makePointCloud(const cvx::util::EPointList3f &coords, const cvx::util::EPointList3f &clrs) ;

    void computeNormals() ;

  //  bool hit(const Ray &ray, Eigen::Vector3f &pos) const override ;

    // create a new
    static MeshPtr flatten(const MeshPtr &src) ;

private:
    vb3_t vertices_, normals_, colors_ ;
    vb2_t tex_coords_[MAX_TEXTURES] ;

    PrimitiveType ptype_ ;

};

} // namespavce viz
} // namespace cvx
#endif
