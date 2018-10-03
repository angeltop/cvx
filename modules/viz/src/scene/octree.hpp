#ifndef __CVX_VIZ_OCTREE_HPP__
#define __CVX_VIZ_OCTREE_HPP__

#include <Eigen/Core>
#include <vector>

#include <cvx/viz/scene/mesh.hpp>

#include "intersect.hpp"

using Eigen::Vector3f ;
using std::vector ;


namespace cvx { namespace viz { namespace detail {

struct OctreeNode
{
    vector<uint32_t> data_;
    OctreeNode* children_[8] = { nullptr } ;
    bool is_leaf_ = true ;

    OctreeNode() = default ;
    ~OctreeNode();
};

class Octree
{
public:

    Octree( const Mesh &mesh, uint max_depth = 5, uint max_count = 100);

    ~Octree() { delete root_; }

    void insert(uint32_t tindex, const Vector3f &v0, const Vector3f &v1, const Vector3f &v2);

    bool intersect(const Ray &ray, uint32_t &tindex, float &t);

private:
    void insertTriangle( OctreeNode *node, const Vector3f &center, const Vector3f &hs, uint depth,
                         uint32_t tindex, const Vector3f &v0, const Vector3f &v1, const Vector3f &v2);

    bool intersect(OctreeNode *node, const Ray &r, const Vector3f &center, const Vector3f &hs, uint32_t &tindex, float &mint);

    void getTriangleVertices(uint32_t tindex, Vector3f &v0, Vector3f &v1, Vector3f &v2);

    static const Vector3f offsets_[8] ;

    Vector3f center_, hs_ ;

    const Mesh::vb3_t::data_container_t &vertices_ ;
    const Mesh::vb3_t::index_container_t &indices_ ;
    uint max_depth_, max_count_ ;
    OctreeNode* root_;


};

}}}

#endif
