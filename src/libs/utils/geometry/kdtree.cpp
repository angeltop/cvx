#include <cvx/util/geometry/kdtree.hpp>
#include <cvx/util/geometry/point_list.hpp>

#include "../3rdparty/nanoflann.hpp"

using namespace std ;
using namespace Eigen ;
using namespace nanoflann ;

namespace cvx { namespace util {

struct PointCloudAdaptor3
{
    typedef EPointList3f point_list_t ;

    PointCloudAdaptor3(const point_list_t &data): pts_(data) {}

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts_.size(); }

    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    inline float kdtree_distance(const float *p1, const size_t idx_p2, size_t /*size*/) const
    {
        const float d0=p1[0] - pts_[idx_p2].x();
        const float d1=p1[1] - pts_[idx_p2].y();
        const float d2=p1[2] - pts_[idx_p2].z();
        return d0*d0+d1*d1+d2*d2;
    }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline float kdtree_get_pt(const size_t idx, int dim) const
    {
        if ( dim == 0 ) return pts_[idx].x();
        else if (dim == 1 ) return pts_[idx].y();
        else return pts_[idx].z() ;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

    const point_list_t pts_ ;
};

class KDTreeIndex3
{
public:

    typedef EPointList3f point_list_t ;

    KDTreeIndex3(const point_list_t &data) {
        data_.reset(new PointCloudAdaptor3(data)) ;
        index_.reset(new  kd_tree_t(3, *data_, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) ) ) ;
        index_->buildIndex() ;
    }

    void knn(const Vector3f &q, uint k, vector<uint> &indices, vector<float> &distances) {
        k = std::min(data_->kdtree_get_point_count(), (size_t)k) ;
        indices.resize(k) ;
        distances.resize(k) ;
        KNNResultSet<float, uint> results(k);
        results.init(indices.data(), distances.data() );
        index_->findNeighbors(results, q.data(), nanoflann::SearchParams());
    }

    void radiusSearch(const Vector3f &q, float radius, vector<uint> &indices, vector<float> &distances) {

        std::vector<std::pair<uint,float> > indices_dists;

        index_->radiusSearch(q.data(), radius, indices_dists, nanoflann::SearchParams());

        for( uint i=0 ; i<indices_dists.size() ; i++ ) {
           indices.push_back(indices_dists[i].first) ;
           distances.push_back(indices_dists[i].second) ;
        }
    }

private:

    typedef nanoflann::KDTreeSingleIndexAdaptor< L2_Simple_Adaptor<float, PointCloudAdaptor3 > , PointCloudAdaptor3,  3 /* dim */, uint > kd_tree_t;

    std::shared_ptr<kd_tree_t> index_ ;
    std::shared_ptr<PointCloudAdaptor3> data_ ;
};

KDTree3::KDTree3(const point_list_t &data): index_(new KDTreeIndex3(data))
{

}

void KDTree3::train(const point_list_t &data)
{
    index_.reset(new KDTreeIndex3(data)) ;
}

uint KDTree3::nearest(const point_t &q)
{
    vector<uint> indices ;
    vector<float> distances ;

    index_->knn(q, 1, indices, distances) ;

    return indices[0] ;
}

uint KDTree3::nearest(const point_t &q, float &dist)
{
    vector<uint> indices ;
    vector<float> distances ;

    index_->knn(q, 1, indices, distances) ;

    dist = distances[0] ;

    return indices[0] ;
}

void KDTree3::knearest(const point_t &q, uint k, std::vector<uint> &indexes)
{
    vector<float> distances ;

    index_->knn(q, k, indexes, distances) ;

}

void KDTree3::knearest(const point_t &q, uint k, std::vector<uint> &indexes, vector<float> &distances)
{
    index_->knn(q, k, indexes, distances) ;
}

void KDTree3::withinRadius(const point_t &q, float radius, std::vector<uint> &indexes)
{
    vector<float> distances ;
    index_->radiusSearch(q, radius, indexes, distances) ;
}

void KDTree3::withinRadius(const point_t &q, float radius, std::vector<uint> &indexes,  vector<float> &distances )
{
    index_->radiusSearch(q, radius, indexes, distances) ;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct PointCloudAdaptor2
{
    typedef PointList<float, 2> point_list_t ;

    PointCloudAdaptor2(const point_list_t &data): pts_(data) {}

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts_.size(); }

    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    inline float kdtree_distance(const float *p1, const size_t idx_p2, size_t /*size*/) const
    {
        const float d0=p1[0] - pts_[idx_p2].x();
        const float d1=p1[1] - pts_[idx_p2].y();
        return d0*d0+d1*d1;
    }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline float kdtree_get_pt(const size_t idx, int dim) const
    {
        if ( dim == 0 ) return pts_[idx].x();
        else if (dim == 1 ) return pts_[idx].y();
        else return 0 ;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

    const point_list_t &pts_;
};

class KDTreeIndex2
{
public:

    typedef PointList<float, 2> point_list_t ;

    KDTreeIndex2(const point_list_t &data) {
        data_.reset(new PointCloudAdaptor2(data)) ;
        index_.reset(new  kd_tree_t(2, *data_, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) ) ) ;
        index_->buildIndex() ;
    }

    void knn(const Vector2f &q, uint k, vector<uint> &indices, vector<float> &distances) {
        k = std::min(data_->kdtree_get_point_count(), (size_t)k) ;
        indices.resize(k) ;
        distances.resize(k) ;
        KNNResultSet<float, uint> results(k);
        results.init(indices.data(), distances.data() );
        index_->findNeighbors(results, q.data(), nanoflann::SearchParams());
    }

    void radiusSearch(const Vector2f &q, float radius, vector<uint> &indices, vector<float> &distances) {

        std::vector<std::pair<uint,float> > indices_dists;

        index_->radiusSearch(q.data(), radius, indices_dists, nanoflann::SearchParams());

        for( uint i=0 ; i<indices_dists.size() ; i++ ) {
           indices.push_back(indices_dists[i].first) ;
           distances.push_back(indices_dists[i].second) ;
        }
    }

private:

    typedef nanoflann::KDTreeSingleIndexAdaptor< L2_Simple_Adaptor<float, PointCloudAdaptor2 > , PointCloudAdaptor2,  2 /* dim */, uint > kd_tree_t;

    std::shared_ptr<kd_tree_t> index_ ;
    std::shared_ptr<PointCloudAdaptor2> data_ ;
};

KDTree2::KDTree2(const point_list_t &data): index_(new KDTreeIndex2(data))
{

}

void KDTree2::train(const point_list_t &data)
{
    index_.reset(new KDTreeIndex2(data)) ;
}

uint KDTree2::nearest(const point_t &q)
{
    vector<uint> indices ;
    vector<float> distances ;

    index_->knn(q, 1, indices, distances) ;

    return indices[0] ;
}

uint KDTree2::nearest(const point_t &q, float &dist)
{
    vector<uint> indices ;
    vector<float> distances ;

    index_->knn(q, 1, indices, distances) ;

    dist = distances[0] ;

    return indices[0] ;
}

void KDTree2::knearest(const point_t &q, uint k, std::vector<uint> &indexes)
{
    vector<float> distances ;

    index_->knn(q, k, indexes, distances) ;

}

void KDTree2::knearest(const point_t &q, uint k, std::vector<uint> &indexes, vector<float> &distances)
{
    index_->knn(q, k, indexes, distances) ;
}

void KDTree2::withinRadius(const point_t &q, float radius, std::vector<uint> &indexes)
{
    vector<float> distances ;
    index_->radiusSearch(q, radius, indexes, distances) ;
}

void KDTree2::withinRadius(const point_t &q, float radius, std::vector<uint> &indexes,  vector<float> &distances )
{
    index_->radiusSearch(q, radius, indexes, distances) ;
}

}}
