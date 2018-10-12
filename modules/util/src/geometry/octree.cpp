#include <cvx/util/geometry/octree.hpp>
#include <functional>
#include <float.h>

using namespace Eigen ;
using namespace std ;

namespace cvx { namespace util {

class OctreeVisitor
{
public:

    OctreeVisitor(const PointList3f &cloud, PointList3f &centers): cloud_(cloud), centers_(centers) {}

    bool visit(const Vector3f &, const Vector3f &, const vector<uint> &data) {

        uint n = data.size() ;

        if ( n > 0 )
        {
            Vector3f c(0, 0, 0) ;
            for(uint i=0 ; i<n ; i++)  c += cloud_[data[i]] ;
            c /= n ;

            float min_dist = std::numeric_limits<float>::max() ;
            Vector3f best_pt ;

            for(uint i=0 ; i<n ; i++) {
                const Vector3f &p = cloud_[data[i]] ;
                float d = ( p - c ).squaredNorm() ;
                if ( d < min_dist ) {
                    min_dist = d ;
                    best_pt = p ;
                }
            }

            centers_.push_back(best_pt) ;
            return false ;
        }

        return true ;
    }

    const PointList3f &cloud_ ;
    PointList3f &centers_ ;

};

void sampleCloudCenters(const PointList3f &cloud, float cell_size, PointList3f &res, const Vector3f &pmin, const Vector3f &pmax) {

    OctreeIndexed tree(pmin, pmax, Vector3f(cell_size, cell_size, cell_size)) ;

    for(uint i=0 ; i<cloud.size() ; i++ )
        tree.insert(cloud[i], i) ;

    using namespace std::placeholders;

    OctreeVisitor v(cloud, res) ;
    tree.traverse(std::bind(&OctreeVisitor::visit, v, _1, _2, _3)) ;
}


}}
