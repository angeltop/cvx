#ifndef __POLYGON_SCANNER_HPP__
#define __POLYGON_SCANNER_HPP__

#include <cvx/util/geometry/point.hpp>

#include <vector>

namespace cvx { namespace util {

class PolygonScanIterator
{
  public:

    // input data should be row-wise matrices
    typedef Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> pts_matrix_t ;
    typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> attribute_matrix_t ;

    // Iterates over all pixels inside a concave polygon.
    // One may also provide attributes for polygon vertices. Each attribute is
    // nelem-dimensional vector. Vertex attributes are stored sequentially in
    // array att. Then the scanner linearly interpolates attributes at pixel centers

    PolygonScanIterator(const pts_matrix_t &p, const attribute_matrix_t &attr = attribute_matrix_t()) ;
    ~PolygonScanIterator() ;

    void reset() ;

    // Checks whether the iterator is still valid
    operator int () const ;

    // Go to the next pixel
    PolygonScanIterator & operator++() ;

    // Get the coordinates of the current pixel
    Point2i point() const { return Point2i(x_, y_) ; }

  // Get the coordinate i of the interpolate attribute vector at the current pixel
    const Eigen::VectorXf & attribute() const { return a_ ; }

private:

    friend class CompareEdge ;

    struct Edge {
        float x_, dx_ ;
        Eigen::VectorXf a_, da_ ;
        int i_;
    } ;

    std::vector<Edge> active_ ;

    const pts_matrix_t &poly_ ;
    const attribute_matrix_t &attr_ ;
    int nact_, jscan_ ;
    int gk_, n_, y0_, y1_, x_, y_, xl_, xr_, nelem_ ;
    Eigen::VectorXf a_, da_ ;
    std::vector<int> ind_ ;
    bool valid_ ;

    void cdelete(int) ;
    void cinsert(int, int) ;

    void findActiveScans() ;
    void findNextValidScan() ;
    void incrementX() ;

} ;

void getPointsInPoly(const Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> &poly,
                     std::vector<Point2i> &pts) ;

/*
class TriangleScanIterator: public PolygonScanIterator
{
public:

    TriangleScanIterator(const Triangle2d &tr, double *at = NULL, int nelem = 0):
        PolygonScanIterator(Polygon2d(tr), at, nelem) {}

} ;


void getPointsInPoly(const Polygon2d &poly, std::vector<Point> &pts) ;
*/

}}

#endif
