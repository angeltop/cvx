#ifndef __POLYGON_2D_HPP__
#define __POLYGON_2D_HPP__

#include <cvx/util/geometry/rectangle.hpp>
#include <cvx/util/geometry/point_list.hpp>

namespace cvx { namespace  util {

template < class T >
class Polygon: public PointList<T, 2>
{
    typedef PointList<T, 2> base_t ;

public:

    Polygon(const PointList<T, 2> &pts): PointList<T, 2>(pts) {}

    Polygon(uint n): PointList<T, 2>(n) {}

    Polygon(T *data, int n, bool row_major = true):
        PointList<T, 2>(data, n, row_major) {}

    Polygon(const std::vector< Point<T, 2> > &pts): PointList<T, 2>(pts) {}

    Polygon(const cv::Mat &src): PointList<T, 2>(src) {}

    size_t numPoints() const { return base_t::mat_.rows() ; }

    T area() const {
        int n = numPoints() ;

        T s = 0 ;
        for( uint i=0 ; i<n ; i++ )
        {
            uint i1 = (i+1)%n ;
            const Point<T, 2> &pi = base_t::mat_.row(i), &pi1 = base_t::mat_.row(i1) ;
            s += pi.x()*pi1.y() - pi1.x()*pi.y() ;
        }

        return 0.5*fabs(s) ;
    }

    Rectangle<T> boundingBox() const {
        Point<T, 2> pmin = base_t::mat_.row(0), pmax = base_t::mat_.row(0) ;

        for( uint i=1 ; i<numPoints() ; i++ ) {
            pmin = min(pmin, base_t::mat_.row(i)) ;
            pmax = max(pmax, base_t::mat_.row(i)) ;
        }

        return Rectangle<T>(pmin, pmax) ;
    }

    bool contains(const Point<T, 2> &p) {
        int i, j, nvert = base_t::size() ;
        bool c = false ;
        for (i = 0, j = nvert-1; i < nvert; j = i++) {
            Point<T, 2> vi = base_t::mat_.row(i), vj = base_t::mat_.row(j) ;
            if ( ( (vi.y() > p.y() ) != ( vj.y() > p.y() ) ) &&
                 ( p.x() < ( vj.x() - vi.x() ) * ( p.y() - vi.y() ) / ( vj.y() - vi.y() ) + vi.x() ) )
                c = !c;
        }
        return c ;
    }

    bool contains(T x, T y) { return contains(Point<T, 2>(x, y)) ; }
} ;

typedef Polygon<double> Polygon2d ;
typedef Polygon<float> Polygon2f;

}}

#endif
