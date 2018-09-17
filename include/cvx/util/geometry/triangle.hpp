#ifndef __TRIANGLE_2D_HPP__
#define __TRIANGLE_2D_HPP__

#include <cvx/util/geometry/polygon.hpp>

namespace cvx { namespace  util {

template < class T >
class Triangle: public Polygon<T>
{
    typedef Polygon<T> base_t ;
    typedef Point<T, 2> point_t ;

public:

    Triangle(const point_t &A, const point_t &B, const point_t &C): Polygon<T>(3) {
        base_t::mat_.row(0)=A ;
        base_t::mat_.row(1)=B ;
        base_t::mat_.row(2)=C ;
    }

    Triangle(const Point2d pl[3]): Polygon<T>(3) {
        base_t::mat_.row(0)=pl[0] ;
        base_t::mat_.row(1)=pl[1] ;
        base_t::mat_.row(2)=pl[2] ;
    }

    point_t A() const { return base_t::mat_.row(0) ; }
    point_t B() const { return base_t::mat_.row(1) ; }
    point_t C() const { return base_t::mat_.row(2) ; }

    static int orientation(const point_t &P, const point_t &Q, const point_t &R) {
        point_t A = Q - P, B = R - P ;

        T d = A.x()*B.y() - A.y()*B.x() ;

        if ( fabs(d) < std::numeric_limits<T>::min() ) return 0 ;
        else return ( d < 0 ? -1 : 1 ) ;
    }

    bool contains(const point_t &p) const {
        int orABP,orBCP,orCAP ;

        orABP = orientation(A(), B(), p) ;
        orBCP = orientation(B(), C(), p) ;
        orCAP = orientation(C(), A(), p) ;

        if ( orCAP == orBCP && orABP == orCAP ) return true ;

        if ( ( orABP == 0 && orCAP == orBCP ) ||
             ( orBCP == 0 && orABP == orCAP ) ||
             ( orCAP == 0 && orBCP == orABP ) ) return true ;  // on edge

        return false ;
    }

} ;


}}

#endif
