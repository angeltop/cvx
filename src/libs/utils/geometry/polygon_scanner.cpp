

/*
 * This is a wrapper of
 *
 * Concave Polygon Scan Conversion
 * by Paul Heckbert
 * from "Graphics Gems", Academic Press, 1990
 */


#include <cvx/util/geometry/polygon_scanner.hpp>
#include <cvx/util/geometry/point.hpp>

#include <vector>

using namespace std ;

namespace cvx { namespace util {

struct CompareEdge {

bool operator () ( const PolygonScanIterator::Edge &u, const PolygonScanIterator::Edge &v )
{
    return (u.x_ < v.x_) ;
}

} ;

struct CompareInd {

    typedef Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> pts_matrix_t ;

    CompareInd(const pts_matrix_t &poly): pt_(poly) {}

    bool operator () ( int u, int v ) {
        return pt_.row(u).y() < pt_.row(v).y() ;
    }

    const pts_matrix_t &pt_ ;

};

PolygonScanIterator::PolygonScanIterator(const pts_matrix_t &pts, const attribute_matrix_t &attrs): poly_(pts), attr_(attrs), nelem_(attrs.cols()) {

    n_ = poly_.rows() ;
    assert( n_ > 2 ) ;

    ind_.resize(n_) ;
    active_.resize(n_) ;

    for( uint i=0 ; i<n_ ; i++ ) {
        active_[i].a_.resize(nelem_) ;
        active_[i].da_.resize(nelem_) ;
    }

    a_.resize(nelem_) ;
    da_.resize(nelem_) ;

    reset() ;
}

PolygonScanIterator::~PolygonScanIterator() {
}

void PolygonScanIterator::incrementX()
{
    float frac = 1.0 ;

    float dx = active_[jscan_+1].x_ - active_[jscan_].x_ ;
    if ( dx == 0. ) dx = 1. ;
    frac = x_ + .5 - active_[jscan_].x_ ;

    float *pda = da_.data(), *pa = a_.data(), *p2 = active_[jscan_+1].a_.data(), *p1 = active_[jscan_].a_.data() ;

    for( int j=0 ; j<nelem_ ; j++, pa++, pda++, p1++, p2++ ) {
        *pda = ( *p2 - *p1)/dx ;
        *pa = *p1 + *pda * frac;
    }
}

void PolygonScanIterator::reset()
{
    for (int k=0; k<n_; k++) ind_[k] = k;

    std::sort(ind_.begin(), ind_.end(), CompareInd(poly_)) ;

    gk_ = 0 ; nact_ = 0;

    y0_ = (int)ceil(poly_.row(ind_[0]).y()-.5);
    y1_ = (int)floor(poly_.row(ind_[n_-1]).y()-.5);

    y_ = y0_ ; jscan_ = 0 ;

    valid_ = true ;

    findActiveScans() ;

    findNextValidScan() ;

    if ( y_ > y1_ || xl_ > xr_ ) {
        valid_ = false ;
        return ;
    }

    x_ = xl_ ;

    incrementX() ;
}

void PolygonScanIterator::findNextValidScan()
{
    while ( y_ <= y1_ )
    {
        jscan_ = 0 ;

        while ( jscan_ < nact_ ) {
            xl_ = (int)ceil(active_[jscan_].x_-.5);
            xr_ = (int)floor(active_[jscan_+1].x_-.5);

            if ( xl_ > xr_ ) {
                active_[jscan_].x_ += active_[jscan_].dx_;
                active_[jscan_+1].x_ += active_[jscan_+1].dx_;
                jscan_ += 2 ;
            }
            else return ;

        } ;

        ++y_ ;

        if ( y_ <= y1_ )
            findActiveScans() ;
    }

}


void PolygonScanIterator::findActiveScans()
{
    int i, j ;

    for ( ; gk_ < n_ && poly_.row(ind_[gk_]).y() <= y_+.5 ; gk_++ )
    {
        i = ind_[gk_];
        j = i>0 ? i - 1 : n_ - 1;

        if ( poly_.row(j).y() <= y_ -.5 ) cdelete(j);
        else if ( poly_.row(j).y() > y_ +.5 ) cinsert(j, y_);

        j = i<n_-1 ? i+1 : 0;

        if ( poly_.row(j).y() <= y_-.5 ) cdelete(i) ;
        else if ( poly_.row(j).y() > y_+.5 ) cinsert(i, y_);
    }

    std::sort(active_.begin(), active_.begin() + nact_, CompareEdge());
}

PolygonScanIterator::operator int () const { return valid_ ; }

PolygonScanIterator & PolygonScanIterator::operator++()
{
    valid_ = false ;

    ++x_ ;

    float *pa = a_.data(), *pda = da_.data() ;
    for(  int j=0 ; j<nelem_ ; j++, pa++, pda++ ) *pa += *pda ;

    if ( x_ <= xr_ ) {
        valid_ = true ;
        return *this ;
    }
    active_[jscan_].x_ += active_[jscan_].dx_;
    active_[jscan_+1].x_ += active_[jscan_+1].dx_;

    pa = active_[jscan_].a_.data() ; pda = active_[jscan_].da_.data()  ;
    for( uint j=0 ; j<nelem_ ; j++, pa++, pda++ ) *pa += *pda ;
    pa = active_[jscan_+1].a_.data() ; pda = active_[jscan_+1].da_.data()  ;
    for( uint j=0 ; j<nelem_ ; j++, pa++, pda++ ) *pa += *pda ;

    jscan_ += 2 ;

    if ( jscan_ < nact_ ) {
        x_ = xl_ = (int)ceil(active_[jscan_].x_-.5);
        xr_ = (int)floor(active_[jscan_+1].x_-.5);

        incrementX() ;

        valid_ = true ;
        return *this ;
    }
    else
    {
        ++y_ ;
        findActiveScans() ;
        findNextValidScan() ;

        if ( y_ <= y1_ ) {
            x_ = xl_ ;
            incrementX() ;
            valid_ = true ;
            return *this ;
        }
        else valid_ = false ;

        return *this ;
    }

    return *this ;
}

void PolygonScanIterator::cdelete(int i)
{
    uint j ;
    for ( j=0; j<nact_ && active_[j].i_ != i; j++ ) ;
    if ( j>=nact_ ) return ;
    nact_-- ;
    active_.erase(active_.begin()+j) ;
}

using namespace Eigen ;

void PolygonScanIterator::cinsert(int i, int y)
{
    int j;
    float dx, day;
    Point2f p, q ;
    VectorXf ap, aq ;

    j = i<poly_.rows()-1 ? i+1 : 0 ;

    if (poly_.row(i).y() < poly_.row(j).y()) {
        p = poly_.row(i); q = poly_.row(j) ;
        if ( nelem_ > 0 ) {
            ap = attr_.row(i) ; aq = attr_.row(j) ;
        }
    }
    else {
        p = poly_.row(j); q = poly_.row(i) ;
        if ( nelem_ > 0 ) {
            ap = attr_.row(j) ; aq = attr_.row(i) ;
        }
    }

    active_[nact_].dx_ = dx = (q.x() - p.x())/(q.y() - p.y());
    active_[nact_].x_ = dx*( y +.5 - p.y()) + p.x();
    active_[nact_].i_ = i ;

    float *paq = aq.data(), *pap = ap.data(), *pa = active_[nact_].a_.data(), *pda = active_[nact_].da_.data() ;

    for( j=0 ; j<nelem_ ; j++, pap++, paq++, pa++, pda++ )
    {
        *pda = day = (*paq - *pap)/( q.y() - p.y() ) ;
        *pa = day * (y + .5 -p.y()) + *pap ;
    }

    nact_++;
}

void getPointsInPoly(const Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> &poly, std::vector<Point2i> &pts)
{
    PolygonScanIterator it(poly) ;

    while (it) {
        pts.push_back(it.point()) ;
        ++it ;
    }

}

} }
