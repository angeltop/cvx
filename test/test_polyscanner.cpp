#include <cvx/util/geometry/point.hpp>
#include <cvx/util/geometry/polygon_scanner.hpp>

#include <iostream>

using namespace std ;
using namespace cvx::util ;

int main(int argc, char *argv[])
{
    PolygonScanIterator::pts_matrix_t pts(5, 2) ;
    PolygonScanIterator::attribute_matrix_t  attrs(5, 3) ;

    pts << 100, 100,  50, 150,  100, 200,  140, 140,  200, 100 ;
    attrs << 255, 20, 30,  40, 50, 255,  30, 255, 20,  0, 255, 128,  255, 255, 255 ;

    cv::Mat_<cv::Vec3b> im(300, 300) ;

    im = 0 ;

    PolygonScanIterator it(pts, attrs) ;

    while ( it ) {
        Point2i p = it.point() ;
        Eigen::VectorXf a = it.attribute() ;
        im[p.y()][p.x()] = cv::Vec3b(a[0], a[1], a[2]) ;
        ++it ;
    }

    cv::imwrite("/tmp/oo.png", im) ;

}

