#include <cvx/util/geometry/point.hpp>
#include <cvx/util/geometry/point_list.hpp>
#include <cvx/util/geometry/line.hpp>
#include <cvx/util/geometry/line_fit.hpp>

#include <boost/random.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#include <iostream>
#include <fstream>

using namespace std ;
using namespace cvx::util ;


void random_pts(vector<Point2f> &pts, const Line2f &l, int n_pts, float var) {
    typedef boost::mt19937 RNGType;
    RNGType rng;

    boost::uniform_real<> ur( -1, 1 );
    boost::variate_generator< RNGType, boost::uniform_real<> > gen(rng, ur) ;

    Vector2f dir = l.dir() ;
    Vector2f n(dir.y(), -dir.x()) ;

    for( int i=0 ; i<n_pts ; i++ ) {
        float t = gen() ;
        Point2f p = l.origin() + t * l.dir() ;
        p += gen() * var * n ;
        pts.push_back(p) ;
    }
}

void save_line(const Line2f &l, int n_pts, const string &fileName) {
       ofstream strm(fileName.c_str()) ;

       typedef boost::mt19937 RNGType;
       RNGType rng;

       boost::uniform_real<> ur( -2, 2 );
       boost::variate_generator< RNGType, boost::uniform_real<> > gen(rng, ur) ;

       Vector2f dir = l.dir() ;

       for( int i=0 ; i<n_pts ; i++ ) {
           float t = gen() ;
           Point2f p = l.origin() + t * l.dir() ;
           strm << p << endl ;
       }
}

void save_pts(const vector<Point2f> &pts, const string &fileName) {
    ofstream strm(fileName.c_str()) ;
    for( uint i=0 ; i<pts.size() ; i++ )
        strm << pts[i] << endl ;
}

int main(int argc, char *argv[])
{
    vector<Point2f> pts ;

    random_pts(pts, Line2f(Point2f(0, 0), Point2f(1, 1)), 100, 0.1) ;
    random_pts(pts, Line2f(Point2f(-0.5, 2.0), Point2f(1.0, 0.5)), 20, 0.1) ;

    save_pts(pts, "/tmp/pts.txt") ;

    PointList2f plist(pts) ;

    Line2f line = fitLineRobust(plist) ;
    float a[3] ;
    line.coeffs(a) ;
    cout << line.origin() << ' ' << line.dir() << endl ;
    save_line(line, 100, "/tmp/line1.txt" ) ;

    cv::Vec4f ll;
    cv::fitLine(plist.toCVMat(), ll, CV_DIST_HUBER, 0, 0.01, 0.01) ;

    Line2f line_cv(Point2f(ll[2], ll[3]), Point2f(ll[0], ll[1])) ;

    save_line(line_cv, 100, "/tmp/line2.txt" ) ;

    cout << ll << endl ;
}

