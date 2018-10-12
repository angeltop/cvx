#include <cvx/util/geometry/point_list.hpp>
#include <cvx/util/geometry/line.hpp>
#include <cvx/util/geometry/line_fit.hpp>
#include <cvx/util/geometry/polygon.hpp>

#include <Eigen/Geometry>


using namespace std ;
using namespace cvx::util ;
using namespace Eigen ;

int main(int argc, char *argv[]) {
    PointList3f a {{ 1.0, 2, 3}, {4.0, 5.0, 6}};
    float data[] = {1.0, 2.0, 3.0, 4.0, 5, 6, 7, 8, 9, 10, 11, 12} ;
    PointList4f b { data, 2, false } ;


    Eigen::MatrixX3f m(4, 3) ;
    m << 1, 2, 3,
         4, 11, 6,
         7, 8, 9,
         10, 4, -2;


    cout << m << endl ;
    PointList3f mm(m) ;



    cv::Mat f(4, 3, CV_32FC1, data) ;

 //   cout << f << endl ;

    PointList3f g(f) ;

    Eigen::Vector3f q(1, 2, 3) ;

 //   Isometry3f tr ;
 //   tr.setIdentity() ;
 //   tr.translate(Vector3f{1, 1, 1});

 //   mm.transform(tr);
    cout << mm.asEigenMap() << endl ;
    cout << mm.bbox().first << endl << mm.bbox().second << endl ;
    Line2f line({0, 0}, {1, 0}) ;

    fitLine(mm) ;

    Polygon2f pp({{1, 2}, {3, 4}, {5, 6}}) ;
}
