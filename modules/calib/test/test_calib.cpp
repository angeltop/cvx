#include <iostream>

#include <cvx/util/misc/path.hpp>
#include <cvx/calib/calibration.hpp>


using namespace std ;
using namespace cvx::camera ;
using namespace cvx::util ;
using namespace Eigen ;

int main(int argc, char *argv[]) {

    auto files = Path::entries("/home/malasiot/Downloads/calib/", DirectoryFilters::Glob("*.jpg"), false) ;

    CameraCalibration calib ;
    CameraCalibration::Data cdata ;

    AprilTagDetector adet ;
    AprilTagGridPattern agrid(cv::Size(4, 5), 0.05, 0.01, adet) ;

    calib.detect(files, agrid, cdata) ;
    cdata.save("/tmp/detect.calib") ;

    cdata.load("/tmp/detect.calib") ;

    PinholeCamera cam ;

    vector<Matrix4d> extrinsics ;
    calib.run(cdata, cam, extrinsics) ;

    calib.refine(cdata, cam, extrinsics) ;

  //  cout << H << ' ' << he.residual(pts1, pts2, H) << endl ;

}
