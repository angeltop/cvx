#include <certh_core/RGBDUtil.h>

using namespace certh_core ;
using namespace Eigen ;
using namespace std ;

int main(int argc, char *argv[]) {

    cv::Mat im = cv::imread("/home/malasiot/Downloads/20objects/data/Kinfu_Audiobox1_spot/depth_noseg/depth_00388.png", -1) ;

    PinholeCamera cam(570, 570, 640/2, 480/2, cv::Size(640, 480)) ;

    vector<Vector4f> coeffs ;
    findAllPlanes(im, cam, coeffs) ;

    cv::Mat mask = segmentPointsAbovePlane(im, cam, coeffs[0]) ;

    cv::imwrite("/tmp/oo.png", mask) ;

    cout << "ok here" << endl ;

}

