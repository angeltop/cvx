#include <iostream>

#include <cvx/util/misc/path.hpp>
#include <cvx/calib/calibration.hpp>
#include <cvx/calib/pose.hpp>
#include <cvx/util/misc/arg_parser.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <regex>

using namespace std ;
using namespace cvx::util ;
using namespace cvx::camera ;
using namespace Eigen ;

istream &operator >> (istream &strm, cv::Size &sz) {
    strm >> sz.width >> sz.height ;
    return strm ;
}

int main(int argc, char *argv[]) {

    ArgumentParser args ;

    string data_folder, output_file ;
    bool compute_markers = false ;
    cv::Size pattern_grid(4, 5) ;
    float pattern_width = 0.04, pattern_gap = 0.01 ;
    bool print_help = false, refine = false ;

    args.setDescription("Usage: camera_intrinsics [options]") ;
    args.addOption("-h|--help", print_help).setMaxArgs(0).setDescription("print this help message").setImplicit("true") ;
    args.addOption("--data", data_folder).required().setName("<folder>").setDescription("Folder with captured images") ;
    args.addOption("--markers", compute_markers).setMaxArgs(0).setDescription("Recompute markers positions on images").setImplicit("true") ;
    args.addOption("--refine", refine).setMaxArgs(0).setDescription("Refine calibration using bundle adjustment").setImplicit("true") ;
    args.addOption("--grid", pattern_grid).setMinArgs(2).setMaxArgs(2).setName("<gx> <gy>").setDescription("Number of tags in x and y dimensions") ;
    args.addOption("--width", pattern_width).setName("<arg>").setDescription("Width of the AprilTag square in meters") ;
    args.addOption("--gap", pattern_gap).setName("<arg>").setDescription("Gap between AprilTags in the grid in meters") ;
    args.addOption("--out", output_file).required().setName("<output>").setDescription("Path to output file to store the camera") ;

    try {
        args.parse(argc, (const char **)argv)  ;

        if ( print_help ) {
            args.printUsage(std::cout) ;
            exit(0);
        }
    } catch ( ArgumentParserException &e ) {
        cerr << e.what() << endl ;
        args.printUsage(std::cerr) ;
        exit(1) ;
    }

    CameraCalibration calib ;
    CameraCalibration::Data cdata ;

    if ( compute_markers ) {
        auto files = Path::entries(data_folder, DirectoryFilters::Glob("*.png"), false) ;

        AprilTagDetector adet(AprilTag36h11, 4) ; // we need decimation othwerwise detection does not work

        AprilTagGridPattern agrid(pattern_grid, pattern_width, pattern_gap, adet) ;

        calib.detect(files, agrid, cdata) ;

        cdata.save(data_folder + "/calib.data") ;
    }
    else
        cdata.load(data_folder + "/calib.data") ;

    PinholeCamera cam ;

    vector<Matrix4d> extrinsics ;
    calib.run(cdata, cam, extrinsics, CV_CALIB_FIX_PRINCIPAL_POINT) ;
    if ( refine )
        calib.refine(cdata, cam, extrinsics, true) ;
    cam.write(output_file) ;
}
