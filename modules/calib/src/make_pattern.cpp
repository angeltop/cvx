#include <cvx/calib/pattern.hpp>

using namespace cvx::camera ;
using namespace std ;

int main(int argc, char *argv[])
{
    string pattern_dir, out_path ;
    cv::Size grid_sz ;
    float tile_width = 0.04, tile_gap = 0.01;

    for(uint c=1 ; c<argc ; c++) {
        string arg = argv[c] ;

        if ( arg == "--patterns" ) {
            if ( ++c < argc ) pattern_dir = argv[c] ;
        }
        else if ( arg == "--out" ) {
            if ( ++c < argc ) out_path = argv[c] ;
        }
        else if ( arg == "--grid" ) {
            uint gx = 0, gy = 0 ;
            if ( ++c < argc ) gx = stoi(argv[c]) ;
            if ( ++c < argc ) gy = stoi(argv[c]) ;
            grid_sz = cv::Size(gx, gy)     ;
        }
        else if ( arg == "--width" ) {
            if ( ++c < argc ) tile_width = stof(argv[c]) ;
        }
        else if ( arg == "--gap" ) {
            if ( ++c < argc ) tile_gap = stof(argv[c]) ;
        }
    }

    if ( pattern_dir.empty() || out_path.empty() || grid_sz.width == 0 || grid_sz.height == 0 ) {
        cout << "Usage: make_pattern --patterns <april tag images path> --out <output path for svg image> --grid <tx> <ty> [--width <tile_width>] [--gap <tile_gap>]" << endl ;
        return 0 ;
    }

    AprilTagGridPattern::makePattern36H11(pattern_dir, out_path, grid_sz, tile_width, tile_gap) ;

}
