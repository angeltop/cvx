#include <certh_core/ShapeContext.h>
#include <certh_core/MFunctions.h>
#include <certh_core/Application.h>
#include <certh_core/Gnuplot.h>

#include <certh_core/gfx.h>

using namespace std ;
using namespace certh_core ;

int main(int argc, char *argv[])
{
    Application app("test_sm", argc, argv) ;

    boost::filesystem::path data = Application::dataFolder() / "shape/DataChui/" ;

    Eigen::MatrixXd a1, a2 ;
    certh_core::load(a1, (data / "save_chinese_def_1_1.mat").string().c_str(), "x1") ;
    certh_core::load(a2, (data / "save_chinese_def_1_1.mat").string().c_str(), "y2") ;


    gfx::Gnuplot gp("oo.png") ;

    gp << "plot " << gp.file(a1) << "with points," << gp.file(a2) << "with points" << endl;

    PointList2d p1(a1), p2(a2) ;

    vector<pair<int, int> > matches ;

    ShapeContextMatcher matcher ;
    matcher.match(p1, p2, matches) ;




    cout << "ok here" ;
}
