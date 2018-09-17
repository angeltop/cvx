#include <certh_core/Point2D.h>
#include <certh_core/Gnuplot.h>
#include <certh_core/Application.h>
#include <certh_core/RandomForest.h>

#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "test_rf.h" 

using namespace certh_core ;
using namespace std ;

class TestDataSet: public DataSetMatd {

public:
    TestDataSet(const vector<double> &rangeMin, const vector<double> &rangeMax, int samplesX, int samplesY) {

        double stepX = (rangeMax[0] - rangeMin[0])/samplesX ;
        double stepY = (rangeMax[1] - rangeMin[1])/samplesY ;

        vector<Point2d> vals ;

        for( double x = rangeMin[0] ; x <= rangeMax[0] ; x += stepX )
            for( double y = rangeMin[1] ; y <= rangeMax[1] ; y += stepY )
            {
                vals.push_back(Point2d(x, y)) ;
            }

        data_.resize(vals.size(), 2) ;

        for(int i=0 ; i<vals.size() ; i++ )
           data_.row(i) = vals[i] ;

        data_labels_.resize(vals.size());
    }

    void addLabel(uint32_t label)
    {
        data_labels_.push_back(label) ;
    }
} ;

typedef RandomForestClassifier<Linear2DLearner<MatStorageTraits<double> >, MatStorageTraits<double> > Classifier ;

int main(int argc, char *argv[])
{
    Application app("test_rf", argc, argv) ;

    Classifier::TrainingParameters tparams ;
    tparams.max_depth = 5 ;
    tparams.gain_threshold = 0.01 ;
    tparams.num_random_samples = 50 ;
    tparams.num_trees = 50 ;
    tparams.num_thresholds_per_sample = 20 ;
    tparams.num_samples_per_tree = 1000 ;
    tparams.min_pts_per_leaf = 5 ;

    TrainingDataSet ds(Application::dataFolder() / "rf/exp5_n2.txt") ;

    vector<double> rangeMin, rangeMax ;
    ds.getRange(rangeMin, rangeMax) ;

    Classifier cls ;
    cls.train(ds, tparams) ;

    {
        ofstream strm("/tmp/oo.bin", ios::binary) ;
        BinaryStream archive(strm) ;
        cls.write(archive) ;
    }

    Classifier cls2 ;

    {
        ifstream strm("/tmp/oo.bin", ios::binary) ;
        BinaryStream archive(strm) ;
        cls2.read(archive) ;
    }

    TestDataSet ts(rangeMin, rangeMax, 100, 100) ;

    DataSetSubset<MatStorageTraits<double> > sub(ts) ;
    sub.random(1000) ;

    DataSetSubset<MatStorageTraits<double> > sub2(sub) ;
    sub2.random(100) ;

    cls2.classify(ts) ;

    certh_core::gfx::Gnuplot plot("/tmp/oo.png") ;

    plot << "set palette defined (0 'blue', 1 'red')" << endl  ;
    plot << "unset colorbox" << endl;

    Eigen::MatrixXd m = ts.getMat() ;
    vector<uint32_t> labels ;
    ts.getSampleLabels(labels);

    plot << "plot " << plot.file(m, labels) << "using 2:3:1 with points palette notitle" << endl ;

    cout << "ok here" ;
}
