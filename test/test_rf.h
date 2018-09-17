#ifndef __TEST_RFH__
#define __TEST_RFH__

#include <certh_core/Point2D.h>
#include <certh_core/DataSet.h>

#include <fstream>
#include <boost/filesystem.hpp>

using namespace certh_core ;


class TrainingDataSet: public DataSetMatd {

public:
    TrainingDataSet(const boost::filesystem::path &p) {
        std::ifstream strm(p.string().c_str()) ;

        int lidx ;
        double x, y ;

        std::vector<Point2d> vals ;

        while ( strm )
        {
            strm >> lidx >> x >> y ;

            vals.push_back(Point2d(x, y)) ;

            data_labels_.push_back(lidx - 1) ;
        }

        data_.resize(vals.size(), 2) ;

        for(int i=0 ; i<vals.size() ; i++ )
        {
            data_.row(i) = vals[i] ;
        }

        labels_.push_back("1") ;
        labels_.push_back("2") ;

    }

    void getRange(std::vector<double> &rangeMin, std::vector<double> &rangeMax)
    {
        rangeMin.resize(2) ;
        rangeMax.resize(2) ;

        for( int i=0 ; i<2 ; i++ )
        {
            coord_t minV = std::numeric_limits<double>::max() ;
            coord_t maxV = std::numeric_limits<double>::min() ;

            for(int j = 0 ; j<numSamples() ; j++ )
            {
                minV = std::min(minV, getSampleCoordinate(j, i)) ;
                maxV = std::max(maxV, getSampleCoordinate(j, i)) ;
            }

            rangeMin[i] = minV ;
            rangeMax[i] = maxV ;
        }
    }


};

#endif
