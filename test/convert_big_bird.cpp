#include <core/MFunctions.h>
#include <core/Application.h>

#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <H5Cpp.h>

#include <core/Calibration.h>
#include <boost/lexical_cast.hpp>

using namespace std ;
using namespace vl ;



cv::Mat readHDF5(const std::string &fileName, const std::string &variable)
{
    using namespace H5;

    try
    {
        Exception::dontPrint();

        H5File file( fileName, H5F_ACC_RDONLY );
        DataSet dataset = file.openDataSet( variable );
        DataSpace dataspace = dataset.getSpace();

        hsize_t dims_out[2];
        int ndims = dataspace.getSimpleExtentDims( dims_out, NULL);
        assert( ndims == 2 ) ;
        const hsize_t rows = dims_out[0], cols = dims_out[1];
        std::vector<unsigned short> data(rows * cols);
        dataset.read(data.data(), H5::PredType::NATIVE_UINT16) ;

        cv::Mat m(rows, cols, CV_16UC1, data.data(), cols*2) ;

        return m.clone() ;

    } // end of try block
    // catch failure caused by the H5File operations
    catch( Exception error )
    {
        error.printError();
        return cv::Mat();
    }

}

void convertDepth(const std::string &datadir)
{
    using namespace boost::filesystem ;

    directory_iterator it(datadir), end ;

    while ( it != end )
    {
        path p = it->path() ;
        cout << p << endl ;


        if ( boost::regex_match(p.filename().string(), boost::regex("NP[1-5]_[0-9]+\\.h5")) )
        {

            cv::Mat d = readHDF5(p.string(), "depth") ;

            path op = p.parent_path() / (p.stem().string() + ".png") ;

            if ( d.data )
               cv::imwrite(op.string(), d) ;
        }

        ++it ;
    }
}


int main(int argc, char *argv[])
{
    cv::VideoCapture cap(0); // open the default camera
//    if(!cap.isOpened())  // check if we succeeded
//          return -1;

 //   AprilTagGridPattern::makePattern36H11("/home/malasiot/source/vl/3rdparty/apriltag/tag36h11/",
 //                                         "/tmp/pattern.svg", cv::Size(7, 10), 0.04, 0.01) ;

    cv::namedWindow("result",1);

    AprilTagDetector pat ;
    AprilTagGridPattern gpat(cv::Size(7, 10), 0.04, 0.01, pat) ;

    for(;;)
    {
          cv::Mat frame;
          cap >> frame; // get a new frame from camera

          vector<cv::Point2f> pts ;
          vector<cv::Point3f> objs ;

          if ( gpat.findPoints(frame, pts, objs) )
              gpat.draw(frame, pts, objs) ;

          cv::imshow("result", frame);
          if( cv::waitKey(30) >= 0) break;
      }
      // the camera will be deinitialized automatically in VideoCapture destructor
      return 0;

}
