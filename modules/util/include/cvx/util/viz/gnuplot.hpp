#ifndef __GNUPLOT_HPP__
#define __GNUPLOT_HPP__

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iterator>
#include <Eigen/Core>

namespace cvx { namespace util {

class FILEoutbuf ;

class GnuplotPipeWrapper: public std::ostream {

    friend class Gnuplot ;

    GnuplotPipeWrapper() {
        assert( init() ) ;
    }

    bool find_gnuplot_path() ;
    bool init() ;
    void close() ;

    int gnuplot_cmd_desc() ;
    int get_fileno() ;

    static std::string  gnuplot_path_,
                        gnuplot_cmd_ ,
                        gnuplot_args_ ;

    FILE *pipe_ ;
    FILEoutbuf *buf_ ;

};

/* An interface to gnuplot program that allows sending commands directly via a pipe and a stream like interface
 *
 * e.g.
 *
 * Gnuplot gp ;
 * gp << "plot sin(x)" << endl ;
 * gp << plot.file(X, Y) << "with lines" << endl ;
 *
 * For the class to work correctly the gnuplot executable should be in the system path. Otherwise you have to define
 * the environment variable GNUPLOT_CMD_PATH as the directory of the executable.
 *
 * The code is based on gnuplot-iostream (https://gitorious.org/gnuplot-iostream) and gnuplot-cpp (https://code.google.com/p/gnuplot-cpp/) projects.
 *
 */


class Gnuplot: public GnuplotPipeWrapper
{
public:

    // Default constructor to establish a gnuplot session
    // Some defaults are set.
    Gnuplot() ;

    // Initialized Gnuplot and sets output to the specified file.
    // The appropriate terminal is set based on file extension (pdf, eps, png) and using the defaults.
    // Alternatinatively you may customize it by using the set terminal command

    Gnuplot(const std::string &outFile) ;
    ~Gnuplot() ;

    // plot via a temporary file

    class FileWritter ;

    template<typename T>
    std::string file(const std::vector<T>& x) ;

    template<typename T>
    std::string file(const std::vector<T>& x, const std::vector<T>& y) ;

    template<typename T, typename K>
    std::string file(const std::vector<T>& x, const std::vector<T>& y, const std::vector<K> &labels) ;

    template<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
    std::string file(const Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>& x) ;

    template<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime, typename K>
    std::string file(const Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>& x, const std::vector<K> &labels) ;

    // version of 2d plot with x, y column or row vectors
    template<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
    std::string file(const Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>& x,
                     const Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>& y) ;

    // version of 2d plot with x, y column or row vectors
    template<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime, typename T>
    std::string file(const Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>& x,
                     const Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>& y,
                     const std::vector<T> &labels) ;


    // repeat the last file
    std::string file() ;

private:

    struct TmpFileWrapper: public std::ofstream {

        TmpFileWrapper(Gnuplot &gp): fname_(gp.make_tmp_file()) {
            open(fname_.c_str()) ;
        }

        std::string fname_ ;
    };

    class FileWriter: public TmpFileWrapper
    {
    public:
        FileWriter(Gnuplot &gp): TmpFileWrapper(gp) {}

        std::string quotedName() {

            flush() ;
            close() ;
           std::ostringstream cmdline;
            cmdline << " '" << fname_ << "' ";
            return cmdline.str();
        }
    };

    std::string make_tmp_file() ;
    std::vector<std::string> tmp_files_ ;

};

template<typename T>
std::string Gnuplot::file(const std::vector<T>& x)
{
    FileWriter strm(*this) ;

    std::ostream_iterator<T> it( strm, "\n" );

    std::copy(x.begin(), x.end(), it) ;

    return strm.quotedName() ;
}

template<typename T>
std::string Gnuplot::file(const std::vector<T>& x, const std::vector<T>& y)
{
    FileWriter strm(*this) ;

    assert(x.size() == y.size()) ;

    for(int i=0 ; i<x.size() ; i++)
        strm << x[i] << ' ' << y[i] << std::endl ;

    return strm.quotedName() ;
}

template<typename T, typename K>
std::string Gnuplot::file(const std::vector<T>& x, const std::vector<T>& y, const std::vector<K> &labels)
{
    FileWriter strm(*this) ;

    assert(x.size() == y.size()) ;
    assert(x.size() == labels.size()) ;

    for(int i=0 ; i<x.size() ; i++)
        strm << labels[i] << ' ' << x[i] << ' ' << y[i] << std::endl ;

    return strm.quotedName() ;
}


template<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
std::string Gnuplot::file(const Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>& x)
{
    FileWriter strm(*this) ;

    for(int i=0 ; i<x.rows() ; i++ )
    {
        for(int j=0 ; j<x.cols() ; j++ )
            strm << x(i, j) << ' ';
        strm << std::endl ;
    }

    return strm.quotedName() ;
}

template<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime, typename K>
std::string Gnuplot::file(const Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>& x, const std::vector<K> &labels)
{
    FileWriter strm(*this) ;

    for(int i=0 ; i<x.rows() ; i++ )
    {
        strm << labels[i] << ' ' ;
        for(int j=0 ; j<x.cols() ; j++ )
            strm << x(i, j) << ' ';
        strm << std::endl ;
    }

    return strm.quotedName() ;
}

// version of 2d plot with x, y column or row vectors
template<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
std::string Gnuplot::file(const Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>& x,
                 const Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>& y)
{
    FileWriter strm(*this) ;

    unsigned nx = std::max(x.rows(), x.cols()) ;
    unsigned ny = std::max(y.rows(), y.cols()) ;

    assert( std::min(x.rows(), x.cols()) == 1 ) ;
    assert( std::min(y.rows(), y.cols()) == 1 ) ;
    assert ( nx == ny ) ;

    for(int i=0 ; i<nx ; i++ )
        strm << x[i] << ' ' << y[i] << std::endl ;

    return strm.quotedName() ;
}

// version of 2d plot with x, y column or row vectors
template<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime, typename T>
std::string Gnuplot::file(const Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>& x,
                 const Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>& y,
                 const std::vector<T> &labels)
{
    FileWriter strm(*this) ;

    unsigned nx = std::max(x.rows(), x.cols()) ;
    unsigned ny = std::max(y.rows(), y.cols()) ;

    assert( std::min(x.rows(), x.cols()) == 1 ) ;
    assert( std::min(y.rows(), y.cols()) == 1 ) ;
    assert ( nx == ny ) ;
    assert( labels.size() == nx ) ;

    for(int i=0 ; i<nx ; i++ )
        strm << labels[i] << ' ' << x[i] << ' ' << y[i] << std::endl ;

    return strm.quotedName() ;
}


// repeat the last file
inline std::string Gnuplot::file()
{
    assert( !tmp_files_.empty() ) ;

    std::ostringstream cmdline;
    cmdline << " '" << tmp_files_.back() << "' ";
    return cmdline.str();
}

} // namespace util
} // namespace cvx

#endif
