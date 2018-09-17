#include <cvx/util/viz/gnuplot.hpp>
#include <cvx/util/misc/path.hpp>
#include <cvx/util/misc/strings.hpp>

#include <stdio.h>
#include <stdlib.h>

using namespace std ;


namespace cvx { namespace util {


string GnuplotPipeWrapper::gnuplot_path_ ;

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)
string GnuplotPipeWrapper::gnuplot_args_ = "-persist 2> NUL" ;
string GnuplotPipeWrapper::gnuplot_cmd_ = "gnuplot.exe" ;
#elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
string GnuplotPipeWrapper::gnuplot_args_ = "-persist" ;
string GnuplotPipeWrapper::gnuplot_cmd_ = "gnuplot" ;
#endif

class FILEoutbuf : public std::streambuf {
  protected:
    FILE *file_;    // file descriptor
  public:
    // constructor
    FILEoutbuf (FILE *fd) : file_(fd) {}
  protected:

    // write one character
    virtual int_type overflow (int_type c) {
        if (c != EOF) {
            if (fputc(c, file_) != c) {
                return EOF;
            }
        }
        return c;
    }
    // write multiple characters
    virtual
    std::streamsize xsputn (const char* s,
                            std::streamsize num) {
        return fwrite(s, 1, num, file_);
    }
};


void GnuplotPipeWrapper::close()
{
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)
        _pclose(pipe_) ;
#elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
        pclose(pipe_) ;
#endif

   delete buf_ ;
}

bool GnuplotPipeWrapper::find_gnuplot_path()
{
    // chech if path has already been previously determined

    if ( !GnuplotPipeWrapper::gnuplot_path_.empty() &&
         Path::exists(GnuplotPipeWrapper::gnuplot_path_ + '/' + GnuplotPipeWrapper::gnuplot_cmd_) )
        return true ;

    // test if environment variable GNUPLOT_CMD_PATH is defined

    const char *epath ;
    if ( epath = getenv("GNUPLOT_CMD_PATH")  )
    {
        string p(epath) ;
        p += '/' + GnuplotPipeWrapper::gnuplot_cmd_ ;

        if ( Path::exists(p) )
        {
            GnuplotPipeWrapper::gnuplot_path_ = epath ;
            return true ;
        }
    }

    if ( epath = getenv("PATH") )
    {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)
        const char *path_separator = ";" ;
#elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
        const char *path_separator = ":" ;
#endif

        vector<string> tokens = split(epath, path_separator) ;

        for( const string &t: tokens) {

            if ( Path::exists(t + '/' + GnuplotPipeWrapper::gnuplot_cmd_) )
            {
                GnuplotPipeWrapper::gnuplot_path_ = t ;
                return true ;
            }

        }
    }

    return false ;
}


bool GnuplotPipeWrapper::init()
{
    if ( !find_gnuplot_path() ) return false ;

    // open pipe

    std::string cmd = GnuplotPipeWrapper::gnuplot_path_ + "/" + GnuplotPipeWrapper::gnuplot_cmd_ +
            ' ' + GnuplotPipeWrapper::gnuplot_args_ ;

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)
    pipe_ = _popen(cmd.c_str(),"w");
#elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
    pipe_ = popen(cmd.c_str(), "w") ;
#endif

    if ( !pipe_ ) return false ;

    buf_ = new FILEoutbuf(pipe_) ;
    rdbuf(buf_) ;

    return true ;

}


Gnuplot::Gnuplot(): GnuplotPipeWrapper()
{
    assert( pipe_ ) ;
}

Gnuplot::Gnuplot(const string &outFile): GnuplotPipeWrapper()
{
    Path epath(outFile) ;

    string ext = epath.extension() ;

    if ( ext == "eps" )
        *this << "set terminal epslatex" << endl ;
    else if ( ext == "pdf" )
        *this << "set terminal pdfcairo" << endl ;
    else if ( ext == "png" )
        *this << "set terminal pngcairo" << endl ;

    *this << "set output \"" << outFile << "\"" << endl ;
}


Gnuplot::~Gnuplot()
{
    *this << "set output" << endl ;
    *this << "set terminal pop" << endl ;

    GnuplotPipeWrapper::close() ;

    for(int i=0 ; i<tmp_files_.size() ; i++)
    {
        if ( Path::exists(tmp_files_[i]) )
            Path::remove(tmp_files_[i]) ;
    }

}

string Gnuplot::make_tmp_file()
{
    Path p = Path::uniquePath(Path::tempPath(), "tmp-gnuplot-") ;
    string tmp_file_path = p.toString() ;
    tmp_files_.push_back(tmp_file_path) ;

    return tmp_file_path ;
}


}
}
