#include <cvx/util/misc/path.hpp>

#include <string>
#include <sstream>
#include <random>
#include <functional>
#include <algorithm>


#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>

using namespace std ;

namespace cvx { namespace util {

void Path::parse(const std::string &path) {

    std::string::const_iterator it  = path.begin();
    std::string::const_iterator end = path.end();

    if ( it == end ) return ; // empty path

    if ( *it == '/' ) {
        root_ = "/" ;
        ++it ;
    }

    std::string item ;

    for( ; it != end ; ++it ) {
        if ( *it == '/' ) {
            if ( !item.empty() ) {
                elements_.emplace_back(item) ;
                item.clear() ;
            }
        }
        else item += *it ;
    }

    if ( !item.empty() )
        elements_.emplace_back(item) ;
}

const char Path::path_separator_ = '/' ;

std::string resolve_sym_link(const std::string &path) {
    struct stat buffer ;
    if ( ::lstat(path.c_str(), &buffer) == 0 && S_ISLNK(buffer.st_mode) ) {
        size_t ls, bs = ( buffer.st_size == 0 ) ? PATH_MAX : buffer.st_size + 1;

        std::unique_ptr<char[]> link_name(new char [bs]);

        if ( ls = ::readlink(path.c_str(), link_name.get(), bs)  )
            return string(link_name.get(), ls) ;
    }
    return string() ;
}


Path Path::executablePath() {
    stringstream fpaths ;
    fpaths << "/proc/" << getpid() << "/exe" ;
    Path fpath(fpaths.str()) ;

    if ( fpath.exists() ) return fpath.canonicalPath().parentPath() ;
    else return Path() ;
}


Path Path::currentWorkingDir() {
    char temp[1024];
    return ( ::getcwd(temp, 1024) ? std::string( temp ) : std::string() );
}


Path Path::homePath() {
    const char *home_dir = getenv("HOME") ;

    if ( home_dir ) return Path(home_dir) ;
    else return Path() ;
}

Path Path::tempPath() {
    return Path("/tmp") ;
}

Path Path::nativeConfigDir(const std::string &app_name) {
    return  homePath() / ".config" / app_name  ;
}

Path Path::nativeDataDir() {
    return homePath() / ".local/share/";
}


bool Path::exists(const string &path) {
    struct stat buffer;
    return ::stat(path.c_str(), &buffer) == 0 ;
}

bool Path::isDirectory(const string &path) {
    struct stat buffer;
    return ( ::stat(path.c_str(), &buffer) == 0 ) && S_ISDIR(buffer.st_mode)  ;
}

bool Path::isFile(const string &path) {
    struct stat buffer;
    return ( ::stat(path.c_str(), &buffer) == 0 ) && S_ISREG(buffer.st_mode)  ;
}

bool Path::mkdir(const string &path) {
    return ::mkdir(path.c_str(), S_IRWXU|S_IRWXG|S_IRWXO) == 0 ;
}

bool Path::remove(const string &path) {
    if ( !exists(path) ) return false ;
    if ( isDirectory(path) )
        return ::rmdir(path.c_str()) == 0 ;
    else
        return ::unlink(path.c_str()) == 0 ;
}

bool Path::rename(const string &old_path, const string &new_path) {
    return ::rename(old_path.c_str(), new_path.c_str())  == 0 ;
}


}} // namespace cvx

