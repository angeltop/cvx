#ifndef __PATH_HPP__
#define __PATH_HPP__

#include <vector>
#include <string>
#include <stdexcept>

#include <cvx/util/misc/dir_iterator.hpp>

namespace cvx { namespace util {

// Filesystem path

class Path {
	public:
		
    Path() = default ;
    Path(const Path &parent, const std::string &child) ;
    Path(const std::string &parent, const std::string &child) ;
    Path(const std::string &pathname) ;

    // filename or directory name
    std::string name() const {
        return elements_.empty() ? std::string() : elements_.back() ;
    }

    std::string extension() const ;

    // convert to string
    std::string toString() const ;

    // get absolute path
    Path absolutePath(const Path &base = Path::currentWorkingDir()) const ;
    std::string absolute(const Path &base = Path::currentWorkingDir()) const {
        return absolutePath(base).toString() ;
    }

    // makes a unique path by eliminating "." and ".." and resolving symbolic links
    Path canonicalPath(const Path &base = Path::currentWorkingDir()) const ;
    std::string canonical(const Path &base = Path::currentWorkingDir()) const {
        return canonicalPath(base).toString() ;
    }

    // check if it is an absolute path
    bool isAbsolute() const { return !root_.empty() ; }

    // check if file exists
    bool exists() const {
        return exists(toString()) ;
    }

    // check if it is a directory
    bool isDirectory() const {
        return isDirectory(toString()) ;
    }

    // check if it is a regular file
    bool isFile() const {
        return isDirectory(toString()) ;
    }

    // get parent directory if exists

    std::string parent() const ;
    Path parentPath() const ;

    DirectoryListing subDirs() const ;

    // append child path

    void append(const Path &child) ;

    Path & operator /= (const Path & rhs) {
        append(rhs) ;
        return *this ;
    }

    Path & operator /= (const char *rhs) {
        append(std::string(rhs)) ;
        return *this ;
    }

    Path operator / (const Path & rhs) const {
        Path tmp(*this) ;
        tmp.append(rhs) ;
        return tmp ;
    }

    Path operator / (const char *rhs) const {
        return *this / Path(rhs) ;
    }

    friend std::ostream &operator << ( std::ostream &strm, const Path &path ) {
        strm << path.toString() ;
        return strm ;
    }

    const std::string &root() const { return root_ ; }
    const std::vector<std::string> &elements() const {
        return elements_ ;
    }

    bool mkdir() const {
        return mkdir(toString()) ;
    }

    bool mkdirs() const ;

    static Path currentWorkingDir() ;
    static Path executablePath();
    static Path homePath();
    static Path tempPath();
    static Path nativeDataDir();
    static Path uniquePath(const Path &dir, const std::string &prefix, const std::string &suffix = "");
    static Path tempFilePath(const std::string &prefix = "tmp-", const std::string &suffix = "");
    static Path nativeConfigDir(const std::string &app_name);

    static bool exists(const std::string &path) ;
    static bool isDirectory(const std::string &path) ;
    static bool isFile(const std::string &path) ;
    static bool mkdir(const std::string &path) ;
    static bool mkdirs(const std::string &path) ;
    static bool remove(const std::string &path) ;
    static bool remove(const Path &path) {
        return remove(path.toString()) ;
    }

    static bool rename(const std::string &orig_path, const std::string &new_path) ;

    // get directory contents matching the filter
    static std::vector<std::string> entries(const std::string &dir, DirectoryFilter filter, bool relative = true) ;
    // all files in dir matching glob pattern
    static std::vector<std::string> glob(const std::string &dir, const std::string &pattern, bool relative = true) {
        return entries(dir, matchFilesWithGlobPattern(pattern), relative) ;
    }

private:
    typedef std::vector<std::string> PathElements ;

    void parse(const std::string &path) ; // can throw InvalidPathException
    std::string toString(PathElements::const_iterator start, PathElements::const_iterator end) const ;

private:

    std::string root_ ;
    PathElements elements_ ;

    static const char path_separator_ ;
} ;


class InvalidPathException: public std::runtime_error {
public:
    InvalidPathException(const std::string &path_name):
        std::runtime_error("Invalid filesystem path: " + path_name) {}
};

} // util
} // cvx

#endif
