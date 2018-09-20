#include <cvx/util/misc/path.hpp>
#include <cvx/util/misc/strings.hpp>
#include <cvx/util/misc/regex.hpp>

#include <cassert>
#include <memory>
#include <random>

using namespace std ;

namespace cvx { namespace util {

Path::Path(const std::string &filepath) {
    parse(filepath) ;
}

string Path::extension() const
{
    string n = name() ;
    if ( n.empty() ) return n ;

    size_t pos = n.find_first_of('.') ;
    return ( pos == string::npos ) ? string() : n.substr(pos+1) ;
}

string Path::toString() const {
    if ( elements_.empty() ) return root_ ;
    return toString(elements_.cbegin(), elements_.cend()) ;
}

Path Path::absolutePath(const Path &base) const {
    assert(base.isAbsolute()) ;

    if ( isAbsolute() ) return *this ;
    Path res(base) ;
    res.append(*this) ;
    return res ;
}

extern std::string resolve_sym_link(const std::string &path) ;

Path Path::canonicalPath(const Path &base) const
{
    Path source(absolutePath(base)) ;
    Path res ;

    bool rescan  ;

    do  {
        rescan = false ;
        res.root_ = source.root_ ;
        res.elements_.clear() ;

        auto end = source.elements_.cend() ;
        for ( auto it = source.elements_.cbegin() ; it != end ; ++it ) {
            if (*it == ".") continue;
            if (*it == ".." ) {
                if ( !res.elements_.empty() ) {
                    res.elements_.pop_back();
                    continue;
                }
            }

            res.elements_.emplace_back(*it) ;

            std::string symlink = resolve_sym_link(res.toString()) ;

            if ( !symlink.empty() ) {
                // if this is a symlink we have to rebuild the source string and restart scanning to properly deal with "." abd ".."

                res.elements_.pop_back();

                Path link(symlink) ;

                if ( link.isAbsolute() ) {
                    Path tmp(link) ;
                    for( ++it ; it != end ; ++it )
                        tmp.elements_.emplace_back(*it) ;
                    source = tmp ;
                }
                else {
                    Path tmp(res) ;
                    tmp.append(link) ;
                    for( ++it ; it != end ; ++it )
                         tmp.elements_.emplace_back(*it) ;
                    source = tmp ;
                }
                rescan = true ;
                break ;

            }
        }
    } while ( rescan ) ;

    return res ;
}

string Path::parent() const {
    if ( elements_.empty() ) return root_ ;
    auto it = elements_.cbegin(), end = std::prev(elements_.cend()) ;
    return toString(it, end) ;
}

Path Path::parentPath() const {
    Path res ;
    res.root_ = root_ ;

    if ( elements_.empty() ) return res ;

    auto it = elements_.cbegin(), end = std::prev(elements_.cend()) ;

    if ( it == end ) return res ;

    res.elements_.assign(it, end) ;

    return res ;
}

Path::Path(const Path &parent, const string &child): Path(parent) {
    append(child) ;
}

Path::Path(const std::string &parent, const string &child): Path(parent) {
    append(child) ;
}


void Path::append(const Path &child_path) {
    std::copy(child_path.elements().begin(), child_path.elements().end(), std::back_inserter(elements_)) ;
}


bool Path::mkdirs() const
{
    string dir_path = root_ ;

    for( auto  it = elements_.begin() ; it != elements_.end() ; ++it ) {
        if ( it != elements_.begin() ) dir_path += path_separator_ ;
        dir_path += *it ;

        if ( exists(dir_path) ) continue ;

        bool res = mkdir(dir_path) ;

        if ( !res ) return false ;
    }

    return true ;
}

bool Path::mkdirs(const string &path) {
    return Path(path).mkdirs() ;
}

vector<string> Path::entries(const string &dir, DirectoryFilter filter, bool relative)
{
    vector<string> entries ;
    DirectoryIterator dit(dir, filter), dend ;
    for( ; dit != dend ; ++dit ) {
        if ( relative )
            entries.emplace_back(dit->path()) ;
        else
            entries.emplace_back(Path(dir, dit->path()).toString()) ;
    }
    return entries ;
}

string Path::toString(PathElements::const_iterator start, PathElements::const_iterator end) const {

    string res = root_ ;
    for( auto it = start ; it != end ; ++it ) {
        if ( it != start ) res += path_separator_ ;
        res += *it ;
    }
    return res ;

}

std::string rand_string(uint sz) {
    //1) create a non-deterministic random number generator
    static char rchars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
    std::default_random_engine rng(std::random_device{}());
    std::uniform_int_distribution<> dist(0, sizeof(rchars)-1);

    std::string str(sz, 0);
    std::generate_n( str.begin(), sz, [&] () { return rchars[ dist(rng) ] ; } );
    return str ;
}

Path Path::uniquePath(const Path &dir, const std::string &prefix, const std::string &suffix) {

    while (1) {
        Path res(dir) ;
        std::string filename(prefix + rand_string(8) + suffix) ;
        res /= filename ;
        if ( !res.exists() ) return res ;
    }
}


Path Path::tempFilePath(const std::string &prefix, const std::string &suffix) {
    return uniquePath(tempPath(), prefix, suffix) ;
}

}}

