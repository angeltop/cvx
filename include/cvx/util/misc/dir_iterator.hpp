#ifndef __CVX_UTIL_DIR_ITERATOR_HPP__
#define __CVX_UTIL_DIR_ITERATOR_HPP__

#include <string>
#include <functional>
#include <memory>

namespace cvx {

class DirectoryIteratorImpl ;
class Path ;

// abstract directory entry and associated path (it is always relative to container)

class DirectoryEntry {
public:
    enum Type { Dir, File, SymLink, Unknown } ;

    Type type() const { return type_ ; }
    std::string path() const { return path_ ; }
    bool isDirectory() const { return type_ == Dir ; }

    operator std::string () const { return path_ ; }

private:

    friend class DirectoryIteratorImpl ;

    DirectoryEntry(): type_(Unknown) {}

    std::string path_ ;
    Type type_ ;
};

// custom filter function for directory iterators ( return true to accept a file and false to skip it)
typedef std::function<bool (const DirectoryEntry &)> DirectoryFilter ;

class DirectoryFilters {
public:
    // filter matching all entries
    static const DirectoryFilter MatchAll  ;
    static const DirectoryFilter MatchDirectories ;
    static const DirectoryFilter MatchFilesWithGlobPattern(const std::string &glob_pattern) ;
};

// iterator class
class DirectoryIterator: public std::iterator<std::forward_iterator_tag, DirectoryEntry> {
   public:

    DirectoryIterator(const std::string &dir, DirectoryFilter filter = DirectoryFilters::MatchAll ) ;
    DirectoryIterator() ;

    const DirectoryEntry& operator*() const ;
    const DirectoryEntry* operator->() const;

    DirectoryIterator& operator++();

    friend bool operator==(DirectoryIterator a, DirectoryIterator b);
    friend bool operator!=(DirectoryIterator a, DirectoryIterator b) {
        return !(a == b) ;
    }

private:

    std::shared_ptr<DirectoryIteratorImpl> impl_ ;

};

// iterator range class
class DirectoryListing {
public:

    DirectoryListing(const std::string &dir,
                     DirectoryFilter filter = DirectoryFilters::MatchAll):
        dir_(dir), filter_(filter) {}

    DirectoryListing(const Path &dir,
                     DirectoryFilter filter = DirectoryFilters::MatchAll) ;


    DirectoryIterator begin() { return DirectoryIterator(dir_, filter_) ; }
    DirectoryIterator end() { return DirectoryIterator() ; }

private:
    std::string dir_ ;
    DirectoryFilter filter_ ;
};


}

#endif
