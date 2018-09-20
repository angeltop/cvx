#include <cvx/util/misc/dir_iterator.hpp>
#include <cvx/util/misc/strings.hpp>
#include <cvx/util/misc/regex.hpp>


#include <sys/types.h>
#include <dirent.h>
#include <string.h>

#include <cassert>


#define PATH_MAX 4096

using namespace std ;

namespace cvx {

class DirectoryIteratorImpl {
public:
    DirectoryIteratorImpl(const std::string &dir, std::function<bool (const DirectoryEntry &)> filter): handle_(NULL), dir_(dir), filter_(filter) {
        first(dir) ;
    }

    bool first(const std::string &dir) {
        if ( ( handle_ = ::opendir(dir.c_str()))== 0 ) return false ;
        return next() ;
    }

    bool next() {
        while ( 1 ) {
            dp_ = ::readdir(handle_) ;
            if ( dp_ == NULL ) break ;
            if ( dp_->d_type == DT_DIR && ( !strcmp(dp_->d_name, ".") || !strcmp(dp_->d_name, "..") ) ) continue ;
            current_.path_ = dp_->d_name ;
            if ( dp_->d_type == DT_UNKNOWN )
                current_.type_ = DirectoryEntry::Unknown ;
            else {
                if ( dp_->d_type == DT_DIR)
                  current_.type_ = DirectoryEntry::Dir ;
                else if ( dp_->d_type == DT_REG)
                    current_.type_ = DirectoryEntry::File ;
                else if ( dp_->d_type == DT_LNK )
                    current_.type_ = DirectoryEntry::SymLink ;
            }
            if ( !filter_(current_) ) continue ;

            break ;
        }
        return ( dp_ != NULL ) ;
    }

    bool isValid() { return handle_ != NULL && dp_ != NULL; }

    ~DirectoryIteratorImpl() {
        if ( handle_)
            ::closedir(handle_) ;
    }

    const DirectoryEntry &current() const { return current_ ; }

private:

    friend class DirectoryIterator ;

    DIR *handle_ ;
    struct dirent *dp_ ;
    string dir_ ;
    DirectoryEntry current_ ;
    std::function<bool (const DirectoryEntry &)> filter_ ;
};


DirectoryIterator::DirectoryIterator(const string &dir, std::function<bool (const DirectoryEntry &)> filter):
    impl_(new DirectoryIteratorImpl(dir, filter)) {}

DirectoryIterator::DirectoryIterator() {
}

const DirectoryEntry &DirectoryIterator::operator*() const
{
    assert(impl_) ;
    return impl_->current() ;
}

const DirectoryEntry *DirectoryIterator::operator->() const
{
    assert(impl_) ;
    return &impl_->current() ;
}

DirectoryIterator &DirectoryIterator::operator++()
{
    impl_->next() ;
    return *this ;
}

bool operator==(DirectoryIterator a, DirectoryIterator b)
{
    return a.impl_ == b.impl_
              || (!a.impl_ && b.impl_ && !b.impl_->isValid())
              || (!b.impl_ && a.impl_ && !a.impl_->isValid());
}


}
