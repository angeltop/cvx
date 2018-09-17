#ifndef __DICTIONARY_HPP__
#define __DICTIONARY_HPP__

#include <map>
#include <cvx/util/misc/regex.hpp>

namespace cvx {
namespace util {

// A class of key/value pairs of strings.

class Dictionary
{
    public:

    Dictionary() ;
    ~Dictionary() {}

    // add a key/val pair
    void add(const std::string &key, const std::string &val) ;
    // remove entry with given key if exists
    void remove(const std::string &key) ;
    void removeSome(const regex &rx) ;

    // remove all items
    void clear() ;

    // get a the value of the given key if exists. Otherwise return defaultValue

    std::string get(const std::string &key, const std::string &defaultVal = std::string()) const ;

    template <typename T>
    static T default_value() { return (T)0; }

    template<class T>
    T value(const std::string &key, const T &defaultVal = default_value<T>()) const {

        if ( !contains(key) ) return defaultVal ;
        std::string val = get(key) ;
        std::istringstream strm(val) ;
        T res ;
        strm >> res ;
        if ( strm.fail() ) return defaultVal ;
        else return res ;
    }

    template<class T>
    void put(const std::string &key, const T &val) {
        std::ostringstream strm ;
        strm << val ;
        std::string res = strm.str() ;
        add(key, res) ;
    }

    std::string operator[] ( const std::string & key ) const ;
    std::string &operator[] ( const std::string & key ) ;

    // check the existance of a key

    bool contains(const std::string &key) const;

    // get a list of the keys in the dictionary

    std::vector<std::string> keys() const ;
    std::vector<std::string> keys(const regex &rx) const ;

    // get values

    std::vector<std::string> values() const ;
    std::vector<std::string> values(const regex &key) const ;

    // number of entries

    int count() const ;
    int count(const regex &kx) const ;
    int count(const std::string &) const ;

    uint64_t capacity() const ;

    bool empty() const ;

    void dump() const ;

    std::string serialize(const char *sep) ;

public:

    typedef std::map<std::string, std::string> ContainerType ;

    // stl style iterators

    typedef typename ContainerType::iterator iterator;
    typedef typename ContainerType::const_iterator const_iterator;

    iterator begin() { return container_.begin(); }
    const_iterator begin() const { return container_.begin(); }
    iterator end() { return container_.end(); }
    const_iterator end() const { return container_.end(); }

private:

    friend class DictionaryIterator ;

    ContainerType container_ ;

} ;

// read-only iterator

class DictionaryIterator
{
    public:

    DictionaryIterator(const Dictionary &dic): dict_(dic), it_(dic.container_.begin()) {}
    DictionaryIterator(const DictionaryIterator &other): dict_(other.dict_), it_(other.it_) {}

    bool operator == (const DictionaryIterator &other) const { return it_ == other.it_ ; }
    bool operator != (const DictionaryIterator &other) const { return it_ != other.it_ ; }

    operator int () const { return it_ != dict_.container_.end() ; }

    DictionaryIterator & operator++() { ++it_ ; return *this ; }
    DictionaryIterator operator++(int) { DictionaryIterator tmp(*this) ; ++it_; return tmp ; }

    std::string key() const { return (*it_).first ; }
    std::string value() const { return (*it_).second ; }

    private:

    const Dictionary &dict_ ;
    Dictionary::ContainerType::const_iterator it_ ;
} ;

}
}

#endif
