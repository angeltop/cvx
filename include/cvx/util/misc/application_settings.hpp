#ifndef __APPLICATION_SETTINGS_HPP__
#define __APPLICATION_SETTINGS_HPP__

#include <cvx/util/misc/strings.hpp>
#include <cvx/util/misc/dictionary.hpp>

namespace cvx {

// provides mechanism to serialize application settings in XML config file
class ApplicationSettings
{
public:
    ApplicationSettings() ;

    // load settings from file
     bool load(const std::string &fileName) ;
    // save settings to file
    bool save(const std::string &fileName) ;

    // get all keys corresponding to the given prefix
    std::vector<std::string> keys(const std::string &prefix = std::string()) const ;

    // get all sections corresponding to the given prefix
    std::vector<std::string> sections(const std::string &prefix = std::string()) const ;


    // set the value corresponding to a key with an optional attribute map (e.g. semantic information).
    // key is in the form <section1>.<section2>...<key>

    typedef Dictionary Attributes ;

    template <class T>
    void set(const std::string &key, const T &val, const Attributes &a = Attributes())  {
        Node::Ptr n = makeNode(make_prefix() + key) ;
        if ( n ) {
            n->serialize(val) ;
            n->attributes_ = a ;
        }
    }

    // helper for writing lists of values
    template <class T>
    void setArray(const std::string &key, const std::vector<T> &val, const Attributes &a = Attributes())  {
        Node::Ptr n = makeNode(make_prefix() + key) ;
        if ( n ) {
            n->attributes_ = a ;
            for( uint i=0 ; i<val.size() ; i++ ) {
                std::string item_key = key + ".item" + std::to_string(i) ;
                set(item_key, val[i]) ;
            }
        }
    }

    // get value of corresponding key or default value if not found
    template <class T>
    T get(const std::string &key, const T &def = T{}) const {
        Node::Ptr n = findNode(make_prefix() + key) ;
        if ( n )  return n->deserialize<T>() ;
        else return def;
    }

    // helper for reading an array of values
    template <class T>
    std::vector<T> getArray(const std::string &key) {
        Node::Ptr n = findNode(make_prefix() + key) ;
        std::vector<T> res ;
        if ( n ) {
            size_t nc = n->children_.size() ;
            res.resize(nc) ;
            for( auto &&p: n->children_ ) {
                if ( p.second->children_.empty() && startsWith(p.first, "item" ) ) {
                    int index = std::stoi(p.first.substr(4)) ;
                    if ( index >= 0 && index < nc)
                        res[index] = p.second->deserialize<T>() ;
                }
            }
        }

        return res ;
    }

    // get attributes of a key
    Dictionary attributes(const std::string &key) ;

    // starts a new section. All subsequent queries will be prefixed with the section(s) key(s).

    void beginSection(const std::string &sec) ;
    void endSection() ;

public:

    struct Node {
        typedef std::shared_ptr<Node> Ptr ;
        std::map<std::string, Node::Ptr> children_ ;
        std::string value_ ;
        Dictionary attributes_ ;

        // you may provide specialization for custom types

        template<class T> void serialize(const T &t) {
            std::ostringstream strm ;
            strm << t ;
            value_ = strm.str() ;
        }

        template<class T> T deserialize() const {
            std::istringstream strm(value_) ;
            T t ;
            strm >> t ;
            return t ;
        }
    } ;

private:

    std::string make_prefix() const;

    Node::Ptr findNode(const std::string &key) const;
    Node::Ptr makeNode(const std::string &key) ;
    void writeXml(std::ostream &strm, Node::Ptr node, int offset);

    Node::Ptr root_ ;

    std::deque<std::string> section_stack_ ;
};

// template specialization for boolean types
template<>
void ApplicationSettings::Node::serialize<bool>(const bool &b) {
    value_ = b ? "true" : "false" ;
}

template<>
bool ApplicationSettings::Node::deserialize<bool>() const {
    if ( value_ == "true" || value_ == "1" ) return true ;
    return false ;
}


}

#endif
