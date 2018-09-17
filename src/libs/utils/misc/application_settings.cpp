#include <cvx/util/misc/application_settings.hpp>
#include <cvx/util/misc/strings.hpp>
#include <cvx/util/misc/xml_pull_parser.hpp>

#include <cassert>
#include <fstream>

using namespace std ;

namespace cvx {

ApplicationSettings::ApplicationSettings(): root_(new Node)
{
}

static const int xml_indent_step = 4 ;

static void writeIndent(ostream &strm, int a) {
    while ( a-- > 0 ) strm << ' ' ;
}

static void writeEscapedXml(ostream &strm, const std::string &src) {
    for( char c: src ) {
        switch(c) {
        case '&':  strm << "&amp;";      break;
        case '\"': strm << "&quot;";     break;
        case '\'': strm << "&apos;";     break;
        case '<':  strm << "&lt;";       break;
        case '>':  strm << "&gt;";       break;
        default:   strm << c;            break;
        }
    }

}

void ApplicationSettings::writeXml(ostream &strm, Node::Ptr node, int indent) {

    for( auto &&p: node->children_ ) {
        writeIndent(strm, indent) ;
        strm << "<" << p.first  ;
        for( auto &&a: node->attributes_ ) {
            strm << ' ' << a.first << "=\"" ;
            writeEscapedXml(strm, a.second) ;
            strm << "\"" ;
        }
        strm << ">\n" ;
        writeXml(strm, p.second, indent + xml_indent_step) ;
        writeIndent(strm, indent) ;
        strm << "</" << p.first  << ">\n" ;
    }
    if ( !node->value_.empty() ) {
        writeIndent(strm, indent) ;
        writeEscapedXml(strm, node->value_);
        strm << endl ;
    }
}

bool ApplicationSettings::save(const std::string &fileName)
{
    ofstream strm(fileName) ;

    strm << R"(<?xml version="1.0" encoding="UTF-8" standalone="no" ?>)" << endl ;
    strm << "<config>" << endl ;
    writeXml(strm, root_, xml_indent_step) ;

    strm << "</config>" ;

    return (bool)strm ;
}
#if 0
class AppSettingsXmlParser: public XMLSAXParser {
public:
    AppSettingsXmlParser(istream &strm, ApplicationSettings &sts): XMLSAXParser(strm), sts_(sts), current_(sts_.root_) {
        node_stack_.emplace_back(current_) ;
    }
private:
    virtual void startElement(const string &qname, const AttributeList &attrs) {
        if ( qname == "config" ) return ;

        ApplicationSettings::Node::Ptr node(new ApplicationSettings::Node) ;

        node->attributes_ = ApplicationSettings::Attributes(attrs.begin(), attrs.end()) ;

        current_->children_[qname] = node ;
        node_stack_.emplace_back(node) ;
        current_ = node ;
    }

    virtual void endElement(const string &qname) {
        if ( qname == "config" ) return ;

        node_stack_.pop_back() ;
        current_ = node_stack_.back() ;
    }

    virtual void characters(const string &text) {
        current_->value_ = trimCopy(text) ;
    }

    ApplicationSettings &sts_ ;
    ApplicationSettings::Node::Ptr current_ ;
    std::deque<ApplicationSettings::Node::Ptr> node_stack_ ;
};
#endif

bool ApplicationSettings::load(const std::string &fileName) {

    ifstream strm(fileName) ;

    XmlPullParser parser(strm) ;

    Node::Ptr current_ = root_ ;
    std::deque<Node::Ptr> node_stack_ ;

    node_stack_.emplace_back(current_) ;

    try {
        while ( parser.next() != XmlPullParser::END_DOCUMENT ) {
            auto et = parser.getEventType() ;
            string tagName = parser.getName() ;

            if ( et == XmlPullParser::START_TAG && tagName != "config" ) {

                Node::Ptr node(new ApplicationSettings::Node) ;

                node->attributes_ = parser.getAttributes() ;

                current_->children_[tagName] = node ;
                node_stack_.emplace_back(node) ;
                current_ = node ;

            } else if ( et == XmlPullParser::END_TAG && tagName != "config" ) {
                node_stack_.pop_back() ;
                current_ = node_stack_.back() ;
            } else if ( et == XmlPullParser::TEXT ) {
                current_->value_ = trimCopy(parser.getText()) ;
            }
        }
        return true ;
    } catch ( XmlPullParserException & ) {
        return false ;
    }
}


string ApplicationSettings::make_prefix() const
{
    if ( section_stack_.empty() ) return string() ;
    else {
        string p ;
        for(int i=0 ; i<section_stack_.size() ; i++ ) {
            p += section_stack_[i] ;
            p += '.' ;
        }

        return p ;
    }
}


std::vector<std::string> ApplicationSettings::keys(const std::string &prefix) const
{
    vector<string> res ;

    Node::Ptr n = findNode(make_prefix() + prefix) ;
    if ( !n ) return res ;

    for( auto &&c: n->children_ ) {
        if ( c.second->children_.empty() )
            res.emplace_back(c.first) ;
    }

    return res ;
}

std::vector<std::string> ApplicationSettings::sections(const std::string &prefix) const
{
    vector<string> res ;

    Node::Ptr n = findNode(make_prefix() + prefix) ;
    if ( !n ) return res ;

    for( auto &&c: n->children_ ) {
        if ( !c.second->children_.empty() )
            res.emplace_back(c.first) ;
    }

    return res ;
}

Dictionary ApplicationSettings::attributes(const std::string &key)
{
    Node::Ptr n = findNode(make_prefix() + key) ;
    if (n) return n->attributes_ ;
    else return {} ;
}

void ApplicationSettings::beginSection(const string &sec)
{
    section_stack_.push_back(sec) ;
}

void ApplicationSettings::endSection()
{
    assert(!section_stack_.empty()) ;
    section_stack_.pop_back() ;
}

static void split_key(const std::string &src, std::string &prefix, std::string &suffix) {
    size_t pos = src.find_last_of('.') ;
    if ( pos == string::npos ) {
        suffix = src ;
    }
    else {
        prefix = src.substr(0, pos) ;
        suffix = src.substr(pos+1) ;
    }
}

ApplicationSettings::Node::Ptr ApplicationSettings::makeNode(const string &key) {

    string prefix, suffix ;
    split_key(key, prefix, suffix) ;

    Node::Ptr parent ;
    if ( prefix.empty() ) parent = root_ ;
    else {
        parent = findNode(prefix) ;
        if ( !parent ) parent = makeNode(prefix) ;
    }

    Node::Ptr node(new Node) ;
    parent->children_[suffix] = node ;

    return node ;
}

ApplicationSettings::Node::Ptr ApplicationSettings::findNode(const string &key) const {

    vector<string> tokens = split(key, ".") ;

    Node::Ptr p = root_ ;

    for( const string &tok: tokens) {
        if ( p ) {
            auto it = p->children_.find(tok) ;
            if ( it != p->children_.end() )
                p = it->second ;
            else return nullptr ;
        }
    }

    return p ;
}


} // namespace cvx
