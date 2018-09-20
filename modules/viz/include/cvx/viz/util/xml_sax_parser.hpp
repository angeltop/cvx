#ifndef __XML_HPP__
#define __XML_HPP__

#include <istream>
#include <map>
#include <string>
#include <iostream>
#include <memory>

#include <vsim/util/dictionary.hpp>

// Very simple SAX like XML parser

namespace vsim { namespace util {

class XMLStreamWrapper ;

class XMLSAXParser {
public:
    enum ErrorCode { InvalidChar, NoClosingQuote, TagMismatch, TagInvalid, AttrValueInvalid, InvalidXml } ;

    XMLSAXParser(std::istream &strm) ;

    // parse input stream and return true if succesfull, errors are reported through an error callback

    bool parse() ;

    virtual void startElement(const std::string &qname, const Dictionary &attr_list) {
        std::cout << "start element: " << qname << std::endl ;
        for ( const auto &p: attr_list )
            std::cout << "key: " << p.first << ", value: " << p.second << std::endl ;
    }
    virtual void endElement(const std::string &qname) {
        std::cout << "end element: " << qname << std::endl ;
    }

    virtual void characters(const std::string &text_data) {
        std::cout << "characters: " << text_data << std::endl ;
    }

    virtual void error(ErrorCode code, uint line, uint column ) {
        std::cout << "error near line: " << line << ", col: " << column << ", code: " << code << std::endl ;
    }


private:

    bool parseXmlDecl() ;
    bool parseAttributeList(Dictionary &at) ;
    bool parseName(std::string &name);
    bool parseAttributeValue(std::string &val);
    bool parseMisc() ;
    bool parseComment() ;
    bool parseElement() ;
    bool parsePI() ;
    bool parseContent() ;
    bool parseCData() ;
    bool parseCharacters() ;
    bool parseDocType() ;
    void fatal(ErrorCode code) ;

    std::shared_ptr<XMLStreamWrapper> stream_ ;

};


} // namespace util
} // namespace vsim

#endif
