#include <cvx/util/misc/xml_sax_parser.hpp>

#include <iterator>
#include <stack>

using namespace std ;

class XMLSAXException {
public:
    XMLSAXException(XMLSAXParser::ErrorCode code, uint line, uint col): code_(code), line_(line), column_(col) {}

    XMLSAXParser::ErrorCode code_ ;
    uint line_, column_ ;
};

class XMLStreamWrapper {
public:
    XMLStreamWrapper(istream &strm): strm_(strm) {
        line_ = 1 ; chars_ = 0 ; column_ = 1 ;
        nla_ = 0 ;
    }

    char next() {
        char c ;
        if ( nla_ > 0 )
            c = look_ahead_[--nla_] ;
        else {
            c = strm_.get() ;
            if ( c == '\r' ) c = strm_.get() ;
        }

        chars_ ++ ;
        column_ ++ ;

        if ( c == '\n' ) {
            line_++ ; column_ = 1 ;
        }

        return c ;
    }

    void putback(char c) {
        look_ahead_[nla_++] = c ;
        chars_ -- ;
        column_ -- ;
        if ( c == '\n' ) {
            line_-- ; column_ = 1 ;
        }
    }

    bool expect(const char *seq) {
        char buf[256] ;
        bool match = true ;
        const char *p = seq ;

        uint k = 0 ;
        while ( *p ) {
            char c = next() ;
            buf[k++] = c ;
            if ( *p != c ) {
                match = false ;
                break ;
            }

            ++p ;
        }

        if ( !match )
            for ( int i=k-1 ; i>=0 ; i-- )
                putback(buf[i]) ;

        return match ;
    }

    bool eatWhite() {
        char c ;
        uint chars = 0 ;
        do {
            c = next() ;
            chars ++ ;
        } while ( strm_ && ( c == ' ' || c == '\t' || c == '\r' || c == '\n' ) ) ;
        putback(c) ;
        return chars > 1 ;
    }

    bool good() {
        return strm_.good() ;
    }

    bool escapeString(string &value) {

        uint k = 0 ;
        string quot ;
        char c = next() ;
        do {
            quot += c ;
            c = next() ;
            ++k ;
        } while ( strm_.good() && c != ';' && k < 5 ) ;

        if ( quot == "amp" ) value += '&' ;
        else if ( quot == "quot" ) value += '\"' ;
        else if ( quot == "lt" ) value += '<' ;
        else if ( quot == "gt" ) value += '>' ;
        else if ( quot == "apos") value += '\'' ;
        else return false ;

        return true ;
    }

    char peek() {
        if ( nla_ > 0 )
            return look_ahead_[nla_ - 1] ;
        else
            return strm_.peek() ;
    }

    istream &strm_ ;
    uint line_, chars_, column_ ;
    char look_ahead_[8] ;
    uint nla_ ;
};

static bool is_valid_name_char(char c) {
    return ( isalnum(c) || c == '-' || c == '_' || c == '.' ) ;
}

bool XMLSAXParser::parseXmlDecl() {
    if ( stream_->expect("<?xml") ) {
        stream_->eatWhite() ;
        AttributeList attrs ;
        if ( !parseAttributeList(attrs) ) return false ;
        stream_->eatWhite() ;
        return stream_->expect("?>") ;
    }

    return false ;
}

bool XMLSAXParser::parseName(std::string &name) {
    do {
        char c = stream_->next() ;
        if ( !is_valid_name_char(c) ) {
            stream_->putback(c) ;
            break ;
        }
        else name += c ;
    } while ( stream_->good() ) ;

    return !name.empty() ;
}

bool XMLSAXParser::parseAttributeValue(std::string &val) {

    char c = stream_->next() ;
    char oc = c ;

    // matching starting quote
    if ( oc != '"' && oc != '\'' ) {
        stream_->putback(c) ;
        return false ;
    }

    // eat characters (no backtracking here)

    while ( stream_->good() ) {
        c = stream_->next() ;

        if ( c == '&' ) {
            if ( !stream_->escapeString(val) ) return false ;
        }
        else if ( c == '<' ) {
            stream_->putback(c) ; return false ;
        }
        else if ( c == oc ) break ;
        else
            val += c ;

    }  ;

    // closing quote

    return c == oc ;
}

bool XMLSAXParser::parseMisc()
{
    bool match = false ;
    while ( stream_->good() ) {
        stream_->eatWhite() ;
        if ( !parseComment() &&
             !parsePI() ) break ;
        match = true ;
    }
    stream_->eatWhite() ;
    return match ;
}

bool XMLSAXParser::parseComment()
{
    if ( stream_->expect("<!--") ) {
        char c ;
        do {
            c = stream_->next() ;
        }
        while ( stream_->good() && c != '-' ) ;

        if ( stream_->expect("->") )
            return true ;
        else
            fatal(InvalidChar) ;
    }

    return false ;
}

bool XMLSAXParser::parseElement()
{
    if ( stream_->expect("<") ) {

        char c = stream_->peek() ;
        if ( c == '/' ) {
            stream_->putback('<') ;
            return false ;
        }

        string tag ;
        AttributeList at ;
        if ( !parseName(tag) ) fatal(TagInvalid) ;
        stream_->eatWhite() ;

        parseAttributeList(at) ;

        if ( stream_->expect("/>") ) {
            startElement(tag, at) ;
            characters("") ;
            endElement(tag) ;
            return true ;
        }
        else if ( stream_->expect(">") ) {
            startElement(tag, at) ;

            parseContent() ;

            if ( stream_->expect("</") ) {
                string closing_tag ;

                if ( !parseName(closing_tag) ) fatal(TagInvalid) ;
                if ( tag != closing_tag ) fatal(TagMismatch) ;

                stream_->eatWhite() ;
                if ( stream_->expect(">") ) {
                    endElement(closing_tag) ;
                    return true ;
                }
            }
        }
        else fatal(InvalidChar) ;
    }
    return false ;
}

bool XMLSAXParser::parsePI()
{
    if ( stream_->expect("<?") ) {
        char c ;
        while ( stream_->good() ) {
            c = stream_->next() ;
            if ( c == '>' ) break ;
        } ;
        return true ;
    }
    return false ;
}

bool XMLSAXParser::parseContent()
{
    while ( 1 ) {
        if ( parseCData() ) continue ;
        if ( parseComment() ) continue ;
        if ( parsePI() ) continue ;
        if ( parseElement() ) continue ;
        if ( parseCharacters() ) continue ;
        break ;
    }
    return true ;
}

bool XMLSAXParser::parseCData()
{
    string text ;

    if ( stream_->expect("<![CDATA[") ) {
        char c ;
        while ( stream_->good() ) {
            c = stream_->next() ;
            if ( c != ']' ) text += c ;
            else {
                if ( stream_->expect("]>")) {
                    if ( !text.empty() )
                        characters(text) ;
                    return true ;
                }
            }
        } ;
    }

    return false ;

}

bool XMLSAXParser::parseCharacters()
{
    string text ;
    while ( stream_->good() ) {
        char c = stream_->next() ;
        if ( c == '<' ) {
            stream_->putback(c) ;
            break ;
        }
        else if ( c == '&' ) {
           if ( !stream_->escapeString(text) )
               fatal(InvalidChar) ;
        }
        else
           text += c ;
    }

    if ( !text.empty() ) {
        characters(text) ;
        return true ;
    }
    return false ;
}

bool XMLSAXParser::parseDocType() {
    if ( stream_->expect("<!DOCTYPE") ) {
        // ignore section
        while ( stream_->good() ) {
            char c = stream_->next() ;

            if ( c == '>' ) break ;
            else if ( c == '[') { // skip until ending bracket (maybe nested)
                uint depth = 1 ;
                while ( stream_->good() )
                {
                    char c = stream_->next() ;
                    if ( c == '[' ) ++depth ;
                    else if ( c == ']' )
                        --depth ;

                    if ( depth == 0 ) break ;
                }
            }
       }

       return true ;
    }

    return false ;
}

bool XMLSAXParser::parseAttributeList(XMLSAXParser::AttributeList &at)
{
    do {
        string attrName, attrValue ;
        if ( !parseName(attrName) ) return false ;
        stream_->eatWhite();
        if ( !stream_->expect("=") ) fatal(InvalidChar) ;
        stream_->eatWhite();
        if ( !parseAttributeValue(attrValue) ) fatal(AttrValueInvalid) ;
        at.add(attrName, attrValue) ;
        stream_->eatWhite();
        char c = stream_->peek() ;
        if ( c == '/' || c == '>' || c == '?' ) break ;
    } while ( stream_->good() ) ;
    return true ;
}

void XMLSAXParser::fatal(ErrorCode code) {
    throw XMLSAXException(code, stream_->line_, stream_->column_) ;
}

XMLSAXParser::XMLSAXParser(std::istream &strm): stream_(new XMLStreamWrapper(strm)) {}

bool XMLSAXParser::parse() {
    try {
        // prolog
        if ( !parseXmlDecl() ) return false ; // maybe we can make this less strict with option
        parseMisc() ;
        while ( 1 ) {
            if ( parseDocType() ) continue ;
            if ( parseMisc() ) continue ;
            break ;
        }

        if ( !parseElement() ) fatal(InvalidXml) ;
        parseMisc() ;

        return true ;
    }
    catch ( XMLSAXException &e ) {
        error( e.code_, e.line_, e.column_ ) ;
        return false ;
    }
}
