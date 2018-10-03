#ifndef __CVX_VIZ_FONT_HPP__
#define __CVX_VIZ_FONT_HPP__

#include <string>
#include <vector>

namespace cvx { namespace viz {

enum class FontStyle { Normal, Oblique, Italic } ;
enum FontWeight { Normal, Bold } ;

class Font {
public:

    Font(const std::string &family_desc, size_t px):  sz_(px),
        style_(FontStyle::Normal),  weight_(FontWeight::Normal) {
        parse_family_names(family_desc) ;
    }

    Font & setStyle(FontStyle style) { style_ = style  ; return *this ; }
    Font & setWeight(FontWeight weight) { weight_ = weight ; return *this ;}
    Font & setSize(double pts) { sz_ = pts ; return *this ; }
    Font & setFamily(const std::string &family_desc ) { parse_family_names(family_desc) ; return *this ; }

    FontStyle style() const { return style_ ; }
    FontWeight weight() const { return weight_ ; }
    size_t size() const { return sz_ ; }
    const std::vector<std::string> familyNames() const { return family_names_ ; }

private:

    void parse_family_names(const std::string &desc) {
        std::string token ;
        for ( char c: desc ) {
            if ( c == ',' ) {
                family_names_.emplace_back(token) ;
                token.clear() ;
            }
            else token += c ;
        }
        if ( !token.empty() ) family_names_.emplace_back(token) ;
    }

    FontStyle style_ ;
    FontWeight weight_ ;
    size_t sz_ ;
    std::vector<std::string> family_names_ ;
} ;


} }

#endif
