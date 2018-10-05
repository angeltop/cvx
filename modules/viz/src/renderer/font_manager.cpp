#include "font_manager.hpp"
#include <cassert>
#include <memory>
#include <fontconfig/fontconfig.h>

using namespace std ;

namespace cvx { namespace viz { namespace detail {

FT_Face FontManager::find(const std::string &key) {
    std::lock_guard<std::mutex> g(mx_) ;
    std::map<std::string, FT_Face>::const_iterator it = cache_.find(key) ;
    if ( it != cache_.end() ) return it->second ;
    else return 0 ;
}

void FontManager::save(const std::string &key, FT_Face face) {
    std::lock_guard<std::mutex> g(mx_) ;
    cache_.insert(std::make_pair(key, face)) ;
}

FontManager::FontManager() {
     assert ( FT_Init_FreeType( &library_ ) == 0 ) ;
}

FontManager::~FontManager() {
     FT_Done_FreeType(library_);
}


string FontManager::font_face_key(const string &family_name, FontStyle font_style, FontWeight font_weight) {
    string key = family_name  + '-' ;

    switch ( font_style )
    {
    case FontStyle::Normal:
        key += "normal-" ;
        break ;
    case FontStyle::Oblique:
        key += "oblique-" ;
        break ;
    case FontStyle::Italic:
        key += "italic-" ;
        break ;
    }

    switch ( font_weight )
    {
    case FontWeight::Normal:
        key += "normal" ;
        break ;
    case FontWeight::Bold:
        key += "bold" ;
        break ;
    }

    return key ;
}

// https://stackoverflow.com/a/4119881

static bool iequals(const string& a, const string& b) {
    unsigned int sz = a.size();
    if (b.size() != sz) return false;

    for (unsigned int i = 0; i < sz; ++i)
        if (tolower(a[i]) != tolower(b[i]))
            return false;
    return true;
}

FT_Face FontManager::queryFace(const std::string &family_name, FontStyle font_style, FontWeight font_weight) {
    string key = font_face_key(family_name, font_style, font_weight) ;

    FT_Face face = find(key) ;

    if ( face ) return face ;

    std::unique_ptr<FcPattern, decltype(&FcPatternDestroy)>pat(FcPatternCreate(), FcPatternDestroy) ;

    FcPatternAddString(pat.get(), FC_FAMILY, (const FcChar8*)(family_name.c_str()));

    if ( font_style == FontStyle::Italic )
        FcPatternAddInteger(pat.get(), FC_SLANT, FC_SLANT_ITALIC) ;
    else if ( font_style == FontStyle::Oblique )
        FcPatternAddInteger(pat.get(), FC_SLANT, FC_SLANT_OBLIQUE) ;
    else
        FcPatternAddInteger(pat.get(), FC_SLANT, FC_SLANT_ROMAN) ;

    if ( font_weight == FontWeight::Bold )
        FcPatternAddInteger(pat.get(), FC_WEIGHT, FC_WEIGHT_BOLD) ;
    else
        FcPatternAddInteger(pat.get(), FC_WEIGHT, FC_WEIGHT_NORMAL) ;

    FcPatternAddBool(pat.get(), FC_SCALABLE, FcTrue);

    FcConfigSubstitute(0, pat.get(), FcMatchPattern);
    FcDefaultSubstitute(pat.get());

    FcChar8* fontConfigFamilyNameAfterConfiguration;
    FcPatternGetString(pat.get(), FC_FAMILY, 0, &fontConfigFamilyNameAfterConfiguration);

    FcResult font_config_result;
    FcPattern *result_pattern = FcFontMatch(0, pat.get(), &font_config_result);
    if ( !result_pattern ) // No match.
        return nullptr ;

    std::unique_ptr<FcPattern, decltype(&FcPatternDestroy)> result(result_pattern, FcPatternDestroy) ;

    FcChar8* fc_family_name_str;
    FcPatternGetString(result.get(), FC_FAMILY, 0, &fc_family_name_str);
    string fc_family_name((char *)fc_family_name_str) ;

    if (!iequals(family_name, fc_family_name)
            && !(iequals(family_name, "sans") || iequals(family_name, "sans-serif")
              || iequals(family_name, "serif") || iequals(family_name, "monospace")
              || iequals(family_name, "fantasy") || iequals(family_name, "cursive")))
        face = nullptr ;
    else {
        FcChar8* file = nullptr;
        if ( FcPatternGetString(result.get(), FC_FILE, 0, &file) == FcResultMatch )  {
              const char *fontFile = (const char*)file;

              // load face from file
              FT_Face     face;
              if ( FT_New_Face( library_, fontFile, 0, &face ) == 0 ) {

                  save(key, face) ;
                  return face ;
              }
              return nullptr ;
        }
    }

    return nullptr ;

}


FT_Face FontManager::queryFontFace(const Font &font)
{
    vector<string> q_family_names(font.familyNames()) ;
    q_family_names.emplace_back("serif") ; // fallback family (OS dependent)

    for( const auto &family: q_family_names ) {
        FT_Face face = FontManager::instance().queryFace(family, font.style(), font.weight()) ;
        if ( face ) return face ;
    }

    return nullptr ;
}

}}}
