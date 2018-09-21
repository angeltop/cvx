#include <cvx/util/misc/dir_iterator.hpp>
#include <cvx/util/misc/strings.hpp>
#include <cvx/util/misc/path.hpp>

using namespace std ;

namespace cvx { namespace util {

const DirectoryFilter DirectoryFilters::MatchAll = [](const DirectoryEntry &) {return true ; } ;
const DirectoryFilter DirectoryFilters::MatchDirectories = [](const DirectoryEntry &e) {return e.isDirectory() ; } ;


static string glob_to_regex(const char *pat)
{
    // Convert pattern
    string rx = "^", be ;

    int i = 0;
    const char *pc = pat ;
    size_t clen = strlen(pat) ;
    bool in_char_class = false ;

    while (i < clen)
    {
        char c = pc[i++];

        switch (c)
        {
            case '*':
                rx += "[^\\\\/]*" ;
                break;
            case '?':
                rx += "[^\\\\/]" ;
                break;
            case '$':  //Regex special characters
            case '(':
            case ')':
            case '+':
            case '.':
            case '|':
                rx += '\\';
                rx += c;
                break;
            case '\\':
                if ( pc[i] == '*' ) rx += "\\*" ;
                else if ( pc[i] == '?' )  rx += "\\?" ;
                ++i ;
            break ;
            case '[':
            {
                if ( in_char_class )  rx += "\\[";
                else {
                    in_char_class = true ;
                    be += c ;
                }
                break ;
            }
            case ']':
            {
                if ( in_char_class ) {
                    in_char_class = false ;
                    rx += be ;
                    rx += c ;
                    rx += "{1}" ;
                    be.clear() ;
                }
                else rx += "\\]" ;

                break ;
            }
            case '%':
            {
#ifdef USE_BOOST_REGEX
                regex rd("(0\\d)?d", regex::extended | regex::icase) ;
#else
                regex rd("(0\\d)?d", regex_constants::extended | regex_constants::icase) ;
#endif

                smatch what;
                string q(pat + i) ;

                if ( regex_match(q, what, rd) )
                {

                    rx += "[[:digit:]]" ;

                    if ( what.size() == 2 )
                    {
                        rx +=  "{" ;
                        rx += what[1] ;
                        rx += "}" ;
                    }
                    else
                        rx += "+" ;

                    i += what.size() ;
                }
                else
                {
                    if ( in_char_class ) be += c ;
                    else rx += c;
                }
                break ;

            }
            default:
                if ( in_char_class ) be += c ;
                else rx += c;
        }
    }

    rx += "$" ;
    return rx ;
}

const DirectoryFilter DirectoryFilters::Glob(const string &glob_pattern)
{
    std::vector<string> el = split(glob_pattern, ";") ;

    std::vector<regex> pats ;
    for(int i=0 ; i<el.size() ; i++) {
        try {
            string rx_str = glob_to_regex(el[i].c_str()) ;
#ifdef USE_BOOST_REGEX
            regex rx(rx_str, regex::extended) ;
#else
            regex rx(rx_str, regex_constants::extended) ;
#endif
            pats.emplace_back(rx) ;
        }
        catch ( regex_error &e ) {

        }
    }

    auto filter = [pats] ( const DirectoryEntry &e ) {
        if ( e.type() == DirectoryEntry::File ) {
            for(int i=0 ; i<pats.size() ; i++ ) {
                if ( regex_match(e.path(), pats[i]) )
                    return true ;
            }
        }
        return false ;
    } ;

    return filter ;
}

DirectoryListing::DirectoryListing(const Path &dir, DirectoryFilter filter): dir_(dir.toString()), filter_(filter) {

}

}}
