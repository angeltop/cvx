#include <cvx/util/misc/arg_parser.hpp>
#include <cvx/util/misc/strings.hpp>

#include <algorithm>
#include <cassert>
#include <iostream>

using namespace std ;

namespace cvx {

void ArgumentParser::parse(int argc, const char *argv[], uint c0) {

    Container::iterator cpos = positional_.begin() ;

    pos_ = c0 ;

    while ( pos_ < argc ) {

        if ( argv[pos_][0] == '-' ) { // we have a non-positional option

            auto match = findMatchingArg(argv[pos_]) ; // find matching arg

            if ( match != options_.end() ) { // found
                ++pos_ ; // skip flag
                consumeArg(match, argc, argv, pos_)  ;
                if ( match->action_ && !match->action_() ) break ;
            }
            else throw InvalidOption(*this, argv[pos_]) ; // unknown option
        } else if ( cpos != positional_.end() ) {  // try positional arguments
            string val = argv[pos_] ;
            consumeArg(cpos, argc, argv, pos_) ;
            if ( cpos->action_ && !cpos->action_() ) break ;
            ++cpos ;
        } else throw InvalidOption(*this, argv[pos_]) ;
    }

    setDefaults() ;
    checkRequired() ;
}



// adapted from boost::program_options code

static void format_paragraph(std::ostream& strm,
                             std::string par,
                             unsigned indent,
                             unsigned line_length)
{
    assert(indent < line_length);
    line_length -= indent;

    // index of tab (if present) is used as additional indent relative
    // to first_column_width if paragrapth is spanned over multiple
    // lines if tab is not on first line it is ignored
    string::size_type par_indent = par.find('\t');

    if ( par_indent == string::npos ) par_indent = 0;
    else  {
        // erase tab from string
        par.erase(par_indent, 1);

        // this assert may fail due to user error or environment conditions!
        assert(par_indent < line_length);

        // ignore tab if not on first line
        if ( par_indent >= line_length ) par_indent = 0 ;
    }

    if (par.size() < line_length) strm << par ; // description is short no formating needed
    else {
        string::const_iterator       line_begin = par.begin();
        const string::const_iterator par_end = par.end();

        bool first_line = true; // of current paragraph!

        while ( line_begin < par_end ) {  // paragraph lines
            if ( !first_line ) {
                // If line starts with space, but second character
                // is not space, remove the leading space.
                // We don't remove double spaces because those
                // might be intentianal.
                if ((*line_begin == ' ') && ((line_begin + 1 < par_end) && (*(line_begin + 1) != ' ')))
                    line_begin += 1;  // line_begin != line_end
            }
            // Take care to never increment the iterator past
            // the end, since MSVC 8.0 (brokenly), assumes that
            // doing that, even if no access happens, is a bug.
            unsigned remaining = static_cast<unsigned>(std::distance(line_begin, par_end));
            string::const_iterator line_end = line_begin +
                    ((remaining < line_length) ? remaining : line_length);

            // prevent chopped words
            // Is line_end between two non-space characters?
            if ((*(line_end - 1) != ' ') && ((line_end < par_end) && (*line_end != ' '))) {
                // find last ' ' in the second half of the current paragraph line
                string::const_iterator last_space =
                        find(reverse_iterator<string::const_iterator>(line_end),
                             reverse_iterator<string::const_iterator>(line_begin),
                             ' ').base();

                if ( last_space != line_begin ) {
                    // is last_space within the second half ot the current line
                    if (static_cast<unsigned>(std::distance(last_space, line_end)) < (line_length / 2))
                        line_end = last_space;
                }
            } // prevent chopped words

            // write line to stream
            copy(line_begin, line_end, ostream_iterator<char>(strm));

            if ( first_line ) {
                indent += static_cast<unsigned>(par_indent);
                line_length -= static_cast<unsigned>(par_indent); // there's less to work with now
                first_line = false;
            }

            // more lines to follow?
            if ( line_end != par_end ) {
                strm << '\n';
                for(unsigned pad = indent; pad > 0; --pad) strm.put(' ') ;
            }

            // next line starts after of this line
            line_begin = line_end;
        } // paragraph lines
    }

}


static void format_description(std::ostream& strm,
                               const std::string& desc,
                               unsigned first_column_width,
                               unsigned line_length)  {

    assert( line_length > 1 );

    --line_length;

    assert(line_length > first_column_width);

    vector<string> lines = split(desc, "\n") ;

    for( auto it = lines.begin() ; it != lines.end() ;  ) {
        format_paragraph(strm, *it, first_column_width, line_length);
        ++it ;

        if ( it != lines.end() ) {
            strm << '\n';
            for ( uint pad = first_column_width ; pad > 0; --pad ) strm.put(' ') ;
        }
    }  // paragraphs
}

static void format_text(std::ostream& strm,  const std::string& str, unsigned line_length) {
    string tmp;
    string::const_iterator it = str.begin() ;
    char cur = 0, last = 0;
    size_t i = 0;

    while ( it != str.end() ) {
        cur = *it++ ;
        if ( ++i == line_length ) {
            ltrim(tmp) ;
            strm << endl << tmp ;
            i = tmp.length() ;
            tmp.clear() ;
        } else if ( cur == '\n' ) {
            strm << tmp;
            tmp.clear();
            i = 0 ;
        } else if (isspace(cur) && !isspace(last)) { // a word
             strm << tmp;
             tmp.clear();
        }

        tmp += cur;
        last = cur;
    }

}
void ArgumentParser::printUsage(ostream &strm, uint line_length, uint min_description_width)
{
    if ( !description_.empty() ) {
        format_text(strm, description_, line_length) ;
        strm << endl << endl ;
    }

    printOptions(strm, line_length, min_description_width) ;

    if ( !epilog_.empty() ) {
        strm << endl ;
        format_text(strm, epilog_, line_length) ;
    }
}

uint ArgumentParser::getOptimalColumnWidth(uint line_length, uint min_description_length)
{
    /* Find the maximum width of the option column */

    uint width = 23 ;
    for ( const Option &o: options_ ) {
        string ss = o.formatOptionFlags() ;
        width = std::max(width, static_cast<uint>(ss.size()));
    }

    const uint start_of_description_column = line_length - min_description_length;

    width = std::min(width, start_of_description_column - 1);

    /* add an additional space to improve readability */
    ++width;
    return width;
}

void ArgumentParser::printOptions(ostream &strm, uint line_length, uint min_description_length)
{
    uint col_width = getOptimalColumnWidth(line_length, min_description_length) ;

    if (!options_caption_.empty())
        strm << options_caption_ << ":\n";
    for( const Option &o: options_ ) {
        o.printDescription(strm, col_width, line_length);
        strm << '\n' ;
    }
}

void ArgumentParser::consumeArg(const Container::iterator &match, int argc, const char *argv[], uint &c) {
    Option &a = *match ;
    a.matched_ = true ;

    uint maxc = ( a.max_args_ == -1 ) ? argc-1 : std::min(c + a.max_args_-1, (uint)argc-1) ;

    std::string args ;

    uint n_args = 0 ;
    while ( c <= maxc ) {

        string arg(argv[c]) ;

        if ( argv[c][0] == '-' ) break ; // too many values
        else {
            n_args ++ ;
            if ( !args.empty() ) args += '\n' ;
            args += arg ;
        }
        ++c ;
    }

    if ( n_args < a.min_args_  ) throw IncorrectArguments(*this, a.short_flag_) ;

    if ( n_args > 0 ) {
        istringstream strm(args) ;
        if ( !a.value_->read(strm) ) throw IncorrectArguments(*this, a.short_flag_) ;
    }
    else if ( !a.implicit_value_.empty() ) { // no args passed so try to store the implicit value
        stringstream strm(a.implicit_value_) ;
        if ( !a.value_->read(strm) ) throw IncorrectArguments(*this, a.short_flag_) ;
    }
}

ArgumentParser::Container::iterator ArgumentParser::findMatchingArg(const string &arg)
{
    return std::find_if(options_.begin(), options_.end(), [&arg] ( const Container::value_type &t) { return t.matches(arg) ;} ) ;
}


void ArgumentParser::setDefaults()
{
    for( auto &&o: options_ ) {
        if ( !o.matched_ && !o.default_value_.empty() ) {
            stringstream strm(o.default_value_) ;
            if ( !o.value_->read(strm) )  ;
        }
    }
}

void ArgumentParser::checkRequired()
{
    for( auto &&o: options_ ) {
        if ( o.is_required_ && !o.matched_ ) throw MissingRequiredOption(*this, o.short_flag_) ;
    }

}

string ArgumentParser::Option::formatOptionFlags() const
{

    stringstream ss ;
    ss << "  " << join(flags_, "|") ;
    if ( !name_.empty() && max_args_ > 0 ) ss << ' ' << name_ ;

    string first_column = ss.str() ;

    return first_column ;
}

void ArgumentParser::Option::printDescription(ostream &strm, uint first_column_width, uint line_length) const
{
    string first_column = formatOptionFlags() ;
    strm << first_column ;

    if ( !description_.empty() ) {
        if ( first_column.size() >= first_column_width ) {
            strm.put('\n');
            for ( uint pad = first_column_width ; pad > 0; --pad ) strm.put(' ') ;
        } else {
            for( uint pad = first_column_width - static_cast<uint>(first_column.size()); pad > 0; --pad ) strm.put(' ') ;
        }

        format_description(strm, description_, first_column_width, line_length);
    }
}

ArgumentParser::Option::Option(const string &flags, std::shared_ptr<Value> val): value_(val), min_args_(1), max_args_(1),
    is_required_(false), matched_(false), is_positional_(false)
{
    name_ = "<arg>" ;
    flags_ = split(flags, "|,") ;
    for(const string &f : flags_) {
        if ( startsWith(f, "--") ) continue ;
        else if ( f.at(0) == '-' )
            short_flag_ = f ;
    }
}

ArgumentParser::Option::Option(std::shared_ptr<Value> val): value_(val), min_args_(1), max_args_(1),
    is_required_(false), matched_(false), is_positional_(true)
{
    name_ = "<arg>" ;
}


bool ArgumentParser::Option::matches(const string &arg) const
{
    return std::find(flags_.begin(), flags_.end(), arg) != flags_.end() ;
}


}
