#ifndef __ARG_PARSER_HPP__
#define __ARG_PARSER_HPP__

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>
#include <istream>
#include <iterator>
#include <sstream>

#include <cvx/util/misc/optional.hpp>

namespace cvx {

typedef std::function<bool(std::istream &)> ValueParser ;

// Minimal command line argument parser
// Add options using addOption and addPositional functions, then call parse function
// The class does not act as a container of parsed values. Instead the values are stored in user variables which have to be valid when parse is called.
// Reading of arguments into values is performed internally be means of stream >> operations and therefore any variable type that supports this operator
// can be used.

class ArgumentParserException ;
class CommandGroup ;

class ArgumentParser {

public:

    ArgumentParser() {}

    void parse(CommandGroup &group, int argc, const char *argv[]) ;

} ;

class CommandGroup {
    class Value ;

public:

    CommandGroup() {}

    CommandGroup(CommandGroup &parent, const std::string &group_name): parent_(&parent), group_name_(group_name) {
        assert(!group_name.empty()) ;
        parent.children_[group_name_] = this ;
    }

    // returns true if the command group has been succesfully matched
    bool isMatched() const { return is_matched_ ; }

    class Option ;

    // add option that stores the arguments in the reference value
    // the template parameter can be any type defining the istream >> operator
    // flags is a list of acceptable flags (delimited by '|' or ',')
    // It is the user to set a default value to the variable before calling the parser. This will not be touched
    // if the option is not given on the command line otherwise one can use the .setDefault function

    template<class T>
    Option &addOption(const std::string &flags, T &value) {
        options_.emplace_back(flags, std::make_shared<ValueHolder<T>>(value)) ;
        return options_.back();
    }

    template<class T>
    Option &addOption(const std::string &flags, optional<T> &value) {
        options_.emplace_back(flags, std::make_shared<OptionalValueHolder<T>>(value)) ;
        return options_.back();
    }

    // add option that stores the arguments to a vector of values (i.e. to accept multiple args)
    // the template parameter can be any type defining the istream >> operator
    // flags is a list of acceptable flags (delimited by '|' or ',')

    template<class T>
    Option &addOption(const std::string &flags, std::vector<T> &value) {
        options_.emplace_back(flags, std::make_shared<ValueListHolder<T>>(value)) ;
        return options_.back() ;
    }

    // Use this for defining a custom argument parser
    // The parser is a lambda that should parse the value and store it to a captured variable
    // flags is a list of acceptable flags (delimited by '|' or ',')

    Option &addOption(const std::string &flags, ValueParser parser) {
        options_.emplace_back(flags, std::make_shared<ValueAdapter>(parser)) ;
        return options_.back() ;
    }

    // Positional options

    template<class T>
    Option &addPositional(T &value) {
        positional_.emplace_back(std::make_shared<ValueHolder<T>>(value)) ;
        return positional_.back();
    }

    template<class T>
    Option &addPositional(std::vector<T> &value) {
        positional_.emplace_back(std::make_shared<ValueListHolder<T>>(value)) ;
        return positional_.back();
    }

    Option &addPositional(ValueParser parser) {
        positional_.emplace_back(std::make_shared<ValueAdapter>(parser)) ;
        return positional_.back();
    }

    // Parse the command line. In case of failure an exception is thrown.
    void parse(uint c, int argc, const char *argv[]) ;

    void printUsage(std::ostream &strm, uint line_length = 80, uint min_description_width = 40) ;

    // set the description of the command printed first in the usage print out.
    void setDescription(const std::string &desc) {
        description_ = desc ;
    }

    // set the caption printed before the list of options:
    void setOptionsCaption(const std::string &caption) {
        options_caption_ = caption ;
    }

    // set the text printed after the list of options:
    void setEpilog(const std::string &epilog) {
        epilog_ = epilog ;
    }

    class Option {
    public:

        // Set minimum args expected ( default 1 )
        Option &setMinArgs(int minargs) {
            min_args_ = minargs ;
            if ( max_args_ >= 0 )
                max_args_ = std::max(min_args_, max_args_) ;
            return *this ;
        }

        // Set maximum args expected ( default 1 ). set to -1 for infinite
        Option &setMaxArgs(int maxargs) {
            max_args_ = maxargs ;
            if ( maxargs >= 0 )
                min_args_ = std::min(min_args_, maxargs) ;
            return *this ;
        }

        // Set constant number of args
        Option &setNumArgs(int numargs) {
            return setMinArgs(numargs) ;
        }

        // Set option as required (not checked for positional arguments)
        Option &required(bool req = true) {
            is_required_ = req ;
            return *this ;
        }

        // Set description paragraph to be written on help output
        Option &setDescription(const std::string &desc) {
            description_ = desc ;
            return *this ;
        }

        // Set variable name to be written on help output (default is <arg>)
        // We let the user provide a concise and meaningful semantic name (e.g. including default values)
        // instead of trying to make this automatically
        Option &setName(const std::string &name) {
            name_ = name ;
            return *this ;
        }

        // if the option did not match and the default value is set then the argument value will be set to this as if provided on the commandline
        Option &setDefault(const std::string &defaultValue) {
            default_value_ = defaultValue ;
            return *this ;
        }

        // if the option matches but no arguments given the implicit value is set to this as if provided in the command line
        Option &setImplicit(const std::string &implicitValue) {
            implicit_value_ = implicitValue ;
            return *this ;
        }

        // use for positional arguments to indicate that this is a group switch and in this case a subgroup of commands matching the value should be called
        Option &setGroupSwitch(bool gs = true) {
            is_group_switch_ = gs  ;
            return *this ;
        }


        Option(const std::string& flags, std::shared_ptr<Value> val) ;
        Option(std::shared_ptr<Value> val) ;

    protected:

        friend class CommandGroup ;
        bool matches(const std::string &arg) const ;

        std::string formatOptionFlags() const ;
        void printHelp(std::ostream &strm) const;
        void printDescription(std::ostream &strm, uint first_col_width, uint line_length) const;

        std::string default_value_ ;
        std::string implicit_value_ ;
        std::vector<std::string> flags_;  // list of flags that this option accepts
        std::string description_ ;        // description of this arg
        std::string short_flag_ ;         // short flag selected from the flags
        std::string name_ ;               // the arg name that will appear after the flag in the help printout
        bool is_required_, is_positional_ ;
        std::shared_ptr<Value> value_ ;

        bool matched_ = false, is_group_switch_ = false ;
        int min_args_, max_args_ ;        // minimum and maximum args acceptable
    };

private:

    class Value {
    public:
        virtual bool read(std::istream &s) = 0 ;
    };


    template <class T>
    class ValueHolder: public Value {
    public:
        ValueHolder(T &t): val_(t) {}

        bool read(std::istream &strm) override {
            strm >> val_ ;
            return (bool)strm ;
        }

    private:
        T &val_ ;
    };

    template <class T>
    class OptionalValueHolder: public Value {
    public:
        OptionalValueHolder(optional<T> &t): val_(t) {}
        bool read(std::istream &strm)  {
            T val ;
            strm >> val ;
            val_ = val ;
            return (bool)strm ;
        }

    private:
        optional<T> &val_ ;
    };


    template <class T>
    class ValueListHolder: public Value {
    public:
        ValueListHolder(std::vector<T> &t): list_(t) {}
        bool read(std::istream &stream)  {
            std::istream_iterator< T > it { stream }, end;
            std::copy( it, end, std::back_inserter(list_) );
            return (bool)stream ;
        }


    private:
        std::vector<T> &list_ ;
    };


    class ValueAdapter: public Value {
    public:
        ValueAdapter(ValueParser writer): writer_(writer) {}

        bool read(std::istream &s) {
            return writer_(s) ;
        }

    private:
        ValueParser writer_ ;
    };


private:
    typedef std::vector<Option> Container ;

    void consumeArg(const Container::iterator &match, int argc, const char *argv[], uint &c);
    Container::iterator findMatchingArg(const std::string &arg) ;
    void checkRequired() ;
    void setDefaults();
    uint getOptimalColumnWidth(uint line_length, uint min_description_length);
    void printOptions(std::ostream &strm, uint line_length = 80, uint min_description_width = 40) ;

    Container options_, positional_ ;
    std::string options_caption_, description_, epilog_ ;
    std::string prog_ ;
    std::string usage_ ;
    std::string group_name_ ;
    std::map<std::string, CommandGroup *> children_ ;
    CommandGroup *parent_ = nullptr ;
    bool is_matched_ = false ;
};

template<>
bool CommandGroup::ValueHolder<bool>::read(std::istream &strm) {
    std::string s ;
    strm >> s ;
    if ( s == "true" || s == "1" || s == "yes" ) {
        val_ = true ;
        return true ;
    }
    else if ( s == "false" || s == "0" || s == "no" ) {
        val_ = false ;
        return true ;
    }
    else return false ;
}

class ArgumentParserException: public std::exception {
public:
public:
    ArgumentParserException(CommandGroup &group, const std::string& msg): msg_(msg), group_(group) {}

    virtual const char*  what() const noexcept {
        return msg_.c_str();
    }

    CommandGroup &group() { return group_ ; }

private:
    std::string msg_ ;
    CommandGroup &group_ ;
};

class InvalidOption: public ArgumentParserException {
public:
    InvalidOption(CommandGroup &group, const std::string& option)
        : ArgumentParserException(group, "Option \'" + option + "\' does not exist")
    {
    }
};

class IncorrectArguments: public ArgumentParserException {
public:
    IncorrectArguments(CommandGroup &group, const std::string& option)
        : ArgumentParserException(group, "Option \'" + option + "\' has inccorect arguments")
    {
    }
};

class MissingRequiredOption: public ArgumentParserException {
public:
    MissingRequiredOption(CommandGroup &group, const std::string& option)
        : ArgumentParserException(group, "Required option \'" + option + "\' is missing")
    {
    }
};

class InvalidCommandGroup: public ArgumentParserException {
public:
    InvalidCommandGroup(CommandGroup &parent_group, const std::string& group)
        : ArgumentParserException(parent_group, "Invalid command group \'" + group + "\'")
    {
    }
};

}
#endif
