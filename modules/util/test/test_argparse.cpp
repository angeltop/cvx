#include <iostream>

#include <cvx/util/misc/arg_parser.hpp>
#include <cvx/util/misc/strings.hpp>
#include <cvx/util/misc/optional.hpp>

using namespace std ;
using namespace cvx::util ;

struct BBox {
    float min_x_, min_y_, max_x_, max_y_ ;
    bool empty_ = true ;

    BBox() = default ;
    BBox(float minx, float miny, float maxx, float maxy):
        min_x_(minx), min_y_(miny), max_x_(maxx), max_y_(maxy), empty_(false) {}

    friend std::istream &operator >> ( std::istream &strm, BBox &box ) {
        strm >>box.min_x_ >> box.min_y_ >> box.max_x_ >> box.max_y_ ;
        box.empty_ = false ;
        return strm ;
    }
};

static void test_simple(int argc, const char *argv[]) {

    ArgumentParser parser ;

    BBox bbox ;
    float f = 2.0;
    int val ;
    std::vector<std::string> files ;
    std::vector<std::string> items ;
    int pos ;
    bool print_help = true ;
    ArgumentParser args ;
    args.setDescription("test_argparse [options] output (input)+ \nList information about the FILEs (the current directory by default).\nSort entries alphabetically if none of -cftuvSUX nor --sort is specified.") ;

    args.addOption("-h|--help", print_help).setMaxArgs(0).setDescription("print this help message").setDefault("false") ;
    args.addOption("-f|--flag", f).setMinArgs(0).setMaxArgs(1).setDescription("first flag").setName("[<arg>]").setImplicit("4.0") ;
    args.addOption("-b|--bbox", bbox).setNumArgs(4).setDescription("bounding box").setName("<minx> <miny> <maxx> <maxy>") ;
    args.addOption("--test", items).setMinArgs(2).setMaxArgs(-1).setDescription("second flag").setName("<arg1> <arg2> [ ... <argN>]") ;
    args.addOption("--custom", [&](istream &strm) {
        strm >> val ;
        return true ;
    }).setDescription("custom flag") ;
    args.addPositional(pos) ;
    args.addPositional(files).setMaxArgs(-1) ;

    try {
        args.parse(argc, argv) ;

        if ( print_help ) {
            cout << "Usage: prog [options] position files" << endl ;
            cout << "Options:" << endl ;
            args.printUsage(std::cout) ;
        }
    } catch ( ArgumentParserException &e ) {
        cout << e.what() << endl ;
        args.printUsage(std::cout) ;
    }


}

void test_git(int argc, const char *argv[]) {

    ArgumentParser root ;
    ArgumentParser clone_group ;
    ArgumentParser init_group ;

    root.setDescription(R"(usage: git [--version] [--help] [-C <path>] [-c name=value]
[--exec-path[=<path>]] [--html-path] [--man-path] [--info-path]
[-p | --paginate | --no-pager] [--no-replace-objects] [--bare]
[--git-dir=<path>] [--work-tree=<path>] [--namespace=<name>]
<command> [<args>]

These are common Git commands used in various situations:

Start a working area (see also: git help tutorial)
    clone      Clone a repository into a new directory
    init       Create an empty Git repository or reinitialize an existing one

'git help -a' and 'git help -g' list available subcommands and some
concept guides. See 'git help <command>' or 'git help <concept>'
to read about a specific subcommand or concept.")") ;

    bool print_help = false ;
    string cmd ;

    root.addOption("-h|--help", print_help).setMaxArgs(0).setDescription("print this help message").setImplicit("true") ;
    root.addPositional(cmd).setAction( [&] {
        if ( cmd == "init" ) {
            try {
                init_group.parse(argc, argv, root.pos()) ;
                if ( print_help )
                    init_group.printUsage(std::cout) ;
            } catch ( ArgumentParserException &e ) {
                cerr << e.what() << endl ;
                init_group.printUsage(std::cerr) ;
            }
        }
        else if ( cmd == "clone" ) {
            try {
                clone_group.parse(argc, argv, root.pos()) ;
                if ( print_help )
                    clone_group.printUsage(std::cout) ;
            } catch ( ArgumentParserException &e ) {
                cerr << e.what() << endl ;
                clone_group.printUsage(std::cerr) ;
            }
        } else {
            if ( !cmd.empty() ) cerr << "Unknown command: " << cmd << endl ;
            root.printUsage(std::cerr) ;
        }

        return false ;
    } ) ;



    bool verbose = false, quiet = false, bare = false  ;
    string origin_name, repo, dir, tmpl ;

    clone_group.setDescription("usage: git clone [<options>] [--] <repo> [<dir>]") ;
    clone_group.addOption("-v|--verbose", verbose).setNumArgs(0).setDescription("be more verbose").setImplicit("true") ;
    clone_group.addOption("-o|--origin", origin_name).setNumArgs(1).setDescription("use <name> instead of 'origin' to track upstream").setName("<name>") ;
    clone_group.addOption("-h|--help", print_help).setMaxArgs(0).setDescription("print this help message").setImplicit("true") ;
    clone_group.addPositional(repo).required() ;
    clone_group.addPositional(dir) ;



    init_group.setDescription("usage: git init [-q | --quiet] [--bare] [--template=<template-directory>] [--shared[=<permissions>]] [<directory>]") ;
    init_group.addOption("--template", tmpl).setDescription("directory from which templates will be used").setName("<template-directory>");
    init_group.addOption("--bare", bare).setNumArgs(0).setDescription("create a bare repository") ;
    init_group.addOption("-h|--help", print_help).setMaxArgs(0).setDescription("print this help message").setImplicit("true") ;
    init_group.addPositional(dir) ;

    try {
        root.parse(argc, argv) ;

        if ( print_help && cmd.empty() )
                root.printUsage(std::cout) ;

    } catch ( ArgumentParserException &e ) {
        cout << e.what() << endl ;
        e.group().printUsage(std::cout) ;
    }
}

int main(int argc, const char *argv[]) {
 //   test_simple(argc, argv) ;
    test_git(argc, argv) ;
}
