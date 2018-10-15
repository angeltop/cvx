#include <cvx/viz/image/view.hpp>

#include <QObject>
#include <QApplication>

#include "main_window.hpp"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv) ;

    QStringList paths ;

    for( uint i=1 ; i<argc ; i++ )
        paths.append(argv[i]) ;

    MainWindow mwin ;
    mwin.show() ;

    mwin.setFilePaths(paths) ;
    mwin.first() ;

    app.exec() ;

}
