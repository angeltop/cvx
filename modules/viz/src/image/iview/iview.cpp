#include <cvx/viz/image/view.hpp>

#include <QObject>
#include <QApplication>

#include "main_window.hpp"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv) ;

    MainWindow mwin ;
    mwin.show() ;

    app.exec() ;

}
