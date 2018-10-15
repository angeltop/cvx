#include <QApplication>
#include <QMainWindow>
#include <QAction>
#include <QToolBar>

#include <cvx/viz/image/view.hpp>
#include <cvx/viz/image/widget.hpp>

using namespace cvx::viz ;
using namespace std ;

class CustomTool: public QPolygonTool {
public:
    CustomTool(QImageView *parent): QPolygonTool(parent) {
        setMaxPoints(3) ;
        drawClosed() ;
        QPen pen(QBrush(Qt::green), 4);
        pen.setCosmetic(true) ;
        setPen(pen) ;
        setBrush(QBrush(QColor(255, 0, 0, 100))) ;

    }

    QString makeLabel(int i) const override {
        if ( i==0) return QString("A") ;
        else if ( i==1 ) return QString("B") ;
        else return QString("C") ;
    }

};

class View: public QImageView {
public:
    View(): QImageView(nullptr) {
        resize(1024, 768) ;

        addSampleTool() ;
        addRectTool() ;

        QAction *polyToolAct = new QAction(QIcon(":/images/polygon-tool.png"), "Polygon Tool", this);
        polyToolAct->setStatusTip("Select points");

        CustomTool *tool = new CustomTool(this) ;

        addTool(polyToolAct, tool) ;
    }

    void onImageLoaded() {
        fitToWindow() ;

        setWindowTitle(pathName) ;
    }

};

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    QDir directory("/home/malasiot/Downloads/");
    QStringList images = directory.entryList(QStringList() << "*.jpg" << "*.png",QDir::Files);
    QStringList imagePaths ;

    for ( const QString &rp: images )
        imagePaths.append(directory.path() + '/' + rp) ;

    View view ;
    view.show() ;
    view.setFilePaths(imagePaths) ;
    view.first() ;

    return app.exec();
}
