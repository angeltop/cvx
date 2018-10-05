#ifndef _QIMAGEVIEW_H_
#define _QIMAGEVIEW_H_

#include <cvx/viz/image/widget.hpp>

#include <QLabel>
#include <QDirIterator>
#include <QLineEdit>
#include <QMainWindow>
#include <QComboBox>

#include <opencv2/opencv.hpp>

class QtColorButton ;

namespace cvx { namespace viz {


    // This is a basic image viewer. It provides a toolbar that may be used for interaction with the image (toy may toggle it by pressing T).
    // It allows browsing the images in the current folder by using PgUp, PgDown, Home, End buttons.

    class QImageView: public QMainWindow
    {
        Q_OBJECT

    public:

        QImageView(QWidget *parent) ;

        QImageWidget *imageWidget() const {
            return pWidget ;
        }

        cv::Mat image() const {
            return pWidget->image() ;
        }

        QString fileName() const {
            return pathName ;
        }

        void setFileName(const QString &name) {
            pathName = name ;
            emit imageLoaded(name) ;
        }

        void setImage(const cv::Mat &im, const QString &name)
        {
            pWidget->setImage(im) ;
            setFileName(name) ;
        }

        void setImage(const QImage &img) {
            pWidget->setImage(img) ;
        }

        bool hasImage() const { return pWidget->hasImage() ; }

        bool save() ;
        bool saveAs() ;

        bool copy(QMimeData *mime) ;

        void previous() ;
        void next() ;
        void first() ;
        void last() ;

    public slots:

        void fitToWindow() ;
        void fitToRect(const QRect &rect) ;

        bool load(const QString &file) ;

    signals:

        // emitted when the filename or zoom factor changed
        void imageLoaded(const QString &fname) ;

        // connect this to the mainwindow status bar to display messages
        void statusMessage(const QString &msg) ;

    private slots:

        void updateZoomCombo(int idx) ;

        void setZoom() ;
        void zoomIn() ;
        void zoomOut() ;
        void zoomRect() ;
        void zoomFit() ;
        void setTool() ;

    protected:

        void registerTool(QAction *act, QImageTool *tool) ;


    private:

        bool saveFile(const QString &fileNameSave ) ;

        QString strippedName(const QString &fullFileName);


        bool eventFilter(QObject *o, QEvent *e) ;
        void closeEvent(QCloseEvent *e) ;

        void createToolBar() ;
        void createActions() ;
        void createTools() ;

        QMainWindow *pContainer ;
        QToolBar *imageToolBar ;
        QImageWidget *pWidget ;
        QString pathName ;
        QStringList dirEntries ;
        int idx ;
        QDirIterator *it ;

        QComboBox *zoomCombo ;
        QAction *zoomInAct, *zoomOutAct, *zoomRectAct, *zoomFitAct ;
        QAction *panToolAct, *rectToolAct, *polyToolAct, *polyToolClosedAct ;
        QAction *polyToolOpenAct, *polyToolLinesAct,  *polyToolPointsAct, *polyToolNumbersAct ;
        QAction *sampleToolAct, *zoomConAct ;
        QActionGroup *toolGroupAct ;

        QMap<QAction *, QImageTool *>  tools ;


    } ;

} }

#endif
