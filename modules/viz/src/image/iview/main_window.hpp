#include <QMainWindow>
#include <QAction>
#include <QMenuBar>
#include <QLabel>
#include <QFileDialog>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QThread>
#include <QMutex>

#include <cvx/viz/image/view.hpp>

class MainWindow: public QMainWindow
{
    Q_OBJECT
public:
    MainWindow() ;

protected:

    void createActions() ;
    void createMenus() ;
    void updateRecentFileActions();
    void writeGuiSettings();
    void readGuiSettings() ;

protected slots:

    void open() ;
    void save() ;
    void saveAs() ;
    void copy() ;
    void paste() ;
    void startCam() ;
    void stopCam() ;
    void showCameraFrame(const QImage &im);

    void setImageName(const QString &q) ;

    void printConsoleMessage(const QString &msg) ;

    void updateMenus();
    void clipboardChanged();
    void openRecentFile();

private:

    void openFile(const QString &file) ;
    void setCurrentFile(const QString &fileName);

    void closeEvent(QCloseEvent *event);

    friend class QPreviewFileOpenDialog ;

private:

    QAction *openAct, *saveAct, *saveAsAct, *copyAct, *pasteAct,
            *optionsAct, *exitAct, *separatorAct,
            *startCamAct, *stopCamAct ;

    QAction *recentFileActs[10];

    QMenu *fileMenu, *editMenu, *cameraMenu ;
    QTabWidget *tabWidget ;

    cvx::viz::QImageView *imageView, *cameraView ;

    QString curFile ;
    QThread cameraThread ;

    friend class CameraGrabber ;
    class CameraGrabber *cameraGrabber ;

};

class CameraGrabber : public QObject
{
    Q_OBJECT

public:

    CameraGrabber() ;

    bool isRunning() {
        QMutexLocker lock(&mutex_) ;
        return is_running_ ;
    }

public slots:

    void grab() ;
    void abort() ;

signals:

    void imageReady(const QImage &result);
    void stopped() ;

private:

    bool abort_, is_running_ ;
    QMutex mutex_;
};

class QPreviewFileOpenDialog: public QFileDialog
{
    Q_OBJECT

public:

    QPreviewFileOpenDialog(MainWindow *);

    bool hasPreview() const {
        return showPreview ;
    }

    QLabel *getPreview() const {
        return preview ;
    }

private slots:

    void fileSelectionChanged(const QString &files) ;

    void setPreview(int) ;

    void done(int) ;

    private:

    QLabel *preview ;
    QCheckBox *buttonPreview ;
    MainWindow *mw ;

    bool showPreview ;
} ;
