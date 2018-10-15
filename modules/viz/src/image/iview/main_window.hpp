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

class MainWindow: public cvx::viz::QImageView
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

    void onImageLoaded() override ;

private:

    QAction *openAct, *saveAct, *saveAsAct, *copyAct, *pasteAct,
            *exitAct, *separatorAct ;

    QAction *recentFileActs[10];

    QMenu *fileMenu, *editMenu ;
    QTabWidget *tabWidget ;

    QString curFile ;

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
