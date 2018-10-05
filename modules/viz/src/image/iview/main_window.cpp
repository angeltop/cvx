#include "main_window.hpp"

#include <cvx/viz/image/view.hpp>

#include <QSettings>
#include <QFileInfo>
#include <QStatusBar>
#include <QApplication>
#include <QMimeData>
#include <QDragEnterEvent>
#include <QUrl>
#include <QSignalMapper>
#include <QClipboard>

#include <opencv2/highgui/highgui.hpp>

using cvx::viz::QImageView ;
using cvx::viz::QImageWidget ;

void MainWindow::createActions()
{
    openAct = new QAction(QIcon(":/images/open.png"), tr("&Open..."), this);
    openAct->setShortcut(tr("Ctrl+O"));
    openAct->setStatusTip(tr("Open an existing image"));
    connect(openAct, SIGNAL(triggered()), this, SLOT(open()));

    saveAct = new QAction(QIcon(":/images/save.png"), tr("&Save"), this);
    saveAct->setShortcut(tr("Ctrl+S"));
    saveAct->setStatusTip(tr("Save the image to disk"));
    connect(saveAct, SIGNAL(triggered()), this, SLOT(save()));

    saveAsAct = new QAction(tr("Save &As..."), this);
    saveAsAct->setStatusTip(tr("Save the image under a new name"));
    connect(saveAsAct, SIGNAL(triggered()), this, SLOT(saveAs()));

    separatorAct = new QAction(this);
    separatorAct->setSeparator(true);

    for (int i = 0; i < 10; ++i) {
        recentFileActs[i] = new QAction(this);
        recentFileActs[i]->setVisible(false);
        connect(recentFileActs[i], SIGNAL(triggered()),
                this, SLOT(openRecentFile()));
    }

    exitAct = new QAction(tr("E&xit"), this);
    exitAct->setShortcut(tr("Ctrl+Q"));
    exitAct->setStatusTip(tr("Exit the application"));
    connect(exitAct, SIGNAL(triggered()), qApp, SLOT(closeAllWindows()));

    copyAct = new QAction(QIcon(":/images/copy.png"), tr("&Copy"), this);
    copyAct->setShortcut(tr("Ctrl+C"));
    copyAct->setStatusTip(tr("Copy the current selection's contents to the "
                             "clipboard"));
    connect(copyAct, SIGNAL(triggered()), this, SLOT(copy()));

    pasteAct = new QAction(QIcon(":/images/paste.png"), tr("&Paste"), this);
    pasteAct->setShortcut(tr("Ctrl+V"));
    pasteAct->setStatusTip(tr("Paste the clipboard's contents into the current "
                              "selection"));
    connect(pasteAct, SIGNAL(triggered()), this, SLOT(paste()));
    pasteAct->setEnabled(false);

    startCamAct = new QAction(tr("Start Camera"), this);
    startCamAct->setStatusTip(tr("Start camera playback"));
    connect(startCamAct, SIGNAL(triggered()), this, SLOT(startCam()));
    startCamAct->setEnabled(true);

    stopCamAct = new QAction(tr("Stop Camera"), this);
    stopCamAct->setStatusTip(tr("Stop camera playback"));
    connect(stopCamAct, SIGNAL(triggered()), this, SLOT(stopCam()));
    stopCamAct->setEnabled(true);


}

void MainWindow::createMenus()
{
    fileMenu = menuBar()->addMenu(tr("&File"));

    fileMenu->addAction(openAct);
    fileMenu->addSeparator() ;

    fileMenu->addAction(saveAct);
    fileMenu->addAction(saveAsAct);

    connect(fileMenu, SIGNAL(aboutToShow()), this, SLOT(updateMenus())) ;

    for (int i = 0; i < 10; ++i)
        fileMenu->addAction(recentFileActs[i]);

    fileMenu->addSeparator();
    fileMenu->addAction(exitAct);

    updateRecentFileActions();

    editMenu = menuBar()->addMenu(tr("&Edit"));

    editMenu->addAction(copyAct);
    editMenu->addAction(pasteAct);
    editMenu->addSeparator() ;

    connect(editMenu, SIGNAL(aboutToShow()), this, SLOT(updateMenus())) ;

    cameraMenu = menuBar()->addMenu(tr("&Camera"));

    cameraMenu->addAction(startCamAct);
    cameraMenu->addAction(stopCamAct);

    connect(cameraMenu, SIGNAL(aboutToShow()), this, SLOT(updateMenus())) ;


}

void MainWindow::updateMenus()
{
    bool hasView =  imageView->hasImage() ;

    saveAct->setEnabled(hasView) ;
    saveAsAct->setEnabled(hasView) ;

    separatorAct->setVisible(hasView);
    copyAct->setEnabled(hasView);

    startCamAct->setEnabled(!cameraGrabber->isRunning()) ;
    stopCamAct->setEnabled(cameraGrabber->isRunning()) ;
}


void MainWindow::clipboardChanged( )
{
    QClipboard *clipboard = QApplication::clipboard() ;
    pasteAct->setEnabled(clipboard->mimeData()->hasImage()) ;
}


void MainWindow::updateRecentFileActions()
{
    QSettings settings;
    QStringList files = settings.value("GuiSettings/recentFileList").toStringList();

    int numRecentFiles = qMin(files.size(), (int)10);

    QStringList toDelete ;

    int k = 1 ;
    for (int i = 0; i < numRecentFiles; ++i)
    {
        if ( !QFileInfo(files[i]).exists() ) {
            toDelete.append(files[i]) ;
            continue ;
        }
        QString text = tr("&%1 %2").arg(k++).arg(QFileInfo(files[i]).fileName());
        recentFileActs[i]->setText(text);
        recentFileActs[i]->setData(files[i]);
        recentFileActs[i]->setVisible(true);
    }

    for (int j = numRecentFiles; j < 10; ++j)
        recentFileActs[j]->setVisible(false);

    if ( !toDelete.isEmpty() )
    {
        foreach(QString s, toDelete)
            files.removeAll(s) ;

        settings.setValue("GuiSettings/recentFileList", files) ;
    }

    separatorAct->setVisible(numRecentFiles > 0);
}


MainWindow::MainWindow()
{
    tabWidget = new QTabWidget(this) ;

    imageView = new QImageView(this) ;
    cameraView = new QImageView(this) ;

    tabWidget->addTab(imageView, "Image") ;
    tabWidget->addTab(cameraView, "Camera") ;

    connect(imageView, SIGNAL(imageLoaded(QString)), this, SLOT(setImageName(QString))) ;

    setCentralWidget(tabWidget);

    connect(QApplication::clipboard(), SIGNAL(dataChanged()), this, SLOT(clipboardChanged( ))) ;

    setWindowTitle(QCoreApplication::applicationName());
    setAcceptDrops(true) ;

    createActions() ;
    createMenus() ;

    readGuiSettings();

    cameraGrabber = new CameraGrabber() ;

    cameraGrabber->moveToThread(&cameraThread);
    connect(&cameraThread, SIGNAL(started()), cameraGrabber, SLOT(grab()));
    connect(cameraGrabber, SIGNAL(imageReady(QImage)), this, SLOT(showCameraFrame(QImage)));
    connect(cameraGrabber, SIGNAL(stopped()), &cameraThread, SLOT(quit()), Qt::DirectConnection);
}


void MainWindow::open()
{
    QPreviewFileOpenDialog dlg(this);

    QStringList filters ;
    filters << "Image Files (*.tif *.pbm *.pgm *.ppm *.png *.tga *.jpg *.bmp *.gif)" ;

    dlg.setNameFilters(filters) ;

    if ( dlg.exec() )
    {
        QStringList selFiles = dlg.selectedFiles() ;

        openFile(selFiles[0]) ;
    }

    updateRecentFileActions() ;
}

void MainWindow::openFile(const QString &fileName)
{
    if ( imageView->load(fileName) )
    {
        imageView->fitToWindow();
        setCurrentFile(fileName) ;
    }
}

void MainWindow::save()
{
    imageView->save() ;
}

void MainWindow::saveAs()
{
    imageView->saveAs() ;
}


void MainWindow::copy()
{

    QClipboard *clipboard = QApplication::clipboard() ;

    clipboard->clear() ;

    QMimeData *data = new QMimeData ;

    imageView->copy(data) ;

    clipboard->setMimeData(data) ;

}

void MainWindow::paste()
{
    static int imageCount = 1 ;

    QClipboard *clipboard = QApplication::clipboard() ;

    const QMimeData *data = clipboard->mimeData() ;

    if ( data->hasImage() )
    {

        QImage img = clipboard->image() ;

        imageView->setImage(QImageWidget::QImageToImage(img), QString("Image%1.tif").arg(imageCount++) ) ;
        imageView->fitToWindow() ;
    }

    clipboard->clear() ;
}

void MainWindow::setImageName(const QString &q)
{
    tabWidget->setTabText(0, QFileInfo(q).fileName());
    tabWidget->setTabToolTip(0, q);
}

void MainWindow::printConsoleMessage(const QString &msg)
{
    statusBar()->showMessage(msg, 2000);
}

void MainWindow::openRecentFile()
{
    QAction *action = qobject_cast<QAction *>(sender());
    if (action)
    {
        QString fileName = action->data().toString();

        openFile(fileName) ;

        updateRecentFileActions() ;
    }

}

void MainWindow::readGuiSettings()
{
    QSettings settings ;
    restoreState(settings.value("GuiSettings/mainWinState").toByteArray()) ;
    QPoint p = settings.value("GuiSettings/pos", QPoint(0, 0)).toPoint() ;
    QSize sz = settings.value("GuiSettings/size", QSize(1000, 800)).toSize() ;
    move(p) ;
    resize(sz) ;
}

void MainWindow::writeGuiSettings()
{
    QSettings settings ;
    settings.setValue("GuiSettings/mainWinState", saveState()) ;
    settings.setValue("GuiSettings/pos", pos()) ;
    settings.setValue("GuiSettings/size", size()) ;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    event->accept();

    writeGuiSettings();

    stopCam() ;

    QClipboard *clipboard = QApplication::clipboard() ;
    clipboard->clear() ;
    QCoreApplication::quit() ;
}

void MainWindow::setCurrentFile(const QString &fileName)
{
    curFile = QFileInfo(fileName).absoluteFilePath() ;

    QSettings settings ;
    QStringList files = settings.value("GuiSettings/recentFileList").toStringList();

    files.removeAll(fileName);
    files.prepend(fileName);
    while (files.size() > 10)
        files.removeLast();

    settings.setValue("GuiSettings/recentFileList", files);

    updateRecentFileActions() ;
}


void MainWindow::startCam()
{

    cameraThread.start();
}

void MainWindow::showCameraFrame(const QImage &im)
{
    cameraView->setImage(im) ;
}


void MainWindow::stopCam()
{
    if ( cameraGrabber->isRunning() )
        cameraGrabber->abort() ;
}

////////////////////////////////////////////////////////////////////

CameraGrabber::CameraGrabber():  abort_(false), is_running_(false)
{

}

void CameraGrabber::grab()
{

    cv::VideoCapture cap(0) ;

    while (cap.isOpened() )
    {
        mutex_.lock();
        bool abort = abort_;
        is_running_ = true ;
        mutex_.unlock();

        if ( abort ) break ;

        cv::Mat frame;
        cap >> frame; // get a new frame from camera

        emit imageReady(QImageWidget::imageToQImage(frame)) ;
    }

    mutex_.lock();
    is_running_ = false;
    abort_ = false ;
    mutex_.unlock();

    emit stopped() ;
}

void CameraGrabber::abort()
{
    mutex_.lock();
    abort_ = true ;
    mutex_.unlock();
}

////////////////////////////////////////////////////////////////////


QPreviewFileOpenDialog::QPreviewFileOpenDialog(MainWindow *par): QFileDialog(par)
{
    mw = par ;

    QSettings st ;

    showPreview = st.value("GuiSettings/FileDialog/showPreview", QVariant(true)).toBool() ;
    restoreState(st.value("GuiSettings/FileDialog/state").toByteArray()) ;

    setFileMode(QFileDialog::ExistingFiles) ;
    setResolveSymlinks(false) ;

    setDirectory(history().last()) ;

    QWidget *prw = new QWidget(this) ;
    prw->setAttribute(Qt::WA_DeleteOnClose) ;

    buttonPreview = new QCheckBox ;
    buttonPreview->setText("Preview") ;
    buttonPreview->setChecked(showPreview) ;

    QVBoxLayout *vbox = new QVBoxLayout ;

    prw->setLayout(vbox) ;

    preview = new QLabel(this) ;
    preview->setMaximumWidth(100) ;
    preview->setMinimumWidth(100) ;
    preview->setMargin(10) ;

    vbox->addWidget(preview, 0, Qt::AlignTop) ;
    vbox->addSpacing(20) ;
    vbox->addWidget(buttonPreview) ;
    vbox->addSpacing(10) ;

    connect(buttonPreview, SIGNAL(stateChanged(int)), this, SLOT(setPreview(int))) ;

    preview->setScaledContents(true) ;

    QLayout* l = layout();
    if (dynamic_cast<QVBoxLayout*>(l) != 0) {
        QVBoxLayout* v = dynamic_cast<QVBoxLayout*>(l);
        v->addWidget(preview);
    }
    if (dynamic_cast<QHBoxLayout*>(l) != 0) {
        //std::cout << "QFileDialog layout is a QHBoxLayout, must be
        // QVBoxLayout or QGridLayout" << std::endl;
        abort();
    }
    if (dynamic_cast<QGridLayout*>(l) != 0) {
        QGridLayout* grid = dynamic_cast<QGridLayout*>(l);
        const int numRows = grid->rowCount();
        const int numCols = grid->columnCount();
        grid->addWidget(prw, 1, numCols, Qt::AlignHCenter | Qt::AlignVCenter);
    }

    connect(this, SIGNAL(currentChanged(QString)), this, SLOT(fileSelectionChanged(QString))) ;

}

void QPreviewFileOpenDialog::setPreview(int flag)
{
    showPreview = ( flag == Qt::Checked ) ? true : false ;
    if ( !showPreview ) preview->hide() ;
    else preview->show() ;

    QSettings st ;
    st.setValue("GuiSettings/FileDialog/showPreview", showPreview) ;
}

void QPreviewFileOpenDialog::done(int r)
{
    QSettings st ;
    st.setValue("GuiSettings/FileDialog/state", saveState()) ;

    QFileDialog::done(r) ;
}


void QPreviewFileOpenDialog::fileSelectionChanged(const QString &file)
{
    if ( hasPreview() )

    if ( !hasPreview() || file.isNull() || !QFileInfo(file).isFile() ) return ;

    QPixmap *pixmap = NULL ;

    cv::Mat im = cv::imread((const char *)file.toUtf8(), -1);

    if ( im.data )
    {
        pixmap = QImageWidget::imageToPixmap(im) ;
        int w = pixmap->width(), h = pixmap->height() ;

        preview->setFixedSize(120, 120*h/w) ;
        preview->setPixmap(*pixmap) ;
    }
}
