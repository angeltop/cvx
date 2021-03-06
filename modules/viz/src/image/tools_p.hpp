#ifndef _TOOLS_P_H_
#define _TOOLS_P_H_

#include <cvx/viz/image/tools.hpp>

#include <QGraphicsItem>
#include <QLabel>
#include <QPen>
#include <QRectF>
#include <QApplication>
#include <QDesktopWidget>
#include <QMouseEvent>



namespace cvx { namespace viz {


class QImageWidget ;


class QGrabHandle : public QGraphicsItem
{
public:

    QGrabHandle(QGraphicsItem *parent);

    QRectF boundingRect() const;
    QPainterPath shape() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    void setPos(const QPointF &p) ;
    void setHighlighted(bool h = true) ;
    void setSmall(bool sm=true) ;

    QPointF pos ;

protected:

    QRectF sceneRect() const;

private:

    bool highlighted ;
    bool isSmall ;
} ;

class QRectRBand : public QGraphicsItemGroup
{
public:

    QRectRBand(QGraphicsItem *parent, QGraphicsScene *scene);
    ~QRectRBand();

    QRectF boundingRect() const;
    QPainterPath shape() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    void setRect(const QRectF &r) ;
    QRect getRect() const { return rect.toRect() ; }

    // -1: means outside, 0-7 means handle, 8 means inside
    int whereIsPoint(const QPointF &p) ;

    QGrabHandle *getHandle(int i) ;

private:

    friend class QRectTool ;
    void hideAll() ;
    void showAll() ;

    QRectF rect, brect ;
    QGrabHandle *handle[8] ;
};

class QImageSamplingPopup : public QLabel
{
    Q_OBJECT

public:

    QImageSamplingPopup(QImageWidget* parent);

    void popup( const QPoint &pos);

protected:

    virtual void mouseMoveEvent( QMouseEvent * ) override ;
    virtual void mouseReleaseEvent( QMouseEvent * ) override ;
    virtual void closeEvent( QCloseEvent * ) override ;

private:

    QImageWidget* popupParent;

};


class QPolyRBand : public QGraphicsItem
{
public:

    enum Flags { ClosedFlag = 0x0001, NoLinesFlag = 0x0002, NoTextFlag = 0x0004, PaintInteriorFlag = 0x0008 } ;

    QPolyRBand(QGraphicsItem *parent, unsigned nFlags = 0) ;

    QPolyRBand(QImageWidget *v, unsigned nFlags = 0) ;
    
    QPolygonF getPolygon() const ;
    void setPolygon(const QPolygonF &poly) ;

    void setPen(const QPen &pen) ;
    void setBrush(const QBrush &brush) ;

    void setLabelPen(const QPen &pen) ;
    void setLabelBrush(const QBrush &brush) ;
    void setLabelFont(const QFont &font) ;

    void setClosed(bool closed) { isClosed = closed ; update(); }
    void showLabels(bool show_labels) { drawText = show_labels ;  update(); }

    ~QPolyRBand() {}

    QRectF boundingRect() const;
    QPainterPath shape() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    QGrabHandle *getHandle(int i) ;
    void appendPoint(const QPointF &p, const QString &label) ;
    void removePoints(int start, int nPts = 1) ;

    int whereIsPoint(const QPointF &pt) ;

private:

    friend class QPolygonTool ;

    QPen pen_, label_pen_ ;
    QBrush brush_, label_brush_ ;
    QFont label_font_ ;


    QRectF brect ;
    QPolygonF poly ;
    QVector<QString> labels_ ;
    QVector<QGrabHandle *> handles ;

    bool isClosed = false ;
    bool multiLines = false ;
    bool drawText = true ;

    void updatePoly() ;
};


class QZoomInTool: public QImageTool
{
    Q_OBJECT

public:
    QZoomInTool(QObject *p) ;
    virtual ~QZoomInTool() {} ;

protected:

    friend class ImageWidget ;

    virtual void registerWithView(QImageWidget *v) ;
    virtual void mousePressed(QGraphicsSceneMouseEvent *) ;
    virtual void mouseReleased(QGraphicsSceneMouseEvent *) {}
    virtual void mouseMoved(QGraphicsSceneMouseEvent *) {}

private:

    QImageWidget *view ;

} ;


class QZoomOutTool: public QImageTool
{
    Q_OBJECT

public:
    QZoomOutTool(QObject *p) ;
    virtual ~QZoomOutTool() {} ;

protected:

    virtual void registerWithView(QImageWidget *v) ;
    virtual void mousePressed(QGraphicsSceneMouseEvent *) ;
    virtual void mouseReleased(QGraphicsSceneMouseEvent *) {}
    virtual void mouseMoved(QGraphicsSceneMouseEvent *) {}

private:

    QImageWidget *view ;

} ;


class QZoomRectTool: public QImageTool
{
    Q_OBJECT

public:
    QZoomRectTool(QObject *p) ;
    virtual ~QZoomRectTool()  ;

protected:

    friend class ImageWidget ;

    virtual void registerWithView(QImageWidget *v) ;
    virtual void mousePressed(QGraphicsSceneMouseEvent *) ;
    virtual void mouseReleased(QGraphicsSceneMouseEvent *) ;
    virtual void mouseMoved(QGraphicsSceneMouseEvent *) ;

private:

    QImageWidget *view ;
    QRubberBand *p_rz ;
    QPoint porigin ;
    QPointF porigins ;
} ;


class QPanTool: public QImageTool
{
    Q_OBJECT

public:
    QPanTool(QObject *p) ;
    virtual ~QPanTool()  ;

protected:

    friend class QImageWidget ;

    virtual void registerWithView(QImageWidget *v) ;
    virtual void mousePressed(QGraphicsSceneMouseEvent *) ;
    virtual void mouseReleased(QGraphicsSceneMouseEvent *) ;
    virtual void mouseMoved(QGraphicsSceneMouseEvent *) ;

private:

    QImageWidget *view ;

} ;


}}







#endif
