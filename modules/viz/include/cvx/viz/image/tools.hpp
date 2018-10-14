#ifndef __CVX_VIZ_IMAGE_TOOLS_HPP__
#define __CVX_VIZ_IMAGE_TOOLS_HPP__

#include <QAction>
#include <QGraphicsSceneMouseEvent>

class QRubberBand ;

namespace cvx { namespace viz {

class QImageTool ;

class QImageWidget ;

// abstract class for handling all dragging operations on image canvas
class QImageTool: public QObject
{
    Q_OBJECT

public:

    QImageTool(QObject *p = nullptr) ;
    virtual ~QImageTool();

    virtual void show(bool sh = true) {}

    //protected:

    friend class QImageView ;

    // should be overriden to perform per view initialization
    virtual void registerWithView(QImageWidget *v) = 0 ;

    // handles mouse-pressed event
    virtual void mousePressed(QGraphicsSceneMouseEvent *pevent) = 0 ;

    // handles mouse-released event
    virtual void mouseReleased(QGraphicsSceneMouseEvent *pevent) = 0 ;

    // handles mouse-move event
    virtual void mouseMoved(QGraphicsSceneMouseEvent *pevent) = 0 ;

signals:

    void showMessage(const QString &) ;
} ;

class QRectRBand ;

class QRectTool: public QImageTool
{
    Q_OBJECT

public:

    QRectTool(QObject *p) ;
    virtual ~QRectTool() ;

    void setRect(const QRectF &rect_) ;
    QRectF getRect() const ;

protected:

    friend class QImageView ;


    virtual void show(bool sh) ;

    virtual void registerWithView(QImageWidget *v) ;

    virtual void mousePressed(QGraphicsSceneMouseEvent *pevent) ;
    virtual void mouseReleased(QGraphicsSceneMouseEvent *pevent) ;
    virtual void mouseMoved(QGraphicsSceneMouseEvent *pevent) ;

    QRectRBand *rb_ ;

private:

    QPointF last_point_, p_initial_, p_final_;
    QImageWidget *view_ ;

    bool start_tracking_ ;
    bool edit_mode_ ;
    QRectF rect_ ;
    
    enum EditMotionType { UPPER_LEFT, UPPER_RIGHT, BOTTOM_LEFT,
                          BOTTOM_RIGHT, TOP_SIDE, BOTTOM_SIDE, LEFT_SIDE, RIGHT_SIDE, MOVE_RECT,
                          NONE } edit_motion_ ;
} ;

class QImageSamplingPopup ;


class QSamplingTool: public QImageTool
{
    Q_OBJECT

public:

    QSamplingTool(QObject *p) ;
    virtual ~QSamplingTool() ;

protected:

    friend class QImageView ;

    virtual void registerWithView(QImageWidget *v) ;
    virtual void mousePressed(QGraphicsSceneMouseEvent *pevent) ;
    virtual void mouseReleased(QGraphicsSceneMouseEvent *pevent) ;
    virtual void mouseMoved(QGraphicsSceneMouseEvent *pevent) ;

private:

    QImageSamplingPopup *popup_ ;

} ;

class QPolyRBand ;

class QPolygonTool: public QImageTool
{
    Q_OBJECT

public:
    QPolygonTool(QObject *p) ;
    virtual ~QPolygonTool() ;

    QPolygonF getPolygon() const ;
    void setPolygon(const QPolygonF &poly) ;

    void setPen(const QPen &pen) ;
    void setBrush(const QBrush &brush) ;

    void setLabelPen(const QPen &pen) ;
    void setLabelBrush(const QBrush &brush) ;
    void setLabelFont(const QFont &font) ;

    void drawLabels(bool draw = true) ;
    void drawClosed(bool draw = true) ;
    void setEditOnly(bool edit = true) ;

    void setMaxPoints(int max_pts = -1) {
        max_pts_ = max_pts ;
    }

    // override
    virtual void polygonChanged() {}

    virtual QString makeLabel(int i) const {
        return QString::number(i) ;
    }

protected:

    friend class QImageView ;

    virtual void show(bool sh) ;

    virtual void registerWithView(QImageWidget *v) ;
    virtual void mousePressed(QGraphicsSceneMouseEvent *pevent) ;
    virtual void mouseReleased(QGraphicsSceneMouseEvent *pevent) ;
    virtual void mouseMoved(QGraphicsSceneMouseEvent *pevent) ;

    QImageWidget *view_ ;

private:

    QPolyRBand *rb_ ;

    QPointF start_move_ ;
    QRubberBand *p_rz_ ;
    QPoint porigin_ ;
    QVector<int> selected_ ;

    bool start_tracking_ ;
    int  index_ ;
    bool edit_mode_ ;
    bool edit_only_ ;
    int rb_flags_ ;

    int max_pts_ = -1;
} ;

}}


#endif
