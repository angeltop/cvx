#ifndef _TTOOLS_H_
#define _TTOOLS_H_

#include <QAction>
#include <QGraphicsSceneMouseEvent>

class QRubberBand ;


namespace cvx { namespace viz {

class QImageTool ;

class  QImageToolCreator
{
public:

    virtual QImageTool *create() const = 0 ;
} ;

class QImageToolFactory
{
public:

    QImageToolFactory() {};

    QImageTool *createImageTool(const char *toolName) {
        QImageToolCreator *cr = clsMap.value(toolName, 0) ;
        if ( cr ) return cr->create() ;
        else return (QImageTool *)0 ;
    }

    QAction *findToolAction(const char *toolName) {
        QAction *ac = actionMap.value(toolName, 0) ;
        return ac ;
    }

    QByteArray toolName(QAction *act)
    {
        QByteArray k = actionMap.key(act, "") ;
        return k ;
    }

    void registerTool(const char *toolName, QImageToolCreator *cr, QAction *act)
    {
        clsMap[toolName] = cr ;
        if ( act ) actionMap[toolName] = act ;
    }

private:

    QMap<QByteArray, QImageToolCreator *> clsMap ;
    QMap<QByteArray, QAction *> actionMap ;

} ;


#define REGISTER_IMAGE_TOOL(classKey, className, act)\
    class QImageToolCreator_##className: public QImageToolCreator \
{\
    public:\
    QImageToolCreator_##className() { } \
    QImageTool *create() const { return new className ; }\
};\
    QImageToolCreator_##className *imageToolInstance_##className = \
    new QImageToolCreator_##className ;\
    toolFactory.registerTool(classKey, imageToolInstance_##className, act) ;



class QImageWidget ;

// abstract class for handling all dragging operations on image canvas
class QImageTool: public QObject
{
    Q_OBJECT

public:

    QImageTool(QObject *p =NULL) ;
    virtual ~QImageTool();

    virtual void show(bool sh=true) {} ;

    //protected:

    friend class QImageView ;

    // should be overriden to perform per view initialization
    virtual void Register(QImageWidget *v) = 0 ;

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

    void setRect(const QRectF &rect) ;
    QRectF getRect() const ;

protected:

    friend class QImageView ;


    virtual void show(bool sh) ;

    virtual void Register(QImageWidget *v) ;

    virtual void mousePressed(QGraphicsSceneMouseEvent *pevent) ;
    virtual void mouseReleased(QGraphicsSceneMouseEvent *pevent) ;
    virtual void mouseMoved(QGraphicsSceneMouseEvent *pevent) ;

    QRectRBand *rb ;

private:

    QPointF lastPoint, p_initial, p_final;
    QImageWidget *view ;

    bool startTracking ;
    bool editMode ;
    QRectF rect ;
    
    enum EditMotionType { UPPER_LEFT, UPPER_RIGHT, BOTTOM_LEFT,
                          BOTTOM_RIGHT, TOP_SIDE, BOTTOM_SIDE, LEFT_SIDE, RIGHT_SIDE, MOVE_RECT,
                          NONE } editMotion ;
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

    virtual void Register(QImageWidget *v) ;
    virtual void mousePressed(QGraphicsSceneMouseEvent *pevent) ;
    virtual void mouseReleased(QGraphicsSceneMouseEvent *pevent) ;
    virtual void mouseMoved(QGraphicsSceneMouseEvent *pevent) ;

private:

    QImageSamplingPopup *popup ;

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

    void drawLines(bool draw = true) ;
    void drawLabels(bool draw = true) ;
    void drawClosed(bool draw = true) ;
    void setEditOnly(bool edit = true) ;

    // override
    virtual void polygonChanged() {} ;

protected:

    friend class QImageView ;

    virtual void show(bool sh) ;

    virtual void Register(QImageWidget *v) ;
    virtual void mousePressed(QGraphicsSceneMouseEvent *pevent) ;
    virtual void mouseReleased(QGraphicsSceneMouseEvent *pevent) ;
    virtual void mouseMoved(QGraphicsSceneMouseEvent *pevent) ;

    QImageWidget *view ;

private:

    QPolyRBand *rb ;

    QPointF startMove ;
    QRubberBand *p_rz ;
    QPoint porigin ;
    QVector<int> selected ;

    bool startTracking ;
    int  index ;
    bool editMode ;
    bool editOnly ;
    int rbflags ;

} ;

}}


#endif
