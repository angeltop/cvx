#include <cvx/viz/image/tools.hpp>
#include <cvx/viz/image/view.hpp>
#include <cvx/viz/image/widget.hpp>

#include "tools_p.hpp"

#include <QRubberBand>
#include <QStatusBar>
#include <QDebug>
#include <QMdiArea>

namespace cvx { namespace viz {

QImageTool::QImageTool(QObject *p): QObject(p) {}
QImageTool::~QImageTool() {}

///////////////////////////////////////////////////////////////////////
const int HRECT_SIZE = 7 ;
const int HRECT_SIZE_SMALL = 5 ;

QGrabHandle::QGrabHandle(QGraphicsItem *p): QGraphicsItem(p)
{
    setZValue(2);
    highlighted = false ;
    isSmall = false ;
    setCursor(Qt::CrossCursor) ;

}


void QGrabHandle::setPos(const QPointF &p)
{
    prepareGeometryChange();
    pos = p ;
}

void QGrabHandle::setHighlighted(bool h)
{
    if ( h != highlighted ) {
        highlighted = h ;
        update() ;
    }
}


void QGrabHandle::setSmall(bool h)
{
    if ( h != isSmall ) {
        isSmall = h ;
        update() ;
    }
}

QRectF QGrabHandle::boundingRect() const
{
    return sceneRect() ; //.adjusted(-20, -20, 20, 20) ;
}

QPainterPath QGrabHandle::shape() const
{
    QPainterPath path;
    path.addRect(sceneRect());
    return path;
}

QRectF QGrabHandle::sceneRect() const
{

    if ( scene()->views().isEmpty() ) return QRectF() ;

    int side = (isSmall) ? HRECT_SIZE_SMALL : HRECT_SIZE ;

    QSizeF sz = scene()->views()[0]->mapToScene(QRect(0, 0, side, side)).boundingRect().size() ;
    qreal sw = sz.width(), sh = sz.height();

    return QRectF(pos.x() - sw/2, pos.y() - sh/2, sw, sh) ;
}

void QGrabHandle::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *)
{
    painter->setPen(Qt::SolidLine);
    painter->setBackgroundMode(Qt::TransparentMode) ;
    
    if ( highlighted )
    {
        painter->setBrush(QBrush(QColor(0, 0, 125, 100), Qt::SolidPattern)) ;

    }
    else
    {
        painter->setBrush(QBrush(QColor(255, 0, 125, 100), Qt::SolidPattern)) ;
    }

    //painter->drawRect(sceneRect());
    painter->drawEllipse(sceneRect()) ;
}

////////////////////////////////////////////////////////////////////////////////////////////////




QRectRBand::QRectRBand(QGraphicsItem *parent, QGraphicsScene *sc): QGraphicsItemGroup(parent)
{
    setZValue(1);

    sc->addItem(this) ;
    int i ;
    for( i=0 ; i<8 ; i++ ) {
        handle[i] = new QGrabHandle(this) ;
        handle[i]->setZValue(2) ;
        addToGroup(handle[i]) ;
    }
}

QRectRBand::~QRectRBand()
{

}


void QRectRBand::setRect(const QRectF &r)
{
    prepareGeometryChange();
    rect = r ;
    
    qreal mx = (rect.left() + rect.right())/2 ;
    qreal my = (rect.top() + rect.bottom())/2 ;

    handle[0]->setPos(rect.topLeft()) ;
    handle[1]->setPos(QPointF(mx, rect.top())) ;
    handle[2]->setPos(rect.topRight()) ;
    handle[3]->setPos(QPointF(rect.right(), my)) ;
    handle[4]->setPos(rect.bottomRight()) ;
    handle[5]->setPos(QPointF(mx, rect.bottom())) ;
    handle[6]->setPos(rect.bottomLeft()) ;
    handle[7]->setPos(QPointF(rect.left(), my)) ;

    qreal pw = HRECT_SIZE;
    brect = shape().controlPointRect().adjusted(-pw/2, -pw/2, pw, pw);

    update() ;
}

QRectF QRectRBand::boundingRect() const
{
    return brect ;
}

QPainterPath QRectRBand::shape() const
{
    QPainterPath path;
    path.addRect(rect);
    return path;
}

void QRectRBand::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *)
{
    QPen pen(Qt::SolidLine) ;
    pen.setColor(QColor(255, 0, 0, 255)) ;
    //
    painter->setBrush(QBrush(QColor(0, 0, 125, 100), Qt::SolidPattern)) ;
    painter->setBackgroundMode(Qt::TransparentMode) ;
    
    painter->setPen(pen);
    painter->drawRect(rect.normalized());
}

int QRectRBand::whereIsPoint(const QPointF &p)
{
    QGraphicsItem *pItem = scene()->itemAt(p, QTransform()) ;

    int idx = -1 ;
    for( int i=0 ; i<8 ; i++ )
    {
        if ( pItem == handle[i] )
        {
            idx = i ;
            break ;
        }
    }

    if ( idx == -1 && pItem == this ) return 8 ;
    else return idx ;
}

QGrabHandle *QRectRBand::getHandle(int i) 
{
    return handle[i] ;
}

void QRectRBand::hideAll() 
{
    for(int i=0 ; i<8 ; i++ )
        handle[i]->hide() ;
    hide() ;
}

void QRectRBand::showAll() 
{
    for(int i=0 ; i<8 ; i++ )
        handle[i]->show() ;
    show() ;

}

///////////////////////////////////////////////////////////////////////////////////////

void QRectTool::mousePressed(QGraphicsSceneMouseEvent *mouseEvent)
{
    if ( !rb ) return ;

    rb->show() ;
    QPointF point = mouseEvent->scenePos() ;

    startTracking = true ;

    if ( !editMode ) {
        rect = QRectF(point, point) ;
        rb->setRect(rect) ;
    }
    else if ( editMotion == NONE )
    {
        editMode = false ;
        rect = QRectF(point, point) ;
        rb->setRect(rect) ;
    }
    else lastPoint = point ;

}

void QRectTool::mouseReleased(QGraphicsSceneMouseEvent *event)
{
    if ( !rb ) return ;
    if (!editMode) editMode = true ;


}


void QRectTool::mouseMoved(QGraphicsSceneMouseEvent *event)
{
    if ( !rb  ) return ;


    QGraphicsView *vw = view->scene()->views().front() ;
    
    QPointF point = event->scenePos() ;

    //QGraphicsPixmapItem *pitem = (QGraphicsPixmapItem *)vw->items()[0] ;

    qreal w = view->scene()->width() ;
    qreal h = view->scene()->height() ;

    point.rx() = qMin(w-1, point.x()) ;
    point.ry() = qMin(h-1, point.y()) ;
    point.rx() = qMax((qreal)0, point.x()) ;
    point.ry() = qMax((qreal)0, point.y()) ;

    if (editMode == false && ( event->buttons() & Qt::LeftButton ) )
    {
        rect.setBottomRight(point) ;
        rb->setRect(rect) ;

        vw->ensureVisible(QRectF(point, point), 10, 10) ;
    }
    else if ( editMode == true && ( event->buttons() & Qt::LeftButton )  )
    {
        switch ( editMotion )
        {
        case UPPER_LEFT:
            rect.setTopLeft(point) ;
            break ;
        case UPPER_RIGHT:
            rect.setTop(point.y()) ;
            rect.setRight(point.x()) ;
            break ;
        case BOTTOM_LEFT:
            rect.setBottom(point.y()) ;
            rect.setLeft(point.x()) ;
            break ;
        case BOTTOM_RIGHT:
            rect.setRight(point.x()) ;
            rect.setBottom(point.y()) ;
            break ;
        case LEFT_SIDE:
            rect.setLeft(point.x()) ;
            break ;
        case RIGHT_SIDE:
            rect.setRight(point.x()) ;
            break ;
        case TOP_SIDE:
            rect.setTop(point.y()) ;
            break ;
        case BOTTOM_SIDE:
            rect.setBottom(point.y()) ;
            break ;
        case MOVE_RECT:
            rect.translate(point - lastPoint) ;
            break ;
        }

        if ( editMotion != NONE ) rb->setRect(rect) ;
        lastPoint = point ;
        vw->ensureVisible(QRectF(point, point), 10, 10) ;

    }
    else
    {
        int idx  = rb->whereIsPoint(point) ;

        for( int i=0 ; i<8 ; i++ )
        {
            rb->getHandle(i)->setHighlighted(idx == i) ;
        }

        switch (idx)
        {
        case 0:
            editMotion = UPPER_LEFT ; break ;
        case 1:
            editMotion = TOP_SIDE ; break ;
        case 2:
            editMotion = UPPER_RIGHT ; break ;
        case 3:
            editMotion = RIGHT_SIDE ; break ;
        case 4:
            editMotion = BOTTOM_RIGHT ; break ;
        case 5:
            editMotion = BOTTOM_SIDE ; break ;
        case 6:
            editMotion = BOTTOM_LEFT ; break ;
        case 7:
            editMotion = LEFT_SIDE ; break ;
        case 8:
            editMotion = MOVE_RECT ; break ;
        default:
            editMotion = NONE ; break ;
        }
    }

    char s[80] ;

    QRect r = rect.normalized().toRect() ;
    sprintf(s, "(%d, %d) -> (%d, %d) [%dx%d]", r.left(), r.top(),
            r.right(), r.bottom(), r.width(), r.height()) ;
    emit showMessage(s) ;

}



QRectTool::QRectTool(QObject *p): QImageTool(p)
{
    rb = NULL ;
    startTracking = false ;
    editMode = false ;
    editMotion = NONE ;
}

QRectTool::~QRectTool()
{
    delete rb ;
}

void QRectTool::Register(QImageWidget *v)
{
    view = v ;
    rb = new QRectRBand(0, view->scene()) ;
    rb->hide() ;

}

void QRectTool::show(bool sh)
{
    if ( rb )
    {
        if ( sh ) rb->show() ;
        else rb->hide() ;
    }
}

QRectF QRectTool::getRect() const 
{
    if ( rb ) return rb->getRect() ;
    else return QRectF() ;
}

void QRectTool::setRect(const QRectF &rect) 
{
    if ( rb ) rb->setRect(rect) ;
}
/////////////////////////////////////////////////////////////////////////////////////////////

#define FP_SIZE_W 60
#define FP_SIZE_H 40

QImageSamplingPopup::QImageSamplingPopup( QImageWidget *iv ):
    QLabel( iv->viewport(), Qt::Popup ) {
    popupParent = iv ;
    setFrameStyle( QFrame::Box );
    setAlignment( Qt::AlignCenter );
    setWindowOpacity(0.5) ;
    resize(FP_SIZE_W,FP_SIZE_H);

    setMouseTracking( false );
}

void QImageSamplingPopup::mouseMoveEvent( QMouseEvent * e){

    QString s;

    QGraphicsView *w = (QGraphicsView *)popupParent ;

    QImageWidget *iv = popupParent ;

    int dw = QApplication::desktop()->width() ;
    int dh = QApplication::desktop()->height() ;

    QPoint gp = e->globalPos() ;
    QRect drect(0, 0, dw-1, dh-1) ;

    if ( drect.contains(gp + QPoint(FP_SIZE_W,FP_SIZE_H) + QPoint(5, 5)) )
    {
        move(gp + QPoint(5, 5)) ;
    }
    else if ( drect.contains(gp - QPoint(FP_SIZE_W,FP_SIZE_H) - QPoint(5, 5) )  )
    {
        move(gp - QPoint(FP_SIZE_W,FP_SIZE_H) - QPoint(5, 5)) ;
    }

    QPoint lp = w->mapFromGlobal(gp) ;
    QPointF pp = w->mapToScene(lp) ;
    if ( iv->hasImage() ) setText(iv->sampleImage(iv->image(), pp)) ;

    if ( !text().isEmpty() ) w->ensureVisible(QRectF(pp, pp), 10, 10) ;
    

    
}

void QImageSamplingPopup::mouseReleaseEvent( QMouseEvent * e){

    close();
}

void QImageSamplingPopup::closeEvent( QCloseEvent *e ){
    e->accept();
    if (!popupParent) return ;
}

void QImageSamplingPopup::popup(const QPoint &gp) {
    int dw = QApplication::desktop()->width() ;
    int dh = QApplication::desktop()->height() ;

    //QPoint gp = e->globalPos() ;
    QRect drect(0, 0, dw-1, dh-1) ;

    if ( drect.contains(gp + QPoint(FP_SIZE_W,FP_SIZE_H) + QPoint(5, 5)) )
    {
        move(gp + QPoint(5, 5)) ;
    }
    else if ( drect.contains(gp - QPoint(FP_SIZE_W,FP_SIZE_H) - QPoint(5, 5) )  )
    {
        move(gp - QPoint(FP_SIZE_W,FP_SIZE_H) - QPoint(5, 5)) ;
    }

    QGraphicsView *w = (QGraphicsView *)popupParent ;

    QImageWidget *iv = popupParent ;

    QPoint lp = w->mapFromGlobal(gp) ;
    QPointF pp = w->mapToScene(lp) ;
    
    if ( iv->hasImage() ) setText(iv->sampleImage(iv->image(), pp)) ;

    show();

}

/////////////////////////////////////////////////////////////////////////////////

QSamplingTool::QSamplingTool(QObject *p): QImageTool(p)
{
    popup = NULL ;
}

QSamplingTool::~QSamplingTool()
{

}

void QSamplingTool::Register(QImageWidget *iv)
{
    popup = new QImageSamplingPopup(iv) ;
}

void QSamplingTool::mousePressed(QGraphicsSceneMouseEvent *mouseEvent)
{
    popup->popup(mouseEvent->screenPos()) ;

}

void QSamplingTool::mouseMoved(QGraphicsSceneMouseEvent *mouseEvent)
{

}

void QSamplingTool::mouseReleased(QGraphicsSceneMouseEvent *mouseEvent)
{

}

///////////////////////////////////////////////////////////////////////////


QPolyRBand::QPolyRBand(QGraphicsItem *parent, QGraphicsScene *scene, unsigned nFlags): 
    QGraphicsItem(parent)
{
    setZValue(1);

    isClosed = nFlags & ClosedFlag ;
    drawLines = !( nFlags & NoLinesFlag ) ;
    drawText = !( nFlags & NoTextFlag ) != 0 ;
    fillInterior = isClosed && ( nFlags & PaintInteriorFlag ) ;

    pen = QPen(Qt::CustomDashLine) ;
    QVector<qreal> dashPattern ;
    dashPattern << 3 << 3 ;
    pen.setDashPattern(dashPattern) ;
    pen.setColor(QColor(255, 0, 0, 255)) ;
}

void QPolyRBand::setMode(unsigned nFlags)
{
    isClosed = nFlags & ClosedFlag ;
    drawLines = !( nFlags & NoLinesFlag ) ;
    drawText = !( nFlags & NoTextFlag ) != 0 ;
    fillInterior = isClosed && ( nFlags & PaintInteriorFlag ) ;

    update() ;

}

/*
void QPolyRBand::animate()
{
  static int offset = 0 ;
  offset = (offset + 1)%6 ;
  pen.setDashOffset(offset);
  update() ;
}
*/



void QPolyRBand::setPolygon(const QPolygonF &ptlist)
{
    for(int i=0 ; i<handles.size() ; i++ ) {
        delete handles[i] ;
    }
    poly.clear() ;
    handles.clear() ;

    unsigned n = ptlist.size() ;
    for(int i=0 ; i<n ; i++ ) {
        poly.push_back(ptlist[i]) ;
        QGrabHandle *pHandle = new QGrabHandle( this ) ;
        pHandle->setPos(poly[i]) ;
        handles.push_back(pHandle) ;
        scene()->addItem(pHandle) ;
    }

    updatePoly() ;
}

void QPolyRBand::updatePoly()
{
    prepareGeometryChange();
    
    int i ;
    for( i=0 ; i<poly.size() ; i++ )
        handles[i]->setPos(poly[i]) ;

    qreal pw = 1.0;

    brect = shape().controlPointRect().adjusted(-pw/2-20, -pw/2-20, pw+20, pw+20);

    update() ;
}

QPolygonF QPolyRBand::getPolygon() const 
{
    return poly ;
}


QRectF QPolyRBand::boundingRect() const
{
    return brect ;
}

QPainterPath QPolyRBand::shape() const
{
    QPainterPath path;
    path.addPolygon(poly);
    return path;
}

void QPolyRBand::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *)
{

    painter->setBrush(QBrush(QColor(0, 0, 125, 100), Qt::SolidPattern)) ;
    painter->setBackgroundMode(Qt::TransparentMode) ;
    
    painter->setPen(pen);


    if ( drawLines )
    {
        if ( isClosed )
            painter->drawPolygon(poly) ;
        else
            painter->drawPolyline(poly);
    }

    if ( drawText )
    {
        for( int i=0 ; i<poly.size() ; i++ )
        {
            char s[10] ;
            sprintf(s, "%d", i) ;

            QPointF pp = painter->transform().map(poly[i]) ;
            painter->save() ;
            painter->resetTransform() ;
            painter->setPen(Qt::black) ;
            painter->drawText(pp + QPointF(6, 10), QString(s)) ;
            painter->setPen(Qt::yellow) ;
            painter->drawText(pp + QPointF(5, 10), QString(s)) ;
            painter->restore() ;
        }
    }

}

QGrabHandle *QPolyRBand::getHandle(int i)  {
    return handles[i] ;
}

void QPolyRBand::appendPoint(const QPointF &p) {
    poly.push_back(p) ;
    QGrabHandle *pHandle = new QGrabHandle( this) ;
    scene()->addItem(pHandle) ;
    handles.push_back(pHandle) ;
    updatePoly() ;
}

void QPolyRBand::removePoints(int start, int nPts)
{
    poly.remove(start, nPts) ;

    for(int i=start ; i<start + nPts ; i++ )
    {
        delete handles[i] ;
    }

    handles.remove(start, nPts) ;

    updatePoly() ;
}

int QPolyRBand::whereIsPoint(const QPointF &pt) 
{
    QGraphicsScene *sc = scene() ;
    QGraphicsItem *pItem = scene()->itemAt(pt, QTransform()) ;

    int idx = -1 ;
    for( int i=0 ; i<poly.size() ; i++ ) {
        if ( pItem == handles[i] ) {
            idx = i ;
            break ;
        }
    }
    
    return idx ;


}

/////////////////////////////////////////////////////////////////////////////////////////

QPolygonTool::QPolygonTool(QObject *p): QImageTool(p)
{
    startTracking = false ;
    editMode = false ;
    editOnly = false ;
    index = -1 ;
    p_rz = NULL ;
    rb = NULL ;
    rbflags = 0 ;

}

QPolygonTool::~QPolygonTool()
{
    delete rb ;
}

void QPolygonTool::Register(QImageWidget *v)
{
    view = v ;
    rb = new QPolyRBand(0, view->scene()) ;
    rb->setMode(rbflags) ;
    rb->hide() ;
}

void QPolygonTool::show(bool sh)
{
    if ( rb )
    {
        if ( sh ) rb->show() ;
        else rb->hide() ;
    }
}

void QPolygonTool::mousePressed(QGraphicsSceneMouseEvent *mouseEvent)
{
    QPointF point = mouseEvent->scenePos() ;

    startTracking = true ;

    int nPts = rb->poly.size() ;

    if ( (mouseEvent->modifiers() & Qt::ShiftModifier) && !editOnly )
    {
        p_rz = new QRubberBand(QRubberBand::Rectangle, view->viewport()) ;
        startMove = point ;
        porigin = view->mapFromScene(point) ;
        p_rz->setGeometry(QRect(porigin, QSize()));
        p_rz->show();

        for( int i=0 ; i<selected.size() ; i++ )
        {
            index = selected[i] ;
            rb->getHandle(index)->setHighlighted(false) ;
        }
        selected.clear() ;
    }
    else if ( (mouseEvent->modifiers() & Qt::AltModifier) && !editOnly )
    {
        startMove = point ;
    }
    else if ( index == -1 && !editOnly )
    {
        QPointF lastPoint = point ;

        if ( nPts == 0 )
        {
            rb->appendPoint(lastPoint) ;
            nPts ++ ;
            polygonChanged() ;
        }
        else
        {
            QPointF startPoint = ( nPts == 0 ) ? lastPoint : rb->poly[nPts-1] ;
            rb->appendPoint(point) ; nPts ++ ;
            polygonChanged() ;
        }

        char s[80] ;

        sprintf(s, "new point: (%d, %d), total %d", (int)point.x(), (int)point.y(), rb->poly.size()) ;

        emit showMessage(s) ;
    }
    else if ( (mouseEvent->modifiers() & Qt::ControlModifier) && !editOnly )
    {
        for(int i=0 ; i<selected.size() ; i++ )
        {
            if ( selected[i] == index )
            {
                selected.remove(i) ;
                break ;
            }
        }

        if ( index == 0 && nPts == 1 )
        {
            rb->removePoints(0) ;
            nPts -- ;
            polygonChanged() ;
        }
        else if ( index == 0 && ( !rb->isClosed || nPts <= 2 ) )
        {
            rb->removePoints(0) ;
            nPts -- ;
            polygonChanged() ;

        }
        else if ( index == nPts-1 && ( !rb->isClosed || nPts<=2 ) )
        {
            rb->removePoints(nPts-1) ;
            nPts -- ;
            polygonChanged() ;
        }
        else
        {
            rb->removePoints(index) ;
            nPts-- ;
            polygonChanged() ;

        }

        char s[80] ;

        sprintf(s, "delete point %d, total %d", index, nPts) ;

        emit showMessage(s) ;
    }
    else
    {
        editMode = true ;
    }

}

void QPolygonTool::mouseReleased(QGraphicsSceneMouseEvent *mouseEvent)
{
    index = -1 ;

    editMode = false ;

    if ( p_rz )
    {
        p_rz->hide();
        delete p_rz ;
        p_rz = NULL ;

        QRectF rect = QRectF(mouseEvent->scenePos(), startMove).normalized() ;

        for( int i=0 ; i<rb->poly.size() ; i++ )
        {
            if ( rect.contains(rb->poly[i]) )
            {
                selected.append(i) ;
                rb->getHandle(i)->setHighlighted() ;
            }
        }
    }
}

void QPolygonTool::mouseMoved(QGraphicsSceneMouseEvent *mouseEvent)
{
    QPointF point = mouseEvent->scenePos() ;
    QGraphicsView *vw = (QGraphicsView *)view ;

    qreal w = view->scene()->width() ;
    qreal h = view->scene()->height() ;

    point.rx() = qMin(w-1, point.x()) ;
    point.ry() = qMin(h-1, point.y()) ;
    point.rx() = qMax((qreal)0, point.x()) ;
    point.ry() = qMax((qreal)0, point.y()) ;

    unsigned nPts = rb->poly.size() ;

    if ( !editMode ) index = -1 ;

    if ( (mouseEvent->modifiers() & Qt::ShiftModifier) && mouseEvent->buttons() & Qt::LeftButton )
    {
        p_rz->setGeometry(QRect(porigin, vw->mapFromScene(point)).normalized());
        vw->ensureVisible(QRectF(point, point), 10, 10) ;
    }
    else if ( (mouseEvent->modifiers() & Qt::AltModifier) && mouseEvent->buttons() & Qt::LeftButton )
    {
        QPointF resPoint = point ;

        if ( selected.isEmpty() )
        {
            for(int i=0 ; i<nPts ; i++ )
            {
                rb->poly[i].rx() += resPoint.x() - startMove.x() ;
                rb->poly[i].ry() += resPoint.y() - startMove.y() ;
            }

        }
        else
        {
            for(int i=0 ; i<selected.size() ; i++ )
            {
                int index = selected[i] ;

                rb->poly[index].rx() += resPoint.x() - startMove.x() ;
                rb->poly[index].ry() += resPoint.y() - startMove.y() ;
            }
        }


        rb->updatePoly() ;
        polygonChanged() ;

        vw->ensureVisible(QRectF(point, point), 10, 10) ;

        startMove = point ;
    }
    else if (editMode == false && ( mouseEvent->buttons() & Qt::LeftButton ) )
    {
        rb->poly[nPts-1] = point ;
        rb->updatePoly() ;
        polygonChanged() ;
        char s[80] ;

        sprintf(s, "new point: (%d, %d)", (int)point.x(), (int)point.y()) ;

        showMessage(s) ;

        vw->ensureVisible(QRectF(point, point), 10, 10) ;

    }
    else if ( editMode && ( mouseEvent->buttons() & Qt::LeftButton ))
    {
        rb->poly[index] = point ;
        rb->updatePoly() ;
        polygonChanged() ;

        char s[80] ;

        sprintf(s, "move point: (%d, %d)", (int)point.x(), (int)point.y()) ;

        // cout << s << std::endl ;
        emit showMessage(s) ;
        vw->ensureVisible(QRectF(point, point), 10, 10) ;

    }
    else
    {
        index = rb->whereIsPoint(point) ;

        if ( index >= 0 )
        {
            //      SetCursor(::LoadCursor(NULL, MAKEINTRESOURCE(IDC_SIZEALL))) ;
        }
    }


}


void QPolygonTool::drawLines(bool draw)
{
    if ( !draw ) rbflags |= QPolyRBand::NoLinesFlag ;
    else rbflags &= ~QPolyRBand::NoLinesFlag ;
    if ( rb ) rb->setMode(rbflags) ;
}

void QPolygonTool::drawLabels(bool draw)
{
    if ( !draw ) rbflags |= QPolyRBand::NoTextFlag ;
    else rbflags &= ~QPolyRBand::NoTextFlag ;
    if ( rb ) rb->setMode(rbflags) ;
}

void QPolygonTool::drawClosed(bool draw)
{
    if ( draw ) rbflags |= QPolyRBand::ClosedFlag ;
    else rbflags &= ~QPolyRBand::ClosedFlag ;
    if ( rb ) rb->setMode(rbflags) ;
}

void QPolygonTool::setEditOnly(bool edit) 
{
    editOnly = edit ;
}

QPolygonF QPolygonTool::getPolygon() const 
{
    if ( rb ) return rb->getPolygon() ;
    else return QPolygonF() ;
}

void QPolygonTool::setPolygon(const QPolygonF &poly) 
{
    if ( rb ) rb->setPolygon(poly) ;
}
////////////////////////////////////////////////////////////////////////////

QZoomInTool::QZoomInTool(QObject *p): QImageTool(p)
{
    view = NULL ;
}

void QZoomInTool::Register(QImageWidget *v)
{
    view = v ;
}

void QZoomInTool::mousePressed(QGraphicsSceneMouseEvent *mouseEvent) { view->zoomToPoint(mouseEvent->scenePos(), +1) ; }

/////////////////////////////////////////////////////////////////////////////

QZoomOutTool::QZoomOutTool(QObject *p): QImageTool(p)
{
    view = NULL ;
}

void QZoomOutTool::Register(QImageWidget *v)
{
    view = v ;
}

void QZoomOutTool::mousePressed(QGraphicsSceneMouseEvent *mouseEvent) { view->zoomToPoint(mouseEvent->scenePos(), -1) ; }

/////////////////////////////////////////////////////////////////////////////

QZoomRectTool::~QZoomRectTool()
{
    delete p_rz ;
}

void QZoomRectTool::Register(QImageWidget *iv)
{
    view = iv ;

}

void QZoomRectTool::mouseMoved(QGraphicsSceneMouseEvent *mouseEvent)
{
    if ( mouseEvent->buttons() & Qt::LeftButton )
    {
        p_rz->setGeometry(QRect(porigin, view->mapFromScene(mouseEvent->scenePos())).normalized());
        view->ensureVisible(QRectF(mouseEvent->scenePos(), mouseEvent->scenePos()), 10, 10) ;
    }
}

void QZoomRectTool::mousePressed(QGraphicsSceneMouseEvent *mouseEvent)
{
    p_rz = new QRubberBand(QRubberBand::Rectangle, view->viewport()) ;
    porigins = mouseEvent->scenePos() ;
    porigin = view->mapFromScene(porigins) ;
    p_rz->setGeometry(QRect(porigin, QSize()));
    p_rz->show();
}

void QZoomRectTool::mouseReleased(QGraphicsSceneMouseEvent *mouseEvent) 
{
    p_rz->hide();
    delete p_rz ;

    QRectF rz(porigins, mouseEvent->scenePos()) ;

    view->zoomToRect(rz) ;
}

QZoomRectTool::QZoomRectTool(QObject *p): QImageTool()
{
    p_rz = NULL ;
    view = NULL ;
}

////////////////////////////////////////////////////////////////////////////////////

QPanTool::QPanTool(QObject *p): QImageTool(p)
{
    view = NULL ;
}

void QPanTool::Register(QImageWidget *v)
{
    view = v ;

}

void QPanTool::mousePressed(QGraphicsSceneMouseEvent *mouseEvent) 
{ 
    view->setDragMode(QGraphicsView::ScrollHandDrag) ;

}

void QPanTool::mouseReleased(QGraphicsSceneMouseEvent *mouseEvent) 
{ 
    view->setDragMode(QGraphicsView::NoDrag) ;
}

void QPanTool::mouseMoved(QGraphicsSceneMouseEvent *mouseEvent) 
{ 


}

QPanTool::~QPanTool()
{
}

}
}

//////////////////////////////////////////////////////////////////////////////////////

