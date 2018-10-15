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
    if ( !rb_ ) return ;

    rb_->show() ;
    QPointF point = mouseEvent->scenePos() ;

    start_tracking_ = true ;

    if ( !edit_mode_ ) {
        rect_ = QRectF(point, point) ;
        rb_->setRect(rect_) ;
    }
    else if ( edit_motion_ == NONE )
    {
        edit_mode_ = false ;
        rect_ = QRectF(point, point) ;
        rb_->setRect(rect_) ;
    }
    else last_point_ = point ;

}

void QRectTool::mouseReleased(QGraphicsSceneMouseEvent *event)
{
    if ( !rb_ ) return ;
    if (!edit_mode_) edit_mode_ = true ;


}


void QRectTool::mouseMoved(QGraphicsSceneMouseEvent *event)
{
    if ( !rb_  ) return ;


    QGraphicsView *vw = view_->scene()->views().front() ;
    
    QPointF point = event->scenePos() ;

    //QGraphicsPixmapItem *pitem = (QGraphicsPixmapItem *)vw->items()[0] ;

    qreal w = view_->scene()->width() ;
    qreal h = view_->scene()->height() ;

    point.rx() = qMin(w-1, point.x()) ;
    point.ry() = qMin(h-1, point.y()) ;
    point.rx() = qMax((qreal)0, point.x()) ;
    point.ry() = qMax((qreal)0, point.y()) ;

    if (edit_mode_ == false && ( event->buttons() & Qt::LeftButton ) )
    {
        rect_.setBottomRight(point) ;
        rb_->setRect(rect_) ;

        vw->ensureVisible(QRectF(point, point), 10, 10) ;
    }
    else if ( edit_mode_ == true && ( event->buttons() & Qt::LeftButton )  )
    {
        switch ( edit_motion_ )
        {
        case UPPER_LEFT:
            rect_.setTopLeft(point) ;
            break ;
        case UPPER_RIGHT:
            rect_.setTop(point.y()) ;
            rect_.setRight(point.x()) ;
            break ;
        case BOTTOM_LEFT:
            rect_.setBottom(point.y()) ;
            rect_.setLeft(point.x()) ;
            break ;
        case BOTTOM_RIGHT:
            rect_.setRight(point.x()) ;
            rect_.setBottom(point.y()) ;
            break ;
        case LEFT_SIDE:
            rect_.setLeft(point.x()) ;
            break ;
        case RIGHT_SIDE:
            rect_.setRight(point.x()) ;
            break ;
        case TOP_SIDE:
            rect_.setTop(point.y()) ;
            break ;
        case BOTTOM_SIDE:
            rect_.setBottom(point.y()) ;
            break ;
        case MOVE_RECT:
            rect_.translate(point - last_point_) ;
            break ;
        }

        if ( edit_motion_ != NONE ) rb_->setRect(rect_) ;
        last_point_ = point ;
        vw->ensureVisible(QRectF(point, point), 10, 10) ;

    }
    else
    {
        int idx  = rb_->whereIsPoint(point) ;

        for( int i=0 ; i<8 ; i++ )
        {
            rb_->getHandle(i)->setHighlighted(idx == i) ;
        }

        switch (idx)
        {
        case 0:
            edit_motion_ = UPPER_LEFT ; break ;
        case 1:
            edit_motion_ = TOP_SIDE ; break ;
        case 2:
            edit_motion_ = UPPER_RIGHT ; break ;
        case 3:
            edit_motion_ = RIGHT_SIDE ; break ;
        case 4:
            edit_motion_ = BOTTOM_RIGHT ; break ;
        case 5:
            edit_motion_ = BOTTOM_SIDE ; break ;
        case 6:
            edit_motion_ = BOTTOM_LEFT ; break ;
        case 7:
            edit_motion_ = LEFT_SIDE ; break ;
        case 8:
            edit_motion_ = MOVE_RECT ; break ;
        default:
            edit_motion_ = NONE ; break ;
        }
    }

    char s[80] ;

    QRect r = rect_.normalized().toRect() ;
    sprintf(s, "(%d, %d) -> (%d, %d) [%dx%d]", r.left(), r.top(),
            r.right(), r.bottom(), r.width(), r.height()) ;
    emit showMessage(s) ;

}



QRectTool::QRectTool(QObject *p): QImageTool(p)
{
    rb_ = NULL ;
    start_tracking_ = false ;
    edit_mode_ = false ;
    edit_motion_ = NONE ;
}

QRectTool::~QRectTool()
{
    delete rb_ ;
}

void QRectTool::registerWithView(QImageWidget *v)
{
    view_ = v ;
    rb_ = new QRectRBand(nullptr, view_->scene()) ;
    rb_->hide() ;

}

void QRectTool::activate() {
    rb_->show() ;
}

void QRectTool::deactivate() {
    rb_->hide() ;
}


QRectF QRectTool::getRect() const 
{
    if ( rb_ ) return rb_->getRect() ;
    else return QRectF() ;
}

void QRectTool::setRect(const QRectF &rect) 
{
    if ( rb_ ) rb_->setRect(rect) ;
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
    popup_ = nullptr ;
}

QSamplingTool::~QSamplingTool()
{

}

void QSamplingTool::registerWithView(QImageWidget *iv)
{
    popup_ = new QImageSamplingPopup(iv) ;
}

void QSamplingTool::mousePressed(QGraphicsSceneMouseEvent *mouseEvent)
{
    popup_->popup(mouseEvent->screenPos()) ;

}

void QSamplingTool::mouseMoved(QGraphicsSceneMouseEvent *) {
}

void QSamplingTool::mouseReleased(QGraphicsSceneMouseEvent *) {
}

///////////////////////////////////////////////////////////////////////////


QPolyRBand::QPolyRBand(QGraphicsItem *parent, unsigned nFlags):
    QGraphicsItem(parent)
{
    setZValue(1);

    pen_ = QPen(Qt::CustomDashLine) ;
    QVector<qreal> dashPattern ;
    dashPattern << 3 << 3 ;
    pen_.setDashPattern(dashPattern) ;
    pen_.setColor(QColor(255, 0, 0, 255)) ;
    pen_.setWidth(2) ;
    pen_.setCosmetic(true) ;

    brush_ = QBrush(QColor(0, 0, 125, 100), Qt::SolidPattern);

    label_pen_ = QPen(Qt::white) ;
    label_pen_.setCosmetic(true) ;
    label_pen_.setWidth(2) ;
    label_brush_ = QBrush(Qt::black) ;
    label_font_ = QFont("arial", 16) ;

}

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

    }

    updatePoly() ;
}

void QPolyRBand::setPen(const QPen &pen) {
    pen_ = pen ;
    update() ;
}

void QPolyRBand::setBrush(const QBrush &brush) {
    brush_ = brush ;
    update() ;
}

void QPolyRBand::setLabelPen(const QPen &pen) {
    label_pen_ = pen ;
    update() ;
}

void QPolyRBand::setLabelBrush(const QBrush &brush) {
    label_brush_ = brush ;
    update() ;
}

void QPolyRBand::setLabelFont(const QFont &font) {
    label_font_ = font ;
    update() ;
}


void QPolyRBand::updatePoly()
{
    prepareGeometryChange();
    
    int i ;
    for( i=0 ; i<poly.size() ; i++ )
        handles[i]->setPos(poly[i]) ;

    qreal pw = 1.0;

    brect = shape().controlPointRect().adjusted(-pw/2-40, -pw/2-40, pw+40, pw+40);

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

    painter->setBrush(brush_) ;
    painter->setBackgroundMode(Qt::TransparentMode) ;
    
    painter->setPen(pen_);

    if ( isClosed )
       painter->drawPolygon(poly) ;
    else
       painter->drawPolyline(poly);

    if ( drawText )
    {
        for( int i=0 ; i<poly.size() ; i++ )
        {
            QString text = labels_[i] ;

            QPointF pp = painter->transform().map(poly[i])  + QPointF(6, 10) ;

            painter->save() ;
            painter->resetTransform() ;

            QPainterPath tpath ;
            tpath.addText(pp, label_font_, text) ;
            painter->fillPath(tpath, label_brush_) ;
            painter->strokePath(tpath, label_pen_) ;

            painter->restore() ;

        }
    }

}

QGrabHandle *QPolyRBand::getHandle(int i)  {
    return handles[i] ;
}

void QPolyRBand::appendPoint(const QPointF &p, const QString &label) {
    poly.push_back(p) ;
    labels_.push_back(label) ;
    QGrabHandle *pHandle = new QGrabHandle( this) ;

    handles.push_back(pHandle) ;
    updatePoly() ;
}

void QPolyRBand::removePoints(int start, int nPts)
{
    poly.remove(start, nPts) ;
    labels_.remove(start, nPts) ;

    for(int i=start ; i<start + nPts ; i++ )
           delete handles[i] ;

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
    start_tracking_ = false ;
    edit_mode_ = false ;
    edit_only_ = false ;
    index_ = -1 ;
    p_rz_ = nullptr ;
    rb_ = new QPolyRBand((QGraphicsItem *)nullptr) ;
    rb_flags_ = 0 ;

}

QPolygonTool::~QPolygonTool()
{
    delete rb_ ;
}

void QPolygonTool::registerWithView(QImageWidget *v)
{
    view_ = v ;
    view_->scene()->addItem(rb_) ;
    rb_->hide() ;
}

void QPolygonTool::activate() {
    rb_->show() ;
}

void QPolygonTool::deactivate() {
    rb_->hide() ;
}


void QPolygonTool::mousePressed(QGraphicsSceneMouseEvent *mouseEvent)
{
    QPointF point = mouseEvent->scenePos() ;

    start_tracking_ = true ;

    int nPts = rb_->poly.size() ;

    if ( (mouseEvent->modifiers() & Qt::ShiftModifier) && !edit_only_ )
    {
        p_rz_ = new QRubberBand(QRubberBand::Rectangle, view_->viewport()) ;
        start_move_ = point ;
        porigin_ = view_->mapFromScene(point) ;
        p_rz_->setGeometry(QRect(porigin_, QSize()));
        p_rz_->show();

        for( int i=0 ; i<selected_.size() ; i++ )
        {
            index_ = selected_[i] ;
            rb_->getHandle(index_)->setHighlighted(false) ;
        }
        selected_.clear() ;
    }
    else if ( (mouseEvent->modifiers() & Qt::AltModifier) && !edit_only_ )
    {
        start_move_ = point ;
    }
    else if ( index_ == -1 && !edit_only_ )
    {
        QPointF lastPoint = point ;

        if ( nPts == 0 )
        {
            rb_->appendPoint(lastPoint, makeLabel(nPts++)) ;
            polygonChanged() ;
        }
        else if ( max_pts_ == -1 || nPts < max_pts_ )
        {
            QPointF startPoint = ( nPts == 0 ) ? lastPoint : rb_->poly[nPts-1] ;
            rb_->appendPoint(point, makeLabel(nPts++)) ;
            polygonChanged() ;
        }

        char s[80] ;

        sprintf(s, "new point: (%d, %d), total %d", (int)point.x(), (int)point.y(), rb_->poly.size()) ;

        emit showMessage(s) ;
    }
    else if ( (mouseEvent->modifiers() & Qt::ControlModifier) && !edit_only_ )
    {
        for(int i=0 ; i<selected_.size() ; i++ )
        {
            if ( selected_[i] == index_ )
            {
                selected_.remove(i) ;
                break ;
            }
        }

        if ( index_ == 0 && nPts == 1 )
        {
            rb_->removePoints(0) ;
            nPts -- ;
            polygonChanged() ;
        }
        else if ( index_ == 0 && ( !rb_->isClosed || nPts <= 2 ) )
        {
            rb_->removePoints(0) ;
            nPts -- ;
            polygonChanged() ;

        }
        else if ( index_ == nPts-1 && ( !rb_->isClosed || nPts<=2 ) )
        {
            rb_->removePoints(nPts-1) ;
            nPts -- ;
            polygonChanged() ;
        }
        else
        {
            rb_->removePoints(index_) ;
            nPts-- ;
            polygonChanged() ;

        }

        char s[80] ;

        sprintf(s, "delete point %d, total %d", index_, nPts) ;

        emit showMessage(s) ;
    }
    else
    {
        edit_mode_ = true ;
    }

}

void QPolygonTool::mouseReleased(QGraphicsSceneMouseEvent *mouseEvent)
{
    index_ = -1 ;

    edit_mode_ = false ;

    if ( p_rz_ )
    {
        p_rz_->hide();
        delete p_rz_ ;
        p_rz_ = nullptr ;

        QRectF rect = QRectF(mouseEvent->scenePos(), start_move_).normalized() ;

        for( int i=0 ; i<rb_->poly.size() ; i++ )
        {
            if ( rect.contains(rb_->poly[i]) )
            {
                selected_.append(i) ;
                rb_->getHandle(i)->setHighlighted() ;
            }
        }
    }
}

void QPolygonTool::mouseMoved(QGraphicsSceneMouseEvent *mouseEvent)
{
    QPointF point = mouseEvent->scenePos() ;
    QGraphicsView *vw = (QGraphicsView *)view_ ;

    qreal w = view_->scene()->width() ;
    qreal h = view_->scene()->height() ;

    point.rx() = qMin(w-1, point.x()) ;
    point.ry() = qMin(h-1, point.y()) ;
    point.rx() = qMax((qreal)0, point.x()) ;
    point.ry() = qMax((qreal)0, point.y()) ;

    unsigned nPts = rb_->poly.size() ;

    if ( !edit_mode_ ) index_ = -1 ;

    if ( (mouseEvent->modifiers() & Qt::ShiftModifier) && mouseEvent->buttons() & Qt::LeftButton )
    {
        p_rz_->setGeometry(QRect(porigin_, vw->mapFromScene(point)).normalized());
        vw->ensureVisible(QRectF(point, point), 10, 10) ;
    }
    else if ( (mouseEvent->modifiers() & Qt::AltModifier) && mouseEvent->buttons() & Qt::LeftButton )
    {
        QPointF resPoint = point ;

        if ( selected_.isEmpty() )
        {
            for(int i=0 ; i<nPts ; i++ )
            {
                rb_->poly[i].rx() += resPoint.x() - start_move_.x() ;
                rb_->poly[i].ry() += resPoint.y() - start_move_.y() ;
            }

        }
        else
        {
            for(int i=0 ; i<selected_.size() ; i++ )
            {
                int index = selected_[i] ;

                rb_->poly[index].rx() += resPoint.x() - start_move_.x() ;
                rb_->poly[index].ry() += resPoint.y() - start_move_.y() ;
            }
        }


        rb_->updatePoly() ;
        polygonChanged() ;

        vw->ensureVisible(QRectF(point, point), 10, 10) ;

        start_move_ = point ;
    }
    else if (edit_mode_ == false && ( mouseEvent->buttons() & Qt::LeftButton ) )
    {
        rb_->poly[nPts-1] = point ;
        rb_->updatePoly() ;
        polygonChanged() ;
        char s[80] ;

        sprintf(s, "new point: (%d, %d)", (int)point.x(), (int)point.y()) ;

        showMessage(s) ;

        vw->ensureVisible(QRectF(point, point), 10, 10) ;

    }
    else if ( edit_mode_ && ( mouseEvent->buttons() & Qt::LeftButton ))
    {
        rb_->poly[index_] = point ;
        rb_->updatePoly() ;
        polygonChanged() ;

        char s[80] ;

        sprintf(s, "move point: (%d, %d)", (int)point.x(), (int)point.y()) ;

        // cout << s << std::endl ;
        emit showMessage(s) ;
        vw->ensureVisible(QRectF(point, point), 10, 10) ;

    }
    else
    {
        index_ = rb_->whereIsPoint(point) ;

        if ( index_ >= 0 )
        {
            //      SetCursor(::LoadCursor(NULL, MAKEINTRESOURCE(IDC_SIZEALL))) ;
        }
    }


}

void QPolygonTool::drawLabels(bool draw) {
    rb_->showLabels(draw) ;
}

void QPolygonTool::drawClosed(bool draw) {
    rb_->setClosed(draw) ;
}

void QPolygonTool::setEditOnly(bool edit){
    edit_only_ = edit ;
}


QPolygonF QPolygonTool::getPolygon() const 
{
    if ( rb_ ) return rb_->getPolygon() ;
    else return QPolygonF() ;
}

void QPolygonTool::setPolygon(const QPolygonF &poly) 
{
    if ( rb_ ) rb_->setPolygon(poly) ;
}

void QPolygonTool::setPen(const QPen &pen)
{
    rb_->setPen(pen) ;
}

void QPolygonTool::setBrush(const QBrush &brush)
{
    rb_->setBrush(brush) ;
}

void QPolygonTool::setLabelPen(const QPen &pen) {
    rb_->setLabelPen(pen) ;
}

void QPolygonTool::setLabelBrush(const QBrush &brush) {
    rb_->setLabelBrush(brush) ;
}

void QPolygonTool::setLabelFont(const QFont &font) {
    rb_->setLabelFont(font) ;
}

////////////////////////////////////////////////////////////////////////////

QZoomInTool::QZoomInTool(QObject *p): QImageTool(p)
{
    view = nullptr ;
}

void QZoomInTool::registerWithView(QImageWidget *v)
{
    view = v ;
}

void QZoomInTool::mousePressed(QGraphicsSceneMouseEvent *mouseEvent) { view->zoomToPoint(mouseEvent->scenePos(), +1) ; }

/////////////////////////////////////////////////////////////////////////////

QZoomOutTool::QZoomOutTool(QObject *p): QImageTool(p)
{
    view = NULL ;
}

void QZoomOutTool::registerWithView(QImageWidget *v)
{
    view = v ;
}

void QZoomOutTool::mousePressed(QGraphicsSceneMouseEvent *mouseEvent) { view->zoomToPoint(mouseEvent->scenePos(), -1) ; }

/////////////////////////////////////////////////////////////////////////////

QZoomRectTool::~QZoomRectTool()
{
    delete p_rz ;
}

void QZoomRectTool::registerWithView(QImageWidget *iv)
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

void QPanTool::registerWithView(QImageWidget *v)
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

