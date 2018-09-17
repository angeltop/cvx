#ifndef __KML_HPP__
#define __KML_HPP__

#include <vector>
#include <string>
#include <map>

// Simple KML DOM

class KmlFeature {

public:

    enum Type { Folder, Placemark, Document } ;

    typedef std::shared_ptr<KmlFeature> Ptr ;

    virtual Type type() const = 0 ;

    std::string id_, name_, description_, style_url_ ;
    bool visibility_, open_ ;

public:

    KmlFeature(): visibility_(true), open_(true) {}
} ;


class KmlFolder: public KmlFeature {
public:
    KmlFolder() {}
    Type type() const { return Folder ; }
    void add(const KmlFeature::Ptr &f) {
        items_.push_back(f) ;
    }

private:
    std::vector<KmlFeature::Ptr> items_ ;
};

struct KmlCoord {
    KmlCoord() = default ;
    KmlCoord(float lat, float lon, float ele = 0): lat_(lat), lon_(lon), ele_(ele) {}

    float lat_, lon_, ele_ ;
};

typedef std::vector<KmlCoord> KmlCoordList ;

class KmlGeometry {
public:
    typedef std::shared_ptr<KmlGeometry> Ptr ;

    enum Type { Point, LineString, LinearRing, Polygon, MultiGeometry } ;
    virtual Type type() const = 0 ;
} ;

class KmlPlacemark: public KmlFeature {
public:
    Type type() const { return Placemark ; }
    KmlGeometry::Ptr geometry_;
} ;

class KmlPoint: public KmlGeometry {
public:
    KmlPoint() = default ;
    KmlPoint(KmlCoord coord): pt_(coord) {}
    Type type() const { return Point ; }
    KmlCoord pt_ ;
};

class KmlLineString: public KmlGeometry {
public:

    Type type() const { return LineString ; }
    void add(const KmlCoord &coord) { pts_.push_back(coord) ; }

    KmlLineString() = default ;
    KmlCoordList pts_ ;
};

class KmlPolygon: public KmlGeometry {
public:

    Type type() const { return Polygon ; }

    KmlCoordList outer_ ;
    std::vector<KmlCoordList> inner_ ;
};


class KmlMultiGeometry: public KmlGeometry {
public:

    Type type() const { return MultiGeometry ; }

    std::vector<KmlGeometry::Ptr> geometries_ ;
};

class KmlStyleTrait {
public:
    enum Type { Icon, Line, Fill } ;

    virtual Type type() const = 0 ;
    typedef std::shared_ptr<KmlStyleTrait> Ptr ;

    int color_ ;
};

class KmlFillStyle: public KmlStyleTrait {
public:
    virtual Type type() const { return Fill ; }
};

class KmlLineStyle: public KmlStyleTrait {
public:
    float width_ ;
    virtual Type type() const { return Line ; }
};

class KmlIconStyle: public KmlStyleTrait {
public:
    float scale_, heading_, hs_x_, hs_y_ ;
    std::string href_ ;
    virtual Type type() const { return Icon ; }
};

class KmlStyleMap {
public:
    typedef std::shared_ptr<KmlStyleMap> Ptr ;

    void add(const std::string &key, const std::string &ref) {
        styles_.emplace(std::make_pair(key, ref)) ;
    }

private:
    std::map<std::string, std::string> styles_ ;
};

class KmlStyle {
public:
    std::shared_ptr<KmlLineStyle> line_style_ ;
    std::shared_ptr<KmlFillStyle> fill_style_ ;
    std::shared_ptr<KmlIconStyle> icon_style_ ;

    typedef std::shared_ptr<KmlStyle> Ptr ;
};

class KmlDocument: public KmlFeature {
public:
    KmlDocument() {}
    Type type() const { return Document ; }
    void add(const KmlFeature::Ptr &f) {
        items_.push_back(f) ;
    }
    void addStyle(const std::string &id, const KmlStyle::Ptr &style) {
        styles_.emplace(std::make_pair(id, style)) ;
    }

    void addStyleMap(const std::string &id, const KmlStyleMap::Ptr &style_map) {
        styles_map_.emplace(std::make_pair(id, style_map)) ;
    }


private:

    std::map<std::string, KmlStyle::Ptr> styles_ ;
    std::map<std::string, KmlStyleMap::Ptr> styles_map_ ;
    std::vector<KmlFeature::Ptr> items_ ;
};




#endif
