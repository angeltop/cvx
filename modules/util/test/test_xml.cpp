#include <cvx/util/misc/xml_sax_parser.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <stack>
#include <regex>

#include "kml.hpp"

using namespace std ;

static string trim(const string &src) {
    std::string::const_iterator it = src.begin();
    while (it != src.end() && isspace(*it))
       it++;

    std::string::const_reverse_iterator rit = src.rbegin();
    while (rit.base() != it && isspace(*rit))
       rit++;

    return std::string(it, rit.base());
}

class KmlParser: public XMLSAXParser {
public:
    KmlParser(istream &strm): XMLSAXParser(strm) {
        feature_ = root_ = KmlFeature::Ptr(new KmlDocument) ;
        in_inner_boundary_ = false ;
    }

    vector<KmlCoord> parseKmlCoordinates(const string &coord_str) {
        string text = coord_str + ' ' ;
        vector<KmlCoord> coords ;
        KmlCoord coord ;
        uint m = 0 ;
        float val ;
        string t ;

        coord.ele_ = 0 ;

        for( string::const_iterator it = text.begin() ; it != text.end() ;  ) {
            char c = *it ;
            if ( c != ',' && !isspace(c) ) {
                t += c ;
                ++it ;
            }
            else {
                val = stof(t) ;
                t.clear() ;
                switch (m) {
                    case 0: coord.lon_ = val ; m = 1 ; break ;
                    case 1: coord.lat_ = val ; m = 2 ; break ;
                    case 2: coord.ele_ = val ; m = 0 ; break ;
                }
                if ( isspace(c) ) {
                    coords.push_back(coord) ;
                    coord.ele_ = 0 ;
                    while ( it != text.end() && isspace(*it) ) ++it ;
                }
                else ++it ;
            }
        }

        return coords ;
    }

    int parseKmlColor(const string &text) {
        string sColor = trim(text_) ;

        while ( sColor.length()<8 )  sColor = "0" + sColor;

        int aa = stoi(sColor.substr(0, 2), 0, 16);
        int bb = stoi(sColor.substr(2, 2), 0, 16);
        int gg = stoi(sColor.substr(4, 2), 0, 16);
        int rr = stoi(sColor.substr(6, 2), 0, 16);

        int iColor = aa + (bb << 8) + (gg << 16) + (rr << 24) ;

        return iColor;
    }

    virtual void startElement(const string &qname, const AttributeList &attrs) {
        if ( qname == "Document" ) {
            feature_ = root_ ;
            feature_->id_ = attrs.get("id") ;
        } else if ( qname == "Folder" ) {
            KmlFeature::Ptr folder(new KmlFolder) ;

            if ( feature_ ) {
                if ( feature_->type() == KmlFeature::Folder ) {
                    std::shared_ptr<KmlFolder> parent = std::static_pointer_cast<KmlFolder>(feature_) ;
                    parent->add(folder) ;
                }
                else if ( feature_->type() == KmlFeature::Document ) {
                    std::shared_ptr<KmlDocument> parent = std::static_pointer_cast<KmlDocument>(feature_) ;
                    parent->add(folder) ;
                }
            }
            feature_ = folder ;
            feature_->id_ = attrs.get("id") ;
            features_.push(feature_) ;
        } else if ( qname == "Placemark" ) {
            KmlFeature::Ptr placemark(new KmlPlacemark) ;

            if ( feature_ ) {
                if ( feature_->type() == KmlFeature::Folder ) {
                    std::shared_ptr<KmlFolder> parent = std::static_pointer_cast<KmlFolder>(feature_) ;
                    parent->add(placemark) ;
                } else if ( feature_->type() == KmlFeature::Document ) {
                    std::shared_ptr<KmlDocument> parent = std::static_pointer_cast<KmlDocument>(feature_) ;
                    parent->add(placemark) ;
                }
            }
            feature_ = placemark ;
            feature_->id_ = attrs.get("id") ;
            features_.push(feature_) ;
        } else if ( qname == "Point" ) {
            geometry_.reset(new KmlPoint()) ;
            geometries_.push(geometry_) ;
        } else if ( qname == "LineString" ) {
            geometry_.reset(new KmlLineString()) ;
            geometries_.push(geometry_) ;
        } else if ( qname == "Polygon" ) {
            geometry_.reset(new KmlPolygon()) ;
            geometries_.push(geometry_) ;
        } else if ( qname == "MultiGeometry" ) {
            geometry_.reset(new KmlMultiGeometry()) ;
            geometries_.push(geometry_) ;
        }
        else if ( qname == "innerBoundaryIs" ) {
            in_inner_boundary_ = true ;
        }
        else if ( qname == "Style" ) {
            style_.reset(new KmlStyle()) ;
            string id = attrs.get("id") ;
            std::shared_ptr<KmlDocument> root = std::static_pointer_cast<KmlDocument>(root_) ;
            if ( !id.empty() ) root->addStyle(id, style_) ;
        } else if ( qname == "StyleMap" ) {
            style_map_.reset(new KmlStyleMap()) ;
            string id = attrs.get("id") ;
            std::shared_ptr<KmlDocument> root = std::static_pointer_cast<KmlDocument>(root_) ;
            if ( !id.empty() ) root->addStyleMap(id, style_map_) ;
        }  else if ( qname == "LineStyle" ) {
            style_->line_style_.reset(new KmlLineStyle) ;
            style_trait_ = style_->line_style_ ;
        } else if ( qname == "IconStyle" ) {
            style_->icon_style_.reset(new KmlIconStyle()) ;
            style_trait_ = style_->icon_style_ ;
        } else if ( qname == "PolyStyle" ) {
            style_->fill_style_.reset(new KmlFillStyle()) ;
            style_trait_ = style_->fill_style_ ;
        } else if ( qname == "hotSpot" ) {
            if ( style_ && style_->icon_style_ && style_trait_ == style_->icon_style_ ){
                 style_->icon_style_->hs_x_ = stof(trim(attrs.get("x", "0"))) ;
                 style_->icon_style_->hs_y_ = stof(trim(attrs.get("y", "0"))) ;
            }
        } else if ( qname == "heading" ) {
            if ( style_ && style_->icon_style_ && style_trait_ == style_->icon_style_ ){
                style_->icon_style_->heading_ = stof(trim(attrs.get("heading", "0"))) ;
            }
        } else if ( qname == "scale" ) {
            if ( style_ && style_->icon_style_ && style_trait_ == style_->icon_style_ ){
                style_->icon_style_->scale_ = stof(trim(attrs.get("scale", "0"))) ;
            }
        }

    }

    virtual void endElement(const std::string &qname) {
        if ( qname == "Document" ) {
             //Document is the root, nothing to do.
        } else if ( qname == "Folder" || qname == "Placemark" ) {
            features_.pop() ;
            if ( !features_.empty() ) feature_ = features_.top() ;
        } else if ( qname == "name" ) {
            feature_->name_ = text_ ;
        } else if ( qname == "description" ) {
            feature_->description_ = text_ ;
        } else if ( qname == "visibility" ) {
            feature_->visibility_ = ( text_ == "1" ) ;
        } else if ( qname == "open" ) {
            feature_->open_ = ( text_ == "1" ) ;
        } else if ( qname == "coordinates" ) {
            vector<KmlCoord> coords = parseKmlCoordinates(trim(text_)) ;

            if ( geometry_->type() == KmlGeometry::Point )  {
                std::shared_ptr<KmlPoint> p = std::dynamic_pointer_cast<KmlPoint>(geometry_) ;
                p->pt_ = coords[0] ;
            } else if ( geometry_->type() == KmlGeometry::LineString ) {
                std::shared_ptr<KmlLineString> p = std::dynamic_pointer_cast<KmlLineString>(geometry_) ;
                p->pts_ = coords ;
            } else if ( geometry_->type() == KmlGeometry::Polygon ) {
                std::shared_ptr<KmlPolygon> p = std::dynamic_pointer_cast<KmlPolygon>(geometry_) ;

                if ( in_inner_boundary_ ) {
                    p->inner_.emplace_back(coords) ;
                }
                else
                    p->outer_ = coords ;
            }

        } else if ( qname == "styleUrl" ) {
            string url = ( text_[0] == '#' ) ? text_.substr(1) : text_ ;
            if ( style_map_ ) {
                style_map_->add(style_key_, url) ;
            } else if ( feature_ ) {
                feature_->style_url_ = url ;
            }
        } else if ( qname == "href" ) {
            if ( style_ && style_->icon_style_ && style_trait_ == style_->icon_style_ ){
                 style_->icon_style_->href_ = text_ ;
            }
        } else if ( qname == "color" ) {
            if ( style_ && style_trait_ ) {
                 style_trait_->color_ = parseKmlColor(text_) ;
            }
        }  else if ( qname == "Style" ) {
            style_.reset() ;
            style_trait_.reset() ;
        } else if ( qname == "StyleMap" ) {
            style_map_.reset() ;
            style_key_.clear() ;
        }
        else if ( qname == "key" ) {
            style_key_ = text_ ;
        } else if ( qname == "innerBoundaryIs" ) {
            in_inner_boundary_ = false ;
        } else if ( qname == "Point" || qname == "LineString" || qname == "Polygon" || qname == "MultiGeometry" ) {
            geometries_.pop() ;
            KmlGeometry::Ptr parent ;
            if ( ! geometries_.empty() ) parent = geometries_.top() ;
            // check to see if current geometry has a multigeometry as parent
            if ( parent && parent->type() == KmlGeometry::MultiGeometry ) {
                std::shared_ptr<KmlMultiGeometry> g = std::static_pointer_cast<KmlMultiGeometry>(parent) ;
                g->geometries_.push_back(geometry_) ;
            }

            if ( !parent ) { // the stack is empty add top level geometry to placemark
                std::shared_ptr<KmlPlacemark> pm = std::static_pointer_cast<KmlPlacemark>(feature_) ;
                if ( pm ) pm->geometry_ = geometry_ ;
            }

            geometry_ = parent ;
        }

        text_.clear() ;
    }

    virtual void characters(const string &text) {
        text_ = text ;
    }

private:

    KmlFeature::Ptr feature_, root_ ;
    KmlGeometry::Ptr geometry_ ;
    KmlStyle::Ptr style_ ;
    KmlStyleMap::Ptr style_map_ ;
    KmlStyleTrait::Ptr style_trait_ ;
    stack<KmlFeature::Ptr> features_ ;
    stack<KmlGeometry::Ptr> geometries_ ;
    string text_, style_key_ ;
    bool in_inner_boundary_ ;
};

int main(int argc, char *argv[]) {

    ifstream strm("/home/malasiot/Downloads/berlin.kml") ;
  //  ifstream strm("/home/malasiot/Downloads/polygon-point.kml") ;
    KmlParser parser(strm) ;
    parser.parse() ;
    cout << "ok here" ;
}
