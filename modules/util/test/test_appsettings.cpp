#include <cvx/util/misc/application_settings.hpp>

#include <iostream>

using namespace std ;
using namespace cvx::util ;

struct GVector {
    vector<int> vals_ ;

    GVector() = default ;

    GVector(const std::initializer_list<int> &vals): vals_(vals) {}
    friend ostream &operator << (ostream &strm, const GVector &v)  {
        for( uint i = 0 ; i<v.vals_.size() ; i++ ) {
            if ( i > 0 ) strm << ',' ;
            strm << v.vals_[i] ;
        }
        return strm ;
    }

    friend istream &operator >> (istream &strm, GVector &v)  {

        while( strm.good() )
        {
            string substr;
            getline( strm, substr, ',' );
            v.vals_.emplace_back(stoi(substr));
        }
        return strm ;
    }
};


int main(int argc, const char *argv[]) {

    ApplicationSettings settings ;

    settings.set("image.threshold", (int)4) ;
    settings.set("image.matching.confidence", GVector{1, 2, 3, 4}) ;
    settings.setArray("image.testing.keys", vector<string>{string("heelo"), string("goodbye")}) ;

    settings.save("/tmp/oo.xml") ;
    ApplicationSettings rs ;
    rs.load("/tmp/oo.xml") ;

    vector<string> res = rs.getArray<string>("image.testing.keys") ;

    GVector v = rs.get<GVector>("image.matching.confidence") ;

    auto keys = rs.keys("image") ;
    auto sections = rs.sections("image") ;

    cout << "ok here" ;
}
