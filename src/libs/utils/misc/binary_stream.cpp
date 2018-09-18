#include <cvx/util/misc/binary_stream.hpp>


#include <cassert>
#include <iterator>
#include <istream>
#include <ostream>
#include <string>

#include <stdexcept>

#ifdef __CHAR_BIT__
#if __CHAR_BIT__ != 8
#error "unsupported char size"
#endif
#else
#ifdef CHAR_BIT
#if CHAR_BIT != 8
#error "unsupported char size"
#endif
#endif
#endif

enum
{
    O32_LITTLE_ENDIAN = 0x03020100ul,
    O32_BIG_ENDIAN = 0x00010203ul,
    O32_PDP_ENDIAN = 0x01000302ul
};

static const union { unsigned char bytes[4]; uint32_t value; } o32_host_order =
{ { 0, 1, 2, 3 } };

static void byte_swap_32(uint32_t &data)
{
    union u {uint32_t v; uint8_t c[4];};
    u un, vn;
    un.v = data ;
    vn.c[0]=un.c[3];
    vn.c[1]=un.c[2];
    vn.c[2]=un.c[1];
    vn.c[3]=un.c[0];
    data = vn.v ;
}

static void byte_swap_64(uint64_t &data)
{
    union u {uint64_t v; uint8_t c[8];};
    u un, vn;
    un.v = data ;
    vn.c[0]=un.c[7];
    vn.c[1]=un.c[6];
    vn.c[2]=un.c[5];
    vn.c[3]=un.c[4];
    vn.c[4]=un.c[3];
    vn.c[5]=un.c[2];
    vn.c[6]=un.c[1];
    vn.c[7]=un.c[0];
    data = vn.v ;
}

static void byte_swap_16(uint16_t &nValue)
{
    nValue = ((( nValue>> 8)) | (nValue << 8));
}



using namespace std ;

namespace cvx {


IBinaryStream::IBinaryStream(std::istream &strm, bool format_little_endian): the_istream_(strm)
{
    bool platform_is_little_endian_ = ( o32_host_order.value == 0x03020100ul ) ;
    compatible_endianess_ =  ( platform_is_little_endian_ == format_little_endian ) ;
}

OBinaryStream::OBinaryStream(std::ostream &strm, bool format_little_endian):
    the_ostream_(strm)
{
    bool platform_is_little_endian_ = ( o32_host_order.value == 0x03020100ul ) ;
    compatible_endianess_ =  ( platform_is_little_endian_ == format_little_endian ) ;
}

void OBinaryStream::write_bytes(const char *data, size_t sz) {
    assert( the_ostream_ ) ;
    the_ostream_.write(data, sz);
}


void IBinaryStream::read_bytes(char *data, size_t sz) {
    assert( the_istream_ );
    the_istream_.read(data, sz);
}

void OBinaryStream::write(const string& str) {
    assert( the_ostream_ ) ;
    uint32_t len = str.length();

    write(len);
    write_bytes(str.c_str(), len);
}

void OBinaryStream::write(const char *str) {
    assert( the_ostream_ ) ;
    uint32_t len = strlen(str);

    write(len);
    write_bytes(str, len);
}

void OBinaryStream::write_v(uint64_t uv) {
    assert( the_ostream_ ) ;

    while ( uv > 0x7F ) {
        the_ostream_.put((static_cast<uint8_t>(uv) & 0x7F) | 0x80);
        uv >>= 7 ;
    }

    the_ostream_.put(static_cast<uint8_t>(uv) & 0x7F) ;
}

void OBinaryStream::write_v(int64_t uv) {
    write_v((uint64_t)((uv << 1) ^ (uv >> 63))) ;
}

void OBinaryStream::write_v(uint32_t uv) {
    assert( the_ostream_ ) ;

    while ( uv > 0x7F ) {
        the_ostream_.put((static_cast<uint8_t>(uv) & 0x7F) | 0x80);
        uv >>= 7 ;
    }

    the_ostream_.put(static_cast<uint8_t>(uv) & 0x7F) ;
}

void OBinaryStream::write_v(int32_t uv) {
    write_v((uint32_t)((uv << 1) ^ (uv >> 31))) ;
}


void IBinaryStream::read_v(uint64_t &uv) {
    assert( the_istream_ ) ;

    uv = UINT64_C(0);
    unsigned int bits = 0;
    uint32_t b ;

    while ( 1 )
    {
        char c ;
        the_istream_.get(c) ;
        b = c ;

        uv |= ( b & 0x7F ) << bits;

        if ( !(b & 0x80) ) break ;

        bits += 7 ;

        if ( bits > 63 ) throw runtime_error("Variable length integer is too long") ;
    }

}

void IBinaryStream::read_v(uint32_t &uv) {
    assert( the_istream_ ) ;

    uv = UINT32_C(0);
    unsigned int bits = 0;
    uint32_t b ;

    while ( 1 )
    {

        char c ;
        the_istream_.get(c) ;
        b = c ;

        uv |= ( b & 0x7F ) << bits;

        if ( !( b & 0x80 ) ) break ;

        bits += 7 ;

        if ( bits > 31 ) throw runtime_error("Variable length integer is too long") ;
    }

}

void IBinaryStream::read_v(int64_t &uv) {
    uint64_t v ;
    read_v(v) ;
    uv = (v >> 1) ^ -static_cast<int64_t>(v & 1) ;
}

void IBinaryStream::read_v(int32_t &uv) {
    uint32_t v ;
    read_v(v) ;
    uv = (v >> 1) ^ -static_cast<int32_t>(v & 1) ;
}

void IBinaryStream::read(string& str) {
    assert( the_istream_ ) ;
    uint32_t count = 0;

    read(count) ;

    str.reserve(count) ;

    std::istreambuf_iterator<char> it(the_istream_) ;

    for( int i=0 ; i<count ; i++ ) str.push_back(*it++) ;
}


void OBinaryStream::write(bool i) {
    write((uint8_t)i) ;
}

void OBinaryStream::write(int8_t i) {
    assert( the_ostream_ ) ;
    the_ostream_.write((const char *)&i, 1) ;
}

void OBinaryStream::write(uint8_t i) {
    assert( the_ostream_ ) ;
    the_ostream_.write((const char *)&i, 1) ;
}

void OBinaryStream::write(int16_t i) {
    write((uint16_t)i) ;
}

void OBinaryStream::write(uint16_t i)
{
    assert( the_ostream_ ) ;

    if ( compatible_endianess_ )
        the_ostream_.write((const char *)&i, 2) ;
    else {
        byte_swap_16(i) ;
        the_ostream_.write((const char *)&i, 2) ;
    }
}

void OBinaryStream::write(int32_t i) {
    write((uint32_t)i) ;
}

void OBinaryStream::write(uint32_t i) {
    assert( the_ostream_ ) ;

    if ( compatible_endianess_ )
        the_ostream_.write((const char *)&i, 4) ;
    else {
        byte_swap_32(i) ;
        the_ostream_.write((const char *)&i, 4) ;
    }
}

void OBinaryStream::write(int64_t i) {
    write((uint64_t)i) ;
}

void OBinaryStream::write(uint64_t i) {
    assert( the_ostream_ ) ;

    if ( compatible_endianess_ )
        the_ostream_.write((const char *)&i, 8) ;
    else {
        byte_swap_64(i) ;
        the_ostream_.write((const char *)&i, 8) ;
    }
}

void OBinaryStream::write(float val) {
    union {float f; uint32_t i;} ;
    f = val ;
    write(i) ;
}

void OBinaryStream::write(double val) {
    union {double f; uint64_t i;} ;
    f = val ;
    write(i) ;
}

void OBinaryStream::write(const double *t, size_t n) {
    assert( the_ostream_ ) ;

    if ( compatible_endianess_ )
        the_ostream_.write((const char *)t, n * 8) ;
    else {
        uint64_t *p = (uint64_t *)t ;
        for(int i=0 ; i<n ; i++)
        {
            uint64_t s = *p++ ;
            byte_swap_64(s) ;
            the_ostream_.write((const char *)&s, 8) ;
        }
    }
}

void OBinaryStream::write(const float *t, size_t n) {
    assert( the_ostream_ ) ;

    if ( compatible_endianess_ )
        the_ostream_.write((const char *)t, n * 4) ;
    else {
        uint32_t *p = (uint32_t *)t ;
        for(int i=0 ; i<n ; i++)
        {
            uint32_t s = *p++ ;
            byte_swap_32(s) ;
            the_ostream_.write((const char *)&s, 4) ;
        }
    }
}

void OBinaryStream::write(const uint8_t *t, size_t n) {
    assert( the_ostream_ ) ;

    the_ostream_.write((const char *)t, n) ;
}

void OBinaryStream::write(const int8_t *t, size_t n) {
    write((uint8_t *)t, n) ;
}

void OBinaryStream::write(const uint16_t *t, size_t n) {
    assert( the_ostream_ ) ;

    if ( compatible_endianess_ )
        the_ostream_.write((const char *)t, n * 2) ;
    else {
        const uint16_t *p = t ;
        for(int i=0 ; i<n ; i++)
        {
            uint16_t s = *p++ ;
            byte_swap_16(s) ;
            the_ostream_.write((const char *)&s, 2) ;
        }
    }
}

void OBinaryStream::write(const int16_t *t, size_t n) {
    write((uint16_t *)t, n) ;
}

void OBinaryStream::write(const uint32_t *t, size_t n) {
    assert( the_ostream_ ) ;

    if ( compatible_endianess_ )
        the_ostream_.write((const char *)t, n * 4) ;
    else {
        const uint32_t *p = t ;
        for(int i=0 ; i<n ; i++)
        {
            uint32_t s = *p++ ;
            byte_swap_32(s) ;
            the_ostream_.write((const char *)&s, 4) ;
        }
    }
}

void OBinaryStream::write(const int32_t *t, size_t n) {
    write((uint32_t *)t, n) ;
}

void OBinaryStream::write(const uint64_t *t, size_t n) {
    assert( the_ostream_ ) ;

    if ( compatible_endianess_ )
        the_ostream_.write((const char *)t, n * 8) ;
    else {
        const uint64_t *p = t ;
        for(int i=0 ; i<n ; i++)
        {
            uint64_t s = *p++ ;
            byte_swap_64(s) ;
            the_ostream_.write((const char *)&s, 8) ;
        }
    }
}

void OBinaryStream::write(const int64_t *t, size_t n) {
    write((uint64_t *)t, n) ;
}

// Read Operations

void IBinaryStream::read(bool &i) {
    assert( the_istream_ ) ;

    char s ;
    the_istream_.get(s) ;
    i = s ;
}


void IBinaryStream::read(uint8_t &i) {
    assert( the_istream_ ) ;

    char s ;
    the_istream_.get(s) ;
    i = (uint8_t)s ;
}

void IBinaryStream::read(int8_t &i) {
    read((uint8_t &)i) ;
}


void IBinaryStream::read(uint16_t &i) {
    assert( the_istream_ ) ;

    if ( compatible_endianess_ )
        the_istream_.read((char *)&i, 2) ;
    else {
        the_istream_.read((char *)&i, 2) ;
        byte_swap_16(i) ;
    }
}

void IBinaryStream::read(int16_t &i)  {
    read((uint16_t &)i) ;
}


void IBinaryStream::read(uint32_t &i) {
    assert( the_istream_ ) ;

    if ( compatible_endianess_ )
        the_istream_.read((char *)&i, 4) ;
    else {
        the_istream_.read((char *)&i, 4) ;
        byte_swap_32(i) ;
    }
}

void IBinaryStream::read(int32_t &i)  {
    read((uint32_t &)i) ;
}


void IBinaryStream::read(uint64_t &i) {
    assert( the_istream_ ) ;

    if ( compatible_endianess_ )
        the_istream_.read((char *)&i, 8) ;
    else {
        the_istream_.read((char *)&i, 8) ;
        byte_swap_64(i) ;
    }
}

void IBinaryStream::read(int64_t &i)  {
    read((uint64_t &)i) ;
}

void IBinaryStream::read(float &val) {
    read((uint32_t &)val) ;
}

void IBinaryStream::read(double &val) {
    read((uint64_t &)val) ;
}

void IBinaryStream::read(double *t, size_t n) {
    assert(the_istream_) ;

    if ( compatible_endianess_ )
        the_istream_.read((char *)t, n * 8) ;
    else {
        uint64_t *p = (uint64_t *)t ;
        for(int i=0 ; i<n ; i++)
        {
            the_istream_.read((char *)p, 8) ;
            byte_swap_64(*p++) ;
        }
    }
}

void IBinaryStream::read(float *t, size_t n) {
    assert(the_istream_) ;

    if ( compatible_endianess_ )
        the_istream_.read((char *)t, n * 4) ;
    else {
        uint32_t *p = (uint32_t *)t ;
        for(int i=0 ; i<n ; i++)
        {
            the_istream_.read((char *)p, 4) ;
            byte_swap_32(*p++) ;
        }
    }
}

void IBinaryStream::read(uint8_t *t, size_t n) {
    assert(the_istream_) ;

    the_istream_.read((char *)t, n) ;
}

void IBinaryStream::read(int8_t *t, size_t n) {
    read((uint8_t *)t, n) ;
}

void IBinaryStream::read(uint16_t *t, size_t n) {
    assert(the_istream_) ;

    if ( compatible_endianess_ )
        the_istream_.read((char *)t, n * 2) ;
    else {
        uint16_t *p = t ;
        for(int i=0 ; i<n ; i++)
        {
            the_istream_.read((char *)p, 2) ;
            byte_swap_16(*p++) ;
        }
    }
}

void IBinaryStream::read(int16_t *t, size_t n) {
    read((uint16_t *)t, n) ;
}

void IBinaryStream::read(uint32_t *t, size_t n) {
    assert(the_istream_) ;

    if ( compatible_endianess_ )
        the_istream_.read((char *)t, n * 4) ;
    else {
        uint32_t *p = t ;
        for(int i=0 ; i<n ; i++)
        {
            the_istream_.read((char *)p, 4) ;
            byte_swap_32(*p++) ;
        }
    }
}

void IBinaryStream::read(int32_t *t, size_t n) {
    read((uint32_t *)t, n) ;
}

void IBinaryStream::read(uint64_t *t, size_t n) {
    assert(the_istream_) ;

    if ( compatible_endianess_ )
        the_istream_.read((char *)t, n * 8) ;
    else {
        uint64_t *p = t ;
        for(int i=0 ; i<n ; i++)
        {
            the_istream_.read((char *)p, 8) ;
            byte_swap_64(*p++) ;
        }
    }
}

void IBinaryStream::read(int64_t *t, size_t n) {
    read((uint64_t *)t, n) ;
}


void OBinaryStream::write(const cv::Mat &m) {
    size_t elem_size = m.elemSize();
    size_t elem_type = m.type();

    write((int)m.cols) ;
    write((int)m.rows) ;
    write(elem_size);
    write(elem_type);

    const size_t data_size = m.cols * m.rows * elem_size;
    write_bytes((const char *)m.ptr(), data_size) ;
}


void IBinaryStream::read(cv::Mat &m)
{
    int cols, rows;
    size_t elem_size, elem_type;

    read(cols) ; read(rows) ;
    read(elem_size) ; read(elem_type) ;

    m.create(rows, cols, elem_type);

    size_t data_size = m.cols * m.rows * elem_size;
    read_bytes((char *)m.ptr(), data_size) ;

}

}

