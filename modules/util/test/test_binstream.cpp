#include "gtest/gtest.h"
#include <core/BinaryStream.h>

#include <iostream>
#include <fstream>

using namespace std;
using namespace vl;

TEST(TestBinaryStream, TestIO)
{
    {

        ofstream strm("data/oo.bin", ios::binary) ;

        BinaryStream s(strm, false) ;

        s.write((uint8_t)10) ;
        s.write((int8_t)-30) ;
        s.write((uint16_t)5664) ;
        s.write((uint16_t)-664) ;
        s.write((uint32_t)673763) ;
        s.write((int32_t)-273763) ;
        s.write((uint64_t)98298499333) ;
        s.write((int64_t)-9348499333) ;

        s.write("hello") ;
        s.write((float)-563.56758) ;
        s.write((double)-.7748e19) ;
        s.write_v((uint32_t)67) ;
        s.write_v((uint32_t)567) ;
        s.write_v((uint32_t)123467) ;
        s.write_v((uint32_t)2233367) ;
        s.write_v((int32_t)-67) ;
        s.write_v((int32_t)-567) ;
        s.write_v((int32_t)-123467) ;
        s.write_v((int32_t)-2233367) ;
        s.write_v((uint64_t)67) ;
        s.write_v((uint64_t)567) ;
        s.write_v((uint64_t)123467) ;
        s.write_v((uint64_t)2233367) ;
        s.write_v((int64_t)-67) ;
        s.write_v((int64_t)-567) ;
        s.write_v((int64_t)-123467) ;
        s.write_v((int64_t)-2233367) ;

        float a_32u[] = { 1, 567, 789 } ;
        s.write(a_32u, 3) ;

    }

    ifstream strm("data/oo.bin", ios::binary) ;

    BinaryStream s(strm, false) ;

    uint8_t v_8u ;
    int8_t v_8s ;
    uint16_t v_16u ;
    int16_t v_16s ;
    uint32_t v_32u ;
    int32_t v_32s ;
    uint64_t v_64u ;
    int64_t v_64s ;
    string v_s ;
    double v_d ;
    float v_f ;

    s.read(v_8u) ;
    EXPECT_EQ((uint8_t)10, v_8u) ;
    s.read(v_8s) ;
    EXPECT_EQ((int8_t)-30, v_8s) ;
    s.read(v_16u) ;
    EXPECT_EQ((uint16_t)5664, v_16u) ;
    s.read(v_16s) ;
    EXPECT_EQ((int16_t)-664, v_16s) ;
    s.read(v_32u) ;
    EXPECT_EQ((uint32_t)673763, v_32u) ;
    s.read(v_32s) ;
    EXPECT_EQ((int32_t)-273763, v_32s) ;
    s >> v_64u >> v_64s;
    EXPECT_EQ((uint64_t)98298499333, v_64u) ;
    EXPECT_EQ((int64_t)-9348499333, v_64s) ;
    s.read(v_s) ;
    EXPECT_STREQ("hello", v_s.c_str()) ;
    s.read(v_f) ;
    EXPECT_FLOAT_EQ((float)-563.56758, v_f) ;
    s.read(v_d) ;
    EXPECT_DOUBLE_EQ((double)-.7748e19, v_d) ;


    s.read_v(v_32u) ;
    EXPECT_EQ((uint32_t)67, v_32u) ;
    s.read_v(v_32u) ;
    EXPECT_EQ((uint32_t)567, v_32u) ;
    s.read_v(v_32u) ;
    EXPECT_EQ((uint32_t)123467, v_32u) ;
    s.read_v(v_32u) ;
    EXPECT_EQ((uint32_t)2233367, v_32u) ;
    s.read_v(v_32s) ;
    EXPECT_EQ((int32_t)-67, v_32s) ;
    s.read_v(v_32s) ;
    EXPECT_EQ((int32_t)-567, v_32s) ;
    s.read_v(v_32s) ;
    EXPECT_EQ((int32_t)-123467, v_32s) ;
    s.read_v(v_32s) ;
    EXPECT_EQ((int32_t)-2233367, v_32s) ;
    s.read_v(v_64u) ;
    EXPECT_EQ((uint64_t)67, v_64u) ;
    s.read_v(v_64u) ;
    EXPECT_EQ((uint64_t)567, v_64u) ;
    s.read_v(v_64u) ;
    EXPECT_EQ((uint64_t)123467, v_64u) ;
    s.read_v(v_64u) ;
    EXPECT_EQ((uint64_t)2233367, v_64u) ;
    s.read_v(v_64s) ;
    EXPECT_EQ((int64_t)-67, v_64s) ;
    s.read_v(v_64s) ;
    EXPECT_EQ((int64_t)-567, v_64s) ;
    s.read_v(v_64s) ;
    EXPECT_EQ((int64_t)-123467, v_64s) ;
    s.read_v(v_64s) ;
    EXPECT_EQ((int64_t)-2233367, v_64s) ;

    float a_32u[3] ;
    s.read(a_32u, 3) ;

    EXPECT_FLOAT_EQ(1, a_32u[0]) ;
    EXPECT_FLOAT_EQ(567, a_32u[1]) ;
    EXPECT_FLOAT_EQ(789, a_32u[2]) ;


}
