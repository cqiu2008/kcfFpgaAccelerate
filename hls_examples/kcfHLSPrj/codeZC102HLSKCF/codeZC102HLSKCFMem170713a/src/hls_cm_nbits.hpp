//================================================================
// Common NBits
// (c) qiu.chao ,2016
//================================================================
#ifndef HLS_CM_NBITS_HPP
#define HLS_CM_NBITS_HPP
// ================================
// = Bit-Width Calculation MACROs =
// ================================
// NBITS(constant) = how many bits needed to represent <constant>
#define NBITS2(n) ((n & 2) ? 1 : 0)
#define NBITS4(n) ((n & (0xC)) ? (2 + NBITS2(n >> 2)) : (NBITS2(n)))
#define NBITS8(n) ((n & 0xF0) ? (4 + NBITS4(n >> 4)) : (NBITS4(n)))
#define NBITS16(n) ((n & 0xFF00) ? (8 + NBITS8(n >> 8)) : (NBITS8(n)))
#define NBITS32(n) ((n & 0xFFFF0000) ? (16 + NBITS16(n >> 16)) : (NBITS16(n)))
#define NBITS(n) ((n) == 0 ? 1 : NBITS32((n)) + 1)

//==== usage methord
//typedef ap_uint<NBITS(300)> dimension_t;
//typedef short apfix16_img_t;
//typedef short apfix16_weight_t;
//typedef int apfix32_img_t;
//typedef short data_t;


#endif
