//==================================================================================================
// hls types sync
// (c) qiu.chao ,2016
//==================================================================================================
#ifndef HLS_TYPES_SYNC_HPP
#define HLS_TYPES_SYNC_HPP

#include "ap_fixed.h"
#include "hls_cm_nbits.hpp"
//#include "hls_fft.h"
#include "hls_fft_pipeline_stream.h"
#include "hls_video.h"
#include <complex>
//#include <string>
#include <cmath>
//#include <cstdlib>
//#include "ap_int.h"

//**************************************************************
//**** fft types
//**************************************************************
const char FFT_INPUT_WIDTH = 32;
const char FFT_OUTPUT_WIDTH = FFT_INPUT_WIDTH;
const char FFT_NFFT_MAX = 5;
// const int FFT_LENGTH = 1 << FFT_NFFT_MAX;
// const int FFT_LENGTH_QC = 256;
const char FFT_CONFIG_WIDTH = 16;

const int FFT_ROWS = 32;
const int FFT_COLS = 32;
const int NUM_FEATURES = 31;
const int SIZE_AREA = FFT_ROWS * FFT_COLS;
// const char fftConfig_WIDTH = FFT_NFFT_MAX >=7 ? 16:8;
using namespace std;
struct config1 : hls::ip_fft::params_t {
  static const unsigned ordering_opt = hls::ip_fft::natural_order;
  static const unsigned config_width = FFT_CONFIG_WIDTH;
  static const unsigned phase_factor_width = 24;
  static const unsigned max_nfft = FFT_NFFT_MAX;
  static const bool has_nfft = true;
  static const unsigned stages_block_ram = 0;
  //  static const unsigned butterfly_type =hls::ip_fft::use_xtremedsp_slices;
};

struct config_radix: hls::ip_fft::params_t {
  static const unsigned ordering_opt = hls::ip_fft::natural_order;
//  static const unsigned config_width = FFT_CONFIG_WIDTH;
  static const unsigned config_width = 24;
  static const unsigned phase_factor_width = 24;
  static const unsigned max_nfft = FFT_NFFT_MAX;
  static const bool has_nfft = true;
  static const unsigned stages_block_ram = 0;
  static const unsigned arch_opt = hls::ip_fft::radix_2_lite_burst_io;
  //  static const unsigned butterfly_type =hls::ip_fft::use_xtremedsp_slices;
//  radix_4_burst_io = 1, radix_2_burst_io,
//  pipelined_streaming_io, radix_2_lite_burst_io
};

typedef hls::ip_fft::config_t<config1> config_t;
typedef hls::ip_fft::status_t<config1> status_t;

typedef hls::ip_fft::config_t<config_radix> config_radix_t;
typedef hls::ip_fft::status_t<config_radix> status_radix_t;

typedef float data_in_prj_t;
// typedef ap_fixed<FFT_INPUT_WIDTH, 1> data_in_t;
// typedef ap_fixed<FFT_OUTPUT_WIDTH, FFT_OUTPUT_WIDTH - FFT_INPUT_WIDTH + 1>
// data_out_t;
typedef float fft_data_t;
typedef float kcf_data_t;
typedef std::complex<fft_data_t> cmpx_data_it; // cmpxDataIn;
typedef std::complex<fft_data_t> cmpx_data_ot; // cmpxDataIn;
typedef ap_uint<64> ap_uint64_t;
typedef ap_uint<32> ap_uint32_t;
typedef hls::stream<fft_data_t> fft_str_t;
typedef ap_uint<NBITS(FFT_ROWS)> fft_rows_t;
typedef ap_uint<NBITS(FFT_COLS)> fft_cols_t;
typedef ap_uint<NBITS(NUM_FEATURES)> num_features_t;
typedef ap_axiu<64, 1, 1, 1> ap_axiu64_t;
typedef ap_axiu<32, 1, 1, 1> ap_axiu32_t;
typedef hls::stream<ap_axiu64_t> axi64_stream_t;
typedef hls::stream<ap_axiu32_t> axi32_stream_t;
typedef hls::stream<ap_uint64_t> apu64_stream_t;
typedef hls::stream<ap_uint32_t> apu32_stream_t;
typedef ap_uint<2> fft_direction_t;
typedef hls::Mat<NUM_FEATURES, SIZE_AREA, HLS_32FC2> hog_mat_2t;
typedef hls::Mat<NUM_FEATURES, SIZE_AREA, HLS_32FC1> hog_mat_t;
typedef hls::Mat<FFT_ROWS, FFT_COLS, HLS_32FC2> fft_mat_2t;
typedef hls::Mat<FFT_ROWS, FFT_COLS, HLS_32FC1> fft_mat_t;
typedef hls::Scalar<HLS_MAT_CN(HLS_32FC2), HLS_TNAME(HLS_32FC2)> fft_scalar_2t;
typedef hls::Scalar<HLS_MAT_CN(HLS_32FC1), HLS_TNAME(HLS_32FC1)> fft_scalar_t;
typedef hls::Point_<fft_data_t> hls_pointf_t;
struct fft_size_t {
  fft_rows_t rows;
  //  fft_rows_t rowsNbit;
  fft_cols_t cols;
  //  fft_cols_t colsNbit;
  num_features_t numFeatures;
  // bool forwardDirection; //
  fft_size_t(fft_rows_t rows1, fft_cols_t cols1, num_features_t numFeatures1)
      : rows(rows1), cols(cols1), numFeatures(numFeatures1){
                                      //    rowsNbit = NBITS(rows) - 1;
                                      //    colsNbit = NBITS(cols) - 1;
                                  };
};
const int KCF_NAME_MAX_LEN = 50;         // max length of layer names
struct kcf_cfg_t {
	char hogName[KCF_NAME_MAX_LEN + 1];
	char kcfName[KCF_NAME_MAX_LEN + 1];
  //// hog
  int image_rows;
  int image_cols;
  int hann_rows;
  int hann_cols;
  kcf_data_t scale_adjust;
  kcf_data_t roi_x;
  kcf_data_t roi_y;
  kcf_data_t roi_width;
  kcf_data_t roi_height;
  kcf_data_t scale;
  int tmpl_sz_width;
  int tmpl_sz_height;
  ////kcf
  kcf_data_t sigma;        ////gaussian kernel bandwidth
  kcf_data_t interpFactor; //(train)linear interpolation factor for adaptation
  kcf_data_t lambda;       ////(train)regularization,
  bool detectEn;           ////1:detect,0:train
  bool firstFrame;         ////active high
  kcf_data_t orgValTmp;

  kcf_cfg_t(const char *nHog,const char *nKcf,int image_rows1, int image_cols1, int hann_rows1, int hann_cols1,
            kcf_data_t scale_adjust1, kcf_data_t roi_x1, kcf_data_t roi_y1,
            kcf_data_t roi_width1, kcf_data_t roi_height1, kcf_data_t scale1,
            int tmpl_sz_width1, int tmpl_sz_height1, kcf_data_t sigma1,
            kcf_data_t interpactor1, kcf_data_t lambda1, bool detectEn1,
            bool firstFrame1,kcf_data_t orgValTmp1)
      : image_rows(image_rows1), image_cols(image_cols1), hann_rows(hann_rows1),
        hann_cols(hann_cols1), scale_adjust(scale_adjust1), roi_x(roi_x1),
        roi_y(roi_y1), roi_width(roi_width1), roi_height(roi_height1),
        scale(scale1), tmpl_sz_width(tmpl_sz_width1),
        tmpl_sz_height(tmpl_sz_height1), sigma(sigma1),
        interpFactor(interpactor1), lambda(lambda1), detectEn(detectEn1),
        firstFrame(firstFrame1),orgValTmp(orgValTmp1){
	    for (int i = 0; i < KCF_NAME_MAX_LEN; i++) {
	    	hogName[i] = nHog[i];
	      if (nHog[i] == 0 ) break;
	    }
	    hogName[KCF_NAME_MAX_LEN] = 0;
	    for (int i = 0; i < KCF_NAME_MAX_LEN; i++) {
	    	kcfName[i] = nKcf[i];
	      if (nKcf[i] == 0) break;
	    }
	    kcfName[KCF_NAME_MAX_LEN] = 0;
  };
  // empty constructor, needed for empty array of layer_t in FPGA BRAM
  kcf_cfg_t()
      : image_rows(0), image_cols(0), hann_rows(0),
        hann_cols(0), scale_adjust(0), roi_x(0),
        roi_y(0), roi_width(0), roi_height(0),
        scale(0), tmpl_sz_width(0),
        tmpl_sz_height(0), sigma(0),
        interpFactor(0), lambda(0), detectEn(0),
        firstFrame(0),orgValTmp(0){
	  hogName[0] = 0;
	  kcfName[0] = 0;
  };
};

//**************************************************************
//**** others
//**************************************************************

// typedef ap_uint<NBITS(300)> dimension_t;
// typedef hls::Mat<ROWS,COLS,HLS_32FC1> hog_mat_2t;
const int EXTMEM_WIDTH = 256;
const int IMAGE_COLS_TMP = 640; //// by cqiu
const int ADDR_DOTS = EXTMEM_WIDTH/32;
const int SIZEOF_EXTMEM = EXTMEM_WIDTH/8;////256/8=32
//const int IMAGE_LINE = IMAGE_COLS_TMP/ADDR_DOTS;//// One 256Bit = 8x32Bit,8Plot
const int IMAGE_LINE = 128;

typedef ap_uint< EXTMEM_WIDTH> extmem_t;

const int DRAM_DEPTH = 320*240/8 ;
typedef ap_uint<32>  image_uint32_t;
typedef unsigned int u32_t;
typedef ap_fixed<16,1,AP_RND>apfix16_1_hog_t;
typedef ap_fixed<16,3,AP_RND>apfix16_3_hog_t;
typedef ap_fixed<32,13,AP_RND>apfix32_13_hog_t;//// max 4096,

// ================================
// = Memory To Image Dots
// ================================
template <int N, typename T_SRC, typename T_DST, int W>
void extMemToImageDots(T_SRC src, T_DST dst[W]) {
#pragma HLS inline
  for(int i = 0; i < W; i++) {
  #pragma HLS UNROLL
     hls::AXIGetBitFields(src, i*N, N, dst[i]);
  }
}

// =================================
// = imageDots To Memory
// =================================
template <typename APFIX_T, typename EXTMEM_T>
void imageDotsoExtMem(APFIX_T apfix16_w[ADDR_DOTS],EXTMEM_T & extmem_data ){
		extmem_data = 0;
	for(short i=0;i<ADDR_DOTS;i++){
		extmem_data += ap_uint<EXTMEM_WIDTH>(apfix16_w[i] << (i*32) );
	}
}


// ================
// =Stream To Cmpx
// ================
template <int W, typename T_DST>
std::complex<T_DST> streamToCmpxSync(hls::stream<ap_uint<W> > &src) {
#pragma HLS inline
  T_DST dst[2];
  ap_uint<W> srcVal;
  src >> srcVal;
  for (int i = 0; i < 2; i++) {
#pragma HLS UNROLL
    hls::AXIGetBitFields(srcVal, i * 32, 32, dst[i]);
  }
  return (cmpx_data_it(dst[0], dst[1]));
}

// ================
// =Stream To array
// ================
template <int W, typename T_DST>
void streamToArraySync(hls::stream<ap_uint<W> > &src, T_DST dst[2]) {
#pragma HLS inline
  ap_uint<W> srcVal;
  src >> srcVal;
  for (int i = 0; i < 2; i++) {
#pragma HLS UNROLL
    hls::AXIGetBitFields(srcVal, i * 32, 32, dst[i]);
  }
}

// ================
// =Cmpx To Stream
// ================
template <int W> ap_uint<W> cmpxToStreamSync(cmpx_data_it &src) {
#pragma HLS inline
  ap_uint<W> srcVal;
  hls::AXISetBitFields(srcVal, 0, 32, src.real());
  hls::AXISetBitFields(srcVal, 32, 32, src.imag());
  return srcVal;
}

// ================
// =Array To Stream
// ================
template <int W, typename T_DST> ap_uint<W> arrayToStreamSync(T_DST src[2]) {
#pragma HLS inline
  ap_uint<W> srcVal;
  hls::AXISetBitFields(srcVal, 0, 32, src[0]);
  hls::AXISetBitFields(srcVal, 32, 32, src[1]);
  return srcVal;
}

// ================================
// =Mat to Apu64Stream
// ================================
template <int W, int ROWS, int COLS, int T>
void Mat2Apu64Stream(hls::Mat<ROWS, COLS, T> &img,
                     hls::stream<ap_uint<W> > &AXI_video_strm) {
  int res = 0;
  hls::Scalar<HLS_MAT_CN(T), HLS_TNAME(T)> pix;
  ap_uint64_t axi;
  int depth = HLS_TBITDEPTH(T);
  // std::cout << W << " " << depth << " " << HLS_MAT_CN(T) << "\n";
  assert(W >= depth * HLS_MAT_CN(T) && "Bit-Width of AXI stream must be "
                                       "greater than the total number of bits "
                                       "in a pixel");
  HLS_SIZE_T rows = img.rows;
  HLS_SIZE_T cols = img.cols;
  assert(rows <= ROWS);
  assert(cols <= COLS);
loop_height:
  for (HLS_SIZE_T i = 0; i < rows; i++) {
  loop_width:
    for (HLS_SIZE_T j = 0; j < cols; j++) {
#pragma HLS loop_flatten off
#pragma HLS pipeline II = 1
      img >> pix;
    loop_channels:
      for (HLS_CHANNEL_T k = 0; k < HLS_MAT_CN(T); k++) {
#pragma HLS UNROLL
        hls::AXISetBitFields(axi, k * depth, depth, pix.val[k]);
      }
      AXI_video_strm << axi;
    }
  }
}

// ================================
// =Apu64Stream To Mat
// ================================
template <int W, int ROWS, int COLS, int T>
void Apu64Stream2Mat(hls::stream<ap_uint<W> > &AXI_video_strm,
                     hls::Mat<ROWS, COLS, T> &img) {
  ap_uint<W> axi;
  hls::Scalar<HLS_MAT_CN(T), HLS_TNAME(T)> pix;
  int depth = HLS_TBITDEPTH(T);
  // std::cout << W << " " << depth << " " << HLS_MAT_CN(T) << "\n";
  assert(W >= depth * HLS_MAT_CN(T) && "Bit-Width of AXI stream must be "
                                       "greater than the total number of bits "
                                       "in a pixel");
  HLS_SIZE_T rows = img.rows;
  HLS_SIZE_T cols = img.cols;
  assert(rows <= ROWS);
  assert(cols <= COLS);

loop_height:
  for (HLS_SIZE_T i = 0; i < rows; i++) {
  loop_width:
    for (HLS_SIZE_T j = 0; j < cols; j++) {
#pragma HLS loop_flatten off
#pragma HLS pipeline II = 1
      AXI_video_strm >> axi;
    loop_channels:
      for (HLS_CHANNEL_T k = 0; k < HLS_MAT_CN(T); k++) {
#pragma HLS UNROLL
        hls::AXIGetBitFields(axi, k * depth, depth, pix.val[k]);
      }
      img << pix;
    }
  }
}

// hls::AXISetBitFields(apAxiu64Val, 0, 32,outFftArray[i].real());
// hls::AXISetBitFields(apAxiu64Val, 32, 32,outFftArray[i].imag() );
// outDataStream << apAxiu64Val;

#endif
