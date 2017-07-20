
#ifndef __HLSKCF_HPP__
#define _HLSKCF_HPP__

#include "hls_video.h"
#include <string.h>
#include "fft_top.h"
#include "../tb/hls_display.hpp"
//#include "hann_ram.h"
using namespace std;
using namespace hls;

#define IMG_HEIGHT 480
#define IMG_WIDTH 640

#define HANN_ROWS 31
//#define HANN_COLS 2048
#define HANN_COLS 1024

#define CELL_SIZE 4
#define NUM_SECTOR 9

//const float PADDING = 2.5;
//const float MAX_SCALE_STEP  = 1.05f;
#define PADDING_SCALE_MAX   3 //// PADDING*MAX_SCALE_STEP=2.625;
#define MAX_HEIGHT    (IMG_HEIGHT * PADDING_SCALE_MAX)
#define  MAX_WIDTH    (IMG_WIDTH  *  PADDING_SCALE_MAX)

typedef hls::stream<ap_axiu<32, 1, 1, 1> > AXI_STREAM;
typedef hls::Scalar<3, unsigned char> RGB_PIXEL;
typedef hls::Mat<IMG_HEIGHT, IMG_WIDTH, HLS_8UC3> RGB_IMAGE;
typedef hls::Mat<MAX_WIDTH, MAX_WIDTH, HLS_8UC3> MAX_IMAGE;


template <int ROWS, int COLS>
void hlsGetHogFeatures(
		RGB_IMAGE &image,
		hog_mat_t &hann,
	   kcf_cfg_t &tracker, int size_patch[3],apu32_stream_t &featureStream );

template <typename T>
void hlsSubWindowLimit(hls::Rect &inWindow, int rows, int cols, T *rowStart,
                       T *rowEnd, T *colStart, T *colEnd);

template <int ROWS, int COLS, int SRC_T>
void hlsGetSubWindow(hls::Mat<ROWS, COLS, SRC_T> &image,
		MAX_IMAGE &subImage,
                     hls::Rect &inWindow);

template <int ROWS, int COLS>
void hlsGetFeatures(RGB_IMAGE &image,
                    hls::Mat<ROWS, COLS, HLS_32FC1> &featuresMat,
                    int size_patch[3]);

template <int ROWS, int COLS, int SRC_T>
void hlsGetHogGradient(hls::Mat<ROWS, COLS, SRC_T> &dx,
                       hls::Mat<ROWS, COLS, SRC_T> &dy,
                       hls::stream<ap_uint<42> > &gradient);

void hlsHogFeatureMaps(hls::stream<ap_uint<42> > &gradient, int rows, int cols,
                       hls::stream<ap_uint<288> > &normFeatureMaps);

void hlsPCAFeatureMaps(hls::stream<ap_uint<288> > &normFeatureMaps, int sizeY,
                       int sizeX, hls::stream<ap_uint<992> > &PCAFeatureMaps);

template <int ROWS, int COLS>
void hlsFeaturesMapToMat(hls::stream<ap_uint<992> > &PCAFeatureMaps, int sizeX,
                         int sizeY,
                         hls::Mat<ROWS, COLS, HLS_32FC1> &featuresMat);
//void genHANN_RAM(hog_mat_t &hann);
void mem2ImageMat(
		extmem_t *SHARED_DRAM_IMAGE,
		unsigned int image_offset,
		RGB_IMAGE &image
);
void mem2HannMat(
		extmem_t *SHARED_DRAM_IMAGE,
		unsigned int hannOffset,
		hog_mat_t &hannMat
);
void mem2ProbMat(
		extmem_t *SHARED_DRAM_PROB,
		unsigned int probOffset,
		fft_mat_t &probMat);

void mat2Ap32Stream(
		hls::Mat<HANN_ROWS, HANN_COLS, HLS_32FC1> &featureMat,
		apu32_stream_t &featureStream);

void fpga_top(
			extmem_t *WEIGHTS_SHARED_DRAM,
			extmem_t *READ_SHARED_DRAM,
			extmem_t *WRITE_SHARED_DRAM,
			unsigned int image_offset,
			unsigned int hannOffset,
			unsigned int probOffset,
			fft_size_t fftSize,
			kcf_cfg_t kcfConfigure, kcf_data_t &out_peak_value,
			hls_pointf_t &piResult);

#endif
