
#include "hlskcf.hpp"
#include <float.h>

using namespace hls;

// ==================
// = debug file out =
// ==================
#ifndef __SYNTHESIS__
FILE *outFileHOG;
FILE *fileHOGOpen(char *fileName) {
  char fname[50];
  char layername[50];
  int i = 0;
  FILE *outFileFFT;
  //	while (char c = layer.name[i])
  while (char c = *fileName++) {
    layername[i++] = (c == '/' | c == ' ') ? '_' : c;
  }
  layername[i] = '\0';
  sprintf(fname, "%s.log", layername);
  outFileFFT = fopen(fname, "w+");
  return outFileFFT;
}
void fileHOGClose(FILE *outFileFFT) { fclose(outFileFFT); }
#endif

//========================
//--global variable --
//========================
  const static int p = 12 * NUM_SECTOR;
  const static int pp = NUM_SECTOR * 3 + 4;
  const static int yp = 4;
  const static int xp = NUM_SECTOR;
  const static float nx = 0.235702276;
  const static float ny = 0.5f;



//============
//--fpga_top--
//============
void fpga_top(
			extmem_t *WEIGHTS_SHARED_DRAM,
			extmem_t *READ_SHARED_DRAM,
			extmem_t *WRITE_SHARED_DRAM,
			unsigned int image_offset,
			unsigned int hannOffset,
			unsigned int probOffset,
			fft_size_t fftSize,
			kcf_cfg_t kcfConfigure, kcf_data_t &out_peak_value,
			hls_pointf_t &piResult) {

#pragma HLS INTERFACE m_axi depth = DRAM_DEPTH port = WEIGHTS_SHARED_DRAM offset = \
    slave bundle = memorybus0 register
#pragma HLS INTERFACE m_axi depth = DRAM_DEPTH port = READ_SHARED_DRAM offset = \
    slave bundle = memorybus1 register
#pragma HLS INTERFACE m_axi depth = DRAM_DEPTH port = WRITE_SHARED_DRAM offset = \
    slave bundle = memorybus2 register

#pragma HLS INTERFACE s_axilite port = image_offset bundle = axilite  register
#pragma HLS INTERFACE s_axilite port = hannOffset bundle = axilite  register
#pragma HLS INTERFACE s_axilite port = probOffset bundle = axilite  register
#pragma HLS INTERFACE s_axilite port = fftSize bundle = axilite  register
#pragma HLS INTERFACE s_axilite port = kcfConfigure bundle = axilite  register
#pragma HLS INTERFACE s_axilite port = out_peak_value bundle = axilite  register
#pragma HLS INTERFACE s_axilite port = piResult bundle = axilite  register
#pragma HLS INTERFACE s_axilite port = return bundle = axilite  register


#pragma HLS DATAFLOW

#ifndef __SYNTHESIS__
  outFileHOG = fileHOGOpen(kcfConfigure.hogName);
#endif

  RGB_IMAGE image(kcfConfigure.image_rows, kcfConfigure.image_cols);

//  hls::Mat<HANN_ROWS, HANN_COLS, HLS_32FC1> featuresMap(kcfConfigure.hann_rows,
//                                                        kcfConfigure.hann_cols);

  apu32_stream_t featureStream;
#pragma HLS STREAM variable = featureStream depth = 512 dim = 1
  int size_patch[3];
  //	hann.rows;
  // RGB_PIXEL mm;
//  hls::AXIvideo2Mat(imageStrm, image);

	hog_mat_t hann(int(kcfConfigure.hann_rows), int(kcfConfigure.hann_cols));
	fft_mat_t probMat(int(fftSize.rows), int(fftSize.cols));

	mem2ImageMat(WEIGHTS_SHARED_DRAM,image_offset,image);
	mem2HannMat(READ_SHARED_DRAM,hannOffset,hann);
	mem2ProbMat(WRITE_SHARED_DRAM,probOffset,probMat);




//	hls::Consume(image);
//	out_peak_value = 1.23f;
//	piResult.x = -3.123f ;
//	piResult.y = 13333.2222f;
   #ifndef __SYNTHESIS__
//     	  hlsMatDisplay(outFileHOG,image,"image",1);
//     	 hlsMatDisplay(outFileHOG,hann,"hann",1);
//     	 hlsMatDisplay(outFileHOG,probMat,"_prob",1);
   #endif

  hlsGetHogFeatures<HANN_ROWS, HANN_COLS>(
		  image,
		  hann,
		  kcfConfigure,
		  size_patch,
		  featureStream);
  #ifndef __SYNTHESIS__
//    	  hlsMatDisplay(outFileHOG,featuresMap,"featuresMap",1);
  #endif

//     hls::Consume(featuresMap);
  hlsCmProc(featureStream, probMat,fftSize, kcfConfigure, out_peak_value, piResult);

//   hls::Mat2AXIvideo(featuresMap, featuresMapStrm);

#ifndef __SYNTHESIS__
  fileHOGClose(outFileHOG);
#endif

}

//============
//--mem2Mat
//============
void mem2ImageMat(
		extmem_t *SHARED_DRAM_IMAGE,
		unsigned int image_offset,
		RGB_IMAGE &image
) {
#pragma HLS INLINE off
	extmem_t imgBuff[IMAGE_LINE];
	int imageLineDots = image.cols/ADDR_DOTS;
	image_uint32_t imgDots[ADDR_DOTS];
	unsigned char imgChannelsData[4];
	RGB_PIXEL imgScalar;
	for(int i=0;i<image.rows;i++){
#pragma HLS LOOP_TRIPCOUNT MIN = 288 AVG = 288 MAX = 288
		int addrOffset = image_offset + i* imageLineDots ;
		memcpy( imgBuff,  &SHARED_DRAM_IMAGE[addrOffset],  sizeof(extmem_t)*imageLineDots   );
		for(int j=0;j<imageLineDots;j++){
#pragma HLS LOOP_TRIPCOUNT MIN = 44 AVG = 44 MAX = 44
			extMemToImageDots<32,extmem_t,image_uint32_t,8>(imgBuff[j],imgDots);
			for(int k=0;k<ADDR_DOTS;k++){
	#pragma HLS LOOP_TRIPCOUNT MIN = 8 AVG = 8 MAX = 8
#pragma HLS pipeline II = 1
				extMemToImageDots<8,image_uint32_t,unsigned char,4>(imgDots[k],imgChannelsData);
				imgScalar.val[0] = imgChannelsData[0];
				imgScalar.val[1] = imgChannelsData[1];
				imgScalar.val[2] = imgChannelsData[2];
				image << imgScalar;
			}
		}
	}
}

//============
//--mem2HannMat
//============
void mem2HannMat(
		extmem_t *SHARED_DRAM_IMAGE,
		unsigned int hannOffset,
		hog_mat_t &hannMat) {
	extmem_t hannBuff[IMAGE_LINE];
	int hannLineDots = hannMat.cols/ADDR_DOTS;
	kcf_data_t hannDots[ADDR_DOTS];
	fft_scalar_t hannScalar;
	for(int i=0;i<hannMat.rows;i++){
#pragma HLS LOOP_TRIPCOUNT MIN = 31 AVG = 31 MAX = 31
		int addrOffset = hannOffset + i* hannLineDots ;
		memcpy( hannBuff,  &SHARED_DRAM_IMAGE[addrOffset],  sizeof(extmem_t)*hannLineDots   );
		for(int j=0;j<hannLineDots;j++){
#pragma HLS LOOP_TRIPCOUNT MIN = 64 AVG = 128 MAX = 128
			extMemToImageDots<32,extmem_t,kcf_data_t,8>(hannBuff[j],hannDots);
			for(int k=0;k<ADDR_DOTS;k++){
#pragma HLS LOOP_TRIPCOUNT MIN = 8 AVG = 8 MAX = 8
#pragma HLS pipeline II = 1
				hannScalar.val[0] = hannDots[k];
				hannMat << hannScalar;
			}
		}
	}
}

//============
//--mem2ProbMat
//============
void mem2ProbMat(
		extmem_t *SHARED_DRAM_PROB,
		unsigned int probOffset,
		fft_mat_t &probMat) {
	extmem_t probBuff[IMAGE_LINE];
	int probLineDots = probMat.cols/ADDR_DOTS;
	kcf_data_t probDots[ADDR_DOTS];
	fft_scalar_t probScalar;
	for(int i=0;i<probMat.rows;i++){
#pragma HLS LOOP_TRIPCOUNT MIN = 32 AVG = 32 MAX = 32
		int addrOffset = probOffset + i* probLineDots ;
		memcpy( probBuff,  &SHARED_DRAM_PROB[addrOffset],  sizeof(extmem_t)*probLineDots   );
		for(int j=0;j<probLineDots;j++){
#pragma HLS LOOP_TRIPCOUNT MIN = 4 AVG = 4 MAX = 4
			extMemToImageDots<32,extmem_t,kcf_data_t,8>(probBuff[j],probDots);
			for(int k=0;k<ADDR_DOTS;k++){
#pragma HLS LOOP_TRIPCOUNT MIN = 8 AVG = 8 MAX = 8
#pragma HLS pipeline II = 1
				probScalar.val[0] = probDots[k];
				probMat << probScalar;
			}
		}
	}
}


//============
//--mem2ProbMat
//============
void mat2Ap32Stream(
		hls::Mat<HANN_ROWS, HANN_COLS, HLS_32FC1> &featureMat,
		apu32_stream_t &featureStream) {

  fft_scalar_t featureMatScalar;
  ap_uint<32> srcVal;

LOOP_MAT2AP32STREAM:
  for (int i = 0; i < featureMat.rows; i++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
    for (int j = 0; j < featureMat.cols; j++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
#pragma HLS PIPELINE II = 1
    	featureMat >> featureMatScalar;
    	hls::AXISetBitFields(srcVal, 0, 32, featureMatScalar.val[0]);
    	featureStream << srcVal;
//      outDataCmpx.real() = inFFTMatScalar.val[0];
//      outDataCmpx.imag() = 0;
//      inFFTStr1ch2 << cmpxToStreamSync<64>(outDataCmpx);
    }
  }
}

//=====================
//--genHANN_RAM--
//=====================
//void genHANN_RAM(hog_mat_t &hann) {
//#pragma HLS INLINE off
////#pragma HLS RESOURCE variable = PROB_RAM core = RAM_S2P_BRAM latency = 3
//	fft_scalar_t hannScalar;
//  for (int i = 0; i < hann.rows * hann.cols; i++) {
//#pragma HLS LOOP_TRIPCOUNT min = 1024 max = 31744 avg = 1024
//#pragma HLS PIPELINE II = 1
//	  hannScalar.val[0] = hann_ram_dat[i];
//	  hann << hannScalar;
//  }
//}

//=====================
//--hlsGetHogFeatures--
//=====================
template <int ROWS, int COLS>
void hlsGetHogFeatures(
		RGB_IMAGE &image,
		hog_mat_t &hann,
	   kcf_cfg_t &tracker, int size_patch[3],
	   apu32_stream_t &featureStream ) {
//	   hls::Mat<ROWS, COLS, HLS_32FC1> &features
#pragma HLS INTERFACE ap_none port = size_patch
#pragma HLS DATAFLOW

	  hls::Mat<HANN_ROWS, HANN_COLS, HLS_32FC1> features(tracker.hann_rows,
			  tracker.hann_cols);

//	hog_mat_t hann(int(tracker.hann_rows), int(tracker.hann_cols));
//	genHANN_RAM(hann);

  hls::Rect extracted_roi;
  float cx = tracker.roi_x + tracker.roi_width / 2;
  float cy = tracker.roi_y + tracker.roi_height / 2;

  extracted_roi.width =
      tracker.scale_adjust * tracker.scale * tracker.tmpl_sz_width;
  extracted_roi.height =
      tracker.scale_adjust * tracker.scale * tracker.tmpl_sz_height;

  // center roi with new size
  extracted_roi.x = cx - extracted_roi.width / 2;
  extracted_roi.y = cy - extracted_roi.height / 2;

  MAX_IMAGE subImage(extracted_roi.height, extracted_roi.width);/// kuang fanwei chaoguo le 640x480
  hlsGetSubWindow(image, subImage, extracted_roi);

//#ifndef __SYNTHESIS__
//  	  hlsMatDisplay(outFileHOG,subImage,"subImage ",1);
//#endif

  RGB_IMAGE resizeSubImage(tracker.tmpl_sz_height, tracker.tmpl_sz_width);
	hls::Resize(subImage, resizeSubImage);
//#ifndef __SYNTHESIS__
//  	  hlsMatDisplay(outFileHOG,resizeSubImage,"resizeSubImage ",1);
//#endif


	hls::Mat<ROWS, COLS, HLS_32FC1> featuresMap(hann.rows, hann.cols);
	hlsGetFeatures(resizeSubImage, featuresMap, size_patch);
//	#ifndef __SYNTHESIS__
//	  	  hlsMatDisplay(outFileHOG,featuresMap,"featuresMapBeforeMul ",1);
//	#endif
  hls::Mul(featuresMap, hann, features);
  mat2Ap32Stream(features,featureStream);
}

template <typename T>
void hlsSubWindowLimit(hls::Rect &inWindow, int rows, int cols, T *rowStart,
                       T *rowEnd, T *colStart, T *colEnd) {
#pragma HLS INLINE
  if (inWindow.y < 0) {
    *rowStart = inWindow.y;
  } else {
    *rowStart = 0;
  }

  if (inWindow.y + inWindow.height > rows) {
    *rowEnd = inWindow.y + inWindow.height;
  } else {
    *rowEnd = rows;
  }

  if (inWindow.x < 0) {
    *colStart = inWindow.x;
  } else {
    *colStart = 0;
  }

  if (inWindow.x + inWindow.width > cols) {
    *colEnd = inWindow.x + inWindow.width;
  } else {
    *colEnd = cols;
  }
}

//===================
//--hlsGetSubWindow--
//===================
template <int ROWS, int COLS, int SRC_T>
void hlsGetSubWindow(hls::Mat<ROWS, COLS, SRC_T> &image,
						MAX_IMAGE &subImage,
                     hls::Rect &inWindow) {
  int rowStart, rowEnd, colStart, colEnd;
  hlsSubWindowLimit(inWindow, image.rows, image.cols, &rowStart, &rowEnd,
                    &colStart, &colEnd);

  Scalar<HLS_MAT_CN(SRC_T), HLS_TNAME(SRC_T)> s;

LP_GETWINDOW_ROWS:
  for (int i = rowStart; i < rowEnd; i++) {
#pragma HLS LOOP_TRIPCOUNT max = 480
  LP_GETWINDOW_COLS:
    for (int j = colStart; j < colEnd; j++) {
#pragma HLS LOOP_TRIPCOUNT max = 640
#pragma HLS pipeline II = 1

      if (i >= 0 && j >= 0 && i < image.rows && j < image.cols) {
        image >> s;
      } else {
        for (int k = 0; k < HLS_MAT_CN(SRC_T); k++) {
#pragma HLS unroll
          s.val[k] = 0;
        }
      }

      if (j >= inWindow.x && j < inWindow.x + inWindow.width &&
          i >= inWindow.y && i < inWindow.y + inWindow.height) {
        subImage << s;
      }
    }
  }
}

//==================
//--hlsGetFeatures--
//==================
template <int ROWS, int COLS>
void hlsGetFeatures(RGB_IMAGE &image,
                    hls::Mat<ROWS, COLS, HLS_32FC1> &featuresMat,
                    int size_patch[3]) {
#pragma HLS DATAFLOW
  int sizeX = image.cols / CELL_SIZE;
  int sizeY = image.rows / CELL_SIZE;

  int height = image.rows;
  int width = image.cols;

  size_patch[0] = sizeY - 2;
  size_patch[1] = sizeX - 2;
  size_patch[2] = 31;

  hls::Mat<IMG_HEIGHT, IMG_WIDTH, HLS_16SC3> dx(height, width);
#pragma HLS STREAM variable = dx depth = 2000 dim = 1
  hls::Mat<IMG_HEIGHT, IMG_WIDTH, HLS_16SC3> dy(height, width);
  RGB_IMAGE src_image1(height, width);
  RGB_IMAGE src_image2(height, width);

  ap_int<3> kernel[3] = {-1, 0, 1};
  hls::Window<1, 3, ap_int<3> > kernel_dx;
  hls::Window<3, 1, ap_int<3> > kernel_dy;
  kernel_dx.insert_row(kernel, 0);
  kernel_dy.insert_col(kernel, 0);

  hls::Point_<ap_int<3> > anchor_dx(-1, 0);
  hls::Point_<ap_int<3> > anchor_dy(0, -1);

  hls::Duplicate(image, src_image1, src_image2);
  hls::Filter2D<BORDER_DEFAULT>(src_image1, dx, kernel_dx, anchor_dx);
  hls::Filter2D<BORDER_DEFAULT>(src_image2, dy, kernel_dy, anchor_dy);

//  	#ifndef __SYNTHESIS__
//  	  	  hlsMatDisplay(outFileHOG,dx,"dx ",1);
//  	  	  hlsMatDisplay(outFileHOG,dy,"dy ",1);
//  	#endif


  hls::stream<ap_uint<42> > gradient;
  hlsGetHogGradient(dx, dy, gradient);

  hls::stream<ap_uint<288> > normFeatureMaps;
  hlsHogFeatureMaps(gradient, height, width, normFeatureMaps);

  hls::stream<ap_uint<992> > PCAFeatureMaps;
#pragma HLS RESOURCE variable = PCAFeatureMaps core = FIFO_LUTRAM
  hlsPCAFeatureMaps(normFeatureMaps, sizeX - 2, sizeY - 2, PCAFeatureMaps);

  hlsFeaturesMapToMat(PCAFeatureMaps, sizeX - 2, sizeY - 2, featuresMat);
}

//=====================
//--hlsGetHogGradient--
//=====================
template <int ROWS, int COLS, int SRC_T>
void hlsGetHogGradient(hls::Mat<ROWS, COLS, SRC_T> &dx,
                       hls::Mat<ROWS, COLS, SRC_T> &dy,
                       hls::stream<ap_uint<42> > &gradient) {
  const static float boundary_x[NUM_SECTOR + 1] = {
      1.0f,         0.939692621, 0.766044443,  0.5,          0.173648178,
      -0.173648178, -0.5,        -0.766044443, -0.939692621, -1.0f};
  const static float boundary_y[NUM_SECTOR + 1] = {
      0.0f,        0.342020143, 0.64278761, 0.866025404, 0.984807753,
      0.984807753, 0.866025404, 0.64278761, 0.342020143, 0.0f};

  int rows = dx.rows;
  int cols = dx.cols;
  int ch = HLS_MAT_CN(SRC_T);

  Scalar<HLS_MAT_CN(SRC_T), HLS_TNAME(SRC_T)> s_x;
  Scalar<HLS_MAT_CN(SRC_T), HLS_TNAME(SRC_T)> s_y;
  apfix32_13_hog_t magApFix;

  for (int i = 0; i < rows; i++) {
#pragma HLS LOOP_TRIPCOUNT max = 480
    for (int j = 0; j < cols; j++) {
#pragma HLS LOOP_TRIPCOUNT max = 640
#pragma HLS PIPELINE II = 1

      dx >> s_x;
      dy >> s_y;
      float r;
      float magnitude = 0;
      int maxCh = 0;
      for (int ch = 0; ch < HLS_MAT_CN(SRC_T); ch++) {
        r = hls::sqrtf(s_x.val[ch] * s_x.val[ch] + s_y.val[ch] * s_y.val[ch]);
        if (r > magnitude) {
          magnitude = r;
          maxCh = ch;
        }
      }

      float max = 0;
      int maxi = 0;
      float dotProd;
      for (int kk = 0; kk < NUM_SECTOR; kk++) {
        dotProd =
            boundary_x[kk] * s_x.val[maxCh] + boundary_y[kk] * s_y.val[maxCh];
        if (dotProd > max) {
          max = dotProd;
          maxi = kk;
        } else if (-dotProd > max) {
          max = -dotProd;
          maxi = kk + NUM_SECTOR;
        }
      }

      ap_uint<5> alfa = maxi % NUM_SECTOR;


//#ifndef __SYNTHESIS__
//      FLOG(outFileHOG,"magnitude(%3d)(%3d)=%f\n",i+1,j+1,magnitude);
//      FLOG(outFileHOG,"alfa(%3d)(%3d)=%d\n",i+1,j+1,int(alfa));
//      FLOG(outFileHOG,"maxi(%3d)(%3d)=%d\n",i+1,j+1,maxi);
//#endif

      if(magnitude > 512){
    	  magApFix = 512;
      }else{
    	  magApFix = magnitude;
      }


      ap_uint<42> value;
      hls::AXISetBitFields(value, 0, 32, magApFix);
      hls::AXISetBitFields(value, 32, 5, alfa);
      hls::AXISetBitFields(value, 37, 5, maxi);

      gradient << value;
    } // for col
  }   // for row
}

//=====================
//--hlsHogFeatureMaps--
//=====================
void hlsHogFeatureMaps(hls::stream<ap_uint<42> > &gradient, int rows, int cols,
                       hls::stream<ap_uint<288> > &normFeatureMaps) {
  const static int nearest[CELL_SIZE] = {-1, -1, 1, 1};
//  const static apfix32_13_hog_t w[CELL_SIZE * 2] = {0.625, 0.375, 0.875, 0.125,
//                                         0.875, 0.125, 0.625, 0.375};

  const static apfix32_13_hog_t w[CELL_SIZE * 2] = {5, 3, 7, 1,
                                         7, 1, 5, 3};

  const static float ALFA = 0.2f;
  apfix32_13_hog_t mapArray[64][64][27];
#pragma HLS ARRAY_PARTITION variable = mapArray complete dim = 3

//  for (int i = 0; i < 32; i++) {
  for (int i = 0; i < 64; i++) {
    for (int j = 0; j < 64; j++) {
#pragma HLS PIPELINE II = 1
      for (int k = 0; k < 27; k++) {
        mapArray[i][j][k] = 0;
      }
    }
  }

  int sizeX = cols / CELL_SIZE;
  int sizeY = rows / CELL_SIZE;
  assert(sizeX <= 32 && sizeY <= 64);

#ifndef __SYNTHESIS__
      FLOG(outFileHOG,"sizeX,sizeY(%3d)(%3d)\n",
    		  sizeX,sizeY);
#endif

LP_HOGFEATUREMAP_ROWS:
  for (int row = 0; row < rows; row++) {
#pragma HLS LOOP_TRIPCOUNT max = 128
  LP_HOGFEATUREMAP_COLS:
    for (int col = 0; col < cols; col++) {
#pragma HLS LOOP_TRIPCOUNT max = 256
#pragma HLS PIPELINE II = 1

      ap_uint<42> value;
      gradient >> value;
      apfix32_13_hog_t r;
      ap_uint<5> alfa0, alfa1;
      hls::AXIGetBitFields(value, 0, 32, r);
      hls::AXIGetBitFields(value, 32, 5, alfa0);
      hls::AXIGetBitFields(value, 37, 5, alfa1);

      if (row > 0 && row < rows - 1 && col > 0 && col < cols - 1) {
        int i = row / CELL_SIZE;
        int ii = row % CELL_SIZE;
        int j = col / CELL_SIZE;
        int jj = col % CELL_SIZE;

        mapArray[i][j][alfa0] += w[ii * 2] * w[jj * 2] * r   >> 6 ;
        mapArray[i][j][alfa1 + NUM_SECTOR] +=  w[ii * 2] * w[jj * 2] * r >> 6;

        if (i + nearest[ii] >= 0 && i + nearest[ii] <= sizeY - 1) {
          mapArray[i + nearest[ii] ][j][alfa0] +=  w[ii * 2 + 1] * w[jj * 2] * r >> 6;
          mapArray[i + nearest[ii] ][j][alfa1 + NUM_SECTOR] +=
               w[ii * 2 + 1] * w[jj * 2] * r >> 6;
        }

        if (j + nearest[jj] >= 0 && j + nearest[jj] <= sizeX - 1) {
          mapArray[i][j + nearest[jj]][alfa0] += w[ii * 2] * w[jj * 2 + 1] * r >> 6;
          mapArray[i][j + nearest[jj]][alfa1 + NUM_SECTOR] +=
               w[ii * 2] * w[jj * 2 + 1] * r >> 6 ;
        }

        if (i + nearest[ii] >= 0 && i + nearest[ii] <= sizeY - 1 &&
            j + nearest[jj] >= 0 && j + nearest[jj] <= sizeX - 1) {
          mapArray[i + nearest[ii] ][j + nearest[jj]][alfa0] +=
               w[ii * 2 + 1] * w[jj * 2 + 1] * r >> 6;
          mapArray[i + nearest[ii] ][j + nearest[jj]][alfa1 + NUM_SECTOR] +=
               w[ii * 2 + 1] * w[jj * 2 + 1] * r >> 6;
        }
      }
    }
  }
//#ifndef __SYNTHESIS__
//
//  for (int i = 0; i < sizeY; i++) {
//    for (int j = 0; j < sizeX; j++) {
//#pragma HLS PIPELINE II = 1
//      for (int k = 0; k < 27; k++) {
//    	  FLOG(outFileHOG,"mapArray(%3d)(%3d)(%3d)=%16f\n",
//    	     		  i+1,j+1,k+1,mapArray[i][j][k]);
//      }
//    }
//  }
//#endif


//  float partOfNorm[32][64];
  float partOfNorm[64][64];

LP_HOGNORMFEATUREMAP_Y:
  for (int i = 0; i < sizeY + 2; i++) {
#pragma HLS LOOP_TRIPCOUNT max = 34
  LP_HOGNORMFEATUREMAP_X:
    for (int j = 0; j < sizeX + 2; j++) {
#pragma HLS LOOP_TRIPCOUNT max = 66
#pragma HLS PIPELINE II = 1

      if (i < sizeY && j < sizeX) {
        float valOfNorm = 0;
        for (int k = 0; k < NUM_SECTOR; k++) {
          valOfNorm += float(mapArray[i][j][k]) * float(mapArray[i][j][k]);
        }
        partOfNorm[i][j] = valOfNorm;
//#ifndef __SYNTHESIS__
//    	  FLOG(outFileHOG,"partOfNorm(%3d)(%3d)%16f\n",
//    	     		  i+1,j+1,partOfNorm[i][j]);
//#endif
      }

      if (i > 2 && i < sizeY + 1 && j > 2 && j < sizeX + 1) {
        float newData[12 * NUM_SECTOR];
        float valOfNorm0 =
            hls::sqrtf(partOfNorm[i - 2][j - 2] + partOfNorm[i - 2][j - 1] +
                       partOfNorm[i - 1][j - 2] + partOfNorm[i - 1][j - 1]) +
            FLT_EPSILON;

        for (int ii = 0; ii < NUM_SECTOR; ii++) {
          float temp = float(mapArray[i - 2][j - 2][ii]) / valOfNorm0;
          newData[ii] = (temp > ALFA) ? ALFA : temp;
        }
        for (int ii = 0; ii < 2 * NUM_SECTOR; ii++) {
          float temp = float(mapArray[i - 2][j - 2][ii + NUM_SECTOR]) / valOfNorm0;
          newData[ii + NUM_SECTOR * 4] = (temp > ALFA) ? ALFA : temp;
        }

        float valOfNorm1 =
            hls::sqrtf(partOfNorm[i - 2][j - 2] + partOfNorm[i - 2][j - 1] +
                       partOfNorm[i - 3][j - 2] + partOfNorm[i - 3][j - 1]) +
            FLT_EPSILON;
        for (int ii = 0; ii < NUM_SECTOR; ii++) {
          float temp = float(mapArray[i - 2][j - 2][ii]) / valOfNorm1;
          newData[ii + NUM_SECTOR] = (temp > ALFA) ? ALFA : temp;
        }
        for (int ii = 0; ii < 2 * NUM_SECTOR; ii++) {
          float temp = float(mapArray[i - 2][j - 2][ii + NUM_SECTOR]) / valOfNorm1;
          newData[ii + NUM_SECTOR * 6] = (temp > ALFA) ? ALFA : temp;
        }

        float valOfNorm2 =
            hls::sqrtf(partOfNorm[i - 2][j - 2] + partOfNorm[i - 2][j - 3] +
                       partOfNorm[i - 1][j - 2] + partOfNorm[i - 1][j - 3]) +
            FLT_EPSILON;

        for (int ii = 0; ii < NUM_SECTOR; ii++) {
          float temp = float(mapArray[i - 2][j - 2][ii]) / valOfNorm2;
          newData[ii + NUM_SECTOR * 2] = (temp > ALFA) ? ALFA : temp;
        }
        for (int ii = 0; ii < 2 * NUM_SECTOR; ii++) {
          float temp = float(mapArray[i - 2][j - 2][ii + NUM_SECTOR]) / valOfNorm2;
          newData[ii + NUM_SECTOR * 8] = (temp > ALFA) ? ALFA : temp;
        }

        float valOfNorm3 =
            hls::sqrtf(partOfNorm[i - 2][j - 2] + partOfNorm[i - 2][j - 3] +
                       partOfNorm[i - 3][j - 2] + partOfNorm[i - 3][j - 3]) +
            FLT_EPSILON;

        for (int ii = 0; ii < NUM_SECTOR; ii++) {
          float temp = float(mapArray[i - 2][j - 2][ii]) / valOfNorm3;
          newData[ii + NUM_SECTOR * 3] = (temp > ALFA) ? ALFA : temp;
        }
        for (int ii = 0; ii < 2 * NUM_SECTOR; ii++) {
          float temp = float(mapArray[i - 2][j - 2][ii + NUM_SECTOR]) / valOfNorm3;
          newData[ii + NUM_SECTOR * 10] = (temp > ALFA) ? ALFA : temp;
        }

        for (int k = 0; k < 12; k++) {

          ap_uint<288> value;
          for (int kk = 0; kk < NUM_SECTOR; kk++) {
//#ifndef __SYNTHESIS__
//      FLOG(outFileHOG,"newData(%3d)(%3d)(%3d)=%f\n",i+1,j+1,(k * NUM_SECTOR + kk+1),newData[k * NUM_SECTOR + kk]);
//#endif
            hls::AXISetBitFields(value, kk * 32, 32,
                                 newData[k * NUM_SECTOR + kk]);
          }
          normFeatureMaps << value;
        }
      }
    }
  }
}
//=====================
//-- genFeatureMap
//=====================
void genFeatureMap(
		hls::stream<ap_uint<288> > &normFeatureMaps,
		 float featureMap[12 * NUM_SECTOR]
	){

#ifndef __SYNTHESIS__
	float featureMapMin = 32767;
	float featureMapMax = -32767;
#endif

	 LP_GEN_FEATUREMAP:
 for (int k = 0; k < 12; k++) {
#pragma HLS PIPELINE II = 1
    ap_uint<288> value;
    normFeatureMaps >> value;
    for (int kk = 0; kk < NUM_SECTOR; kk++) {
      hls::AXIGetBitFields(value, kk * 32, 32,
                           featureMap[k * NUM_SECTOR + kk]);
#ifndef __SYNTHESIS__
      if(featureMap[k * NUM_SECTOR + kk] > featureMapMax ){
    	  featureMapMax = featureMap[k * NUM_SECTOR + kk] ;
      }
      if(featureMap[k * NUM_SECTOR + kk] < featureMapMax ){
    	  featureMapMin = featureMap[k * NUM_SECTOR + kk] ;
      }
//      FLOG(outFileHOG,"featureMap(%3d)(%3d)(%3d)(%3d)=%16f\n",
//    		 i,j,k,kk,featureMap[k * NUM_SECTOR + kk]);
#endif
    }
  }

#ifndef __SYNTHESIS__
      FLOG(outFileHOG,"featureMapMax,Min=%16f,%16f\n",
      		featureMapMax,featureMapMin);
#endif
}
//=====================
//-- genNewData1
//=====================
void genNewData1(
		float featureMap[12 * NUM_SECTOR],
		 float newData1[18]
		){
    LP_GEN_NEWDATA1:
	  //// newData			::	0-17
	  //// featureMap	:: 36+[0,18,36,54:1,19,37,55:2,20,38,56...]
	  //// featureMap	:: 36+[0-71]= 36-107
    for (int jj = 0; jj < NUM_SECTOR * 2; jj++) {
  	  apfix16_1_hog_t val = 0;
      for (int ii = 0; ii < yp; ii++) {//// 4
#pragma HLS PIPELINE II = 1
        val += apfix16_1_hog_t(featureMap[yp * xp + ii * xp * 2 + jj]);
      }
      newData1[jj] = val/2 ;
    }
}
//=====================
//-- genNewData2
//=====================
void genNewData2(
		float featureMap[12 * NUM_SECTOR],
		 float newData2[9]
		){
    LP_GEN_NEWDATA2:
		//// newData			::	18-26
		//// featureMap	::  [0,9,18,27:1,10,19,28...]
		//// featureMap	::  0-35
    for (int jj = 0; jj < NUM_SECTOR; jj++) {
  	  apfix16_1_hog_t val = 0;
      for (int ii = 0; ii < yp; ii++) {//// 4
#pragma HLS PIPELINE II = 1
        val += apfix16_1_hog_t( featureMap[ii * xp + jj]);
      }
//      newData2[jj + xp * 2] = val /2 ;
      newData2[jj ] = val /2 ;
    }
}
//=====================
//-- genNewData3
//=====================
void genNewData3(
		float featureMap[12 * NUM_SECTOR],
		 float newData3[4]
		){
    LP_GEN_NEWDATA3:
		//// newData			::	27-30
		//// featureMap	:: 36+ [0,1,2,3..17:   18,19,20...35:   36,37,...53:  54,55,56,...71]
		//// featureMap	::  36,37,38,39...107
    for (int ii = 0; ii < yp; ii++) {
  	  apfix16_3_hog_t val = 0;
      for (int jj = 0; jj < xp * 2; jj++) {//// 18
#pragma HLS PIPELINE II = 1
        val += apfix16_3_hog_t( featureMap[yp * xp + ii * xp * 2 + jj]);
      }
//      newData3[ii + xp * 3] = float(val) * nx;
      newData3[ii] = float(val) * nx;
    }

}

//=====================
//--hlsPCAFeatureMaps--
//=====================
void hlsPCAFeatureMaps(hls::stream<ap_uint<288> > &normFeatureMaps, int sizeX,
                       int sizeY, hls::stream<ap_uint<992> > &PCAFeatureMaps) {

//  const static int p = 12 * NUM_SECTOR;
//  const static int pp = NUM_SECTOR * 3 + 4;
//  const static int yp = 4;
//  const static int xp = NUM_SECTOR;
//  const static float nx = 0.235702276;
//  const static float ny = 0.5f;

  float newData[31];
#pragma HLS ARRAY_PARTITION variable = newData complete dim = 0

  float featureMap[12 * NUM_SECTOR];
//#pragma HLS RESOURCE variable = featureMap core = RAM_2P_LUTRAM
//#pragma HLS ARRAY_PARTITION variable = featureMap cyclic factor = 9 dim = 1
#pragma HLS ARRAY_PARTITION variable = featureMap complete dim = 0


  float newData1[18];
  float newData2[9];
  float newData3[4];

LP_PCAFEATURE_Y:
  for (int i = 0; i < sizeY; i++) {
#pragma HLS LOOP_TRIPCOUNT max = 30
  LP_PCAFEATURE_X:
    for (int j = 0; j < sizeX; j++) {
#pragma HLS LOOP_TRIPCOUNT max = 62


    	genFeatureMap(normFeatureMaps,featureMap);
    	genNewData1(featureMap,newData1);
    	genNewData2(featureMap,newData2);
    	genNewData3(featureMap,newData3);

      ap_uint<992> tmpValue;

      LP_GEN_PCAFEATUREMAPS_STREAM1:
      for (int kk = 0; kk < 18; kk++) {
#pragma HLS UNROLL
        hls::AXISetBitFields(tmpValue, kk * 32, 32, newData1[kk]);
      }
      LP_GEN_PCAFEATUREMAPS_STREAM2:
      for (int kk = 0; kk < 9; kk++) {
#pragma HLS UNROLL
        hls::AXISetBitFields(tmpValue, (kk+18) * 32, 32, newData2[kk]);
      }
      LP_GEN_PCAFEATUREMAPS_STREAM3:
      for (int kk = 0; kk < 4; kk++) {
#pragma HLS UNROLL
        hls::AXISetBitFields(tmpValue, (kk+27) * 32, 32, newData3[kk]);
      }

      PCAFeatureMaps << tmpValue;
    }
  }




}

//=======================
//--hlsFeaturesMapToMat--
//=======================
template <int ROWS, int COLS>
void hlsFeaturesMapToMat(hls::stream<ap_uint<992> > &PCAFeatureMaps, int sizeX,
                         int sizeY,
                         hls::Mat<ROWS, COLS, HLS_32FC1> &featuresMat) {
  Scalar<HLS_MAT_CN(HLS_32FC1), HLS_TNAME(HLS_32FC1)> d;
  float featureArray[2048][31];
#pragma HLS ARRAY_PARTITION variable = featureArray complete dim = 2
  assert(sizeX * sizeY <= 2048);

  for (int i = 0; i < sizeX * sizeY; i++) {
#pragma HLS PIPELINE II = 1
    ap_uint<992> value;
    PCAFeatureMaps >> value;
    for (int j = 0; j < 31; j++) {
      float feature;
      hls::AXIGetBitFields(value, j * 32, 32, feature);
      featureArray[i][j] = feature;
    }
  }

  for (int j = 0; j < 31; j++) {
    for (int i = 0; i < sizeX * sizeY; i++) {
#pragma HLS PIPELINE II = 1
      d.val[0] = featureArray[i][j];
      featuresMat << d;
    }
  }
}
