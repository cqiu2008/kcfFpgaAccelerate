/*******************************************************************************
Vendor: Xilinx
Associated Filename: fftTop.cpp
Purpose: Xilinx FFT IP-XACT IP in Vivado HLS
Revision History: September 26, 2013 - initial release

*******************************************************************************
#-  (c) Copyright 2011-2016 Xilinx, Inc. All rights reserved.
#-
#-  This file contains confidential and proprietary information
#-  of Xilinx, Inc. and is protected under U.S. and
#-  international copyright and other intellectual property
#-  laws.
#-
#-  DISCLAIMER
#-  This disclaimer is not a license and does not grant any
#-  rights to the materials distributed herewith. Except as
#-  otherwise provided in a valid license issued to you by
#-  Xilinx, and to the maximum extent permitted by applicable
#-  law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
#-  WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
#-  AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
#-  BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
#-  INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
#-  (2) Xilinx shall not be liable (whether in contract or tort,
#-  including negligence, or under any other theory of
#-  liability) for any loss or damage of any kind or nature
#-  related to, arising under or in connection with these
#-  materials, including for any direct, or any indirect,
#-  special, incidental, or consequential loss or damage
#-  (including loss of data, profits, goodwill, or any type of
#-  loss or damage suffered as a result of any action brought
#-  by a third party) even if such damage or loss was
#-  reasonably foreseeable or Xilinx had been advised of the
#-  possibility of the same.
#-
#-  CRITICAL APPLICATIONS
#-  Xilinx products are not designed or intended to be fail-
#-  safe, or for use in any application requiring fail-safe
#-  performance, such as life-support or safety devices or
#-  systems, Class III medical devices, nuclear facilities,
#-  applications related to the deployment of airbags, or any
#-  other applications that could lead to death, personal
#-  injury, or severe property or environmental damage
#-  (individually and collectively, "Critical
#-  Applications"). Customer assumes the sole risk and
#-  liability of any use of Xilinx products in Critical
#-  Applications, subject only to applicable laws and
#-  regulations governing limitations on product liability.
#-
#-  THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
#-  PART OF THIS FILE AT ALL TIMES.
#- ************************************************************************


This file contains confidential and proprietary information of Xilinx, Inc. and
is protected under U.S. and international copyright and other intellectual
property laws.

DISCLAIMER
This disclaimer is not a license and does not grant any rights to the materials
distributed herewith. Except as otherwise provided in a valid license issued to
you by Xilinx, and to the maximum extent permitted by applicable law:
(1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND WITH ALL FAULTS, AND XILINX
HEREBY DISCLAIMS ALL WARRANTIES AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY,
INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-INFRINGEMENT, OR
FITNESS FOR ANY PARTICULAR PURPOSE; and (2) Xilinx shall not be liable (whether
in contract or tort, including negligence, or under any other theory of
liability) for any loss or damage of any kind or nature related to, arising
under
or in connection with these materials, including for any direct, or any
indirect,
special, incidental, or consequential loss or damage (including loss of data,
profits, goodwill, or any type of loss or damage suffered as a result of any
action brought by a third party) even if such damage or loss was reasonably
foreseeable or Xilinx had been advised of the possibility of the same.

CRITICAL APPLICATIONS
Xilinx products are not designed or intended to be fail-safe, or for use in any
application requiring fail-safe performance, such as life-support or safety
devices or systems, Class III medical devices, nuclear facilities, applications
related to the deployment of airbags, or any other applications that could lead
to death, personal injury, or severe property or environmental damage
(individually and collectively, "Critical Applications"). Customer assumes the
sole risk and liability of any use of Xilinx products in Critical Applications,
subject only to applicable laws and regulations governing limitations on product
liability.

THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS PART OF THIS FILE AT
ALL TIMES.

*******************************************************************************/

#include "fft_top.h"
#include <float.h>
// ==================
// = Golbal Variable  =
// ==================
kcf_data_t PROB_RAM[FFT_ROWS * FFT_COLS];
kcf_data_t ALPHAF_RAM[FFT_ROWS * FFT_COLS][2];
kcf_data_t TMPL_RAM[NUM_FEATURES][FFT_ROWS * FFT_COLS];

// ==================
// = debug file out =
// ==================
#ifndef __SYNTHESIS__
FILE *outFileFFT;
FILE *fileFFTOpen(char *fileName) {
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
void fileFFTClose(FILE *outFileFFT) { fclose(outFileFFT); }
#endif

void readLine(apu64_stream_t &inDataStream,
              cmpx_data_it outFftArray[FFT_LENGTH_QC], fft_size_t fftSize,
              fft_direction_t forward) {
#pragma HLS INLINE off
  cmpx_data_it cmpxTmp;
LOOP_ASSIGN:
  for (int i = 0; i < fftSize.cols * fftSize.rows * fftSize.numFeatures; i++) {
#pragma HLS LOOP_TRIPCOUNT min = 256 max = 1024 avg = 256
#pragma HLS PIPELINE II = 1
    cmpxTmp = streamToCmpxSync<64, fft_data_t>(inDataStream);
    if (forward == 1) {
      outFftArray[i] = cmpxTmp;
    } else if (forward == 2) {
      ////      outFftArray[i].real = cmpxTmp.real >> fftSize.rowsNbit;
      ////      outFftArray[i].imag = -cmpxTmp.imag >> fftSize.rowsNbit;
      outFftArray[i].real() = cmpxTmp.real() / fftSize.cols;
      outFftArray[i].imag() = -cmpxTmp.imag() / fftSize.cols;
    } else {
      outFftArray[i].real() = cmpxTmp.real() / fftSize.cols;
      outFftArray[i].imag() = cmpxTmp.imag() / fftSize.cols;
    }
  }
}

void writeLine(cmpx_data_it outFftArray[FFT_LENGTH_QC],
               apu64_stream_t &outDataStream, fft_size_t fftSize) {
#pragma HLS INLINE off
LOOP_OUTFFT:
  for (int i = 0; i < fftSize.cols * fftSize.rows * fftSize.numFeatures; i++) {
#pragma HLS LOOP_TRIPCOUNT min = 256 max = 1024 avg = 256
#pragma HLS PIPELINE II = 1
    outDataStream << cmpxToStreamSync<64>(outFftArray[i]);
    //    printf("writeLineOut[%2d][%2d]=%16f,%16f\n", int(i / fftSize.cols),
    //           int(i % fftSize.cols), float(outFftArray[i].real()),
    //           float(outFftArray[i].imag()));
  }
  //	  *ovflo = status_in->getOvflo() & 0x1;
}

fft_size_t setFFTSize(fft_rows_t rows, fft_cols_t cols,
                      num_features_t numFeatures) {
#pragma HLS INLINE
  fft_size_t fftSizeInv(rows, cols, numFeatures);
  return fftSizeInv;
}

void fft1d(apu64_stream_t &inDataStream, apu64_stream_t &outDataStream,
           fft_size_t fftSize, fft_direction_t forward) {
#pragma HLS INLINE off
#pragma HLS DATAFLOW
  cmpx_data_it outFftArray0[FFT_LENGTH_QC];
  cmpx_data_it outFftArray1[FFT_LENGTH_QC];
#pragma HLS STREAM variable = outFftArray0 depth = 512 dim = 1
#pragma HLS STREAM variable = outFftArray1 depth = 512 dim = 1
  config_t fftConfig;
  status_t fftStatus;
  bool *ovFlo;
  fftConfig.setDir(1);
  fftConfig.setSch(0x2AB);
  fftConfig.setNfft(NBITS(fftSize.cols) - 1);
  readLine(inDataStream, outFftArray0, fftSize, forward);
  hls::fftMultiFramesPipelinedStream<config1>(
      outFftArray0, outFftArray1, &fftStatus, &fftConfig,
      fftSize.rows * fftSize.numFeatures);
  writeLine(outFftArray1, outDataStream, fftSize);
}

void fft1dLittleResource(apu64_stream_t &inDataStream, apu64_stream_t &outDataStream,
           fft_size_t fftSize, fft_direction_t forward) {
#pragma HLS INLINE off
#pragma HLS DATAFLOW
  cmpx_data_it outFftArray0[FFT_LENGTH_QC];
  cmpx_data_it outFftArray1[FFT_LENGTH_QC];
#pragma HLS STREAM variable = outFftArray0 depth = 512 dim = 1
#pragma HLS STREAM variable = outFftArray1 depth = 512 dim = 1
  config_radix_t fftConfig;
  status_radix_t fftStatus;
  bool *ovFlo;
  fftConfig.setDir(1);
  fftConfig.setSch(0x2AB);
  fftConfig.setNfft(NBITS(fftSize.cols) - 1);
  readLine(inDataStream, outFftArray0, fftSize, forward);
  hls::fftMultiFramesPipelinedStream<config_radix>(
      outFftArray0, outFftArray1, &fftStatus, &fftConfig,
      fftSize.rows * fftSize.numFeatures);
  writeLine(outFftArray1, outDataStream, fftSize);
}

void arrayTransposition(apu64_stream_t &inDataStream,
                        apu64_stream_t &outDataStream, fft_size_t fftSize) {
#pragma HLS INLINE off

  fft_data_t fftMem[FFT_ROWS * FFT_COLS][2];
#pragma HLS ARRAY_PARTITION variable = fftMem complete dim = 2
#pragma HLS RESOURCE variable = fftMem core = RAM_T2P_BRAM latency = 3
  for (int k = 0; k < fftSize.numFeatures; k++) {
#pragma HLS LOOP_FLATTEN off
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8

  LOOP_ARRAY_TRANS_ROWS:
    for (int i = 0; i < fftSize.rows; i++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
      for (int j = 0; j < fftSize.cols; j++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
#pragma HLS PIPELINE II = 1
        streamToArraySync<64, fft_data_t>(inDataStream,
                                          fftMem[int(j * fftSize.rows + i)]);
      }
    }
  LOOP_ARRAY_TRANS_COLS:
    for (int i = 0; i < fftSize.cols; i++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
      for (int j = 0; j < fftSize.rows; j++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
#pragma HLS PIPELINE II = 1
        outDataStream << arrayToStreamSync<64, fft_data_t>(
            fftMem[int(i * fftSize.rows + j)]);
      }
    }
  }
}

void fftTopWrapper(apu64_stream_t &inDataStream, apu64_stream_t &outDataStream,
                   fft_size_t fftSize, bool  forward) {
#pragma HLS INTERFACE ap_none port = fftSize
#pragma HLS INLINE off
#pragma HLS DATAFLOW
  apu64_stream_t dataStream1;
  apu64_stream_t dataStream2;
  apu64_stream_t dataStream3;
#pragma HLS STREAM variable = dataStream1 depth = 512 dim = 1
#pragma HLS STREAM variable = dataStream2 depth = 512 dim = 1
  fft_direction_t forward1 = forward ? 1:2;
  fft_direction_t forward2 = forward ? 1:3;
  fft1d(inDataStream, dataStream1, fftSize, forward1);
  arrayTransposition(dataStream1, dataStream2, fftSize);
  fft1d(dataStream2, outDataStream,
        setFFTSize(fftSize.cols, fftSize.rows, fftSize.numFeatures), forward2);
}


void fftTopWrapperLittleResource(apu64_stream_t &inDataStream, apu64_stream_t &outDataStream,
                   fft_size_t fftSize, bool  forward) {
#pragma HLS INTERFACE ap_none port = fftSize
#pragma HLS INLINE off
#pragma HLS DATAFLOW
  apu64_stream_t dataStream1;
  apu64_stream_t dataStream2;
  apu64_stream_t dataStream3;
#pragma HLS STREAM variable = dataStream1 depth = 512 dim = 1
#pragma HLS STREAM variable = dataStream2 depth = 512 dim = 1
  fft_direction_t forward1 = forward ? 1:2;
  fft_direction_t forward2 = forward ? 1:3;
  fft1dLittleResource(inDataStream, dataStream1, fftSize, forward1);
  arrayTransposition(dataStream1, dataStream2, fftSize);
  fft1dLittleResource(dataStream2, outDataStream,
        setFFTSize(fftSize.cols, fftSize.rows, fftSize.numFeatures), forward2);
}

void hlsMulSpectrums(apu64_stream_t &inDataStream1,
                     apu64_stream_t &inDataStream2,
                     apu64_stream_t &outDataStream, fft_size_t fftSize) {
  cmpx_data_it inDataCmpx1;
  cmpx_data_it inDataCmpx2;
  cmpx_data_it outDataCmpx;
  for (int k = 0; k < fftSize.numFeatures; k++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
    for (int i = 0; i < fftSize.cols; i++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
      for (int j = 0; j < fftSize.rows; j++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
#pragma HLS PIPELINE II = 1
        inDataCmpx1 = streamToCmpxSync<64, fft_data_t>(inDataStream1);
        inDataCmpx2 = streamToCmpxSync<64, fft_data_t>(inDataStream2);

        outDataCmpx.real() = inDataCmpx1.real() * inDataCmpx2.real() +
                             inDataCmpx1.imag() * inDataCmpx2.imag();
        outDataCmpx.imag() = inDataCmpx1.imag() * inDataCmpx2.real() -
                             inDataCmpx1.real() * inDataCmpx2.imag();

#ifndef __SYNTHESIS__
//  	  LOG("inDataCmpx1,%16f,%16f,simulation\n",inDataCmpx1.real(),inDataCmpx1.imag());
//  	  LOG("inDataCmpx2,%16f,%16f,simulation\n",inDataCmpx2.real(),inDataCmpx2.imag());
//  	LOG("outDataCmpx,%16f,%16f,simulation\n",outDataCmpx.real(),outDataCmpx.imag());
#endif

        outDataStream << cmpxToStreamSync<64>(outDataCmpx);
        //        outDataStream << cmpxToStreamSync<64>(inDataCmpx1);
      }
    }
  }
}

void arrayCSum(apu64_stream_t &inDataStream, apu32_stream_t &outDataStream,
               fft_size_t fftSize) {
#pragma HLS INLINE off

  fft_data_t cSumMem[FFT_ROWS * FFT_COLS];
  fft_data_t cSumTmp[2];
#pragma HLS RESOURCE variable = cSumMem core = RAM_T2P_BRAM latency = 3
#pragma HLS DEPENDENCE variable = cSumMem inter false
LOOP_CSUM_CACULC:
  for (int k = 0; k < fftSize.numFeatures; k++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
    for (int i = 0; i < fftSize.rows; i++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
      for (int j = 0; j < fftSize.cols; j++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
#pragma HLS PIPELINE II = 1
        streamToArraySync<64, fft_data_t>(inDataStream, cSumTmp);
        if (k == 0) {
          cSumMem[int(i * fftSize.cols + j)] = cSumTmp[0];
        } else {
          cSumMem[int(i * fftSize.cols + j)] += cSumTmp[0];
        }
      }
    }
  }
LOOP_CSUM_WRITE:
  for (int i = 0; i < fftSize.rows; i++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
    for (int j = 0; j < fftSize.cols; j++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
#pragma HLS PIPELINE II = 1
      ap_uint32_t srcVal;
      hls::AXISetBitFields(srcVal, 0, 32, cSumMem[int(i * fftSize.cols + j)]);
      outDataStream << srcVal;
    }
  }
}

void ap32Stream2Ap64Stream(apu32_stream_t &inFFTStr1ch1,
                           apu64_stream_t &inFFTStr1ch2, fft_size_t fftSize) {

  fft_data_t inDataTmp;
  cmpx_data_it outDataCmpx;
  ap_uint32_t srcVal;

LOOP_AP32STREAM2AP64STERAM:
  for (int k = 0; k < fftSize.numFeatures; k++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
    for (int i = 0; i < fftSize.rows; i++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
      for (int j = 0; j < fftSize.cols; j++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
#pragma HLS PIPELINE II = 1
        inFFTStr1ch1 >> srcVal;
        hls::AXIGetBitFields(srcVal, 0, 32, inDataTmp);
        outDataCmpx.real() = inDataTmp;
        outDataCmpx.imag() = 0;
        inFFTStr1ch2 << cmpxToStreamSync<64>(outDataCmpx);
      }
    }
  }
}

void mat2Ap64Stream(fft_mat_t &inFFTMat, apu64_stream_t &inFFTStr1ch2) {

  fft_scalar_t inFFTMatScalar;
  cmpx_data_it outDataCmpx;

LOOP_AP32STREAM2AP64STERAM:

  for (int i = 0; i < inFFTMat.rows; i++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
    for (int j = 0; j < inFFTMat.cols; j++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
#pragma HLS PIPELINE II = 1
      inFFTMat >> inFFTMatScalar;
      outDataCmpx.real() = inFFTMatScalar.val[0];
      outDataCmpx.imag() = 0;
      inFFTStr1ch2 << cmpxToStreamSync<64>(outDataCmpx);
    }
  }
}

fft_data_t squareSum(hog_mat_t &inDataMat) {

  fft_data_t sqSumValue;
  fft_scalar_t sqSumScalar;
  //#pragma HLS DEPENDENCE variable = TMPL_RAM inter false
  for (int i = 0; i < inDataMat.rows; i++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
    for (int j = 0; j < inDataMat.cols; j++) {
#pragma HLS LOOP_TRIPCOUNT min = 256 max = 1024 avg = 256
#pragma HLS PIPELINE II = 1
      inDataMat >> sqSumScalar;
      if ((i == 0) && (j == 0)) {
        sqSumValue = sqSumScalar.val[0] * sqSumScalar.val[0];
      } else {
        sqSumValue += sqSumScalar.val[0] * sqSumScalar.val[0];
      }
    }
  }
  //#ifndef __SYNTHESIS__
  //  	  LOG("x1Sum,%16f,simulation\n",sqSumValue);
  //#endif
  return sqSumValue;
}

void calculateK(fft_data_t x1Sum, fft_data_t x2Sum, apu32_stream_t &cSumStr,
                apu32_stream_t &kStr, fft_size_t fftSize, fft_data_t sigma) {
#pragma HLS INLINE off
//#pragma HLS INTERFACE ap_none port = x1Sum
//#pragma HLS INTERFACE ap_none port = x2Sum
#pragma HLS INTERFACE ap_none port = sigma

  ap_uint32_t srcVal;
  ap_uint32_t dstVal;
  fft_data_t inDataTmp;
  fft_data_t outDataTmp;
  fft_data_t outDataTmp2;
  fft_data_t outData;
  for (int i = 0; i < fftSize.rows; i++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
    for (int j = 0; j < fftSize.cols; j++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
#pragma HLS PIPELINE II = 1
      cSumStr >> srcVal;
      hls::AXIGetBitFields(srcVal, 0, 32, inDataTmp);
      outDataTmp = (x1Sum + x2Sum - 2 * inDataTmp) /
                   (fftSize.rows * fftSize.cols * fftSize.numFeatures);
      if (outDataTmp > 0) {
        outDataTmp2 = outDataTmp;
      } else {
        outDataTmp2 = 0;
      }
      outData = exp(-outDataTmp2 / (sigma * sigma));
      //      #ifndef __SYNTHESIS__
      //      				FLOG(outFileFFT,"gaussianCorrelation[%3d][%3d]=%f\n",i,j,outData);
      //      #endif
      hls::AXISetBitFields(dstVal, 0, 32, outData);
      kStr << dstVal;
    }
  }
}

void hlsGaussianCorrelationMat(hog_mat_t &inDataMat1, hog_mat_t &inDataMat2,
                               fft_mat_t &outDataMat, fft_size_t fftSize,
                               fft_data_t sigma) {
#pragma HLS INLINE off
#pragma HLS DATAFLOW

  apu32_stream_t inFFTStr1ch1;
  apu32_stream_t inFFTStr2ch1;

  apu64_stream_t inFFTStr1;
  apu64_stream_t inFFTStr2;
  apu64_stream_t outFFTStr1;
  apu64_stream_t outFFTStr2;
  apu64_stream_t inIFFTStr;
  apu64_stream_t outIFFTStr;
  apu32_stream_t cSumStr;
  apu32_stream_t kStr;
#pragma HLS STREAM variable = inFFTStr1ch1 depth = 512 dim = 1
#pragma HLS STREAM variable = inFFTStr2ch1 depth = 512 dim = 1
#pragma HLS STREAM variable = inFFTStr1 depth = 512 dim = 1
#pragma HLS STREAM variable = inFFTStr2 depth = 512 dim = 1
#pragma HLS STREAM variable = outFFTStr1 depth = 512 dim = 1
#pragma HLS STREAM variable = outFFTStr2 depth = 512 dim = 1
#pragma HLS STREAM variable = inIFFTStr depth = 512 dim = 1
#pragma HLS STREAM variable = outIFFTStr depth = 512 dim = 1
#pragma HLS STREAM variable = cSumStr depth = 512 dim = 1
#pragma HLS STREAM variable = kStr depth = 512 dim = 1

  hog_mat_t inDataMat1A(int(fftSize.numFeatures),
                        int(fftSize.cols * fftSize.rows));
  hog_mat_t inDataMat1B(int(fftSize.numFeatures),
                        int(fftSize.cols * fftSize.rows));
  hog_mat_t inDataMat2A(int(fftSize.numFeatures),
                        int(fftSize.cols * fftSize.rows));
  hog_mat_t inDataMat2B(int(fftSize.numFeatures),
                        int(fftSize.cols * fftSize.rows));
  hls::Duplicate(inDataMat1, inDataMat1A, inDataMat1B);
  hls::Duplicate(inDataMat2, inDataMat2A, inDataMat2B);

  Mat2Apu64Stream(inDataMat1A, inFFTStr1ch1);
  Mat2Apu64Stream(inDataMat2A, inFFTStr2ch1);
  ap32Stream2Ap64Stream(inFFTStr1ch1, inFFTStr1, fftSize);
  ap32Stream2Ap64Stream(inFFTStr2ch1, inFFTStr2, fftSize);
  fftTopWrapper(inFFTStr1, outFFTStr1, fftSize, 1); //// forward FFT
  fftTopWrapper(inFFTStr2, outFFTStr2, fftSize, 1); //// forward FFT
  hlsMulSpectrums(outFFTStr1, outFFTStr2, inIFFTStr, fftSize);
  fftTopWrapper(inIFFTStr, outIFFTStr,
                   setFFTSize(fftSize.cols, fftSize.rows, fftSize.numFeatures),
                   0); //// backward FFT

//  ifftTopWrapper(inIFFTStr, outIFFTStr,
//                 setFFTSize(fftSize.cols, fftSize.rows, fftSize.numFeatures),
//                 0); //// backward FFT
  arrayCSum(outIFFTStr, cSumStr, fftSize);
  calculateK(squareSum(inDataMat1B), squareSum(inDataMat2B), cSumStr, kStr,
             fftSize, sigma);
  Apu64Stream2Mat(kStr, outDataMat);
}

// void fft2d(apu64_stream_t &inDataStream, apu64_stream_t &outDataStream,
//            fft_size_t fftSize, fft_direction_t forward) {
// #pragma HLS INTERFACE ap_none port = fftSize
// #pragma HLS INLINE off
// #pragma HLS DATAFLOW
//   apu64_stream_t dataStream1;
//   apu64_stream_t dataStream2;
//   apu64_stream_t dataStream3;
// #pragma HLS STREAM variable = dataStream1 depth = 512 dim = 1
// #pragma HLS STREAM variable = dataStream2 depth = 512 dim = 1
// #pragma HLS STREAM variable = dataStream3 depth = 512 dim = 1
//   fft1d(inDataStream, dataStream1, fftSize, forward);
//   arrayTransposition(dataStream1, dataStream2, fftSize);
//   fft1d(dataStream2, dataStream3,
//         setFFTSize(fftSize.cols, fftSize.rows, fftSize.numFeatures),
//         forward);
//   arrayTransposition(
//       dataStream3, outDataStream,
//       setFFTSize(fftSize.cols, fftSize.rows, fftSize.numFeatures));
// }

// Calculate sub-pixel peak for one dimension
kcf_data_t hlsSubPixelPeak(kcf_data_t left, kcf_data_t center,
                           kcf_data_t right) {
#pragma HLS INLINE
  kcf_data_t divisor = 2 * center - right - left;
  if (divisor == 0) {
    return 0;
  } else {
    return 0.5 * (right - left) / divisor;
  }
}

void postDetect(apu64_stream_t &inStream, kcf_data_t &out_peak_value,
                hls_pointf_t &pi, fft_size_t fftSize) {
#pragma HLS INLINE off
  fft_data_t resMem[FFT_ROWS * FFT_COLS];
#pragma HLS RESOURCE variable = resMem core = RAM_T2P_BRAM latency = 3

  cmpx_data_it cmpxTmp;
  hls_pointf_t piTmp(0, 0);

  kcf_data_t maxValue = 0;
LOOP_POSTDETECT_ROWS:
  for (int i = 0; i < fftSize.rows; i++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
    for (int j = 0; j < fftSize.cols; j++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
#pragma HLS PIPELINE II = 1
      cmpxTmp = streamToCmpxSync<64, fft_data_t>(inStream);

      //#ifndef __SYNTHESIS__
      //	    	  FLOG(outFileFFT,"resMem(%3d,%3d)=%16f
      //;\n",i+1,j+1,cmpxTmp.real());
      //#endif
      resMem[i * fftSize.cols + j] = cmpxTmp.real();
      if (maxValue < cmpxTmp.real()) {
        maxValue = cmpxTmp.real();
        piTmp.x = kcf_data_t(j);
        piTmp.y = kcf_data_t(i);
      }
    }
  }

  hls_pointf_t piResult(piTmp.x, piTmp.y);

  if (piTmp.x > 0 && piTmp.x < fftSize.cols - 1) {
    piResult.x += hlsSubPixelPeak(
        resMem[int(piTmp.y * fftSize.cols + piTmp.x - 1)], maxValue,
        resMem[int(piTmp.y * fftSize.cols + piTmp.x + 1)]);
  }

  if (piTmp.y > 0 && piTmp.y < fftSize.rows - 1) {
    piResult.y += hlsSubPixelPeak(
        resMem[int((piTmp.y - 1) * fftSize.cols + piTmp.x)], maxValue,
        resMem[int((piTmp.y + 1) * fftSize.cols + piTmp.x)]);
  }

  piResult.x -= (fftSize.cols) / 2;
  piResult.y -= (fftSize.rows) / 2;
  out_peak_value = maxValue;
  pi.x = kcf_data_t(piResult.x);
  pi.y = kcf_data_t(piResult.y);
}

void genPROB_RAM(fft_mat_t &probMat) {
#pragma HLS INLINE off
#pragma HLS RESOURCE variable = PROB_RAM core = RAM_S2P_BRAM latency = 3
	fft_scalar_t probScalar;
  for (int i = 0; i < probMat.rows * probMat.cols; i++) {
#pragma HLS LOOP_TRIPCOUNT min = 256 max = 1024 avg = 512
#pragma HLS PIPELINE II = 1
	  probMat >> probScalar;
	  PROB_RAM[i] = probScalar.val[0];
//    PROB_RAM[i] = prob_ram_inv_dat[i];
    //#ifndef __SYNTHESIS__
    //      FLOG(outFileFFT, "_probInit(%3d)=%16f ;\n", i,
    //    		  PROB_RAM[i]);
    //#endif
  }
}

void genGaussianCorrelationInput(
		apu32_stream_t &featureStream,
		bool firstFrame,
        bool detectEn, kcf_data_t interpFactor,
        hog_mat_t &xOutMat,
		hog_mat_t &zOutMat) {
#pragma HLS INLINE off
#pragma HLS ARRAY_PARTITION variable = TMPL_RAM complete dim = 1 // Features
#pragma HLS RESOURCE variable = TMPL_RAM core = RAM_T2P_BRAM latency = 3
#pragma HLS DEPENDENCE variable = TMPL_RAM inter false
	ap_uint32_t srcVal;
  fft_scalar_t xInScalar;
  fft_scalar_t xOutScalar;
  fft_scalar_t zOutScalar;
LOOP_GAUSSIAN_INPUT:
  for (int i = 0; i < xOutMat.rows; i++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
    for (int j = 0; j < xOutMat.cols; j++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
#pragma HLS PIPELINE II = 1
    	featureStream >> srcVal;
        hls::AXIGetBitFields(srcVal, 0, 32, xInScalar.val[0]);
//      xInMat >> xInScalar;
#ifndef __SYNTHESIS__
      FLOG(outFileFFT, "getFeatures(%3d)(%3d)=%16f ;\n", i+1, j+1,
    		  xInScalar.val[0]);
#endif


      if (firstFrame) {
        TMPL_RAM[i][j] = xInScalar.val[0];
#ifndef __SYNTHESIS__
//      FLOG(outFileFFT, "_tmplInit(%3d)(%3d)=%16f ;\n", i+1, j+1,
//    		  xInScalar.val[0]);
//		FLOG(outFileFFT, "_tmplInitA(%3d)(%3d)=%16f ;\n", i+1, j+1,
//				TMPL_RAM[i][j] );
#endif
        zOutScalar.val[0] = xInScalar.val[0];
      } else if (detectEn) { //// detect
        zOutScalar.val[0] = TMPL_RAM[i][j];
      } else { //// train
        TMPL_RAM[i][j] = (1 - interpFactor) * TMPL_RAM[i][j] +
                         interpFactor * xInScalar.val[0];
        zOutScalar.val[0] = xInScalar.val[0];
      }
      xOutMat << xInScalar;
      zOutMat << zOutScalar;

#ifndef __SYNTHESIS__
//      FLOG(outFileFFT, "_tmplInitB(%3d)(%3d)=%16f ;\n", i+1, j+1,
//    		  TMPL_RAM[i][j]);
#endif
    }
  }
}

cmpx_data_it cmpxCompute(cmpx_data_it inDataCmpx1, cmpx_data_it inDataCmpx2,
                         bool detectEn) {

#pragma HLS INLINE
  cmpx_data_it outDataCmpx;
  fft_data_t divisor1;

  divisor1 = inDataCmpx2.real() * inDataCmpx2.real() +
             inDataCmpx2.imag() * inDataCmpx2.imag();
  if (detectEn) {
    outDataCmpx.real() = inDataCmpx1.real() * inDataCmpx2.real() -
                         inDataCmpx1.imag() * inDataCmpx2.imag();
    //		outDataCmpx.imag() = -( inDataCmpx1.real() * inDataCmpx2.imag()+
    //				inDataCmpx1.imag() * inDataCmpx2.real() )  ;
    //////
    //此处是值要是负数，不明白为啥，有bug
    outDataCmpx.imag() =
        (inDataCmpx1.real() * inDataCmpx2.imag() -
         inDataCmpx1.imag() *
             inDataCmpx2.real()); //// 此处是值要是负数，不明白为啥，有bug
  } else {
    outDataCmpx.real() = (inDataCmpx1.real() * inDataCmpx2.real() +
                          inDataCmpx1.imag() * inDataCmpx2.imag()) /
                         divisor1;
    outDataCmpx.imag() = (inDataCmpx1.imag() * inDataCmpx2.real() +
                          inDataCmpx1.real() * inDataCmpx2.imag()) /
                         divisor1;
  }
  return outDataCmpx;
}

void hlsComplexCompute(apu64_stream_t &inDataStr, apu64_stream_t &outDataStr,
                       bool firstFrame, bool detectEn, kcf_data_t lambda,
                       kcf_data_t interpFactor, fft_size_t fftSize) {
#pragma HLS INLINE off
#pragma HLS ARRAY_PARTITION variable = ALPHAF_RAM complete dim = 2 // Features
#pragma HLS RESOURCE variable = ALPHAF_RAM core = RAM_T2P_BRAM latency = 3
#pragma HLS DEPENDENCE variable = ALPHAF_RAM inter false

  cmpx_data_it inDataCmpx1;
  cmpx_data_it inDataCmpx2;
  cmpx_data_it inDataCmpxLambda;
  cmpx_data_it outDataCmpx;

  for (int i = 0; i < fftSize.cols; i++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
    for (int j = 0; j < fftSize.rows; j++) {
#pragma HLS LOOP_TRIPCOUNT min = 8 max = 32 avg = 8
#pragma HLS PIPELINE II = 1
      if (detectEn) {
        inDataCmpx1.real() = ALPHAF_RAM[i * fftSize.rows + j][0];
        inDataCmpx1.imag() = ALPHAF_RAM[i * fftSize.rows + j][1];
      } else {
        inDataCmpx1.real() = PROB_RAM[i * fftSize.rows + j];
        inDataCmpx1.imag() = 0;
      }
      inDataCmpx2 = streamToCmpxSync<64, fft_data_t>(inDataStr);
      inDataCmpxLambda.real() = inDataCmpx2.real() + lambda;
      inDataCmpxLambda.imag() = inDataCmpx2.imag();
      outDataCmpx = cmpxCompute(inDataCmpx1, inDataCmpxLambda, detectEn);
      if (firstFrame) {
        ALPHAF_RAM[i * fftSize.rows + j][0] = interpFactor * outDataCmpx.real();
        ALPHAF_RAM[i * fftSize.rows + j][1] = interpFactor * outDataCmpx.imag();
      } else {
        ALPHAF_RAM[i * fftSize.rows + j][0] =
            (1 - interpFactor) * ALPHAF_RAM[i * fftSize.rows + j][0] +
            interpFactor * outDataCmpx.real();
        ALPHAF_RAM[i * fftSize.rows + j][1] =
            (1 - interpFactor) * ALPHAF_RAM[i * fftSize.rows + j][1] +
            interpFactor * outDataCmpx.imag();
      }
      outDataStr << cmpxToStreamSync<64>(outDataCmpx);
//#ifndef __SYNTHESIS__
//      FLOG(outFileFFT, "_alphafInit( 1,%3d)=%16f, %16f ;\n",
//           int(i * fftSize.rows + j + 1), ALPHAF_RAM[i * fftSize.rows + j][0],
//           ALPHAF_RAM[i * fftSize.rows + j][1]);
//#endif
    }
  }
}

void hlsCmProc(
		apu32_stream_t &featureStream,
		 fft_mat_t &probMat,fft_size_t fftSize, kcf_cfg_t kcfConfigure,
               kcf_data_t &out_peak_value, hls_pointf_t &piResult) {

// #pragma HLS INTERFACE axis port = xInStream bundle = INPUT_STREAM
// #pragma HLS INTERFACE s_axilite port = fftSize bundle = CONTROL_BUS register
// #pragma HLS INTERFACE s_axilite port = kcfConfigure bundle =                   \
//     CONTROL_BUS register
// #pragma HLS INTERFACE s_axilite port = out_peak_value bundle =                 \
//     CONTROL_BUS register
// #pragma HLS INTERFACE s_axilite port = piResult bundle = CONTROL_BUS register
// #pragma HLS INTERFACE s_axilite port = return bundle = CONTROL_BUS
#pragma HLS DATAFLOW

#ifndef __SYNTHESIS__
  outFileFFT = fileFFTOpen(kcfConfigure.kcfName);
#endif
//  hog_mat_t xInMat(int(fftSize.numFeatures), int(fftSize.cols * fftSize.rows));
  //		hog_mat_t xInMatA(int(fftSize.numFeatures), int(fftSize.cols *
  // fftSize.rows));
  //		hog_mat_t xInMatB(int(fftSize.numFeatures), int(fftSize.cols *
  // fftSize.rows));
  hog_mat_t xOutMat(int(fftSize.numFeatures), int(fftSize.cols * fftSize.rows));
  hog_mat_t zOutMat(int(fftSize.numFeatures), int(fftSize.cols * fftSize.rows));
  fft_mat_t kOutMat(int(fftSize.rows), int(fftSize.cols));
  apu64_stream_t kInFFTStr;
  apu64_stream_t kOutFFTStr;
  apu64_stream_t mulCmpxStr;
  apu64_stream_t mulCmpxIFFTStr;

#pragma HLS STREAM variable = kInFFTStr depth = 512 dim = 1
#pragma HLS STREAM variable = kOutFFTStr depth = 512 dim = 1
#pragma HLS STREAM variable = mulCmpxStr depth = 512 dim = 1
#pragma HLS STREAM variable = mulCmpxIFFTStr depth = 512 dim = 1

  genPROB_RAM(probMat);

  genGaussianCorrelationInput(featureStream, kcfConfigure.firstFrame,
                              kcfConfigure.detectEn, kcfConfigure.interpFactor,
                              xOutMat, zOutMat);
  hlsGaussianCorrelationMat(xOutMat, zOutMat, kOutMat, fftSize,
                            kcfConfigure.sigma);
//  #ifndef __SYNTHESIS__
//    	  hlsMatDisplay(outFileFFT,kOutMat,"kOutMat",1);
//  #endif

  mat2Ap64Stream(kOutMat, kInFFTStr);
  fftTopWrapperLittleResource(kInFFTStr, kOutFFTStr,
                setFFTSize(fftSize.rows, fftSize.cols, 1), 1); //// forward FFT

  hlsComplexCompute(kOutFFTStr, mulCmpxStr, kcfConfigure.firstFrame,
                    kcfConfigure.detectEn, kcfConfigure.lambda,
                    kcfConfigure.interpFactor, fftSize);

//  ifftTopWrapper(mulCmpxStr, mulCmpxIFFTStr,
//                 setFFTSize(fftSize.cols, fftSize.rows, 1),
//                 0); //// backward FFT
  fftTopWrapperLittleResource(mulCmpxStr, mulCmpxIFFTStr,
                 setFFTSize(fftSize.cols, fftSize.rows, 1),
                 0); //// backward FFT
  postDetect(mulCmpxIFFTStr, out_peak_value, piResult, fftSize);

#ifndef __SYNTHESIS__
  fileFFTClose(outFileFFT);
#endif
}

//   hls::Duplicate(inDataMat1, inDataMat1A, inDataMat1B);
//	#pragma HLS ARRAY_PARTITION variable = outFftArray complete dim = 1
//	#pragma HLS STREAM variable = outFftArray depth = 2048 dim = 1
// #pragma HLS interface ap_fifo depth=1024 port=inData,outData
// XSIP watermark, do not delete
// 67d7842dbbe25473c3c32b93c0da8047785f30d78e8a024de1b57352245f9689
