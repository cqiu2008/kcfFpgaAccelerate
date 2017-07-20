/*

Tracker based on Kernelized Correlation Filter (KCF) [1] and Circulant Structure
with Kernels (CSK) [2].
CSK is implemented by using raw gray level features, since it is a
single-channel filter.
KCF is implemented by using HOG features (the default), since it extends CSK to
multiple channels.

[1] J. F. Henriques, R. Caseiro, P. Martins, J. Batista,
"High-Speed Tracking with Kernelized Correlation Filters", TPAMI 2015.

[2] J. F. Henriques, R. Caseiro, P. Martins, J. Batista,
"Exploiting the Circulant Structure of Tracking-by-detection with Kernels", ECCV
2012.

Authors: Joao Faro, Christian Bailer, Joao F. Henriques
Contacts: joaopfaro@gmail.com, Christian.Bailer@dfki.de, henriques@isr.uc.pt
Institute of Systems and Robotics - University of Coimbra / Department Augmented
Vision DFKI


Constructor parameters, all boolean:
    hog: use HOG features (default), otherwise use raw pixels
    fixed_window: fix window size (default), otherwise use ROI size (slower but
more accurate)
    multiscale: use multi-scale tracking (default; cannot be used with
fixed_window = true)

Default values are set for all properties of the tracker depending on the above
choices.
Their values can be customized further before calling init():
    interp_factor: linear interpolation factor for adaptation
    sigma: gaussian kernel bandwidth
    lambda: regularization
    cell_size: HOG cell size
    padding: area surrounding the target, relative to its size
    output_sigma_factor: bandwidth of gaussian target
    template_size: template size in pixels, 0 to use ROI size
    scale_step: scale step for multi-scale estimation, 1 to disable it
    scale_weight: to downweight detection scores of other scales for added
stability

For speed, the value (template_size/cell_size) should be a power of 2 or a
product of small prime numbers.

Inputs to init():
   image is the initial frame.
   roi is a cv::Rect with the target positions in the initial frame

Inputs to update():
   image is the current frame.

Outputs of update():
   cv::Rect with target positions for the current frame


By downloading, copying, installing or using the software you agree to this
license.
If you do not agree to this license, do not download, install,
copy or use the software.


                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed.
In no event shall copyright holders or contributors be liable for any direct,
indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
 */

#ifndef _KCFTRACKER_HEADERS
#include "kcftracker.hpp"
// #include "../src/hls_types_sync.hpp"
#include "ffttools.hpp"
#include "fhog.hpp"
#include "labdata.hpp"
#include "recttools.hpp"
#endif

#ifndef BOARD_EN
//#include "../src/fft_top.h"
#include "../src/hlskcf.hpp"
#include "hls_opencv.h"
#endif

// typedef cv::Vec<uchar, 1> Vec1b;
// typedef cv::Vec<uchar, 2> Vec2b;
// typedef cv::Vec<uchar, 3> Vec3b;

// typedef cv::Vec<float, 1> Vec1f;
// typedef cv::Vec<float, 2> Vec2f;
// typedef cv::Vec<float, 3> Vec3f;




// Constructor
KCFTracker::KCFTracker(bool hog, bool fixed_window, bool multiscale, bool lab) {

#ifndef __SYNTHESIS__
  char kcfFileName[] = {"KCFTracker"};
  outFile = fileOpen(kcfFileName);
#endif

  // Parameters equal in all cases
  lambda = 0.0001;
  padding = 2.5;
  // output_sigma_factor = 0.1;
  output_sigma_factor = 0.125;

  if (hog) { // HOG
    // VOT
    interp_factor = 0.012;
    sigma = 0.6;
    // TPAMI
    // interp_factor = 0.02;
    // sigma = 0.5;
    cell_size = 4;
    _hogfeatures = true;

    if (lab) {
      interp_factor = 0.005;
      sigma = 0.4;
      // output_sigma_factor = 0.025;
      output_sigma_factor = 0.1;

      _labfeatures = true;
      _labCentroids = cv::Mat(nClusters, 3, CV_32FC1, &data);
      cell_sizeQ = cell_size * cell_size;
    } else {
      _labfeatures = false;
    }
  } else { // RAW
    interp_factor = 0.075;
    sigma = 0.2;
    cell_size = 1;
    _hogfeatures = false;

    if (lab) {
      printf("Lab features are only used with HOG features.\n");
      _labfeatures = false;
    }
  }

  if (multiscale) { // multiscale
    template_size = 96;
    // template_size = 100;
    scale_step = 1.05;
    scale_weight = 0.95;
    if (!fixed_window) {
      // printf("Multiscale does not support non-fixed window.\n");
      fixed_window = true;
    }
  } else if (fixed_window) { // fit correction without multiscale
    template_size = 96;
    // template_size = 100;
    scale_step = 1;
  } else {
    template_size = 1;
    scale_step = 1;
  }
  for(int i=0;i<4;i++){
	  inithannArray[i] = 0;
	  scaleStepArray[i] = 1.0f;
	  peakValueArray[i] = 0;
	  lambdaArray[i] = 0;
	  detectEnArray[i] = 1;
	  interp_factorArray[i] = 0;
  }



//  inithannArray[5] = {0, 0, 0, 0};
//  scaleStepArray[5] = {1.0f, 1.0 / 1.05f, 1.05f, 1.0f};
//  peakValueArray[5] = {0, 0, 0, 0};
//  lambdaArray[5] = {0, 0, 0, lambda};
//  detectEnArray[5] = {1, 1, 1, 0};
//  interp_factorArray[5] = {0, 0, 0, interp_factor};

}

  // Initialize tracker
  void KCFTracker::init(const cv::Rect &roi)
  {
//      _roi = roi;
//      assert(roi.width >= 0 && roi.height >= 0);
//      _tmpl = getFeatures(image, 1);

//      _alphaf = cv::Mat(size_patch[0], size_patch[1], CV_32FC2, float(0));
//      train(_tmpl, 1.0); // train with initial frame
	  _roi = roi;
	    assert(roi.width >= 0 && roi.height >= 0);
	  scaleStepArray[1] = 1.0/1.05f;
	  scaleStepArray[2] = 1.05f;
	  lambdaArray[3] = lambda;
	  detectEnArray[3] = 0;
	  interp_factorArray[3] = interp_factor;
      int padded_w = _roi.width * padding;
      int padded_h = _roi.height * padding;

      if (template_size >
          1) { // Fit largest dimension to the given template size
        if (padded_w >= padded_h) // fit to width
          _scale = padded_w / (float)template_size;
        else
          _scale = padded_h / (float)template_size;

        _tmpl_sz.width = padded_w / _scale;
        _tmpl_sz.height = padded_h / _scale;
      } else { // No template size given, use ROI size
        _tmpl_sz.width = padded_w;
        _tmpl_sz.height = padded_h;
        _scale = 1;
      }

      // Round to cell size and also make it even
      _tmpl_sz.width =
          (((int)(_tmpl_sz.width / (2 * cell_size))) * 2 * cell_size) +
          cell_size * 2;
      _tmpl_sz.height =
          (((int)(_tmpl_sz.height / (2 * cell_size))) * 2 * cell_size) +
          cell_size * 2;
//////////////// Modified by cqiu Begin 2017 04 19
#if 1
      float _tmplSzWidthPowerFloat = cv::log(_tmpl_sz.width) / cv::log(2);
      float _tmplSzHeightPowerFloat = cv::log(_tmpl_sz.height) / cv::log(2);
      int _tmplSzWidthPower = cvCeil(_tmplSzWidthPowerFloat);
      int _tmplSzHeightPower = cvCeil(_tmplSzHeightPowerFloat);
      LOG("_tmplSzWidthPower=%d\n", _tmplSzWidthPower);
      LOG("_tmplSzHeightPower=%d\n", _tmplSzHeightPower);
      _tmpl_sz.width = 1;
      for (int i = 0; i < _tmplSzWidthPower; i++) {
        _tmpl_sz.width = _tmpl_sz.width * 2;
      }
      size_patch[1] = _tmpl_sz.width / 4;
      _tmpl_sz.width =
          _tmpl_sz.width + 8; ////normalizeAndTruncate,has -2,so here +2*4
      _tmpl_sz.height = 1;
      for (int i = 0; i < _tmplSzHeightPower; i++) {
        _tmpl_sz.height = _tmpl_sz.height * 2;
      }
      size_patch[0] = _tmpl_sz.height / 4;
      _tmpl_sz.height =
          _tmpl_sz.height + 8; ////normalizeAndTruncate,has -2,so here +2*4
      LOG("_tmpl_sz.width=%d\n", _tmpl_sz.width);
      LOG("_tmpl_sz.height=%d\n", _tmpl_sz.height);
      size_patch[2] = 31;
#endif
      //////////////// Modified by cqiu End 2017 04 19
      createHanningMats();
      _prob = createGaussianPeak(size_patch[0], size_patch[1]);
      _prob = _prob.t();
   }

// Update position based on the new frame
cv::Rect KCFTracker::update(
		extmem_t *SHARED_DRAM_IMAGE ,
		extmem_t *SHARED_DRAM_HANN ,
		extmem_t *SHARED_DRAM_PROB  ,
		cv::Mat image, const cv::Rect &roi,
        int frameNum) {
#if 1

	bool firstFrame=(frameNum == 0);
  cv::Point2f resArray[4];
  cv::Point2f res;
  float peak_value = 0;
  int totTimes = 0;
  float cx = 0;
  float cy = 0;
  if (firstFrame) {
    _roi = roi;
    totTimes = 1;
    inithannArray[0] = 1;
    detectEnArray[0] = 0; //// enable train
    interp_factorArray[0] = 1.0f;
    lambdaArray[0] = lambda;
  } else {
    inithannArray[0] = 0;
    detectEnArray[0] = 1;
    interp_factorArray[0] = 0;
    lambdaArray[0] = 0;
    ////
    if (_roi.x + _roi.width <= 0)
      _roi.x = -_roi.width + 1;
    if (_roi.y + _roi.height <= 0)
      _roi.y = -_roi.height + 1;
    if (_roi.x >= image.cols - 1)
      _roi.x = image.cols - 2;
    if (_roi.y >= image.rows - 1)
      _roi.y = image.rows - 2;
    cx = _roi.x + _roi.width / 2.0f;
    cy = _roi.y + _roi.height / 2.0f;
    ////
    totTimes = 4;
  }
  fft_size_t fftSize(32, 32, 31);
  fftSize.rows = fft_rows_t(size_patch[0]);
  fftSize.cols = fft_cols_t(size_patch[1]);
  fftSize.numFeatures = num_features_t(size_patch[2]);
  //  cvMatDisplay(outFile, image, "image");
  //  cvMatDisplay(outFile, hann, "hann");
  for (int iTime = 0; iTime < totTimes; iTime++) {
#if 1
    kcf_data_t peakValueSim;
    hls_pointf_t hlsResultPoint;
    axi32_stream_t imageStream;
    axi32_stream_t hannStream;
    int zeroNum=3;
    char hogName[50];
    char kcfName[50];
    sprintf(hogName, "fpgaHog%0*dFrame%0*dTime",zeroNum,frameNum,zeroNum,iTime);
    sprintf(kcfName, "fpgaKcf%0*dFrame%0*dTime",zeroNum,frameNum,zeroNum,iTime);
    kcf_cfg_t kcfConfigure(
    		hogName,
			kcfName,
        //// hog
        image.rows, image.cols, size_patch[2], size_patch[0] * size_patch[1],
        scaleStepArray[iTime], _roi.x, _roi.y, _roi.width, _roi.height, _scale,
        _tmpl_sz.width, _tmpl_sz.height,
        //// cmPro
        sigma, interp_factorArray[iTime], lambdaArray[iTime],
        detectEnArray[iTime], firstFrame,iTime);

//    LOG("_roi.x=%d\n", kcfConfigure.image_rows);
    LOG("The %d Time of the frame\n",iTime);
//    LOG("fftSize.rows=%d\n", int(fftSize.rows));
//    LOG("fftSize.cols=%d\n", int(fftSize.cols));
//    LOG("fftSize.numFeatures=%d\n", int(fftSize.numFeatures));
//    LOG("kcfConfigure.image_rows=%d\n", kcfConfigure.image_rows);
//    LOG("kcfConfigure.image_cols=%d\n", kcfConfigure.image_cols);
//    LOG("kcfConfigure.hann_rows=%d\n", kcfConfigure.hann_rows);
//    LOG("kcfConfigure.hann_cols=%d\n", kcfConfigure.hann_cols);
//    LOG("kcfConfigure.scale_adjust=%f\n", kcfConfigure.scale_adjust);
//    LOG("kcfConfigure.roi_x=%f\n", kcfConfigure.roi_x);
//    LOG("kcfConfigure.roi_y=%f\n", kcfConfigure.roi_y);
//    LOG("kcfConfigure.roi_width=%f\n", kcfConfigure.roi_width);
//    LOG("kcfConfigure.roi_height=%f\n", kcfConfigure.roi_height);
//    LOG("kcfConfigure.scale=%f\n", kcfConfigure.scale);
//    LOG("kcfConfigure.tmpl_sz_width=%d\n", kcfConfigure.tmpl_sz_width);
//    LOG("kcfConfigure.tmpl_sz_height=%d\n", kcfConfigure.tmpl_sz_height);
//    LOG("kcfConfigure.sigma=%f\n", kcfConfigure.sigma);
//    LOG("kcfConfigure.interpFactor=%f\n", kcfConfigure.interpFactor);
//    LOG("kcfConfigure.lambda=%f\n", kcfConfigure.lambda);
//    LOG("kcfConfigure.detectEn=%d\n", kcfConfigure.detectEn);
//    LOG("kcfConfigure.firstFrame=%d\n", kcfConfigure.firstFrame);

//    cvMat2AXIvideo(image, imageStream);
//    cvMat2AXIvideo(hann, hannStream);
//    cvMatDisplay(outFile, hann, "hann");
    fpga_top(SHARED_DRAM_IMAGE,SHARED_DRAM_HANN,
    		SHARED_DRAM_PROB,
			0,0,0,
    		fftSize, kcfConfigure, peakValueSim,
             hlsResultPoint);

    cv::Point2f resSim(hlsResultPoint.x, hlsResultPoint.y);
    peakValueArray[iTime] = peakValueSim;
    resArray[iTime].x = resSim.x;
    resArray[iTime].y = resSim.y;

    if (detectEnArray[iTime]) {
      LOG("peakVaue=%16f,Point.x,y=%16f,%16f Simulation \n",
          peakValueArray[iTime], resArray[iTime].x, resArray[iTime].y);
    }
//////// Test Begin
    cv::Mat getFeatureMat =
        getFeatures(image, inithannArray[iTime], scaleStepArray[iTime]);
    cvMatDisplay(outFile, getFeatureMat, "featuresMap");
    cv::Point2f resArraySF[4];
    float peakValueArraySF[4];
    cmProcess(getFeatureMat,
    		peakValueArraySF[iTime], lambdaArray[iTime], detectEnArray[iTime],
			  resArraySF[iTime], interp_factorArray[iTime], firstFrame);
    if (detectEnArray[iTime]) {
      LOG("peakVaue=%16f,Point.x,y=%16f,%16f SoftWare \n",
    		  peakValueArraySF[iTime], resArraySF[iTime].x, resArraySF[iTime].y);
    }
//////// Test End
#endif

#if 0
    cmProcess(getFeatures(image, inithannArray[iTime], scaleStepArray[iTime]),
              peakValueArray[iTime], lambdaArray[iTime], detectEnArray[iTime],
              resArray[iTime], interp_factorArray[iTime], firstFrame);
#endif

    if (scale_weight * peakValueArray[iTime] > peak_value) {
      res = resArray[iTime];
      peak_value = peakValueArray[iTime];
      _scale *= scaleStepArray[iTime];
      _roi.width *= scaleStepArray[iTime];
      _roi.height *= scaleStepArray[iTime];
    }
    if (iTime == 2) {
      // Adjust by cell size and _scale
      _roi.x = cx - _roi.width / 2.0f + ((float)res.x * cell_size * _scale);
      _roi.y = cy - _roi.height / 2.0f + ((float)res.y * cell_size * _scale);
      if (_roi.x >= image.cols - 1)
        _roi.x = image.cols - 1;
      if (_roi.y >= image.rows - 1)
        _roi.y = image.rows - 1;
      if (_roi.x + _roi.width <= 0)
        _roi.x = -_roi.width + 2;
      if (_roi.y + _roi.height <= 0)
        _roi.y = -_roi.height + 2;
      assert(_roi.width >= 0 && _roi.height >= 0);
    }
  }
  return _roi;
#endif
}

cv::Point2f KCFTracker::calculateP(cv::Mat res, float &peak_value) {
  // minMaxLoc only accepts doubles for the peak, and integer points for the
  // coordinates
  cv::Point2i pi;
  double pv;
  cv::minMaxLoc(res, NULL, &pv, NULL, &pi);
  peak_value = (float)pv;

  // subpixel peak estimation, coordinates will be non-integer
  cv::Point2f p((float)pi.x, (float)pi.y);

  if (pi.x > 0 && pi.x < res.cols - 1) {
    p.x += subPixelPeak(res.at<float>(pi.y, pi.x - 1), peak_value,
                        res.at<float>(pi.y, pi.x + 1));
  }

  if (pi.y > 0 && pi.y < res.rows - 1) {
    p.y += subPixelPeak(res.at<float>(pi.y - 1, pi.x), peak_value,
                        res.at<float>(pi.y + 1, pi.x));
  }

  p.x -= (res.cols) / 2;
  p.y -= (res.rows) / 2;

  return p;
}

void KCFTracker::cmProcess(cv::Mat x, float &peak_value, float lambdaTmp,
                           bool detectEn, cv::Point2f &pDetect,
                           float train_interp_factor_tmp, bool firstFrame) {
//    cvMatDisplay(outFile,x,"xABCD");
#if 0
  kcf_data_t peakValueSim;
  hls_pointf_t hlsResultPoint;
  axi32_stream_t xInStream;
  kcf_cfg_t kcfConfigure(sigma, train_interp_factor_tmp, lambdaTmp, detectEn,
                         firstFrame);
  fft_size_t fftSize(size_patch[0], size_patch[1], size_patch[2]);
  cvMat2AXIvideo(x, xInStream);

  fftTopWrapperStream(xInStream, fftSize, kcfConfigure, peakValueSim,
                      hlsResultPoint);
  cv::Point2f resSim(hlsResultPoint.x, hlsResultPoint.y);

  peak_value = peakValueSim;
  pDetect = resSim;
  if (detectEn) {
    LOG("peakVaue=%16f,Point.x,y=%16f,%16f Simulation \n", peak_value,
        pDetect.x, pDetect.y);
  }
#endif

// #if BOARD_EN
#if 1
  using namespace FFTTools;
  cv::Mat k;

  if (firstFrame) {
    _tmpl = x;
    _prob = createGaussianPeak(size_patch[0], size_patch[1]);
    _alphaf = cv::Mat(size_patch[0], size_patch[1], CV_32FC2, float(0));
  } else {
    _tmpl = (1 - train_interp_factor_tmp) * _tmpl + (train_interp_factor_tmp)*x;
  }
  //    cvMatDisplay(outFile,_tmpl,"_tmplInit");
  if (detectEn) {
    k = gaussianCorrelation(x, _tmpl);
  } else {
    k = gaussianCorrelation(x, x);
  }

  	cvMatDisplay(outFile,k,"kOutMat");

  cv::Mat fftdK = fftd(k);
  cv::Mat fftdKLambda = fftdK + lambdaTmp;
  cv::Mat fftdMul;
  if (detectEn) {
    fftdMul = complexMultiplication(_alphaf, fftdKLambda);
    cv::Mat res = (real(fftd(fftdMul, true)));
    pDetect = calculateP(res, peak_value);
    LOG("peakVaue=%16f,Point.x,y=%16f,%16f Software \n", peak_value, pDetect.x,
        pDetect.y);
  } else {
    fftdMul = complexDivision(_prob, fftdKLambda);
    _alphaf = (1 - train_interp_factor_tmp) * _alphaf +
              (train_interp_factor_tmp)*fftdMul;

    //	    cv::Mat _probTmp = _prob.t();
    //	    cv::Mat _prob1Row = _probTmp.reshape(0,1);
    //	    cvMatDisplay(outFile,_prob1Row,"_probInit");

    //	    cv::Mat _alphafTmp = _alphaf.t();
    //	    cv::Mat  _alphaf1Row = _alphafTmp.reshape(0,1);
    //	    cvMatDisplay(outFile, _alphaf1Row,"_alphafInit");
  }

#endif
}

// Evaluates a Gaussian kernel with bandwidth SIGMA for all relative shifts
// between input images X and Y, which must both be MxN. They must    also be
// periodic (ie., pre-processed with a cosine window).
cv::Mat KCFTracker::gaussianCorrelation(cv::Mat x1, cv::Mat x2) {
  using namespace FFTTools;
  cv::Mat c =
      cv::Mat(cv::Size(size_patch[1], size_patch[0]), CV_32F, cv::Scalar(0));
  // HOG features
  if (_hogfeatures) {
    cv::Mat caux;
    cv::Mat x1aux;
    cv::Mat x2aux;
    for (int i = 0; i < size_patch[2]; i++) {
      x1aux = x1.row(i); // Procedure do deal with cv::Mat multichannel bug
      x1aux = x1aux.reshape(1, size_patch[0]);
      x2aux = x2.row(i).reshape(1, size_patch[0]);
      cv::mulSpectrums(fftd(x1aux), fftd(x2aux), caux, 0, true);
      caux = fftd(caux, true);
      //            rearrange(caux);
      caux.convertTo(caux, CV_32F);
      c = c + real(caux);
    }
  }
  // Gray features
  else {
    cv::mulSpectrums(fftd(x1), fftd(x2), c, 0, true);
    c = fftd(c, true);
    rearrange(c);
    c = real(c);
  }
  cv::Mat d;
  cv::max(((cv::sum(x1.mul(x1))[0] + cv::sum(x2.mul(x2))[0]) - 2. * c) /
              (size_patch[0] * size_patch[1] * size_patch[2]),
          0, d);

  LOG("x1Sum,x2Sum %16f,%16f,software\n", cv::sum(x1.mul(x1))[0],
      cv::sum(x2.mul(x2))[0]);
  cv::Mat k;
  cv::exp((-d / (sigma * sigma)), k);
  return k;
}

// Create Gaussian Peak. Function called only in the first frame.
cv::Mat KCFTracker::createGaussianPeak(int sizey, int sizex) {
  cv::Mat_<float> res(sizey, sizex);

  int syh = (sizey) / 2;
  int sxh = (sizex) / 2;

  float output_sigma =
      std::sqrt((float)sizex * sizey) / padding * output_sigma_factor;
  LOG("output_sigma=%16f\n", output_sigma);
  float mult = -0.5 / (output_sigma * output_sigma);
  LOG("mult=%16f\n", mult);
  for (int i = 0; i < sizey; i++)
    for (int j = 0; j < sizex; j++) {
      int ih = i - syh;
      int jh = j - sxh;
      res(i, j) = std::exp(mult * (float)(ih * ih + jh * jh));
    }
  //    cvMatDisplay(outFile,res,"_probRes              kkkkk");
  //    std::string strTst="asdfasdf";
  //    const char *strArray =strTst.data();
  //    LOG("const char %s\n",strArray);
  //    char strArray[50];

  FLOG(outFile, "%4s createGaussianPeakRes.rows=%d\n", SPACE1, res.rows);
  FLOG(outFile, "%4s createGaussianPeakRes.cols=%d\n", SPACE1, res.cols);
  FLOG(outFile, "%4s createGaussianPeakRes.channels=%d\n", SPACE1,
       res.channels());
  return FFTTools::fftd(res);
}

// Obtain sub-window from image, with replication-padding and extract features
cv::Mat KCFTracker::getFeatures(const cv::Mat &image, bool inithann,
                                float scale_adjust) {
  cv::Rect extracted_roi;

  float cx = _roi.x + _roi.width / 2;
  float cy = _roi.y + _roi.height / 2;

  // if (inithann) {
  //   int padded_w = _roi.width * padding;
  //   int padded_h = _roi.height * padding;

  //   if (template_size >
  //       1) { // Fit largest dimension to the given template size
  //     if (padded_w >= padded_h) // fit to width
  //       _scale = padded_w / (float)template_size;
  //     else
  //       _scale = padded_h / (float)template_size;

  //     _tmpl_sz.width = padded_w / _scale;
  //     _tmpl_sz.height = padded_h / _scale;
  //   } else { // No template size given, use ROI size
  //     _tmpl_sz.width = padded_w;
  //     _tmpl_sz.height = padded_h;
  //     _scale = 1;
  //   }

  //       if (_hogfeatures) {
  //         // Round to cell size and also make it even
  //         _tmpl_sz.width =
  //             (((int)(_tmpl_sz.width / (2 * cell_size))) * 2 * cell_size)
  //             +
  //             cell_size * 2;
  //         _tmpl_sz.height =
  //             (((int)(_tmpl_sz.height / (2 * cell_size))) * 2 *
  //             cell_size) +
  //             cell_size * 2;
  // //////////////// Modified by cqiu Begin 2017 04 19
  // #if 1
  //         float _tmplSzWidthPowerFloat = cv::log(_tmpl_sz.width) /
  //         cv::log(2);
  //         float _tmplSzHeightPowerFloat = cv::log(_tmpl_sz.height) /
  //         cv::log(2);
  //         int _tmplSzWidthPower = cvCeil(_tmplSzWidthPowerFloat);
  //         int _tmplSzHeightPower = cvCeil(_tmplSzHeightPowerFloat);
  //         LOG("_tmplSzWidthPower=%d\n", _tmplSzWidthPower);
  //         LOG("_tmplSzHeightPower=%d\n", _tmplSzHeightPower);
  //         _tmpl_sz.width = 1;
  //         for (int i = 0; i < _tmplSzWidthPower; i++) {
  //           _tmpl_sz.width = _tmpl_sz.width * 2;
  //         }
  //         _tmpl_sz.width =
  //             _tmpl_sz.width + 8; ////normalizeAndTruncate,has -2,so here
  //             +2*4
  //         _tmpl_sz.height = 1;
  //         for (int i = 0; i < _tmplSzHeightPower; i++) {
  //           _tmpl_sz.height = _tmpl_sz.height * 2;
  //         }
  //         _tmpl_sz.height =
  //             _tmpl_sz.height + 8; ////normalizeAndTruncate,has -2,so
  //             here +2*4
  //         LOG("_tmpl_sz.width=%d\n", _tmpl_sz.width);
  //         LOG("_tmpl_sz.height=%d\n", _tmpl_sz.height);
  // #endif
  //         //////////////// Modified by cqiu End 2017 04 19
  //       } else { // Make number of pixels even (helps with some logic
  //       involving
  //                // half-dimensions)
  //         _tmpl_sz.width = (_tmpl_sz.width / 2) * 2;
  //         _tmpl_sz.height = (_tmpl_sz.height / 2) * 2;
  //       }
  //     }

  extracted_roi.width = scale_adjust * _scale * _tmpl_sz.width;
  extracted_roi.height = scale_adjust * _scale * _tmpl_sz.height;

  // center roi with new size
  extracted_roi.x = cx - extracted_roi.width / 2;
  extracted_roi.y = cy - extracted_roi.height / 2;

  cv::Mat FeaturesMap;
  //  cv::Mat z = RectTools::subwindow(image, extracted_roi,
  //  cv::BORDER_REPLICATE);
  cv::Mat z = RectTools::subwindow(image, extracted_roi, cv::BORDER_CONSTANT);

  //  cvMatDisplay(outFile, z, "subImage");

  if (z.cols != _tmpl_sz.width || z.rows != _tmpl_sz.height) {
    cv::resize(z, z, _tmpl_sz);
  }
  //  cvMatDisplay(outFile, z, "zImage");

  // HOG features
  if (_hogfeatures) {
    IplImage z_ipl = z;
    CvLSVMFeatureMapCaskade *map;
    getFeatureMaps(&z_ipl, cell_size, &map);
    normalizeAndTruncate(map, 0.2f);
    PCAFeatureMaps(map);
    size_patch[0] = map->sizeY;
    size_patch[1] = map->sizeX;
    size_patch[2] = map->numFeatures;

    FeaturesMap =
        cv::Mat(cv::Size(map->numFeatures, map->sizeX * map->sizeY), CV_32F,
                map->map); // Procedure do deal with cv::Mat multichannel bug
    FeaturesMap = FeaturesMap.t();
    freeFeatureMapObject(&map);

    // Lab features
    if (_labfeatures) {
      cv::Mat imgLab;
      cvtColor(z, imgLab, CV_BGR2Lab);
      unsigned char *input = (unsigned char *)(imgLab.data);

      // Sparse output vector
      cv::Mat outputLab = cv::Mat(
          _labCentroids.rows, size_patch[0] * size_patch[1], CV_32F, float(0));

      int cntCell = 0;
      // Iterate through each cell
      for (int cY = cell_size; cY < z.rows - cell_size; cY += cell_size) {
        for (int cX = cell_size; cX < z.cols - cell_size; cX += cell_size) {
          // Iterate through each pixel of cell (cX,cY)
          for (int y = cY; y < cY + cell_size; ++y) {
            for (int x = cX; x < cX + cell_size; ++x) {
              // Lab components for each pixel
              float l = (float)input[(z.cols * y + x) * 3];
              float a = (float)input[(z.cols * y + x) * 3 + 1];
              float b = (float)input[(z.cols * y + x) * 3 + 2];

              // Iterate trough each centroid
              float minDist = FLT_MAX;
              int minIdx = 0;
              float *inputCentroid = (float *)(_labCentroids.data);
              for (int k = 0; k < _labCentroids.rows; ++k) {
                float dist =
                    ((l - inputCentroid[3 * k]) * (l - inputCentroid[3 * k])) +
                    ((a - inputCentroid[3 * k + 1]) *
                     (a - inputCentroid[3 * k + 1])) +
                    ((b - inputCentroid[3 * k + 2]) *
                     (b - inputCentroid[3 * k + 2]));
                if (dist < minDist) {
                  minDist = dist;
                  minIdx = k;
                }
              }
              // Store result at output
              outputLab.at<float>(minIdx, cntCell) += 1.0 / cell_sizeQ;
              //((float*) outputLab.data)[minIdx *
              //(size_patch[0]*size_patch[1])
              //+ cntCell] += 1.0 / cell_sizeQ;
            }
          }
          cntCell++;
        }
      }
      // Update size_patch[2] and add features to FeaturesMap
      size_patch[2] += _labCentroids.rows;
      FeaturesMap.push_back(outputLab);
    }
  } else {
    FeaturesMap = RectTools::getGrayImage(z);
    FeaturesMap -= (float)0.5; // In Paper;
    size_patch[0] = z.rows;
    size_patch[1] = z.cols;
    size_patch[2] = 1;
  }

//  if (inithann) {
//    createHanningMats();
//  }
  //    cvMatDisplay(outFile,hann,"hann",1);

  FeaturesMap = hann.mul(FeaturesMap);
  //    cvMatDisplayAttribute(outFile,hann,"hann",1);
  return FeaturesMap;
}

// Initialize Hanning window. Function called only in the first frame.
void KCFTracker::createHanningMats() {
  cv::Mat hann1t = cv::Mat(cv::Size(size_patch[1], 1), CV_32F, cv::Scalar(0));
  cv::Mat hann2t = cv::Mat(cv::Size(1, size_patch[0]), CV_32F, cv::Scalar(0));

  for (int i = 0; i < hann1t.cols; i++)
    hann1t.at<float>(0, i) =
        0.5 *
        (1 - std::cos(2 * 3.14159265358979323846 * i / (hann1t.cols - 1)));
  for (int i = 0; i < hann2t.rows; i++)
    hann2t.at<float>(i, 0) =
        0.5 *
        (1 - std::cos(2 * 3.14159265358979323846 * i / (hann2t.rows - 1)));

  cv::Mat hann2d = hann2t * hann1t;
  // HOG features
  if (_hogfeatures) {
    cv::Mat hann1d =
        hann2d.reshape(1, 1); // Procedure do deal with cv::Mat multichannel bug

    hann = cv::Mat(cv::Size(size_patch[0] * size_patch[1], size_patch[2]),
                   CV_32F, cv::Scalar(0));
    for (int i = 0; i < size_patch[2]; i++) {
      for (int j = 0; j < size_patch[0] * size_patch[1]; j++) {
        hann.at<float>(i, j) = hann1d.at<float>(0, j);
      }
    }
  }
  // Gray features
  else {
    hann = hann2d;
  }
}

// Calculate sub-pixel peak for one dimension
float KCFTracker::subPixelPeak(float left, float center, float right) {
  float divisor = 2 * center - right - left;

  if (divisor == 0)
    return 0;

  return 0.5 * (right - left) / divisor;
}
