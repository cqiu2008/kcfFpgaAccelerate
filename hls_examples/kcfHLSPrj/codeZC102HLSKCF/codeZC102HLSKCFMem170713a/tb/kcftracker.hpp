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
    padding: horizontal area surrounding the target, relative to its size
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

#pragma once
#include <algorithm>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>

#include "cv_display.hpp"
#include "hls_cm_log.hpp"
#include "tracker.h"
#include <sys/time.h> // gettimeofday() for performance measurement

// #define BOARD_EN;

#ifndef _OPENCV_KCFTRACKER_HPP_
#define _OPENCV_KCFTRACKER_HPP_
#endif

using namespace std;
using namespace cv;

class KCFTracker : public Tracker {
public:
  // Constructor
  KCFTracker(bool hog = true, bool fixed_window = true, bool multiscale = true,
             bool lab = true);

  // Initialize tracker
      virtual void init(const cv::Rect &roi);

  // Update position based on the new frame
  virtual cv::Rect update(
		  extmem_t *SHARED_DRAM_IMAGE ,
		  extmem_t *SHARED_DRAM_HANN ,
		  extmem_t *SHARED_DRAM_PROB ,
		  cv::Mat image, const cv::Rect &roi,  int frameNum);


  float interp_factor;       // linear interpolation factor for adaptation
  float sigma;               // gaussian kernel bandwidth
  float lambda;              // regularization
  int cell_size;             // HOG cell size
  int cell_sizeQ;            // cell size^2, to avoid repeated operations
  float padding;             // extra area surrounding the target
  float output_sigma_factor; // bandwidth of gaussian target
  int template_size;         // template size
  float scale_step;          // scale step for multi-scale estimation
  float scale_weight; // to downweight detection scores of other scales for
                      // added stability
  int size_patch[3];
  cv::Mat hann;
  cv::Mat _prob;   // ��ʼ������prob�����ٸ��ģ�����ѵ��

  bool inithannArray[4];
  float scaleStepArray[4];
  float peakValueArray[4];
  float lambdaArray[4];
  bool detectEnArray[4];
  float interp_factorArray[4];

#ifndef __SYNTHESIS__
  FILE *outFile;
#endif
protected:
  // Evaluates a Gaussian kernel with bandwidth SIGMA for all relative shifts
  // between input images X and Y, which must both be MxN. They must    also be
  // periodic (ie., pre-processed with a cosine window).
  cv::Mat gaussianCorrelation(cv::Mat x1, cv::Mat x2);

  // Create Gaussian Peak. Function called only in the first frame.
  cv::Mat createGaussianPeak(int sizey, int sizex);

  // Obtain sub-window from image, with replication-padding and extract features
  cv::Mat getFeatures(const cv::Mat &image, bool inithann,
                      float scale_adjust = 1.0f);

  // Initialize Hanning window. Function called only in the first frame.
  void createHanningMats();

  // Calculate sub-pixel peak for one dimension
  float subPixelPeak(float left, float center, float right);
  // add by cqiu
  void cmProcess(cv::Mat x, float &peak_value, float lambdaTmp, bool detectEn,
                 cv::Point2f &pDetect, float train_interp_factor_tmp,
                 bool firstFrame);
  cv::Point2f calculateP(cv::Mat res, float &peak_value);

  cv::Mat _alphaf; // ��ʼ��/ѵ������alphaf�����ڼ��ⲿ���н����ļ���

  cv::Mat _tmpl; // ��ʼ��/ѵ���Ľ���������detect��z
  cv::Mat _num;
  cv::Mat _den;
  cv::Mat _labCentroids;

private:


  cv::Size _tmpl_sz;
  float _scale;
  int _gaussian_size;
  bool _hogfeatures;
  bool _labfeatures;
};

// void  groundTruth2Pos(  ifstream &groundtruthFile,Rect_<float> &rectGTruth );
template <typename TYPEF>
void groundTruth2Pos(ifstream &groundtruthFile, Rect_<TYPEF> &rectGTruth) {
  // Read groundtruth like a dumb
  float x1, y1, x2, y2, x3, y3, x4, y4;
  char ch;
  string firstLine;
  //	    Rect_<T> rectResult;
  getline(groundtruthFile, firstLine);
  istringstream ss(firstLine);
  ss >> x1;
  ss >> ch;
  ss >> y1;
  ss >> ch;
  ss >> x2;
  ss >> ch;
  ss >> y2;
  ss >> ch;
  ss >> x3;
  ss >> ch;
  ss >> y3;
  ss >> ch;
  ss >> x4;
  ss >> ch;
  ss >> y4;
  //    LOG("x1,y1,x2,y2,x3,y3,x4,y4=%f,%f----%f,%f----%f,%f----%f,%f\n",
  //  		  x1,y1,x2,y2,x3,y3,x4,y4);
  // Using min and max of X and Y for groundtruth rectangle
  float xMin = min(x1, min(x2, min(x3, x4)));
  float yMin = min(y1, min(y2, min(y3, y4)));
  float width = max(x1, max(x2, max(x3, x4))) - xMin;
  float height = max(y1, max(y2, max(y3, y4))) - yMin;
  rectGTruth.x = xMin;
  rectGTruth.y = yMin;
  rectGTruth.width = width;
  rectGTruth.height = height;
  ////////End to Generate xMin,yMin,width,height////////
}

template <typename TYPEF>
void inTest2Pos(ifstream &inTestFile, Rect_<TYPEF> &rectGTruth) {

  float xMin;
  float yMin;
  float width;
  float height;
  char ch;
  string firstLine;
  //	    Rect_<T> rectResult;
  getline(inTestFile, firstLine);
  istringstream ss(firstLine);
  ss >> xMin;
  ss >> ch;
  ss >> yMin;
  ss >> ch;
  ss >> width;
  ss >> ch;
  ss >> height;
  //    LOG("xMin,yMin,width,height=%f,%f----%f,%f\n",
  //    		xMin,yMin,width,height);
  // Using min and max of X and Y for groundtruth rectangle

  rectGTruth.x = xMin;
  rectGTruth.y = yMin;
  rectGTruth.width = width;
  rectGTruth.height = height;
  ////////End to Generate xMin,yMin,width,height////////
};
