//================================================================
//  Open CV Log Display
// (c) qiu.chao , 2017
//================================================================

#ifndef CV_DISPLAY_HPP
#define CV_DISPLAY_HPP

#include "hls_cm_log.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

typedef cv::Vec<uchar, 1> Vec1b;
typedef cv::Vec<uchar, 2> Vec2b;
typedef cv::Vec<uchar, 3> Vec3b;

typedef cv::Vec<float, 1> Vec1f;
typedef cv::Vec<float, 2> Vec2f;
typedef cv::Vec<float, 3> Vec3f;

//#define CV_8U   0
//#define CV_8S   1
//#define CV_16U  2
//#define CV_16S  3
//#define CV_32S  4
//#define CV_32F  5
//#define CV_64F  6
//#define CV_USRTYPE1 7

#ifndef __SYNTHESIS__
FILE *fileOpen(char *fileName);
void flieClose(FILE *outFile);
//void cvMatDisplay(FILE *outFile, cv::Mat cvMatA, char *StringName,bool logEn=1);
void cvMatDisplay(FILE *outFile, cv::Mat cvMatA, std::string StringName,bool logEn=1);
void cvMatDisplayAttribute(FILE *outFile, cv::Mat cvMatA, char *StringName,bool logEn=1);
bool cvMatDiff(cv::Mat cvMatA,cv::Mat cvMatB,std::string stringName);
#endif

#endif
