//================================================================
//  Open CV Log Display
// (c) qiu.chao , 2017
//================================================================

#ifndef HLS_DISPLAY_HPP
#define HLS_DISPLAY_HPP
#include "hls_video.h"
#include "hls_cm_log.hpp"
#include <string>

//#define HLS_8U       0
//#define HLS_8S       1
//#define HLS_16U      2
//#define HLS_16S      3
//#define HLS_32S      4
//#define HLS_32F      5
//#define HLS_64F      6
//#define HLS_USRTYPE1 7
//#define HLS_10U      8
//#define HLS_10S      9
//#define HLS_12U      10
//#define HLS_12S      11
// and HLS_MAT_DEPTH(T) = HLS_8U or HLS_8S,,,.HLS_32F... and so on
// The T is in the " hls::Mat<ROWS, COLS, T>& hlsMatA "


#ifndef __SYNTHESIS__
using namespace hls;
//====================================
//= hlsMatDisplay   =
//====================================
template<int ROWS, int COLS, int T>
void hlsMatDisplay(
		FILE *outFile,
		hls::Mat<ROWS, COLS, T>& hlsMatA ,
		std::string StringName,bool logEn) {
		int lenStr = StringName.length();
		char hlsMatString[50];
		for(int i=0;i<lenStr;i++){
			char c = StringName.at(i);
			hlsMatString[i] = (c == '/' || c == ' ') ? '_' : c;
		}
		hlsMatString[lenStr] = '\0';


	    HLS_SIZE_T rows = hlsMatA.rows;
	    HLS_SIZE_T cols = hlsMatA.cols;
	    assert(rows <= ROWS);
	    assert(cols <= COLS);
	    hls::Scalar<HLS_MAT_CN(T), HLS_TNAME(T)> pix;
		  for (HLS_SIZE_T i = 0; i < rows; i++) {
			for (HLS_SIZE_T j = 0; j < cols ; j++) {
				FLOG(outFile, "%s(%3d,%3d)=", hlsMatString,int( i + 1), int(j + 1));
				hlsMatA >> pix;
				if(logEn) {
					  for (HLS_CHANNEL_T k = 0; k < HLS_MAT_CN(T); k++) {
						  if( (HLS_MAT_DEPTH(T) == HLS_32F) ||
								(HLS_MAT_DEPTH(T) == HLS_64F)   ){
							  FLOG(outFile, "%16f  ", pix.val[k]);
						  } else{
							  FLOG(outFile, "%8x  ",pix.val[k]);
						  }
					  }
					  FLOG(outFile, ";\n");
				}
				hlsMatA << pix;
			}
		  }
//		  free(hlsMatString);
}
#endif

#endif
