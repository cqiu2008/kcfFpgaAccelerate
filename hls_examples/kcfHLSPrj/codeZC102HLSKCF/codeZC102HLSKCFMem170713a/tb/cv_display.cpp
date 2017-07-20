//================================================================
//  Open CV Log Display
// (c) qiu.chao , 2017
//================================================================

#include "cv_display.hpp"
#include "hls_cm_log.hpp"

//====================================
//= debug fileOpen =
//====================================
#ifndef __SYNTHESIS__

FILE *fileOpen(char *fileName) {
  char fname[50];
  char layername[50];
  int i = 0;
  FILE *outFile;
  //	while (char c = layer.name[i])
  while (char c = *fileName++) {
    layername[i++] = (c == '/' || c == ' ') ? '_' : c;
  }
  layername[i] = '\0';
  sprintf(fname, "%s.log", layername);
  outFile = fopen(fname, "w+");
  return outFile;
}
//====================================
//= debug flieClose=
//====================================
void flieClose(FILE *outFile) { fclose(outFile); }
//====================================
//= cvMatDisplayAttribute
//====================================
void cvMatDisplayAttribute(FILE *outFile, cv::Mat cvMatA, char *StringName,bool logEn) {
	if(logEn){
		  char cvMatString[50];
		  int i = 0;
		  while (char c = *StringName++) {
			cvMatString[i++] = (c == '/' || c == ' ') ? '_' : c;
		  }
		  cvMatString[i] = '\0';
		  FLOG(outFile, "%s.channels,rows,cols=%d,%d,%d\n", cvMatString,
			   cvMatA.channels(), cvMatA.rows, cvMatA.cols);
	}

}

//====================================
//= cvMatDisplay   =
//====================================
//void cvMatDisplay(FILE *outFile, cv::Mat cvMatA, char *StringName,bool logEn) {
void cvMatDisplay(FILE *outFile, cv::Mat cvMatA, std::string StringName,bool logEn) {
	if(logEn){

		int lenStr = StringName.length();
//		char *cvMatString = (char *)malloc((lenStr+1)*sizeof(char));

		char cvMatString[50];
		for(int i=0;i<lenStr;i++){
			char c = StringName.at(i);
			cvMatString[i] = (c == '/' || c == ' ') ? '_' : c;
		}
		cvMatString[lenStr] = '\0';

////		*cvMatString = '\0';
//		for(int i=0;i<lenStr;i++){
//			char c = StringName.at(i);
//			*cvMatString++ = (c == '/' || c == ' ') ? '_' : c;
//		}
//		*cvMatString = '\0';
//		StringName.copy(cvMatString,lenStr,0);

		  for (int i = 0; i < cvMatA.rows; i++) {
			for (int j = 0; j < cvMatA.cols; j++) {
			  FLOG(outFile, "%s(%3d,%3d)=", cvMatString, i + 1, j + 1);
//				FLOG(outFile, "%s(%3d,%3d)=", StringName, i + 1, j + 1);
			  for (int k = 0; k < cvMatA.channels(); k++) {
				if (cvMatA.depth() > 4) { //// means CV_32F,CV_64F,CV_USRTYPE1
				  switch (cvMatA.channels()) {
				  case 1:
					FLOG(outFile, "%16f  ", cvMatA.at<Vec1f>(i, j)[k]);
					break;
				  case 2:
					FLOG(outFile, "%16f  ", cvMatA.at<Vec2f>(i, j)[k]);
					break;
				  case 3:
					FLOG(outFile, "%16f  ", cvMatA.at<Vec3f>(i, j)[k]);
					break;
				  default:
					FLOG(outFile, "%16f  ", cvMatA.at<Vec1f>(i, j)[k]);
				  }
				} else {
				  switch (cvMatA.channels()) {
				  case 1:
					FLOG(outFile, "%8x  ", cvMatA.at<Vec1b>(i, j)[k]);
					break;
				  case 2:
					FLOG(outFile, "%8x  ", cvMatA.at<Vec2b>(i, j)[k]);
					break;
				  case 3:
					FLOG(outFile, "%8x  ", cvMatA.at<Vec3b>(i, j)[k]);
					break;
				  default:
					FLOG(outFile, "%8x  ", cvMatA.at<Vec1b>(i, j)[k]);
				  }
				}
			  }
			  FLOG(outFile, ";\n");
			}
		  }
//		  free(cvMatString);
	}
}


//====================================
//= cvMatDiff
//====================================
bool cvMatDiff(cv::Mat cvMatA,cv::Mat cvMatB,std::string stringName){

  for (int i = 0; i < cvMatA.rows; i++) {
		for (int j = 0; j < cvMatA.cols; j++) {
			for (int k = 0; k < cvMatA.channels(); k++) {
				if (cvMatA.depth() > 4) { //// means CV_32F,CV_64F,CV_USRTYPE1
					  switch (cvMatA.channels()) {
					  case 1:
						  if( cvMatA.at<Vec1f >(i, j)[k] != cvMatB.at<Vec1f >(i, j)[k] ){
//							  std::cout<<"cvMat A="<< cvMatA.at<Vec1f >(i, j)[k]<<"cvMat B="<<cvMatB.at<Vec1f >(i, j)[k]<<std::endl;
//							  std::cout<<"Err cvMat channels "<<cvMatA.channels()<<" rows"<<cvMatA.rows<<" cols"<<cvMatA.cols<<std::endl;
							  return 0;
						  }
						  break;
					  case 2:
						  if( cvMatA.at<Vec2f >(i, j)[k] != cvMatB.at<Vec2f >(i, j)[k] ){
//							  std::cout<<"cvMat A="<< cvMatA.at<Vec2f >(i, j)[k]<<"cvMat B="<<cvMatB.at<Vec2f >(i, j)[k]<<std::endl;
//							  std::cout<<"Err cvMat channels "<<cvMatA.channels()<<" rows"<<cvMatA.rows<<" cols"<<cvMatA.cols<<std::endl;
							  return 0;
						  }
						  break;
					  case 3:
						  if( cvMatA.at<Vec3f >(i, j)[k] != cvMatB.at<Vec3f >(i, j)[k] ){
//							  std::cout<<"cvMat A="<< cvMatA.at<Vec3f >(i, j)[k]<<"cvMat B="<<cvMatB.at<Vec3f >(i, j)[k]<<std::endl;
//							  std::cout<<"Err cvMat channels "<<cvMatA.channels()<<" rows"<<cvMatA.rows<<" cols"<<cvMatA.cols<<std::endl;
							  return 0;
						  }
						  break;
					  default:{
//						  std::cout<<"Mat differ channels err "<<std::endl;
							return 0;
							}
					  	  break;
					  } //// end channels
				}else{
					  switch (cvMatA.channels()) {
					  case 1:
						  if( cvMatA.at<Vec1b >(i, j)[k] != cvMatB.at<Vec1b >(i, j)[k] ){
//							  std::cout<<"cvMat A="<< cvMatA.at<Vec1b >(i, j)[k]<<"cvMat B="<<cvMatB.at<Vec1b >(i, j)[k]<<std::endl;
//							  std::cout<<"Err cvMat channels "<<cvMatA.channels()<<" rows"<<cvMatA.rows<<" cols"<<cvMatA.cols<<std::endl;
							  return 0;
						  }
						  break;
					  case 2:
						  if( cvMatA.at<Vec2b >(i, j)[k] != cvMatB.at<Vec2b >(i, j)[k] ){
//							  std::cout<<"cvMat A="<< cvMatA.at<Vec2b >(i, j)[k]<<"cvMat B="<<cvMatB.at<Vec2b >(i, j)[k]<<std::endl;
//							  std::cout<<"Err cvMat channels "<<cvMatA.channels()<<" rows"<<cvMatA.rows<<" cols"<<cvMatA.cols<<std::endl;
							  return 0;
						  }
						  break;
					  case 3:
						  if( cvMatA.at<Vec3b >(i, j)[k] != cvMatB.at<Vec3b >(i, j)[k] ){
//							  std::cout<<"cvMat A="<< cvMatA.at<Vec3b >(i, j)[k]<<"cvMat B="<<cvMatB.at<Vec3b >(i, j)[k]<<std::endl;
//							  std::cout<<"Err cvMat channels "<<cvMatA.channels()<<" rows"<<cvMatA.rows<<" cols"<<cvMatA.cols<<std::endl;
							  return 0;
						  }
						  break;
					  default:{
//						  std::cout<<"Mat differ channels err "<<std::endl;
//						  std::cout<<"Err cvMat channels "<<cvMatA.channels()<<" rows"<<cvMatA.rows<<" cols"<<cvMatA.cols<<std::endl;
							return 0;
							}
					  	  break;
					  } //// end channels
				}

		} //// channels
	}//// cols
  }//// rows
  	  return 1;
}

#endif
