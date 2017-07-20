#include "kcftracker.hpp"
#include <dirent.h>
#include <arpa/inet.h>

using namespace std;
using namespace cv;

// Pointers to Shared DRAM Memory
char *SHARED_DRAM;
extmem_t *SHARED_DRAM_IMAGE;
extmem_t *SHARED_DRAM_HANN;
extmem_t *SHARED_DRAM_PROB;

int main(int argc, char* argv[]){
	if (argc > 5) return -1;
	bool HOG = true;
	bool FIXEDWINDOW = false;
	bool MULTISCALE = true;
    bool SILENT = false;
	bool LAB = false;
  int numFrames = 2;
//  int numFrames =1;
//	int numFrames=2;
//	int numFrames=5;
//  int numFrames=271;
  	  printf("\nVersion 17-05-12-1452\n");

#ifndef __SYNTHESIS__
  char fileNameTest[] = "trackRun";
  std::FILE *outFile = fileOpen(fileNameTest);
#endif
	// Create KCFTracker object
	KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
	// Frame readed
	Mat frame;
	// Tracker results
	Rect result;
  	// Read groundtruth for the 1st frame
  	ifstream groundtruthFile;
	char groundtruth[] = "region.txt";
  	groundtruthFile.open(groundtruth);
	Rect_<float> rectGroundTruth;
	groundTruth2Pos( groundtruthFile,rectGroundTruth );
	LOG("rectGroundTruth.x,y,width,height %4f, %4f, %4f, %4f\n",
			rectGroundTruth.x,rectGroundTruth.y,rectGroundTruth.width,rectGroundTruth.height);
	groundtruthFile.close();
 	// Read Images
	ifstream listFramesFile;
	char listFrames[] = "images.txt";
	listFramesFile.open(listFrames);
	string frameName;
	// Write Results
	ofstream resultsFile;
	char resultsPath[] = "output.txt";
	resultsFile.open(resultsPath);
	// Frame counter
	int nFrames = 0;
	#ifdef BOARD_EN
		// Start time
		timeval fpgaDetetcStartT, fpgaDetetcEndT;
		double fpgaDetetcEclipse;
		gettimeofday(&fpgaDetetcStartT, NULL);
	#endif

		extmem_t imageArray[640*480/8];


	for (int i = 0; i < numFrames; i++) {
		getline(listFramesFile, frameName);
		frame = imread(frameName, CV_LOAD_IMAGE_COLOR);
//		cvMatDisplay(outFile,frame,"image",1);
		unsigned int imgSize;
		unsigned int hannSize;
		unsigned int probSize;

		if(nFrames==0){
			tracker.init( rectGroundTruth );
		}
		imgSize = frame.rows*frame.cols*4 ;
		hannSize =tracker.size_patch[0]*tracker.size_patch[1]* NUM_FEATURES*4;///float so *4
		probSize =tracker.size_patch[0]*tracker.size_patch[1]*4;///float so *4
		imgSize = std::ceil(imgSize / (SIZEOF_EXTMEM*1.0f) ) * SIZEOF_EXTMEM ;
		hannSize = std::ceil(hannSize / (SIZEOF_EXTMEM*1.0f) ) * SIZEOF_EXTMEM ;
		probSize = std::ceil(probSize / (SIZEOF_EXTMEM*1.0f) ) * SIZEOF_EXTMEM ;
		unsigned int totalSize =imgSize + hannSize + probSize ;
		  // Memory Allocation
		  SHARED_DRAM = (char *)malloc(totalSize);
		  SHARED_DRAM_IMAGE = (extmem_t *)(SHARED_DRAM);
		  SHARED_DRAM_HANN  = (extmem_t *)(SHARED_DRAM+imgSize);
		  SHARED_DRAM_PROB   = (extmem_t *)(SHARED_DRAM+imgSize+hannSize);
		  printf("CPU: FPGA DRAM Memory Allocation:\n");
		  printf("     Bytes allocated: %12dB (config) + %12dB (image) + %12dB (hann) + %12dB (prob)\n",
		         0, imgSize, hannSize,probSize);

//		  cvMatDisplay(outFile, tracker.hann, "hann",1);
//		  cvMatDisplay(outFile, tracker._prob, "_prob",1);
		  // Copy prob:
			  u32_t		*putProb =  (u32_t*)imageArray;
			  for (int i = 0; i < tracker._prob.rows; i++) {
				  Vec2f *p = tracker._prob.ptr<Vec2f>(i);
				for (int j = 0; j < tracker._prob.cols; j++) {
					*putProb++ = *(u32_t*)&p[j][0];
				}
			  }
			  memcpy(SHARED_DRAM_PROB, imageArray, probSize);

		  // Copy hanns:
			  u32_t		*putHann =  (u32_t*)imageArray;
			  for (int i = 0; i < tracker.hann.rows; i++) {
				  Vec1f *p = tracker.hann.ptr<Vec1f>(i);
				for (int j = 0; j < tracker.hann.cols; j++) {
					*putHann++ = *(u32_t*)&p[j][0];
				}
			  }
			  memcpy(SHARED_DRAM_HANN, imageArray, hannSize);

		// Copy images:
		  u32_t		*putmp =  (u32_t*)imageArray;
		  for (int i = 0; i < frame.rows; i++) {
			  Vec3b *p = frame.ptr<Vec3b>(i);
			for (int j = 0; j < frame.cols; j++) {
				*putmp++ = *(u32_t*)&p[j][0];
			}
		  }
		  memcpy(SHARED_DRAM_IMAGE, imageArray, imgSize);

		result = tracker.update(
				SHARED_DRAM_IMAGE,SHARED_DRAM_HANN,SHARED_DRAM_PROB,
				frame,rectGroundTruth, nFrames );
		rectangle( frame, Point( result.x, result.y ), Point( result.x+result.width, result.y+result.height), Scalar( 0, 255, 255 ), 1, 8 );
		resultsFile << result.x << "," << result.y << "," << result.width << "," << result.height << endl;
		cout << result.x << "," << result.y << "," << result.width << "," << result.height << endl;
		nFrames++;
		if (!SILENT){
			imshow("Image", frame);
			waitKey(1);
		}
		free(SHARED_DRAM);
	}
	#ifdef BOARD_EN
		// End time
		gettimeofday(&fpgaDetetcEndT, NULL) ;
		fpgaDetetcEclipse = (fpgaDetetcEndT.tv_sec - fpgaDetetcStartT.tv_sec) * 1000;
		printf("\nFpga detect function eclipse : %16fms\n", fpgaDetetcEclipse);
	#endif
	resultsFile.close();
	listFramesFile.close();
#ifndef __SYNTHESIS__
  flieClose(outFile);
#endif
}
