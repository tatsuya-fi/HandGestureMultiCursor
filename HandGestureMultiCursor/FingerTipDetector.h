#pragma once

// This class uses OpenCV
#include "MyLib.h"

using namespace std;
using namespace cv;


class FingerTipDetector
{
	public:
		FingerTipDetector(void);
		~FingerTipDetector(void);

		// Informations of finger tips
		typedef struct{
			Point2i points[5];
			Point2i centroid;

			int numTips;
		} FingerTips;

		// <Sumarry>
		// Detect finger tips from the binary image
		// [Input]
		// - inHandImg:
		//		Binary image whcih hand resion are 255 and other resions are 0.
		//		Image should be CV_8UC1
		// [Output]
		// - Fingertips
		// </Sumarry>
		FingerTips FindFingerTips(Mat& inHandImg);
		//FingerTips FindFingerTips(const Mat& inHandImg);

		// <Sumarry>
		// Draw finger tips on a image
		// [Input]
		// - const Mat& src:
		//		Tips are drawn on this image 
		//		Image should be CV_8UC3 or CV_8UC4
		// - const FingerTips& tips:
		//		Information of finger tips
		// </Sumarry>
		void DrawTips(Mat& src, const FingerTips& tips);
	};
