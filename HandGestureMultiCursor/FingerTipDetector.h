#pragma once

// This class uses OpenCV
#include "stdafx.h"

namespace ftd
{
	// Informations of finger tips
	typedef struct{
		cv::Point2i points[5];
		cv::Point2i centroid;

		int numTips;
	} FingerTips;

}

class FingerTipDetector
{
	public:
		FingerTipDetector(void);
		~FingerTipDetector(void);

		void GetHandInfo(cv::Mat& handRegion, cv::Mat& headRegion, cv::Mat& cameraSpacePoints, hgmc::HandInfo& handInfoR, hgmc::HandInfo& handInfoL);

		bool CheckIsHandOnTable(hgmc::HandInfo handInfo, cv::Mat& tableParam);

		void CursorMove(hgmc::UserData userData, cv::Mat& TKinect2Table);

		void FindHands(cv::Mat& handRegion, cv::Mat& headRegion, cvb::CvBlob& handR, cvb::CvBlob& handL, cvb::CvBlob& head, cv::Mat& handLabel);

		CameraSpacePoint DetectHandCenter(cvb::CvBlob& hand, cvb::CvBlob& head, cv::Mat& handLabel, cvb::CvLabel label, const cv::Mat& cameraPoints);

		// <Sumarry>
		// Detect finger tips from the binary image
		// [Input]
		// - inHandImg:
		//		Binary image whcih hand resion are 255 and other resions are 0.
		//		Image should be CV_8UC1
		// [Output]
		// - Fingertips
		// </Sumarry>
		ftd::FingerTips FindFingerTips(cv::Mat& handRegion);
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
		void DrawTips(cv::Mat& src, const ftd::FingerTips& tips);


private:

	// Debug
	Mat resultImg;


	cvb::CvBlobs LabelingMat(const cv::Mat& src, cv::Mat& label = cv::Mat());
};
