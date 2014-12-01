#pragma once

// This class uses OpenCV
#include "stdafx.h"

class FingerTipDetector
{
	public:
		FingerTipDetector(void);
		~FingerTipDetector(void);

		cv::Mat GetHandInfo(cv::Mat& handRegion, cv::Mat& headRegion, cv::Mat& cameraSpacePoints, hgmc::HandInfo& handInfoR, hgmc::HandInfo& handInfoL);

		

private:

	// Debug
	cv::Mat resultImg;

	bool CheckIsHandOnTable(hgmc::HandInfo handInfo, cv::Mat& tableParam);

	void CursorMove(hgmc::UserData userData, cv::Mat& TKinect2Table);

	void FingerTipDetector::FindHands(cv::Mat& handRegion, cv::Mat& headRegion, cv::RotatedRect& handR, cv::RotatedRect& handL, cv::RotatedRect& head);
	//void FindHands(cv::Mat& handRegion, cv::Mat& headRegion, cvb::CvBlob& handR, cvb::CvBlob& handL, cvb::CvBlob& head, cv::Mat& handLabel);

	cv::Point3f FingerTipDetector::DetectHandCenter(cv::Mat handRegion, const cv::RotatedRect& hand, const cv::RotatedRect& head, const cv::Mat& cameraPoints, cv::Point2i& handPoint = cv::Point2i());
	//CameraSpacePoint DetectHandCenter(cvb::CvBlob& hand, cvb::CvBlob& head, cv::Mat& handLabel, cvb::CvLabel label, const cv::Mat& cameraPoints);

	cvb::CvBlobs LabelingMat(const cv::Mat& src, cv::Mat& label = cv::Mat());
};
