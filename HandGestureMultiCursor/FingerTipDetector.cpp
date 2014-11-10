#include "stdafx.h"
#include "FingerTipDetector.h"

FingerTipDetector::FingerTipDetector(void)
{
}


FingerTipDetector::~FingerTipDetector(void)
{
}

FingerTipDetector::FingerTips FingerTipDetector::FindFingerTips( Mat& inHandImg)
{
	FingerTips tips;

	// Calcurate centroid
//	Moments m = moments(inHandImg, true); // Calcurate moments
	//tips.centroid.x = (int)(m.m10/m.m00); // Centroid X
	//tips.centroid.y = (int)(m.m01/m.m00); // Centroid Y


	//circle(inHandImg,tips.centroid, 2, Scalar(255, 255, 0), 1);



	return tips;
}

void FingerTipDetector::DrawTips(Mat& src, const FingerTips& tips)
{
	for (int i = 0; i < tips.numTips; ++i)
	{
		circle(src, tips.points[i], 3, Scalar(0, 0, 255), 2);
	}
}
