#include "stdafx.h"
#include "FingerTipDetector.h"

using namespace std;
using namespace cv;
using namespace cvb;
using namespace ftd;
using namespace hgmc;

FingerTipDetector::FingerTipDetector(void)
{
}


FingerTipDetector::~FingerTipDetector(void)
{
}

void FingerTipDetector::GetHandInfo(cv::Mat& handRegion, cv::Mat& headRegion, cv::Mat& cameraSpacePoints, hgmc::HandInfo& handInfoR, hgmc::HandInfo& handInfoL)
{
	// Separate hand information to R and L hand
	CvBlob handR, handL, head;
	Mat handLabel;
	FindHands(handRegion, headRegion, handR, handL, head, handLabel);

	// Set informations of each hands
	if (handR.label > 0)
	{
		handInfoR.area = handR.area;
		handInfoR.isTracked = true;

		CameraSpacePoint handPoint = DetectHandCenter(handR, head, handLabel, handR.label, cameraSpacePoints);
		handInfoR.cameraPoint.x = handPoint.X + 0.1;
		handInfoR.cameraPoint.y = handPoint.Y - 0.075;
		handInfoR.cameraPoint.z = handPoint.Z - 0.05;
	}
	else
	{
		handInfoR.isTracked = false;
	}

	if (handL.label > 0)
	{
		handInfoL.area = handL.area;
		handInfoL.isTracked = true;

		CameraSpacePoint handPoint = DetectHandCenter(handL, head, handLabel, handL.label, cameraSpacePoints);
		handInfoL.cameraPoint.x = handPoint.X + 0.1;
		handInfoL.cameraPoint.y = handPoint.Y - 0.075;
		handInfoL.cameraPoint.z = handPoint.Z - 0.05;
	}
	else
	{
		handInfoL.isTracked = false;
	}

	// Show debug image
	if (!resultImg.empty())
	{
		imshow("test2", resultImg);
	}
}

bool FingerTipDetector::CheckIsHandOnTable(hgmc::HandInfo handInfo, cv::Mat& tableParam)
{
	if (!handInfo.isTracked) { return false; }

	Mat handPoint = (Mat_<double>(4, 1) << handInfo.cameraPoint.x * 1000, handInfo.cameraPoint.y * 1000, handInfo.cameraPoint.z * 1000, 1.0);
	// Convert hand point to table coordinate 
	//handPoint = TKinect2Table * handPoint;
	cout << handPoint << endl;

	return true;
}

void FingerTipDetector::CursorMove(hgmc::UserData userData, cv::Mat& TKinect2Table)
{

}


void FingerTipDetector::FindHands(cv::Mat& handRegion, cv::Mat& headRegion, cvb::CvBlob& handR, cvb::CvBlob& handL, cvb::CvBlob& head, cv::Mat& handLabel)
{
	CvBlobs handBlobs = LabelingMat(handRegion, handLabel);
	CvBlobs headBlobs = LabelingMat(headRegion);

	if (headBlobs.empty() || handBlobs.empty()) { return; }
	
	// Set head blob
	head = *headBlobs.begin()->second;

	// 右手左手を判別 / Detect each hand
	int countNumHands = 0;
	CvBlobs::iterator itHead = headBlobs.begin();
	for (CvBlobs::iterator itHand = handBlobs.begin(); itHand != handBlobs.end(); ++itHand)
	{
		// 2直線(体軸と手軸)の交点を求める / Calc intersection point between 2lines (body axis and hand axis)
		double angleHead = cvAngle(itHead->second);
		double angleHand = cvAngle(itHand->second);

		Mat A = (Mat_<double>(2, 2) <<
			sin(angleHead), -cos(angleHead),
			sin(angleHand), -cos(angleHand)
			);
		Mat B = (Mat_<double>(2, 1) <<
			itHead->second->centroid.x * sin(angleHead) - itHead->second->centroid.y * cos(angleHead),
			itHand->second->centroid.x * sin(angleHand) - itHand->second->centroid.y * cos(angleHand)
			);

		Mat C = A.inv() * B;

		resultImg = (headRegion + handRegion);
		circle(resultImg, Point(*C.ptr<double>(0, 0), *C.ptr<double>(1, 0)), 4, Scalar(200, 200, 100), 3);



		// 上下どちらを向いているかを判別 / Detect which user is looking up or down
		bool isLookingUp = true;
		0 < (sin(angleHead) * (itHand->second->centroid.x - itHead->second->centroid.x) - cos(angleHead) * (itHand->second->centroid.y - itHead->second->centroid.y)) ?
			isLookingUp = true :
			isLookingUp = false;
		//isLookingUp ? cout << "Up" << endl : cout << "Down" << endl;

		// 右手左手の判別
		bool isLeftHand = true;
		if (isLookingUp)
		{
			0 < (cos(angleHead) * (*C.ptr<double>(0, 0) - itHead->second->centroid.x) + sin(angleHead) * (*C.ptr<double>(1, 0) - itHead->second->centroid.y)) ?
				isLeftHand = true :
				isLeftHand = false;
		}
		else
		{
			0 < (cos(angleHead) * (*C.ptr<double>(0, 0) - itHead->second->centroid.x) + sin(angleHead) * (*C.ptr<double>(1, 0) - itHead->second->centroid.y)) ?
				isLeftHand = false:
				isLeftHand = true ;
		}
		//isLeftHand ? cout << "left" << endl : cout << "right" << endl;

		// Set hand info
		isLeftHand ?
			handL = *itHand->second :
			handR = *itHand->second;

		++countNumHands;
		if (countNumHands > 2) { break; }
	}
}

CameraSpacePoint FingerTipDetector::DetectHandCenter(cvb::CvBlob& hand, cvb::CvBlob& head, cv::Mat& handLabel, cvb::CvLabel label, const cv::Mat& cameraPoints)
{
	// Calc intersection point between rectangle of hand and hand line
	double angle = cvAngle(&hand);
	double absAngle = abs(angle);
	Point2i anchor1, anchor2;
	if (abs(angle) < M_PI / 4)
	{
		anchor1.x = hand.minx;
		anchor1.y = (int)(tan(angle) * (hand.minx - hand.centroid.x) + hand.centroid.y);

		anchor2.x = hand.maxx;
		anchor2.y = (int)(tan(angle) * (hand.maxx - hand.centroid.x) + hand.centroid.y);
	}
	else
	{
		anchor1.x = (int)(1 / tan(angle) * (hand.miny - hand.centroid.y) + hand.centroid.x);
		anchor1.y = hand.miny;

		anchor2.x = (int)(1 / tan(angle) * (hand.maxy - hand.centroid.y) + hand.centroid.x);
		anchor2.y = hand.maxy;
	}

	int distance1 = pow(head.centroid.x - anchor1.x, 2) + pow(head.centroid.y - anchor1.y, 2);
	int distance2 = pow(head.centroid.x - anchor2.x, 2) + pow(head.centroid.y - anchor2.y, 2);

	// Decide far point as a anchor point of the hand
	Point2i anchor;
	if (distance1 > distance2)
	{
		anchor.x = anchor1.x;
		anchor.y = anchor1.y;
	}
	else
	{
		anchor.x = anchor2.x;
		anchor.y = anchor2.y;
	}

	circle(resultImg, anchor, 4, Scalar(200, 100, 200), 3);

	// Calc centroid of the hand
	const int offset = 50;
	CameraSpacePoint aveHand; aveHand.X = 0; aveHand.Y = 0; aveHand.Z = 0;
	Point2i debugPoint;
	int count = 0;
	for (int y = anchor.y - offset / 2; y < anchor.y + offset / 2; ++y)
	{
		if (y < 0) { continue; }
		else if (y > handLabel.rows) { break; }
		for (int x = anchor.x - offset / 2; x < anchor.x + offset / 2; ++x)
		{
			if (x < 0) { continue; }
			else if (x > handLabel.cols) { break; }

			if (*handLabel.ptr<unsigned long>(y, x) == static_cast<unsigned long>(label))
			{
				aveHand.X += cameraPoints.ptr<float>(y, x)[0];
				aveHand.Y += cameraPoints.ptr<float>(y, x)[1];
				aveHand.Z += cameraPoints.ptr<float>(y, x)[2];
				++count;

				debugPoint.x += x;
				debugPoint.y += y;
			}
		}
	}

	if (count > 0)
	{
		aveHand.X /= count;
		aveHand.Y /= count;
		aveHand.Z /= count;

		debugPoint.x /= count;
		debugPoint.y /= count;
		circle(resultImg, debugPoint, 4, Scalar(200, 100, 100), 3);


		return aveHand;
	}
	else
	{
		return CameraSpacePoint();
	}
}


FingerTips FingerTipDetector::FindFingerTips(Mat& handRegion)
{
	FingerTips tips;

	// Find the biggest region
	CvBlobs blobs = LabelingMat(handRegion);
	//cout << blobs[0]->contour.startingPoint.x << ", " << blobs[0]->contour.startingPoint.y << endl;
	
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


CvBlobs FingerTipDetector::LabelingMat(const Mat& src, Mat& label)
{
	/* Use IplImage (Labeling for Mat is not fully implemented) */
	// Convert to IplImage
	IplImage srcIpl = (IplImage)src;
	// Convert to gray scale
	IplImage* srcIplBinary = cvCreateImage(cvGetSize(&srcIpl), IPL_DEPTH_8U, 1);
	cvCvtColor(&srcIpl, srcIplBinary, CV_BGR2GRAY);
	// Get binary image
	cvThreshold(srcIplBinary, srcIplBinary, 100, 255, CV_THRESH_BINARY);
	// Get blobs
	IplImage* labelImg = cvCreateImage(cvGetSize(srcIplBinary), IPL_DEPTH_LABEL, 1);

	CvBlobs blobs;
	UINT result = cvLabel(srcIplBinary, labelImg, blobs);

	Mat labelBuf(labelImg, true); // Convert to Mat
	// Assert if Mat data is not copied
	CV_Assert(reinterpret_cast<uchar*>(labelImg->imageData) != label.data);
	label = labelBuf;

	// Filter noise / ノイズ点の消去
	cvFilterByArea(blobs, 1000, 1000000);

	// Render blobs
	cvRenderBlobs(labelImg, blobs, &srcIpl, &srcIpl);

	// Free unused IplImages
	cvReleaseImage(&labelImg);
	cvReleaseImage(&srcIplBinary);

	return blobs;
}