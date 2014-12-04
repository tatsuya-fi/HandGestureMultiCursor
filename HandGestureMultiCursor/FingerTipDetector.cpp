#include "stdafx.h"
#include "FingerTipDetector.h"

using namespace std;
using namespace cv;
using namespace cvb;
using namespace hgmc;

FingerTipDetector::FingerTipDetector(void)
{
}


FingerTipDetector::~FingerTipDetector(void)
{
}

cv::Mat FingerTipDetector::GetHandInfo(cv::Mat& handRegion, cv::Mat& headRegion, cv::Mat& cameraSpacePoints, hgmc::HandInfo& handInfoR, hgmc::HandInfo& handInfoL)
{
	Mat handRegionBuf = handRegion.clone();
	Mat headRegionBuf = headRegion.clone();

	erode(handRegionBuf, handRegionBuf, Mat(), Point(-1, -1), 1);
	dilate(handRegionBuf, handRegionBuf, Mat(), Point(-1, -1), 2);

	RotatedRect handL, handR, head;
	FindHands(handRegionBuf, headRegionBuf, handR, handL, head);

	// ���F���o����������
	handInfoR.isTracked = (handR.size.width == 0 || handR.size.height == 0) ? false : true;
	handInfoL.isTracked = (handL.size.width == 0 || handL.size.height == 0) ? false : true;

	// ��̍��W�l(2����, 3��������)�擾
	handInfoR.cameraPoint = DetectHandCenter(handRegionBuf, handR, head, cameraSpacePoints, handInfoR.depthPoint);
	handInfoL.cameraPoint = DetectHandCenter(handRegionBuf, handL, head, cameraSpacePoints, handInfoL.depthPoint);

	// ��̈�̍��W�l���Ƃ��Ă���
	handInfoR.centroid3f = Point3f(cameraSpacePoints.ptr<float>(handR.center.y, handR.center.x)[0], cameraSpacePoints.ptr<float>(handR.center.y, handR.center.x)[1], cameraSpacePoints.ptr<float>(handR.center.y, handR.center.x)[2]);
	handInfoL.centroid3f = Point3f(cameraSpacePoints.ptr<float>(handL.center.y, handL.center.x)[0], cameraSpacePoints.ptr<float>(handL.center.y, handL.center.x)[1], cameraSpacePoints.ptr<float>(handL.center.y, handL.center.x)[2]);
	
	// Debug�\���p�̉摜�쐬
	Mat userArea = handRegionBuf + headRegionBuf;

	return userArea;
}

void FingerTipDetector::FindHands(cv::Mat& handRegion, cv::Mat& headRegion, cv::RotatedRect& handR, cv::RotatedRect& handL, cv::RotatedRect& head)
{
	// ���ɑȉ~�t�B�b�e�B���O
	Mat grayHead, binHead;
	cvtColor(headRegion, grayHead, CV_BGR2GRAY);
	vector<vector<Point> > contoursHead;
	// �摜�̓�l��
	cv::threshold(grayHead, binHead, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	// �֊s�̌��o
	cv::findContours(binHead, contoursHead, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	// �ł��傫���̈�𓪂Ƃ���
	int largestHeadID = -1, largestHeadCount = 0;
	for (int i = 0; i < contoursHead.size(); ++i)
	{
		size_t count = contoursHead[i].size();

		if (count < 100 || count > 1000) { continue; }

		if (count > largestHeadCount)
		{
			largestHeadCount = count;
			largestHeadID = i;
		}
	}
	// ���̈�ɑȉ~�t�B�b�e�B���O
	if (largestHeadID >= 0)
	{
		Mat pointsf;
		Mat(contoursHead[largestHeadID]).convertTo(pointsf, CV_32F);
		// �ȉ~�t�B�b�e�B���O
		head = cv::fitEllipse(pointsf);
		// �ȉ~�̕`��
		ellipse(headRegion, head, cv::Scalar(0, 0, 255), 2, CV_AA);

	}


	// ���r�ɑȉ~�t�B�b�e�B���O
	Mat grayHand, binHand;
	cvtColor(handRegion, grayHand, CV_BGR2GRAY);
	vector<vector<Point> > contoursHand;
	// �摜�̓�l��
	cv::threshold(grayHand, binHand, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	// �֊s�̌��o
	cv::findContours(binHand, contoursHand, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	// �傫���̈�2��r�Ƃ���
	int handID[2] = { -1, -1 }, handCount[2] = { 0, 0 };
	for (int i = 0; i < contoursHand.size(); ++i)
	{
		size_t count = contoursHand[i].size();
		if (count < 120 || count > 500) { continue; }

		int id = i;
		if (count > handCount[0])
		{
			int handIDBuf = handID[0];
			int handCountBuf = handCount[0];

			handID[0] = id;
			handCount[0] = count;

			id = handIDBuf;
			count = handCountBuf;
		}
		if (count < handCount[0] && count > handCount[1])
		{
			handID[1] = id;
			handCount[1] = count;
		}
	}
	RotatedRect boxHands[2];
	for (int i = 0; i < 2; ++i) {
		if (handCount[i] < 5) { continue; }

		Mat pointsf;
		Mat(contoursHand[handID[i]]).convertTo(pointsf, CV_32F);
		// �ȉ~�t�B�b�e�B���O
		boxHands[i] = cv::fitEllipse(pointsf);
	}

	bool isLeftHand[2] = { true, true };
	bool isLookingUp[2] = { true, true };
	bool isFoundFirstHand = false;
	for (int i = 0; i < 2; ++i)
	{
		// ���o�~�X�ɂ��m�C�Y����
		if (boxHands[i].size.width <= 0 || boxHands[i].size.height <= 0
			|| boxHands[i].size.height / boxHands[i].size.width < 2.5)	// �~�ɋ߂����̂͘r�Ƃ݂Ȃ��Ȃ�
		{
			continue;
		}

		// �ȉ~�̉�]�p [pi]
		float angleHead = (head.angle - 90.0f) / 180.0f * M_PI;
		float angleHand = (boxHands[i].angle - 90.0f) / 180.0f * M_PI;

		// 2����(�̎��Ǝ莲)�̌�_�����߂� / Calc intersection point between 2lines (body axis and hand)
		Mat A = (Mat_<float>(2, 2) <<
			sin(angleHead), -cos(angleHead),
			sin(angleHand), -cos(angleHand)
			);
		Mat B = (Mat_<float>(2, 1) <<
			head.center.x * sin(angleHead) - head.center.y * cos(angleHead),
			boxHands[i].center.x * sin(angleHand) - boxHands[i].center.y * cos(angleHand)
			);

		Mat C = A.inv() * B;
		
		circle(headRegion, Point(*C.ptr<float>(0, 0), *C.ptr<float>(1, 0)), 4, Scalar(200, 200, 100), 3);
				
		// �㉺�ǂ���������Ă��邩�𔻕� / Detect which user is looking up or down
		0 < (sin(angleHead) * (boxHands[i].center.x - head.center.x) - cos(angleHead) * (boxHands[i].center.y - head.center.y)) ?
			isLookingUp[i] = true :
			isLookingUp[i] = false;
		//isLookingUp ? cout << "Up" << endl : cout << "Down" << endl;

		// ���E�ő̂̌��������������ꍇ��2�ڂ̘r�͌��o���Ȃ�
		if (i == 1)
		{
			if (isLookingUp[0] != isLookingUp[1])
			{
				return;
			}
		}

		// �E�荶��̔���
		if (isLookingUp[i])
		{
			0 < (cos(angleHead) * (*C.ptr<float>(0, 0) - head.center.x) + sin(angleHead) * (*C.ptr<float>(1, 0) - head.center.y)) ?
				isLeftHand[i] = true :
				isLeftHand[i] = false;
		}
		else
		{
			0 < (cos(angleHead) * (*C.ptr<float>(0, 0) - head.center.x) + sin(angleHead) * (*C.ptr<float>(1, 0) - head.center.y)) ?
				isLeftHand[i] = false :
				isLeftHand[i] = true;
		}
		isLeftHand ? cout << "left" << endl : cout << "right" << endl;

		// 2�{�Ƃ������r�̏ꍇ����
		if (i == 1)
		{
			if (isLeftHand[0] == isLeftHand[1])
			{
				return;
			}
		}

		// Set hand info
		isLeftHand[i] ?
			handL = boxHands[i] :
			handR = boxHands[i];

		isFoundFirstHand = true;
	}
}

cv::Point3f FingerTipDetector::DetectHandCenter(cv::Mat handRegion, const cv::RotatedRect& hand, const cv::RotatedRect& head, const cv::Mat& cameraPoints, cv::Point2i& handPoint)
{
	// Calc intersection point between rectangle of hand and hand line
	float angle = (hand.angle - 90.0f) / 180.0f * M_PI;
	float absAngle = abs(angle);
	Point2i anchor1, anchor2;
	if (abs(angle) < M_PI / 4)
	{
		anchor1.x = hand.boundingRect().tl().x;
		anchor1.y = (int)(tan(angle) * (hand.boundingRect().tl().x - hand.center.x) + hand.center.y);

		anchor2.x = hand.boundingRect().br().x;
		anchor2.y = (int)(tan(angle) * (hand.boundingRect().br().x - hand.center.x) + hand.center.y);
	}
	else
	{
		anchor1.x = (int)(1 / tan(angle) * (hand.boundingRect().tl().y - hand.center.y) + hand.center.x);
		anchor1.y = hand.boundingRect().tl().y;

		anchor2.x = (int)(1 / tan(angle) * (hand.boundingRect().br().y - hand.center.y) + hand.center.x);
		anchor2.y = hand.boundingRect().br().y;
	}

	int distance1 = pow(head.center.x - anchor1.x, 2) + pow(head.center.y - anchor1.y, 2);
	int distance2 = pow(head.center.x - anchor2.x, 2) + pow(head.center.y - anchor2.y, 2);

	// Decide far point as a anchor point of the hand
	if (distance1 > distance2)
	{
		handPoint.x = anchor1.x;
		handPoint.y = anchor1.y;
	}
	else
	{
		handPoint.x = anchor2.x;
		handPoint.y = anchor2.y;
	}

	// Calc centroid of the hand
	const int offset = 50;
	Point3f aveHand; aveHand.x = 0; aveHand.y = 0; aveHand.z = 0;
	Point2i debugPoint;
	int count = 0;
	for (int y = handPoint.y - offset / 2; y < handPoint.y + offset / 2; ++y)
	{
		if (y < 0 || kinectBasics.heightDepth < y) { continue; }
		//else if (y > handRegion.rows || y > hand.boundingRect().br().y) { break; }
		for (int x = handPoint.x - offset / 2; x < handPoint.x + offset / 2; ++x)
		{
			if (x < 0 || kinectBasics.widthDepth < x) { continue; }
			//else if (x > handRegion.cols || x > hand.boundingRect().br().x) { break; }

			if (handRegion.at<Vec3b>(y, x)[0] == 255 && handRegion.at<Vec3b>(y, x)[1] == 255 && handRegion.at<Vec3b>(y, x)[2] == 255
				&& 0.4 < cameraPoints.ptr<float>(y, x)[2] && cameraPoints.ptr<float>(y, x)[2] < KINECT_HEIGHT)
			{
				//circle(handRegion, Point(x, y), 1, Scalar(0, 0, 200), -1);
				aveHand.x += cameraPoints.ptr<float>(y, x)[0];
				aveHand.y += cameraPoints.ptr<float>(y, x)[1];
				aveHand.z += cameraPoints.ptr<float>(y, x)[2];
				++count;

				debugPoint.x += x;
				debugPoint.y += y;
			}
		}
	}

	if (count > 0)
	{

		aveHand.x /= count;
		aveHand.y /= count;
		aveHand.z /= count;

		debugPoint.x /= count;
		debugPoint.y /= count;
		handPoint = debugPoint;

		// Debug
		// �r�̈�
		ellipse(handRegion, hand, cv::Scalar(0, 0, 255), 2, CV_AA);
		// �r�̐�
		//circle(handRegion, handPoint, 4, Scalar(200, 100, 200), 3);
		// �蒆�S
		circle(handRegion, debugPoint, 4, Scalar(200, 100, 100), 3);

		return aveHand;
	}
	else
	{
		return Point3f();
	}
}


bool FingerTipDetector::CheckIsHandOnTable(hgmc::HandInfo handInfo, cv::Mat& tableParam)
{
	if (!handInfo.isTracked) { return false; }

	Mat handPoint = (Mat_<double>(4, 1) << handInfo.cameraPoint.x * 1000, handInfo.cameraPoint.y * 1000, handInfo.cameraPoint.z * 1000, 1.0);
	// Convert hand point to table coordinate 
	//handPoint = TKinect2Table * handPoint;
	//cout << handPoint << endl;

	return true;
}

void FingerTipDetector::CursorMove(hgmc::UserData userData, cv::Mat& TKinect2Table)
{

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

	// Filter noise / �m�C�Y�_�̏���
	cvFilterByArea(blobs, 900, 1000000);
	
	// Render blobs
	cvRenderBlobs(labelImg, blobs, &srcIpl, &srcIpl);

	// Free unused IplImages
	cvReleaseImage(&labelImg);
	cvReleaseImage(&srcIplBinary);

	return blobs;
}



// �ȉ�cvblob���g���o�[�W���� �ᑬ�̂��ߎg��Ȃ����Ƃ�
//void FingerTipDetector::FindHands(cv::Mat& handRegion, cv::Mat& headRegion, static cvb::CvBlob& handR, static cvb::CvBlob& handL, cvb::CvBlob& head, cv::Mat& handLabel)
//{
//	CvBlobs handBlobs = LabelingMat(handRegion, handLabel);
//	CvBlobs headBlobs = LabelingMat(headRegion);
//
//	if (headBlobs.empty() || handBlobs.empty()) { return; }
//	
//	// Set head blob
//	head = *headBlobs.begin()->second;
//
//	// �E�荶��𔻕� / Detect each hand
//	int countNumHands = 0;
//	CvBlobs::iterator itHead = headBlobs.begin();
//	for (CvBlobs::iterator itHand = handBlobs.begin(); itHand != handBlobs.end(); ++itHand)
//	{
//		// 2����(�̎��Ǝ莲)�̌�_�����߂� / Calc intersection point between 2lines (body axis and hand axis)
//		double angleHead = cvAngle(itHead->second);
//		double angleHand = cvAngle(itHand->second);
//
//		Mat A = (Mat_<double>(2, 2) <<
//			sin(angleHead), -cos(angleHead),
//			sin(angleHand), -cos(angleHand)
//			);
//		Mat B = (Mat_<double>(2, 1) <<
//			itHead->second->centroid.x * sin(angleHead) - itHead->second->centroid.y * cos(angleHead),
//			itHand->second->centroid.x * sin(angleHand) - itHand->second->centroid.y * cos(angleHand)
//			);
//
//		Mat C = A.inv() * B;
//
//		resultImg = (headRegion + handRegion);
//		circle(resultImg, Point(*C.ptr<double>(0, 0), *C.ptr<double>(1, 0)), 4, Scalar(200, 200, 100), 3);
//
//
//
//		// �㉺�ǂ���������Ă��邩�𔻕� / Detect which user is looking up or down
//		bool isLookingUp = true;
//		0 < (sin(angleHead) * (itHand->second->centroid.x - itHead->second->centroid.x) - cos(angleHead) * (itHand->second->centroid.y - itHead->second->centroid.y)) ?
//			isLookingUp = true :
//			isLookingUp = false;
//		//isLookingUp ? cout << "Up" << endl : cout << "Down" << endl;
//
//		// �E�荶��̔���
//		bool isLeftHand = true;
//		if (isLookingUp)
//		{
//			0 < (cos(angleHead) * (*C.ptr<double>(0, 0) - itHead->second->centroid.x) + sin(angleHead) * (*C.ptr<double>(1, 0) - itHead->second->centroid.y)) ?
//				isLeftHand = true :
//				isLeftHand = false;
//		}
//		else
//		{
//			0 < (cos(angleHead) * (*C.ptr<double>(0, 0) - itHead->second->centroid.x) + sin(angleHead) * (*C.ptr<double>(1, 0) - itHead->second->centroid.y)) ?
//				isLeftHand = false:
//				isLeftHand = true ;
//		}
//		//isLeftHand ? cout << "left" << endl : cout << "right" << endl;
//
//		// Set hand info
//		isLeftHand ?
//			handL = *itHand->second :
//			handR = *itHand->second;
//
//		++countNumHands;
//		if (countNumHands > 2) { break; }
//	}
//}

//void FingerTipDetector::GetHandInfo(cv::Mat& handRegion, cv::Mat& headRegion, cv::Mat& cameraSpacePoints, hgmc::HandInfo& handInfoR, hgmc::HandInfo& handInfoL)
//{
//	// Separate hand information to R and L hand
//	CvBlob handRbuf, handLbuf, head;
//	Mat handLabel;
//	FindHands(handRegion, headRegion, handRbuf, handLbuf, head, handLabel);
//
//	// ���߂Ċi�[ �������Ȃ��Ɖ��̂�����ɊJ������
//	CvBlob handR = handRbuf;
//	CvBlob handL = handLbuf;
//	// Set informations of each hands
//	if (0 < handR.area && handR.area < 100000)
//	{
//		handInfoR.area = handR.area;
//		handInfoR.isTracked = true;
//
//		CameraSpacePoint handPoint = DetectHandCenter(handR, head, handLabel, handR.label, cameraSpacePoints);
//		handInfoR.cameraPoint.x = handPoint.X + 0.1;
//		handInfoR.cameraPoint.y = handPoint.Y - 0.035;
//		handInfoR.cameraPoint.z = handPoint.Z - 0.075;
//
//		handInfoR.centroid3f.x = cameraSpacePoints.ptr<float>(handR.centroid.y, handR.centroid.x)[0];
//		handInfoR.centroid3f.y = cameraSpacePoints.ptr<float>(handR.centroid.y, handR.centroid.x)[1];
//		handInfoR.centroid3f.z = cameraSpacePoints.ptr<float>(handR.centroid.y, handR.centroid.x)[2];
//	}
//	else
//	{
//		handInfoR.isTracked = false;
//	}
//	if (0 < handL.area && handL.area < 100000)
//	{
//		handInfoL.area = handL.area;
//		handInfoL.isTracked = true;
//
//		CameraSpacePoint handPoint = DetectHandCenter(handL, head, handLabel, handL.label, cameraSpacePoints);
//		handInfoL.cameraPoint.x = handPoint.X + 0.1;
//		handInfoL.cameraPoint.y = handPoint.Y - 0.075;
//		handInfoL.cameraPoint.z = handPoint.Z - 0.075;
//
//		handInfoL.centroid3f.x = cameraSpacePoints.ptr<float>(handL.centroid.y, handL.centroid.x)[0];
//		handInfoL.centroid3f.y = cameraSpacePoints.ptr<float>(handL.centroid.y, handL.centroid.x)[1];
//		handInfoL.centroid3f.z = cameraSpacePoints.ptr<float>(handL.centroid.y, handL.centroid.x)[2];
//	}
//	else
//	{
//		handInfoL.isTracked = false;
//	}
//
//	// Show debug image
//	if (!resultImg.empty())
//	{
//		imshow("test2", resultImg);
//	}
//}
//
//CameraSpacePoint FingerTipDetector::DetectHandCenter(cvb::CvBlob& hand, cvb::CvBlob& head, cv::Mat& handLabel, cvb::CvLabel label, const cv::Mat& cameraPoints)
//{
//	// Calc intersection point between rectangle of hand and hand line
//	double angle = cvAngle(&hand);
//	double absAngle = abs(angle);
//	Point2i anchor1, anchor2;
//	if (abs(angle) < M_PI / 4)
//	{
//		anchor1.x = hand.minx;
//		anchor1.y = (int)(tan(angle) * (hand.minx - hand.centroid.x) + hand.centroid.y);
//
//		anchor2.x = hand.maxx;
//		anchor2.y = (int)(tan(angle) * (hand.maxx - hand.centroid.x) + hand.centroid.y);
//	}
//	else
//	{
//		anchor1.x = (int)(1 / tan(angle) * (hand.miny - hand.centroid.y) + hand.centroid.x);
//		anchor1.y = hand.miny;
//
//		anchor2.x = (int)(1 / tan(angle) * (hand.maxy - hand.centroid.y) + hand.centroid.x);
//		anchor2.y = hand.maxy;
//	}
//
//	int distance1 = pow(head.centroid.x - anchor1.x, 2) + pow(head.centroid.y - anchor1.y, 2);
//	int distance2 = pow(head.centroid.x - anchor2.x, 2) + pow(head.centroid.y - anchor2.y, 2);
//
//	// Decide far point as a anchor point of the hand
//	Point2i anchor;
//	if (distance1 > distance2)
//	{
//		anchor.x = anchor1.x;
//		anchor.y = anchor1.y;
//	}
//	else
//	{
//		anchor.x = anchor2.x;
//		anchor.y = anchor2.y;
//	}
//
//	//circle(resultImg, anchor, 4, Scalar(200, 100, 200), 3);
//
//	// Calc centroid of the hand
//	const int offset = 70;
//	CameraSpacePoint aveHand; aveHand.X = 0; aveHand.Y = 0; aveHand.Z = 0;
//	Point2i debugPoint;
//	int count = 0;
//	for (int y = anchor.y - offset / 2; y < anchor.y + offset / 2; ++y)
//	{
//		if (y < 0) { continue; }
//		else if (y > handLabel.rows) { break; }
//		for (int x = anchor.x - offset / 2; x < anchor.x + offset / 2; ++x)
//		{
//			if (x < 0) { continue; }
//			else if (x > handLabel.cols) { break; }
//
//			if (*handLabel.ptr<unsigned long>(y, x) == static_cast<unsigned long>(label))
//			{
//				aveHand.X += cameraPoints.ptr<float>(y, x)[0];
//				aveHand.Y += cameraPoints.ptr<float>(y, x)[1];
//				aveHand.Z += cameraPoints.ptr<float>(y, x)[2];
//				++count;
//
//				debugPoint.x += x;
//				debugPoint.y += y;
//			}
//		}
//	}
//
//	if (count > 0)
//	{
//		aveHand.X /= count;
//		aveHand.Y /= count;
//		aveHand.Z /= count;
//
//		debugPoint.x /= count;
//		debugPoint.y /= count;
//		circle(resultImg, debugPoint, 4, Scalar(200, 100, 100), 3);
//
//
//		return aveHand;
//	}
//	else
//	{
//		return CameraSpacePoint();
//	}
//}