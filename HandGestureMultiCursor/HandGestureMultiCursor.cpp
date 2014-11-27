// HandGestureMultiCursoror.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//
#include "stdafx.h"
#include "HandGestureMultiCursor.h"

using namespace std;
using namespace cv;
using namespace cvb;
using namespace ftd;
using namespace hgmc;

HandGestureMultiCursor::HandGestureMultiCursor():
	isShowDebugWindows(true)
{
}

HandGestureMultiCursor::~HandGestureMultiCursor()
{
	// �I������
#ifdef USE_KINECT_V1
	if (kinect != 0) {
		//kinect->NuiShutdown();
		kinect->Release();
	}
#else
	// Kinect V2�̏I��������KinectV2Basics�̃f�X�g���N�^���s���܂�
#endif

	// CV�̃E�B���h�E�j���i�O�̂���)
	destroyAllWindows();

}


#pragma region KINECT V1
#ifdef USE_KINECT_V1

void HandGestureMultiCursor::createInstance()
{
	// �ڑ�����Ă���Kinect�̐����擾����
	int count = 0;
	if (S_OK != ::NuiGetSensorCount(&count)) {
		cout << "Error: NuiGetSensorCount()" << endl;
		exit(0);
	}
	if (count == 0) {
		throw std::runtime_error("Kinect ��ڑ����Ă�������");
	}

	// �ŏ���Kinect�̃C���X�^���X���쐬����
	if (S_OK != ::NuiCreateSensorByIndex(0, &kinect)) {
		cout << "Error: NuiCreateSensorByIndex" << endl;
		exit(0);
	}

	// Kinect�̏�Ԃ��擾����
	HRESULT status = kinect->NuiStatus();
	if (status != S_OK) {
		throw std::runtime_error("Kinect �����p�\�ł͂���܂���");
	}
}

void HandGestureMultiCursor::initKinectV1()
{
	createInstance();

	// Kinect�̐ݒ������������
	if (S_OK != kinect->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH)) {
		cout << "Error: NuiInitialize() " << endl;
		exit(0);
	}

	// RGB�J����������������
	const NUI_IMAGE_RESOLUTION KINECT_RGB_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;
	if (S_OK != (kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, KINECT_RGB_RESOLUTION,
		0, 2, 0, &imageStreamHandle))) {
		cout << "Error: NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR)" << endl;
		exit(0);
	}
	
	// �����J����������������
	if (S_OK != (kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, KINECT_RESOLUTION,
		0, 2, 0, &depthStreamHandle))) {
		cout << "Error: NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH)" << endl;
		exit(0);
	}

#ifdef NEAR_MODE
	// Near���[�h
	if (S_OK != (kinect->NuiImageStreamSetImageFrameFlags(
		depthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE))) {
		cout << "Error: NuiImageStreamSetImageFrameFlags()" << endl;
		exit(0);
	}
#endif

	// �t���[���X�V�C�x���g�̃n���h�����쐬����
	streamEvent = ::CreateEvent(0, TRUE, FALSE, 0);
	if (S_OK != (kinect->NuiSetFrameEndEvent(streamEvent, 0))) {
		cout << "Error: NuiSetFrameEndEvent()" << endl;
		exit(0);
	}

	// �w�肵���𑜓x�́A��ʃT�C�Y���擾����
	DWORD width, height;
	::NuiImageResolutionToSize(KINECT_RESOLUTION, width, height);
	CAMERA_WIDTH = (int)width;
	CAMERA_HEIGHT = (int)height;
}

bool HandGestureMultiCursor::getDepthImageV1()
{
	// Initialize matrix for each data
	userAreaMat = Mat::zeros(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC3);
	point3fMatrix = Mat::zeros(CAMERA_HEIGHT, CAMERA_WIDTH, CV_32FC3);
	heightMatrix = Mat::zeros(CAMERA_HEIGHT, CAMERA_WIDTH, CV_16U);


	// Get the frame data of the depth camera
	NUI_IMAGE_FRAME depthFrame = { 0 };
	if (kinect->NuiImageStreamGetNextFrame(depthStreamHandle, 0, &depthFrame) < 0) {
		return false;
	}

	// Get the actual depth data
	NUI_LOCKED_RECT depthData = { 0 };
	depthFrame.pFrameTexture->LockRect(0, &depthData, 0, 0);

	USHORT* depth = (USHORT*)depthData.pBits;
	for (int i = 0; i < (depthData.size / sizeof(USHORT)); ++i) {
		USHORT distance = ::NuiDepthPixelToDepth(depth[i]);

		LONG depthX = i % CAMERA_WIDTH;
		LONG depthY = i / CAMERA_WIDTH;

		int index = ((depthY * CAMERA_WIDTH) + depthX) * 3;
		UCHAR* dataDepth = &userAreaMat.data[index];

		// ���������L�^ / Set the height from floor
		USHORT heightFromFloor;
		(0 < distance && distance < KINECT_HEIGHT) ? heightFromFloor = KINECT_HEIGHT - distance : heightFromFloor = 0;
		*heightMatrix.ptr<USHORT>(depthY, depthX) = heightFromFloor;

		// ���[�U�̈���L�� / Define user area
		if (USER_HEIGHT_THRESHOLD <= heightFromFloor && heightFromFloor <= HEAD_HEIGHT_MAX) {
			dataDepth[0] = 255;
			dataDepth[1] = 255;
			dataDepth[2] = 255;
		}
		else {
			dataDepth[0] = 0;
			dataDepth[1] = 0;
			dataDepth[2] = 0;
		}

		// �|�C���g�N���E�h���L�� / Set 3D point data
		Vector4 realPoint = NuiTransformDepthImageToSkeleton(depthX, depthY, distance << 3, KINECT_RESOLUTION);
		point3fMatrix.ptr<float>(depthY, depthX)[0] = realPoint.x;
		point3fMatrix.ptr<float>(depthY, depthX)[1] = realPoint.y;
		point3fMatrix.ptr<float>(depthY, depthX)[2] = realPoint.z;
	}

	// Release each data
	if (S_OK != (kinect->NuiImageStreamReleaseFrame(depthStreamHandle, &depthFrame))) {
		cout << "Error: NuiImageStreamReleaseFrame()" << endl;
		exit(0);
	}

	return true;
}

void HandGestureMultiCursor::getRgbImageV1()
{
	// RGB�J�����̃t���[���f�[�^���擾����
	NUI_IMAGE_FRAME imageFrame = { 0 };
	if (kinect->NuiImageStreamGetNextFrame(imageStreamHandle, 0, &imageFrame) < 0) {
		return;
	}

	// �摜�f�[�^���擾����
	NUI_LOCKED_RECT colorData = { 0 };
	imageFrame.pFrameTexture->LockRect(0, &colorData, NULL, 0);

	// �摜�f�[�^���R�s�[����
	rgbImage = cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC4, colorData.pBits);

	// �t���[���f�[�^���������
	if (S_OK != (kinect->NuiImageStreamReleaseFrame(imageStreamHandle, &imageFrame))){
		cout << "Error: NuiImageStreamReleaseFrame()" << endl;
		exit(0);
	}
}

#endif
#pragma endregion


#pragma region KINECT V2

bool HandGestureMultiCursor::getDepthImageV2()
{
	// Get depth frame data
	//if (!kinectBasics.GetDepthMat(depthImage, heightMatrix, point3fMatrix)) { return false; }
	bool isGetFrame = false;
	while (!isGetFrame)
	{
		isGetFrame = kinectBasics.GetDepthMat(depthImage, heightMatrix, point3fMatrix);
	}
	// Init Mat
	userAreaMat = Mat::zeros(kinectBasics.heightDepth, kinectBasics.widthDepth, CV_8UC3);
	heightFromTable = Mat::ones(kinectBasics.heightDepth, kinectBasics.widthDepth, CV_32F) * (-1);

	for (int y = 0; y < kinectBasics.heightDepth; ++y)
	{
		for (int x = 0; x < kinectBasics.widthDepth; ++x)
		{
			// ������̍��������߂�
			USHORT distance = *heightMatrix.ptr<USHORT>(y, x);

			// �e�[�u������̍���
			float tHeight = -(
				*tableParam.ptr<float>(0, 0) * point3fMatrix.ptr<float>(y, x)[0]
				+ *tableParam.ptr<float>(1, 0) * point3fMatrix.ptr<float>(y, x)[1]
				+ *tableParam.ptr<float>(2, 0) * point3fMatrix.ptr<float>(y, x)[2] - 1)
				/ sqrt(*tableParam.ptr<float>(0, 0) * *tableParam.ptr<float>(0, 0)
				+ *tableParam.ptr<float>(1, 0) * *tableParam.ptr<float>(1, 0)
				+ *tableParam.ptr<float>(2, 0) * *tableParam.ptr<float>(2, 0))
				* 1000;	// convert to [mm]
			

			// �e�[�u����荂���_�̂݃��[�U�G���A�Ƃ��Ďc��
			//if (offset > tHeight)
			if (TABLE_THRESHOLD < tHeight && tHeight < KINECT_HEIGHT)
				//if (0 < distance && distance < KINECT_HEIGHT - DESK_HEIGHT - offset)
			{
				// ������̍���
				//*heightMatrix.ptr<USHORT>(y, x) = (USHORT)(KINECT_HEIGHT - distance);
				*heightFromTable.ptr<float>(y, x) = tHeight;
				// ���[�U�G���A
				int index = ((y * kinectBasics.widthDepth) + x) * 3;
				UCHAR* dataDepth = &userAreaMat.data[index];
				dataDepth[0] = 255;
				dataDepth[1] = 255;
				dataDepth[2] = 255;
			}
			else
			{
				*heightMatrix.ptr<USHORT>(y, x) = 0;
			}
		}
	}
	return true;
}

#ifdef USE_COLOR_V2
void HandGestureMultiCursor::getRgbImageV2()
{
	int width = 1920;
	int height = 1080;
	unsigned int bufferSize = width * height * 4 * sizeof(unsigned char);
	cv::Mat bufferMat(height, width, CV_8UC4);
	rgbImage = Mat(height / 2, width / 2, CV_8UC4);

	// Frame
	IColorFrame* pColorFrame = nullptr;
	HRESULT hResult = S_OK;
	hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
	if (SUCCEEDED(hResult)){
		hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(bufferMat.data), ColorImageFormat_Bgra);
		if (SUCCEEDED(hResult)){
			cv::resize(bufferMat, rgbImage, cv::Size(), 0.5, 0.5);
		}
	}
	SafeRelease(pColorFrame);
}
#endif

#pragma endregion


// Main loop
void HandGestureMultiCursor::run()
{
	/* 1. Get frame data and prepear data needed */
	bool isGetFrameData = getFrameData();

	if (isGetFrameData)
	{
		/* 2. Labeling users' area */
		CvBlobs blobs = labelingUserArea(userAreaMat);

		/* 3. Detect users' head postiions */
		detectHeadPosition(blobs);

		/* 4. Detect users' hand positions */
		detectArm(blobs);

		/* 5. Detect finger tips of users */
		detectFingerTips(handRegions, headRegions);
		if (!headRegions.empty())
		{
			FingerTipDetector fingerTipDetector;
			for (int i = 0; i < headRegions.size(); ++i)
			{
				FingerTips fingerTips = fingerTipDetector.FindFingerTips(headRegions[i]);


				imshow("test", headRegions[0]);
			}
		}

		/* 6. Calcurate cursors position */
		calcCursorPos(blobs);

		/* Check whether user set cursor's position by pointing gesture */
		checkSettingCursor();


		/* #. Replace previous users' informations with current data  */
		updatePreData();

		/* Show images */
		isShowDebugWindows ? showDebugWindows() : destroyAllWindows();
	}
}

void HandGestureMultiCursor::showDebugWindows()
{
	if (!userAreaMat.empty()) { imshow("Hand/Head detection", userAreaMat); }

#ifndef USE_KINECT_V1
	if (!depthImage.empty()) { imshow("Depth image", depthImage); }	// ����Kinect2�̂�
#endif

#ifdef USE_COLOR_V2
	if (!rgbImage.empty()) { imshow("Color image", rgbImage); }
#endif
}


void HandGestureMultiCursor::updatePreData()
{
	// Replace preUserData by current userData	/ �O�t���[���̃f�[�^�����t���[���̃f�[�^�Œu��������
	preUserData.clear();
	preUserData = userData;
	for (vector<UserData>::iterator p = preUserData.begin(); p != preUserData.end(); p++)
	{
		p->isDataFound = false; // ������
	}
}

bool HandGestureMultiCursor::getFrameData()
{
#ifdef USE_KINECT_V1
	// Wait for updating frame data
	DWORD ret = ::WaitForSingleObject(streamEvent, 0);
	::ResetEvent(streamEvent);

	if (ret != WAIT_TIMEOUT)
	{
		// Get depth image
		if (!getDepthImageV1()) { return false; }

		// Get color image
		getRgbImageV1();
	}
#else
	// Get depth image from kinect v2
	if (!getDepthImageV2()) { return false; }

#ifdef USE_COLOR_V2
	getRgbImageV2();
#endif

#endif
	return true;
}


CvBlobs HandGestureMultiCursor::labelingUserArea(Mat& src)
{
	// Make image dilating for stable labeling
#ifdef USE_KINECT_V1
	dilate(src, src, Mat(), Point(-1, -1), 3);
#else
	//dilate(src, src, Mat(), Point(-1, -1), 1);
#endif

	/* Use IplImage since labeling library for cv::Mat is not fully implemented */
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

	Mat labelMatBuf(labelImg, true); // �f�[�^���R�s�[����
	labelMat = labelMatBuf;
	CV_Assert(reinterpret_cast<uchar*>(labelImg->imageData) != labelMat.data);
	
	// Filter noise / �m�C�Y�_�̏���
	cvFilterByArea(blobs, 2000, 1000000);

	// Render blobs
	cvRenderBlobs(labelImg, blobs, &srcIpl, &srcIpl);

	// Free unused IplImages
	cvReleaseImage(&labelImg);
	cvReleaseImage(&srcIplBinary);
	
	return blobs;
}

void HandGestureMultiCursor::detectHeadPosition(CvBlobs blobs)
{
	// Reset userData
	userData.clear();


	// �O�t���[���̃��x�������čł����[���̑����������x����O�t���[���̑Ή��̈�Ƃ���
	for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it)
	{
		UserData newUserData;
		newUserData.isDataFound = false;
		newUserData.preDataID = -1;
		if (!preLabelMat.empty())
		{
			// �O�t���[���̃��x���𓊕[
			vector<Point2i> checkLabel;
			checkLabel.push_back(Point2i(0, 0));	// init checkLabel;
			for (int y = it->second->miny; y < it->second->maxy; ++y)
			{
				for (int x = it->second->minx; x < it->second->maxx; ++x)
				{
					unsigned long preLabel = preLabelMat.at<unsigned long>(y, x);
					if (it->first == labelMat.at<unsigned long>(y, x) && preLabel != 0)
					{
						bool isFoundLabel = false;
						for (int i = 0; i < checkLabel.size(); ++i)
						{
							if (checkLabel[i].x == preLabel)
							{
								++checkLabel[i].y;
								isFoundLabel = true;
								break;
							}
						}
						if (!isFoundLabel)
						{
							Point2i newPoint(preLabel, 1);		// (Label ID, # of label)
							checkLabel.push_back(newPoint);
						}
					}
				}
			}

			// �ł����[���̑������x����T��
			Point2i mainLabel(0, 0);
			for (size_t i = 0; i < checkLabel.size(); ++i)
			{
				if (checkLabel[i].y > mainLabel.y)
				{
					mainLabel.x = checkLabel[i].x;	// Label ID 
					mainLabel.y = checkLabel[i].y;	// # of label
				}
			}

			// �O�t���[���̃f�[�^�ƑΉ��t���Č��t���[���̃f�[�^�����
			
			newUserData.isDataFound = false;
			if (mainLabel.y > it->second->area * 0.7)	// �����ȏ���߂�̈悪�Ȃ��Ƃ��͖���
			{

				for (size_t i = 0; i < preUserData.size(); ++i)
				{
					if (preUserData[i].labelID == mainLabel.x)
					{
						preUserData[i].isDataFound = true;
						newUserData.isDataFound = true;
						newUserData.headInfo.height = preUserData[i].headInfo.height;
						newUserData.headInfo.depthPoint.x = preUserData[i].headInfo.depthPoint.x;
						newUserData.headInfo.depthPoint.y = preUserData[i].headInfo.depthPoint.y;
						newUserData.preDataID = i;
						break;
					}
				}
			}
			
		}
		newUserData.labelID = it->first;
		newUserData.centroid.x = it->second->centroid.x;
		newUserData.centroid.y = it->second->centroid.y;
		userData.push_back(newUserData);
	}
	// Update preLabel
	preLabelMat = labelMat;


	// Find the highest point of each user area
	float* headHeights = new float[blobs.size()];
	//USHORT* headHeights = new USHORT[blobs.size()];
	Point2i* newHighestPositions = new Point2i[blobs.size()];
	int blobID = 0;
	for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); it++)
	{
		headHeights[blobID] = 0;
		newHighestPositions[blobID].x = 0;
		newHighestPositions[blobID].y = 0;
		for (int y = it->second->miny; y <= it->second->maxy; y++)
		{
			for (int x = it->second->minx; x <= it->second->maxx; x++)
			{
				if (0 <= blobID && blobID < blobs.size())
				{
					//USHORT height = *heightMatrix.ptr<USHORT>(y, x);
					//if (headHeights[blobID] < height && height < HEAD_HEIGHT_MAX)
					float height = *heightFromTable.ptr<float>(y, x) * 1000;
					if (headHeights[blobID] < height && height < KINECT_HEIGHT)
					{
						headHeights[blobID] = height;
						newHighestPositions[blobID].x = x;
						newHighestPositions[blobID].y = y;
					}
				}
			}
		}
		// Debug: Show the highest point of each users
		//circle(userAreaMat, Point(newHighestPositions[blobID].x, newHighestPositions[blobID].y), 5, Scalar(255, 0, 255), 3);
		blobID++;
	}	

	// Define users' head positions
	Point2i* newHeadPositions = new Point2i[blobs.size()];
	INT* numHeadPoints = new INT[blobs.size()];
	blobID = 0;
	for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it)
	{
		// Set the highest position as a base point of searching
		userData[blobID].headInfo.depthPoint.x = newHighestPositions[blobID].x;
		userData[blobID].headInfo.depthPoint.y = newHighestPositions[blobID].y;
		userData[blobID].headInfo.height = headHeights[blobID];

		if (userData[blobID].isDataFound)
		{
			// Check distance between 2d positions in current frame and in preframe
			float distance = sqrt(
				pow(userData[blobID].headInfo.depthPoint.x - preUserData[blobID].headInfo.depthPoint.x, 2)
				+ pow(userData[blobID].headInfo.depthPoint.y - preUserData[blobID].headInfo.depthPoint.y, 2)
				);	// [pixel]
			//float distanceZ = abs(heightMatrix.at<float>(userData[blobID].headInfo.depthPoint.y, userData[blobID].headInfo.depthPoint.x) - preUserData[blobID].headInfo.height);
			float distanceZ = abs(heightFromTable.at<float>(userData[blobID].headInfo.depthPoint.y, userData[blobID].headInfo.depthPoint.x) - preUserData[blobID].headInfo.height);  // [m]
			//cout << distance << endl;
			// If the point is far from predata, just use pre-data	/ �����O��̃t���[�����傫�����̈ʒu������Ă�����O��̒l���g��
			if (distance > 100.0f || distanceZ > 500)  // [pixel] || [mm]
			{
				userData[blobID].headInfo.height = preUserData[blobID].headInfo.height;
				userData[blobID].headInfo.depthPoint.x = preUserData[blobID].headInfo.depthPoint.x;
				userData[blobID].headInfo.depthPoint.y = preUserData[blobID].headInfo.depthPoint.y;
			}
		}


		// Estimate exact head positions (Get average)
		numHeadPoints[blobID] = 0;
		newHeadPositions[blobID].x = 0;
		newHeadPositions[blobID].y = 0;
#if 1
		int offset_head = 40;	// �~�[�e�B���O���[���p
#else
		int offset_head = 100;  // ��Ə�p
#endif
		int minY = userData[blobID].headInfo.depthPoint.y - offset_head;  if (minY < 0) minY = 0;
		int maxY = userData[blobID].headInfo.depthPoint.y + offset_head;  if (maxY > kinectBasics.heightDepth) maxY = kinectBasics.heightDepth;
		int minX = userData[blobID].headInfo.depthPoint.x - offset_head;  if (minX < 0) minX = 0;
		int maxX = userData[blobID].headInfo.depthPoint.x + offset_head;  if (maxX > kinectBasics.widthDepth) maxX = kinectBasics.widthDepth;
		for (int y = minY; y <= maxY; y++)
		{
			for (int x = minX; x <= maxX; x++)
			{
				//USHORT height = *heightMatrix.ptr<USHORT>(y, x);
				float height = *heightFromTable.ptr<float>(y, x);
				//cout << it->first << ",  " << labelMat.at<unsigned long>(y, x) << endl;
				if ((userData[blobID].headInfo.height - HEAD_LENGTH) < height && height < HEAD_HEIGHT_MAX
					&& it->first == labelMat.at<unsigned long>(y, x)	// ����blob���̂ݒT��
					)
				{
					newHeadPositions[blobID].x += x;
					newHeadPositions[blobID].y += y;
					numHeadPoints[blobID]++;
				}	
			}
		}
		blobID++;
	}

	// Make avarage pixel value of each head positions the head positions of users
	for (int i = 0; i < blobs.size(); i++)
	{
		if (numHeadPoints[i] != 0)
		{
			// Calculate head position in 2D pixel
			userData[i].headInfo.depthPoint.x = newHeadPositions[i].x / numHeadPoints[i];
			userData[i].headInfo.depthPoint.y = newHeadPositions[i].y / numHeadPoints[i];
		}
		else
		{
			// �_��������Ȃ������ꍇ�͍ł������_�𓪂ɂ���
			userData[i].headInfo.depthPoint.x = newHighestPositions[i].x;
			userData[i].headInfo.depthPoint.y = newHighestPositions[i].y;
		}
		// Calculate head position in 3D point
		float* headPosition = point3fMatrix.ptr<float>(userData[i].headInfo.depthPoint.y, userData[i].headInfo.depthPoint.x);
		userData[i].headInfo.cameraPoint.x = headPosition[0];
		userData[i].headInfo.cameraPoint.y = headPosition[1];
		userData[i].headInfo.cameraPoint.z = headPosition[2] + 0.13;	// �ڐ��̈ʒu�ɕ␳

		// Debug: Show the head point
		circle(userAreaMat, Point(userData[i].headInfo.depthPoint.x, userData[i].headInfo.depthPoint.y), 7, Scalar(255, 0, 0), 3);
	}

	delete[] headHeights;
	delete[] newHighestPositions;
	delete[] newHeadPositions;
	delete[] numHeadPoints;

}

void HandGestureMultiCursor::detectArm(CvBlobs blobs)
{
	int blobID = 0;

	handRegions.clear();
	headRegions.clear();

	for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it) {
		Mat newHandRegion = Mat::zeros(kinectBasics.heightDepth, kinectBasics.widthDepth, CV_8UC3);
		Mat newHeadRegion = Mat::zeros(kinectBasics.heightDepth, kinectBasics.widthDepth, CV_8UC3);

		int numIntersectionPoints = 0;
		Vector4 handPosition;
		handPosition.w = 1;
		handPosition.x = 0.0f;
		handPosition.y = 0.0f;
		handPosition.z = 0.0f;
		Point3f center3f = Point3_<FLOAT>(userData[blobID].headInfo.cameraPoint.x, userData[blobID].headInfo.cameraPoint.y, userData[blobID].headInfo.cameraPoint.z);

		// ���[�U�̗̈����T��
		for (int y = it->second->miny; y <= it->second->maxy; y++) {
			for (int x = it->second->minx; x <= it->second->maxx; x++)
			{
				float length = sqrt(
					pow(center3f.x - point3fMatrix.ptr<float>(y, x)[0], 2)
					+ pow(center3f.y - point3fMatrix.ptr<float>(y, x)[1], 2)
					+ pow(center3f.z - point3fMatrix.ptr<float>(y, x)[2], 2)
					);
				// Define the intersection point of the sphere which its center is head and the hand as the hand position 
				//if (*heightMatrix.ptr<USHORT>(y, x) > userData[blobID].headInfo.height - HEAD_LENGTH - SHOULDER_LENGTH	// ����荂���_��
				//	&& 0 < point3fMatrix.ptr<float>(y, x)[2] * 1000 && point3fMatrix.ptr<float>(y, x)[2] * 1000 < KINECT_HEIGHT	// Kinect�̌��o�ł���͈͂̒l��
				//	&& it->first == labelMat.at<unsigned long>(y, x)	// ����blob���̂ݒT��
				//	) // Don't include desk
				if (it->first == labelMat.at<unsigned long>(y, x)	// ����blob���̂ݒT��
					&& TABLE_THRESHOLD< *heightFromTable.ptr<float>(y, x) && *heightFromTable.ptr<float>(y, x) < KINECT_HEIGHT) // Don't include desk
				{
					if (SENCIG_CIRCLE_RADIUS < length ) {	// ���ړ_�����ƌ������Ă��邩�ǂ���
						handPosition.x += point3fMatrix.ptr<float>(y, x)[0];
						handPosition.y += point3fMatrix.ptr<float>(y, x)[1];
						handPosition.z += point3fMatrix.ptr<float>(y, x)[2];

						// ��̗̈���L�^����
						int index = ((y * kinectBasics.widthDepth) + x) * 3;
						UCHAR* dataHand = &newHandRegion.data[index];
						dataHand[0] = 255;
						dataHand[1] = 255;
						dataHand[2] = 255;


						circle(userAreaMat, Point(x, y), 0.5, Scalar(255, 255, 0), -1);
						numIntersectionPoints++;
					//}
					//else if (length > SENCIG_CIRCLE_RADIUS) {

//#ifdef USE_KINECT_V1
//						LONG handPositionX2d;
//						LONG handPositionY2d;
//						USHORT dis;
//						NuiTransformSkeletonToDepthImage(handPosition, &handPositionX2d, &handPositionY2d, &dis, KINECT_RESOLUTION);
//						circle(userAreaMat, Point(handPositionX2d, handPositionY2d), 7, Scalar(0, 200, 0), 3);
//#else
//						//DepthSpacePoint depthPoint;
//						//CameraSpacePoint cameraPoint;
//						//cameraPoint.X = point3fMatrix.ptr<float>(y, x)[0];
//						//cameraPoint.Y = point3fMatrix.ptr<float>(y, x)[1];
//						//cameraPoint.Z = point3fMatrix.ptr<float>(y, x)[2];
//						//kinectBasics.GetMapper()->MapCameraPointToDepthSpace(cameraPoint, &depthPoint);
//
//						////int index = ((y * kinectBasics.widthDepth) + x) * 3;
//						//UCHAR* dataDepth = &userAreaMat.data[index];
//						//dataDepth[0] = 0;
//						//dataDepth[1] = 200;
//						//dataDepth[2] = 255;
//						//circle(userAreaMat, Point(depthPoint.X, depthPoint.Y), 1, Scalar(0, 255, 255), 3);
//#endif
					}
					else
					{
						// ��̗̈���L�^����
						int index = ((y * kinectBasics.widthDepth) + x) * 3;
						UCHAR* dataHead = &newHeadRegion.data[index];
						dataHead[0] = 255;
						dataHead[1] = 255;
						dataHead[2] = 255;


					}
				}
			}
		}


		if (numIntersectionPoints > 0)
		{
			userData[blobID].handInfo.isTracked = true;
		}
		else{
			userData[blobID].handInfo.isTracked = false;
		}

		// Add hand resion
		handRegions.push_back(newHandRegion);
		headRegions.push_back(newHeadRegion);

		blobID++;
	}
}

void HandGestureMultiCursor::detectFingerTips(vector<Mat> handRegions, vector<Mat> headRegions)
{
	if (handRegions.size() == 0 || headRegions.size() == 0)	{ return; }
	
	FingerTipDetector fingerTipDetector;
	
	for (int i = 0; i < handRegions.size(); ++i)
	{
		if (!userData[i].handInfo.isTracked) { continue; }

		// �E�荶������o
		CvBlob handR, handL;
		fingerTipDetector.GetHandInfo(handRegions[i], headRegions[i], point3fMatrix, userData[i].handInfoR, userData[i].handInfoL);
		if (fingerTipDetector.CheckIsHandOnTable(userData[i].handInfoR, tableParam))
		{

		}
		//if (fingerTipDetector.CheckIsHandOnTable(userData[i].handInfoL, TKinect2Table))
		//{

		//}
		//fingerTipDetector.CursorMove(userData[i], TKinect2Table);


		
	}

	// Show hand regions
	int numShownUser = 5;
	for (int i = 0; i < numShownUser; ++i)
	{
		stringstream ii;
		ii << i;
		string iStr = ii.str();

		if (i >= handRegions.size())
		{
			destroyWindow(iStr);
		}
		else
		{
			imshow(iStr, handRegions[i]);
		}
	}
}


void HandGestureMultiCursor::calcCursorPos(CvBlobs blobs)
{
#ifdef USE_KINECT_V1
	INT blobID = 0;
	for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it) {
		if (userData[blobID].handInfo.isTracked)
		{
			Mat handPoint = (cv::Mat_<float>(4, 1) << userData[blobID].handInfo.cameraPoint.x, userData[blobID].handInfo.cameraPoint.y, userData[blobID].handInfo.cameraPoint.z, 1);
			Mat headPoint = (cv::Mat_<float>(4, 1) << userData[blobID].headInfo.cameraPoint.x, userData[blobID].headInfo.cameraPoint.y, userData[blobID].headInfo.cameraPoint.z, 1);
			Mat handPointScreen = T_WorldToScreen * T_KinectCameraToWorld * handPoint;
			Mat headPointScreen = T_WorldToScreen * T_KinectCameraToWorld * headPoint;


			// Caliculate the intersection point of vector and screen
			float xvec = *handPointScreen.ptr<float>(0, 0) - *headPointScreen.ptr<float>(0, 0);
			float yvec = *handPointScreen.ptr<float>(1, 0) - *headPointScreen.ptr<float>(1, 0);
			float zvec = *handPointScreen.ptr<float>(2, 0) - *headPointScreen.ptr<float>(2, 0);

			float val = -*handPointScreen.ptr<float>(2, 0) / zvec;

			// Calculate cursor position in real scall
			Point3f cursorScreen3d;
			cursorScreen3d.x = val * xvec + *headPointScreen.ptr<float>(0, 0);
			cursorScreen3d.y = val * yvec + *headPointScreen.ptr<float>(1, 0);
			cursorScreen3d.z = 0.0f;

			// Calculate cursor position in pixel coordinate
			float screen3dTo2d = 246 / 0.432f;
			userData[blobID].cursorInfo.position.x = cursorScreen3d.x * screen3dTo2d;
			userData[blobID].cursorInfo.position.y = cursorScreen3d.y * screen3dTo2d;
			userData[blobID].cursorInfo.isShownCursor;
		}
		++blobID;
	}
#else
	int blobID = 0;
	for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it) 
	{
		userData[blobID].cursorInfo.isShownCursor = false;
		if (userData[blobID].handInfoR.isTracked)
		{
			Mat handPoint = (cv::Mat_<float>(4, 1) << userData[blobID].handInfoR.cameraPoint.x * 1000, userData[blobID].handInfoR.cameraPoint.y * 1000, userData[blobID].handInfoR.cameraPoint.z * 1000, 1);
			Mat headPoint = (cv::Mat_<float>(4, 1) << userData[blobID].headInfo.cameraPoint.x * 1000, userData[blobID].headInfo.cameraPoint.y * 1000, userData[blobID].headInfo.cameraPoint.z * 1000, 1);

			for (size_t i = 0; i < TKinect2Display.size(); ++i)
			{
				Mat handPointScreen = TKinect2Display[i] * handPoint;
				Mat headPointScreen = TKinect2Display[i] * headPoint;

				// Caliculate the intersection point of vector and screen
				float xvec = *handPointScreen.ptr<float>(0, 0) - *headPointScreen.ptr<float>(0, 0);
				float yvec = *handPointScreen.ptr<float>(1, 0) - *headPointScreen.ptr<float>(1, 0);
				float zvec = *handPointScreen.ptr<float>(2, 0) - *headPointScreen.ptr<float>(2, 0);

				float val = -*handPointScreen.ptr<float>(2, 0) / zvec;

				// Calculate cursor position in real scall
				Point3f cursorScreen3d;
				cursorScreen3d.x = val * xvec + *headPointScreen.ptr<float>(0, 0);
				cursorScreen3d.y = val * yvec + *headPointScreen.ptr<float>(1, 0);
				cursorScreen3d.z = 0.0f;

				// Calculate cursor position in pixel coordinate
				Mat cursor3d = (Mat_<float>(3, 1) << cursorScreen3d.x, cursorScreen3d.y, 1);
				Mat cursor2d = TDisplay2Pixel[i] * cursor3d;
				cursor2d /= *cursor2d.ptr<float>(2, 0);

				//cout << cursor2d << endl;
				// Set cursor position in pixel coordinate if the cursor is in the display
				if (0 < *cursor2d.ptr<float>(0, 0) && *cursor2d.ptr<float>(0, 0) < VEC_WIN_WIDTH[0]
					&& 0 < *cursor2d.ptr<float>(1, 0) && *cursor2d.ptr<float>(1, 0) < VEC_WIN_HEIGHT[0])
				{
					userData[blobID].cursorInfo.isShownCursor = true;
					userData[blobID].cursorInfo.displayNum = i;
					userData[blobID].cursorInfo.position.x = *cursor2d.ptr<float>(0, 0) ;
					userData[blobID].cursorInfo.position.y = *cursor2d.ptr<float>(1, 0);
				}


			}
		}
		++blobID;
	}
#endif
}

void HandGestureMultiCursor::checkSettingCursor()
{
	const double timeLimit = 0.4;

	for (size_t i = 0; i < userData.size(); ++i)
	{
		
		if (!userData[i].cursorInfo.isShownCursor) { continue; }

		if (userData[i].preDataID < 0)
		{
			timer.reset();
			timer.start();
			continue;
		}

		int distance = sqrt(
			pow(userData[i].cursorInfo.position.x - preUserData[userData[i].preDataID].cursorInfo.position.x, 2)
			+ pow(userData[i].cursorInfo.position.y - preUserData[userData[i].preDataID].cursorInfo.position.y, 2)
			); 
		if (i > 0) cout << preUserData[userData[i].preDataID].cursorInfo.position.y << endl;
		if (distance > 50)
		{
			timer.reset();
			timer.start();
		}
		else
		{
			timer.stop();
			if (timer.getTimeSec() > timeLimit)
			{
				MouseControl(userData[i].cursorInfo.position.x, userData[i].cursorInfo.position.y);
				timer.reset();
			}
			else
			{
				timer.start();
			}
		}
	}
}


void HandGestureMultiCursor::MouseControl(float x, float y)
{
	// �X�N���[�����W��mouse_event()�p�̍��W�ϊ�
	DWORD dwX = x * 65535 / ::GetSystemMetrics(SM_CXSCREEN);
	DWORD dwY = y * 65535 / ::GetSystemMetrics(SM_CYSCREEN);

	// Set mouse cursor position
	::mouse_event(MOUSEEVENTF_ABSOLUTE | MOUSEEVENTF_MOVE, dwX, dwY, NULL, NULL);
}

#pragma region OpenGL

void HandGestureMultiCursor::initGL(int argc, char* argv[])
{
	glutInit(&argc, argv);

	WinIDs = new int[TKinect2Display.size()];
	for (size_t i = 0; i < TKinect2Display.size(); ++i)
	{
		glutInitWindowPosition(i*20, 0);
		//glutInitWindowPosition(windowOffsetX[i], 0);
		glutInitWindowSize(VEC_WIN_WIDTH[i], VEC_WIN_HEIGHT[i]);

		glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
		char winName[8];
		_itoa((int)i, winName, 10);
		WinIDs[i] = glutCreateWindow(winName);

		//// Register callback functions
		//glutReshapeFunc(sreshape);
		glutDisplayFunc(sdisplay);
		glutIdleFunc(sidle);
		glutKeyboardFunc(skeyboard);
		glutMouseFunc(smouse);

		glClearColor(1.0, 1.0, 1.0, 1.0);

		/* Camera setup */
		glViewport(0, 0, kinectBasics.widthDepth, kinectBasics.heightDepth);
		glLoadIdentity();

		if (i == 0){
			/* GL�̃E�B���h�E���t���X�N���[���� */
			//GL�̃f�o�C�X�R���e�L�X�g�n���h���擾
			glutSetWindow(WinIDs[i]);
			HDC glDc = wglGetCurrentDC();
			//�E�B���h�E�n���h���擾
			HWND hWnd = WindowFromDC(glDc);
			//�E�B���h�E�̑����ƈʒu�ύX
			SetWindowLong(hWnd, GWL_STYLE, WS_POPUP);
			//SetWindowPos(hWnd, HWND_TOP, windowOffsetX[i], 0, 100, 100, SWP_SHOWWINDOW);
			SetWindowPos(hWnd, HWND_TOP, windowOffsetX[i], 0, VEC_WIN_WIDTH[i], VEC_WIN_HEIGHT[i], SWP_SHOWWINDOW);
		}
	}
}

// �J�[�\����`�悷��
void HandGestureMultiCursor::display(void)
{
	const float divisionX = 12;
	float cellLength = VEC_WIN_WIDTH[0] / divisionX;

	glClearColor(1.0, 1.0, 1.0, 1.0);
	for (size_t i = 0; i < VEC_WIN_WIDTH.size(); ++i)
	{
		glutSetWindow(WinIDs[(int)0]);
		glClear(GL_COLOR_BUFFER_BIT);
	}
	if (userData.size() > 0)
	{
		for (int i = 0; i < userData.size(); ++i)
		{
			if (!userData.empty() && userData[i].cursorInfo.isShownCursor)
			{
				// �`��E�B���h�E���Z�b�g
				glutSetWindow(WinIDs[userData[i].cursorInfo.displayNum]);

				int windowWidth = VEC_WIN_WIDTH[userData[i].cursorInfo.displayNum];
				int windowHeight = VEC_WIN_HEIGHT[userData[i].cursorInfo.displayNum];
				windowWidth = windowWidth;
				windowHeight = windowHeight;

#if 0			// ���������O���b�h�ŃJ�[�\���`��
				int posX = (int)(userData[i].cursorInfo.position.x) / (int)(cellLength) * cellLength;
				int posY = ((int)(windowHeight - userData[i].cursorInfo.position.y) / (int)(cellLength) + 1) * cellLength;
					
				float posXGL = ((float)posX - (float)windowWidth / 2) / ((float)windowWidth / 2);
				float posYGL = ((float)posY - (float)windowHeight / 2) / ((float)windowHeight / 2);
				float ofX = cellLength / ((float)windowWidth / 2);
				float ofY = cellLength / ((float)windowHeight / 2);
				glColor4f(0.1f, 1.0f, 0.0f, 1.0f);
				glBegin(GL_QUADS);
				glVertex2f(posXGL, posYGL);
				glVertex2f(posXGL + ofX, posYGL);
				glVertex2f(posXGL + ofX, posYGL - ofY);
				glVertex2f(posXGL, posYGL - ofY);
				glEnd();
				//cout << posXGL << ", " << posYGL << endl;
#endif

#if 1		// ���m�Ȉʒu��\���|�C���^�`��
#ifdef USE_KINECT_V1
				Point2f cursorPos((userData[i].cursorInfo.position.x - windowWidth / 2) / windowWidth, -(userData[i].cursorInfo.position.y - windowHeight / 2) / windowHeight);
#else
				Point2f cursorPos((userData[i].cursorInfo.position.x - windowWidth / 2) / windowWidth * 2, (windowHeight - userData[i].cursorInfo.position.y - windowHeight / 2) / windowHeight * 2);
#endif
				glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
				glBegin(GL_QUADS);
				float offset = 40;
				float offsetX = (offset / (float)windowWidth);
				float offsetY = (offset / (float)windowHeight);
				glVertex2f(cursorPos.x, cursorPos.y);
				glVertex2f(cursorPos.x + offsetX, cursorPos.y);
				glVertex2f(cursorPos.x + offsetX, cursorPos.y + offsetY);
				glVertex2f(cursorPos.x, cursorPos.y + offsetY);
				glEnd();
#endif
			}
		}
	}

	glFlush();
	glutSwapBuffers();

}

void HandGestureMultiCursor::idle(void)
{
	this->run();

	for (size_t i = 0; i < VEC_WIN_WIDTH.size(); ++i)
	{
		glutSetWindow(WinIDs[i]);
		glutPostRedisplay();
	}
}

void HandGestureMultiCursor::keyboard(unsigned char key, int x, int y)
{
	float mouseX = 0.0f, mouseY = 0.0f;

	switch (key) {
	case 'q':
	case 'Q':
	case '\033':	// ESC
		exit(0);
		break;
	case 'd':
	case 'D':
		isShowDebugWindows = !isShowDebugWindows;
		break;
	case 'r':
	case 'R':
		preUserData.clear();


		break;
	case ' ':
		srand((unsigned int)time(NULL));	// ����������
#if 0	// �E�B���h�E���Ƀ����_���ɃJ�[�\���ړ�
		switch (rand() % 4) {
		case 0:
			mouseX = 0;
			mouseY = rand() % VEC_WIN_HEIGHT[0];
			break;
		case 1:
			mouseX = rand() % VEC_WIN_WIDTH[0];
			mouseY = 0;
			break;
		case 2:
			mouseX = VEC_WIN_WIDTH[0];
			mouseY = rand() % VEC_WIN_HEIGHT[0];
			break;
		case 3:
			mouseX = rand() % VEC_WIN_WIDTH[0];
			mouseY = VEC_WIN_HEIGHT[0];
			break;
		default:
			break;
		}
		
#else	// ���E�̃f�B�X�v���C�Ƀ����_���ɃJ�[�\���\��
		if (rand() % 2 == 0)
		{
			// �E�f�B�X�v���C
			mouseX = rand() % 1280 + 2560;
			mouseY = rand() % 1024;
		}
		else
		{
			// ���f�B�X�v���C
			mouseX = -rand() % 1920;
			mouseY = rand() % 1200;
		}
#endif
		MouseControl(mouseX, mouseY);

		break;
	default:
		break;
	}
}

static void sdisplay()
{
	app.display();
}

static void sidle(void)
{
	app.idle();
}

static void skeyboard(unsigned char key, int x, int y)
{
	app.keyboard(key, x, y);
}

#pragma endregion

void HandGestureMultiCursor::loadCalibData()
{
	windowOffsetX.push_back(0);

	// Load each display informations
	for (size_t i = 0; i < DISP_INFO_FILENAMES.size(); ++i)
	{
		FileStorage cvfs(DISP_INFO_FILENAMES[i].c_str(), CV_STORAGE_READ);
		FileNode node(cvfs.fs, NULL);

		// Loat window size
		int winWidth = node["WindowWidth"];
		int winHeight = node["WindowHeight"];
		FileNode fn = node[string("mat_array")];
		
		// Load transformation matrixes
		Mat TK2D, TD2P;
		read(fn[0], TK2D);	// Load transformation matrix from kinect depth camera to display plane
		//*TK2D.ptr<float>(0, 3) *= -1;	// ���E���]�𒼂�
		read(fn[1], TD2P);	// Load transformation matrix from display plane to display pixel image 

		if (winWidth > 0 && winHeight > 0 && !TK2D.empty() && !TD2P.empty())
		{
			VEC_WIN_WIDTH.push_back(winWidth);
			VEC_WIN_HEIGHT.push_back(winHeight);
			TKinect2Display.push_back(TK2D);
			TDisplay2Pixel.push_back(TD2P);

			int offsetX = windowOffsetX[i - 1] + winWidth;
			windowOffsetX.push_back(offsetX);
			
			cout << "Succeeded to load display[" << i << "]" << endl;
			cout << "(width, height) = " << winWidth << ", " << winHeight << endl;
			cout << "TKinect2Display: " << endl << TKinect2Display[i] << endl;
			cout << "TDisplay2Pixel: " << endl << TDisplay2Pixel[i] << endl;
		}
		else
		{
			cout << "Failed to load display[" << i << "]" << endl;
		}
	}

	if (VEC_WIN_WIDTH.size() <= 0)
	{
		cout << "Error(loadCalibData): No display information was loaded" << endl;
		exit(0);
	}

	// Import table information
	FileStorage cvfs(tableInfo_filename, CV_STORAGE_READ);
	FileNode node(cvfs.fs, NULL);
	FileNode fn = node[string("mat_array")];
	read(fn[0], tableParam);
	cout << "Transformation matrix from kinet to table: " << endl << tableParam << endl;
}

void HandGestureMultiCursor::showHelp()
{
	cout << "Use ";
#ifdef USE_KINECT_V1
	cout << "Kinect V1" << endl;
#else
	cout << "Kinect V2" << endl;
#endif
	cout << "<Help>" << endl;
	cout << "p, P, ESC: Quit" << endl;
	cout << "d, D     : Toggle showing debug windows" << endl;
}

// �N���X��`
static HandGestureMultiCursor app;
static KinectV2Basics kinectBasics;

int main(int argc, char* argv[])
{
	app.showHelp();
	
	app.loadCalibData();
	
	app.initGL(argc, argv);
#ifdef USE_KINECT_V1
	app.initKinect();
#else

#ifndef USE_COLOR_V2
	kinectBasics.SelectUsingData(true, false);
#endif

	if (!kinectBasics.SetupKinectV2()) { return -1; }
#endif
	glutMainLoop();

	return 0;
}




// ���̂Ƃ���g��Ȃ�

void HandGestureMultiCursor::mouse(int button, int state, int mouse_x, int mouse_y)
{
	switch (button) {
	case GLUT_LEFT_BUTTON:
		printf("left");
		break;
	case GLUT_MIDDLE_BUTTON:
		printf("middle");
		break;
	case GLUT_RIGHT_BUTTON:
		printf("right");
		break;
	default:
		break;
	}

	printf(" button is ");

	switch (state) {
	case GLUT_UP:
		printf("up");
		break;
	case GLUT_DOWN:
		printf("down");
		break;
	default:
		break;
	}

	printf(" at (%d, %d)\n", mouse_x, mouse_y);
}

void smouse(int button, int state, int mouse_x, int mouse_y)
{
	app.mouse(button, state, mouse_x, mouse_y);
}

//static void sreshape(int w, int h)
//{
//	appSub.reshape(w, h);
//}

//void HandGestureMultiCursor::reshape(int w, int h)
//{
//	// Make the whole window as a viewport
//	glViewport(0, 0, w, h);
//	// Initialize transformation matrix
//	glLoadIdentity();
//}