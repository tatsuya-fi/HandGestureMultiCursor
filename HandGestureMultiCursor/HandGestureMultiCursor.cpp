// HandGestureMultiCursoror.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//
#include "stdafx.h"
#include "HandGestureMultiCursor.h"

using namespace std;
using namespace cv;
using namespace cvb;
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

	// �E�B���h�E�n���h���̊J��
	delete[] WinIDs;

	// CV�̃E�B���h�E�j���i�O�̂���)
	destroyAllWindows();

	// �O�̂��߃{�^���A�b�v
	::mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, NULL, NULL);
	::keybd_event(VK_CONTROL, 0, KEYEVENTF_KEYUP, 0);
}

// Main loop
void HandGestureMultiCursor::run()
{
	fpsTimer.reset();
	fpsTimer.start();


	/* 1. Get frame data and prepear data needed */
	bool isGetFrameData = getFrameData();

	if (isGetFrameData)
	{
		/* 2. Labeling users' area */
		CvBlobs blobs = labelingUserArea(userAreaMat);

		/* 3. Detect users' head postiions */
		detectHeadPosition(blobs);

		///* 4. Detect users' hand positions */
		detectArm(blobs);

		/* 5. Distinguish each hands of users */
		detectHand(handRegions, headRegions);

		/* 6. Calcurate pointed cursors position */
		calcCursorPos(blobs);

		/* 7. Check hand movement on talble */
		relativeCursorControl();

		/* 8. Check whether user set cursor's position by pointing gesture */
		pointingCursorControl();

		/* #. Replace previous users' informations with current data  */
		updatePreData();

		//if (userData[0].cursorInfo.isShownCursor)
		//{
		//	cout << userData[0].cursorInfo.position << endl;
		//}

		/* Show fps */
		fpsTimer.stop();
		// n�t���[���Ɉ��\��
		int n = 10;
		if (fpsCount % n == 0)
		{
			std::ostringstream os;
			os << "fps:" << 1.0 / (fps / (double)n);
			fpsStr = os.str();
			fpsCount = 1;
			fps = 0.0;
		}
		else
		{
			fps += fpsTimer.getTimeSec();
			++fpsCount;
		}
		putText(userAreaMat, fpsStr, Point(2, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 200), 1, CV_AA);

		/* Show images */
		isShowDebugWindows ? showDebugWindows() : destroyAllWindows();

	}
}



#pragma region KINECT V1
#ifdef USE_KINECT_V1

void HandGestureMultiCursor::createInstance()
{
	// �ڑ�����Ă���Kinect�̐����擾����
	int count = 0;
	if (S_OK != ::NuiGetSensorCount(&count)) {
		cout << "Error: NuiGetSensorCount()" << endl;
		ErrorExit();

	}
	if (count == 0) {
		throw std::runtime_error("Kinect ��ڑ����Ă�������");
	}

	// �ŏ���Kinect�̃C���X�^���X���쐬����
	if (S_OK != ::NuiCreateSensorByIndex(0, &kinect)) {
		cout << "Error: NuiCreateSensorByIndex" << endl;
		ErrorExit();

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
		ErrorExit();

	}

	// RGB�J����������������
	const NUI_IMAGE_RESOLUTION KINECT_RGB_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;
	if (S_OK != (kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, KINECT_RGB_RESOLUTION,
		0, 2, 0, &imageStreamHandle))) {
		cout << "Error: NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR)" << endl;
		ErrorExit();

	}
	
	// �����J����������������
	if (S_OK != (kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, KINECT_RESOLUTION,
		0, 2, 0, &depthStreamHandle))) {
		cout << "Error: NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH)" << endl;
		ErrorExit();

	}

#ifdef NEAR_MODE
	// Near���[�h
	if (S_OK != (kinect->NuiImageStreamSetImageFrameFlags(
		depthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE))) {
		cout << "Error: NuiImageStreamSetImageFrameFlags()" << endl;
		ErrorExit();

	}
#endif

	// �t���[���X�V�C�x���g�̃n���h�����쐬����
	streamEvent = ::CreateEvent(0, TRUE, FALSE, 0);
	if (S_OK != (kinect->NuiSetFrameEndEvent(streamEvent, 0))) {
		cout << "Error: NuiSetFrameEndEvent()" << endl;
		ErrorExit();
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
		ErrorExit();
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
		ErrorExit();
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
	//while (!isGetFrame)
	//{
		isGetFrame = kinectBasics.GetDepthMat(depthImage, heightMatrix, point3fMatrix);
		if (!isGetFrame) { return false; }
	//}
	// Init Mat
	userAreaMat = Mat::zeros(kinectBasics.heightDepth, kinectBasics.widthDepth, CV_8UC3);
	heightFromTable = Mat::ones(kinectBasics.heightDepth, kinectBasics.widthDepth, CV_32F) * (-1);

	for (int y = 0; y < kinectBasics.heightDepth; ++y)
	{
		for (int x = 0; x < kinectBasics.widthDepth; ++x)
		{
			// ������̍��������߂�
			USHORT distance = *heightMatrix.ptr<USHORT>(y, x);

			if (*heightMatrix.ptr<USHORT>(y, x) < 500 || (USHORT)KINECT_HEIGHT < *heightMatrix.ptr<USHORT>(y, x)) { continue; }

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
			if (TABLE_THRESHOLD < tHeight && tHeight < KINECT_HEIGHT)
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

	// Denoising by median filter
	//for (int i = 0; i < 5; ++i)
	//{
	//imshow("11", userAreaMat);
		medianBlur(userAreaMat, userAreaMat, 3);
		//imshow("22", userAreaMat);
		erode(userAreaMat, userAreaMat, Mat(), Point(-1, -1), 2);
		//imshow("33", userAreaMat);
		dilate(userAreaMat, userAreaMat, Mat(), Point(-1, -1), 1);

		//imshow("44", userAreaMat);

	//}
	//medianBlur(userAreaMat, userAreaMat, 3);

	return true;
}

bool HandGestureMultiCursor::getInfraredImageV2()
{
	static Mat infraredImageBuf;
	//if (!kinectBasics.GetInfraredMat(infraredImageBuf))
	//{
	//	return false;
	//}
	//infraredImage = infraredImageBuf.clone();
	//imshow("IR image1", infraredImage);
	//cout << 123123 << endl;
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



void HandGestureMultiCursor::showDebugWindows()
{
	if (!userAreaMat.empty())
	{ 
#if 0	// Debug: Show Table Area
		for (int i = 0; i < 4; ++i)
		{
			line(userAreaMat, Point(*tableCorners.ptr<int>(i, 0), *tableCorners.ptr<int>(i, 1)), Point(*tableCorners.ptr<int>((i+1)%4, 0), *tableCorners.ptr<int>((i+1)%4, 1)), Scalar(100, 255, 100), 2);
			//circle(userAreaMat, Point(*tableCorners.ptr<int>(i, 0), *tableCorners.ptr<int>(i, 1)), 2, Scalar(100, 255, 100), 2);
		}
#endif
		imshow("Hand/Head detection", userAreaMat);
	}

#ifndef USE_KINECT_V1
	//if (!depthImage.empty()) { imshow("Depth image", depthImage); }	// ����Kinect2�̂�
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
	
	// Debug
	//cout << "# of Blobs: " << blobs.size() << endl;
	//cvRenderBlobs(labelImg, blobs, &srcIpl, &srcIpl);

	// Filter noise / �m�C�Y�_�̏���
	cvFilterByArea(blobs, 2000, 1000000);

	// ��ʊO�ɐڂ��Ă��郆�[�U�̈�͍l�����Ȃ�
	for (CvBlobs::iterator it = blobs.begin(); it != blobs.end();)
	{
		if (it->second->minx <= 0 || kinectBasics.widthDepth <= it->second->maxx
			|| it->second->miny <= 0 || kinectBasics.heightDepth <= it->second->maxy)
		{
			it = blobs.erase(it);
			continue;
		}
			++it;
	}


	// Render blobs
	cvRenderBlobs(labelImg, blobs, &srcIpl, &srcIpl);
	imshow("55", userAreaMat);

	// Release unused IplImages
	cvReleaseImage(&labelImg);
	cvReleaseImage(&srcIplBinary);

	for (int y = 0; y < kinectBasics.heightDepth; ++y)
	{
		for (int x = 0; x < kinectBasics.widthDepth; ++x)
		{
			if (labelMat.ptr<uchar>(y, x) <= 0)
			{
				int index = ((y * kinectBasics.widthDepth) + x) * 3;
				UCHAR* dataDepth = &userAreaMat.data[index];
				dataDepth[0] = 0;
				dataDepth[1] = 0;
				dataDepth[2] = 0;
			}
		}
	}	

	return blobs;
}

void HandGestureMultiCursor::detectHeadPosition(CvBlobs blobs)
{
	// ���x���`�F�b�N�p�̍\����
	typedef struct
	{
		unsigned long labelID;
		int labelCount;
	}LabelTable;

	// Reset userData
	userData.clear();

	// �O�t���[���̃��x�������čł����[���̑����������x����O�t���[���̑Ή��̈�Ƃ���
	for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it)
	{
		// Define new user data
		UserData newUserData;
		newUserData.labelID = it->first;

		// �����t���[���͂��̂܂܃��[�U�ǉ�
		if (preLabelMat.empty())
		{
			userData.push_back(newUserData);
			continue;
		}

		// �����łȂ��ꍇ�͑O�t���[���Ƃ̑Ή��𒲂ׂ�
		vector<LabelTable> preLabelTable;
		// �����[�U�̈�̊e�s�N�Z�����S�t���[���ł͂ǂ̃��x�����������𒲂ׂ�
		for (int y = it->second->miny; y < it->second->maxy; ++y)
		{
			for (int x = it->second->minx; x < it->second->maxx; ++x)
			{
				// ���t���[���Ń��[�U�̈�łȂ��Ȃ�X�L�b�v
				if (*labelMat.ptr<unsigned long>(y, x) <= 0 || 10000 < *labelMat.ptr<unsigned long>(y, x)) { continue; }

				// �O�t���[���Ń��[�U�̈�łȂ��Ȃ�X�L�b�v
				unsigned long preLabel = *preLabelMat.ptr<unsigned long>(y, x);
				if (preLabel <= 0 || 10000 < preLabel) { continue; }

				// ���x���ɓ��[
				bool isFoundLabel = false;
				for (int i = 0; i < preLabelTable.size(); ++i)
				{
					if (preLabelTable[i].labelID == preLabel)
					{
						++preLabelTable[i].labelCount;
						isFoundLabel = true;
						break;
					}
				}
				// ���[���ꂽ���Ƃ̂Ȃ����x���������ꍇ�͐V���ɃJ�E���g�p�̃e�[�u�����쐬
				if (!isFoundLabel)
				{
					LabelTable newLabelTable;
					newLabelTable.labelID = preLabel;
					newLabelTable.labelCount = 1;
					preLabelTable.push_back(newLabelTable);
				}
					
			}
		}

		// �ł����[���̑������x����T��
		LabelTable labelTable;
		labelTable.labelCount = 0;
		labelTable.labelID = 0;
		for (size_t i = 0; i < preLabelTable.size(); ++i)
		{
			if (preLabelTable[i].labelCount > labelTable.labelCount)
			{
				labelTable.labelCount = preLabelTable[i].labelCount;
				labelTable.labelID = preLabelTable[i].labelID;
			}
		}

		//
		// �O�t���[���̃f�[�^�������p���Ƃ��͂����ōs��
		// �O�t���[���̃f�[�^�ƑΉ��t���Č��t���[���̃f�[�^�����
		// 
		if ((float)labelTable.labelCount / it->second->area > 0.50)	// ���ȏ���߂�̈悪�Ȃ��Ƃ��͖���
		{
			for (size_t i = 0; i < preUserData.size(); ++i)
			{
				if (preUserData[i].labelID == labelTable.labelID
					&& preUserData[i].headInfo.depthPoint.x > 0 && preUserData[i].headInfo.depthPoint.y > 0
					&& labelTable.labelID == *preLabelMat.ptr<unsigned long>(preUserData[i].headInfo.depthPoint.y, preUserData[i].headInfo.depthPoint.x))	   // �G���[���
				{
					preUserData[i].isDataFound = true;
					newUserData.isDataFound = true;
					//newUserData.headInfo.height = preUserData[i].headInfo.height;
					newUserData.headInfo.depthPoint.x = preUserData[i].headInfo.depthPoint.x;
					newUserData.headInfo.depthPoint.y = preUserData[i].headInfo.depthPoint.y;
					newUserData.cursorInfo.stayingTime = preUserData[i].cursorInfo.stayingTime;
					newUserData.cursorInfo.preCursor2d = preUserData[i].cursorInfo.preCursor2d;
					if (preUserData[i].cursorInfo.isDragging)
					{
						newUserData.cursorInfo.isDragging = true;
						newUserData.cursorInfo.sPt = preUserData[i].cursorInfo.sPt;
						newUserData.cursorInfo.cursorMove = preUserData[i].cursorInfo.cursorMove;
						newUserData.cursorInfo.TKinect2Table = preUserData[i].cursorInfo.TKinect2Table;
						newUserData.cursorInfo.preTKinect2Table = preUserData[i].cursorInfo.preTKinect2Table;
						
					}
					newUserData.preDataID = i;
					break;
				}
			}
		}

		userData.push_back(newUserData);
	}
	// Update preLabel
	preLabelMat = labelMat;


	// Find the base point of each user area
	//Point2i* newHighestPositions = new Point2i[blobs.size()];
	float highestHeight;
	Point2i* newHeadPositions = new Point2i[blobs.size()];
	INT* numHeadPoints = new INT[blobs.size()];
	int blobID = 0;
	for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); it++)
	{
		highestHeight = 0.0f;
		userData[blobID].headInfo.depthPoint.x = 0;
		userData[blobID].headInfo.depthPoint.y = 0;
		// �g���b�L���O���������ꍇ�͈ȑO�̓��̈ʒu�����Ɍ������ʒu���v�Z
		if (userData[blobID].isDataFound)
		{
			//userData[blobID].headInfo.height = preUserData[userData[blobID].preDataID].headInfo.height;
			userData[blobID].headInfo.depthPoint.x = preUserData[userData[blobID].preDataID].headInfo.depthPoint.x;
			userData[blobID].headInfo.depthPoint.y = preUserData[userData[blobID].preDataID].headInfo.depthPoint.y;
		}
		// �g���b�L���O���s���͍ł������ʒu�����Ɍ������ʒu���v�Z
		else
		{
			
			for (int y = it->second->miny; y <= it->second->maxy; y++)
			{
				for (int x = it->second->minx; x <= it->second->maxx; x++)
				{
					if (*labelMat.ptr<unsigned long>(y, x) == it->first)
					{
						float height = *heightFromTable.ptr<float>(y, x);
						if (highestHeight < height && height < KINECT_HEIGHT)
						{
							highestHeight = height;

							userData[blobID].headInfo.depthPoint.x = x;
							userData[blobID].headInfo.depthPoint.y = y;
						}
					}
				}
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

		// Debug: Show the search area
		//cv::rectangle(userAreaMat, cv::Point(minX, minY), cv::Point(maxX, maxY), cv::Scalar(200, 0, 0), 2, 4);

		float headHeight = *heightFromTable.ptr<float>(userData[blobID].headInfo.depthPoint.y, userData[blobID].headInfo.depthPoint.x);
		for (int y = minY; y <= maxY; y++)
		{
			for (int x = minX; x <= maxX; x++)
			{
				float height = *heightFromTable.ptr<float>(y, x);
				if (abs(headHeight - height) < HEAD_LENGTH && height < KINECT_HEIGHT
					&& it->first == labelMat.at<unsigned long>(y, x)	// ����blob���̂ݒT��
					)
				{
					newHeadPositions[blobID].x += x;
					newHeadPositions[blobID].y += y;
					numHeadPoints[blobID]++;
				}	
			}
		}

		++blobID;
	}

	// Make avarage pixel value of each head positions the head positions of users
	for (int i = 0; i < blobs.size(); i++)
	{
		// ���ς����܂�Γ��ʒu�X�V
		if (numHeadPoints[i] != 0)
		{
			// Calculate head position in 2D pixel
			userData[i].headInfo.depthPoint.x = newHeadPositions[i].x / numHeadPoints[i];
			userData[i].headInfo.depthPoint.y = newHeadPositions[i].y / numHeadPoints[i];
		}

		// Calculate head position in 3D point
		float* headPosition = point3fMatrix.ptr<float>(userData[i].headInfo.depthPoint.y, userData[i].headInfo.depthPoint.x);
		userData[i].headInfo.cameraPoint.x = headPosition[0];
		userData[i].headInfo.cameraPoint.y = headPosition[1];
		userData[i].headInfo.cameraPoint.z = headPosition[2];

		// Debug: Show the head point
		circle(userAreaMat, Point(userData[i].headInfo.depthPoint.x, userData[i].headInfo.depthPoint.y), 7, Scalar(255, 0, 0), 3);
	}

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

		Mat newUserRegion = Mat::zeros(kinectBasics.heightDepth, kinectBasics.widthDepth, CV_8UC3);

		int numIntersectionPoints = 0;
		Point3f center3f = Point3_<FLOAT>(userData[blobID].headInfo.cameraPoint.x, userData[blobID].headInfo.cameraPoint.y, userData[blobID].headInfo.cameraPoint.z);

		// ���[�U�̗̈����T��
		for (int y = it->second->miny; y <= it->second->maxy; y++) {
			for (int x = it->second->minx; x <= it->second->maxx; x++)
			{
				float lenX = center3f.x - point3fMatrix.ptr<float>(y, x)[0];
				float lenY = center3f.y - point3fMatrix.ptr<float>(y, x)[1];
				float lenZ = center3f.z - point3fMatrix.ptr<float>(y, x)[2];

				// Caliclate length from uaer's head 
				float length = sqrt(lenX * lenX + lenY * lenY);		// Z�͎g��Ȃ�
				// Define the intersection point of the sphere which its center is head and the hand as the hand position 
				if (it->first == labelMat.at<unsigned long>(y, x)	// ����blob���̂ݒT��
					&& TABLE_THRESHOLD < *heightFromTable.ptr<float>(y, x) && *heightFromTable.ptr<float>(y, x) < KINECT_HEIGHT) // Don't include desk
				{
					int index = ((y * kinectBasics.widthDepth) + x) * 3;

					if (SENCIG_CIRCLE_RADIUS < length ) {	// ���ړ_�����ƌ������Ă��邩�ǂ���
						// ��̗̈���L�^����
						UCHAR* dataHand = &newHandRegion.data[index];
						dataHand[0] = 255;
						dataHand[1] = 255;
						dataHand[2] = 255;

						numIntersectionPoints++;
					}
					else
					{
						// ���̗̈���L�^����
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
			userData[blobID].isArmTracked = true;
		}
		else{
			userData[blobID].isArmTracked = false;
		}

		// Add hand resion
		handRegions.push_back(newHandRegion);
		headRegions.push_back(newHeadRegion);

		blobID++;
	}
}

void HandGestureMultiCursor::detectHand(vector<Mat> handRegions, vector<Mat> headRegions)
{
	if (handRegions.size() == 0 || headRegions.size() == 0 || userData.size() == 0)	{ return; }
	
	FingerTipDetector fingerTipDetector;

	vector<Mat> userRegions;
	for (int i = 0; i < handRegions.size(); ++i)
	{
		if (!userData[i].isArmTracked) { break; }

		//cout << userData[i].isDataFound << endl;
		

		// �E�荶������o
		CvBlob handR, handL;
		Mat newUserRegions = fingerTipDetector.GetHandInfo(handRegions[i], headRegions[i], point3fMatrix, userData[i].handInfoR, userData[i].handInfoL);
		userRegions.push_back(newUserRegions);

#if 0
		// �ċA�����ˍރ}�[�J���g���Ă�萳�m�Ɏ�̈ʒu�����߂�
		if (userData[i].handInfoR.isTracked)
		{
			Mat infraredImageBuf;
			waitKey(10);

			if (!kinectBasics.GetInfraredMat(infraredImageBuf)) { break; }

			imshow("IR image", infraredImageBuf);
			
			const int OFFSET = 20;
			if (userData[i].handInfoR.depthPoint.x - OFFSET < 0) { userData[i].handInfoR.depthPoint.x = OFFSET; }
			if (userData[i].handInfoR.depthPoint.x + OFFSET > kinectBasics.widthInfrared) { userData[i].handInfoR.depthPoint.x = kinectBasics.widthInfrared - OFFSET; }
			if (userData[i].handInfoR.depthPoint.y - OFFSET < 0) { userData[i].handInfoR.depthPoint.y = OFFSET; }
			if (userData[i].handInfoR.depthPoint.y + OFFSET > kinectBasics.widthInfrared) { userData[i].handInfoR.depthPoint.y = kinectBasics.widthInfrared - OFFSET; }
			

			Mat handAreaIR = infraredImageBuf(Rect(userData[i].handInfoR.depthPoint.x - OFFSET, userData[i].handInfoR.depthPoint.y - OFFSET, 2 * OFFSET, 2 * OFFSET));
			
			Mat handAreaIRGray;
			cv::cvtColor(handAreaIR, handAreaIRGray, CV_BGR2GRAY);
			const int scale = 3;
			cv::resize(handAreaIRGray, handAreaIRGray, cv::Size(), scale, scale);
			//handAreaIRGray = ~handAreaIRGray;
			// Hough�ϊ��̂��߂̑O�����i�摜�̕��������s�Ȃ�Ȃ��ƌ댟�o���������₷���j
			//cv::GaussianBlur(handAreaIRGray, handAreaIRGray, cv::Size(11, 11), 2, 2);

			// Hough�ϊ��ɂ��~�̌��o�ƌ��o�����~�̕`��
			std::vector<cv::Vec3f> circles;
			cv::HoughCircles(handAreaIRGray, circles, CV_HOUGH_GRADIENT, 1, 100, 200, 10, 0, 10);

			std::vector<cv::Vec3f>::iterator it = circles.begin();
			if (it == circles.end()) {
				userData[i].handInfoR.isTracked = false;
				//if (userData[i].isDataFound && preUserData[userData[i].preDataID].handInfoR.isTracked)
				//{
				//	userData[i].handInfoR.depthPoint = preUserData[userData[i].preDataID].handInfoR.depthPoint;

				//	//	//cout << pow(center2.x - preUserData[userData[i].preDataID].handInfoR.depthPoint.x, 2) + pow(center2.y - preUserData[userData[i].preDataID].handInfoR.depthPoint.y, 2) << endl;
				//	userData[i].handInfoR.cameraPoint = Point3f(preUserData[userData[i].preDataID].handInfoR.cameraPoint);
				//	cout << 111 << endl;
				//}
				cout << "aaa" << endl;

				continue;
			}
			//for (; it != circles.end(); ++it) {
				Point center(cv::saturate_cast<float>((*it)[0])/scale, cv::saturate_cast<float>((*it)[1])/scale);
				int radius = cv::saturate_cast<float>((*it)[2]/scale);
				circle(handAreaIR, center, radius, cv::Scalar(0, 0, 255), 1);
				circle(handAreaIRGray, Point(center.x*scale, center.y*scale), radius*scale, cv::Scalar(255, 255, 255), 3);
			//}
			imshow("IR", handAreaIR);

			imshow("aaa", handAreaIRGray);

			DepthSpacePoint depthSpacePoint;
			depthSpacePoint.X = userData[i].handInfoR.depthPoint.x - OFFSET + saturate_cast<float>((*it)[0]/scale);
			depthSpacePoint.Y = userData[i].handInfoR.depthPoint.y - OFFSET + saturate_cast<float>((*it)[1]/scale);
			
			
			//cout << userData[i].handInfoR.depthPoint << endl;
			Point center2(userData[i].handInfoR.depthPoint.x - OFFSET + saturate_cast<float>((*it)[0] / scale), userData[i].handInfoR.depthPoint.y - OFFSET + saturate_cast<float>((*it)[1] / scale));
			cout << center << endl;


			//if (userData[i].isDataFound && preUserData[userData[i].preDataID].handInfoR.isTracked
			//	&& pow(center2.x - preUserData[userData[i].preDataID].handInfoR.depthPoint.x, 2) 
			//	+ pow(center2.y - preUserData[userData[i].preDataID].handInfoR.depthPoint.y, 2) < 2
			//	)
			//{
			//	userData[i].handInfoR.depthPoint = preUserData[userData[i].preDataID].handInfoR.depthPoint;

			//	//cout << pow(center2.x - preUserData[userData[i].preDataID].handInfoR.depthPoint.x, 2) + pow(center2.y - preUserData[userData[i].preDataID].handInfoR.depthPoint.y, 2) << endl;
			//	userData[i].handInfoR.cameraPoint =  Point3f(preUserData[userData[i].preDataID].handInfoR.cameraPoint);
			//}
			//else
			{
				userData[i].handInfoR.depthPoint.x = depthSpacePoint.X;
				userData[i].handInfoR.depthPoint.y = depthSpacePoint.Y;

				CameraSpacePoint handPosition;
				kinectBasics.GetMapper()->MapDepthPointToCameraSpace(depthSpacePoint, *heightMatrix.ptr<USHORT>(depthSpacePoint.Y, depthSpacePoint.X), &handPosition);
				userData[i].handInfoR.cameraPoint.x = handPosition.X;
				userData[i].handInfoR.cameraPoint.y = handPosition.Y;
				userData[i].handInfoR.cameraPoint.z = handPosition.Z;
				cout << userData[i].handInfoR.cameraPoint << endl;
			} 
			//else cout << "no" << endl;
			//if (userData[i].isDataFound && preUserData[userData[i].preDataID].handInfoR.isTracked)
			//cout << pow(center2.x - preUserData[userData[i].preDataID].handInfoR.depthPoint.x, 2) + pow(center2.y - preUserData[userData[i].preDataID].handInfoR.depthPoint.y, 2) << endl;


			/*CameraSpacePoint handPosition;
			kinectBasics.GetMapper()->MapDepthPointToCameraSpace(depthSpacePoint, *heightMatrix.ptr<USHORT>(depthSpacePoint.Y, depthSpacePoint.X), &handPosition);
			userData[i].handInfoR.cameraPoint.x = handPosition.X;
			userData[i].handInfoR.cameraPoint.y = handPosition.Y;
			userData[i].handInfoR.cameraPoint.z = handPosition.Z;*/
			//cout << handPosition.X << ", " << handPosition.Y << ", " << handPosition.Z << endl;

			
		}
#endif
		
	}

#if 0
	// Show hand regions
	int numShownUserMax = 5;
	for (int i = 0; i < numShownUserMax; ++i)
	{
		stringstream ii;
		ii << "User"<< i;
		string iStr = ii.str();

		if (i >= userRegions.size())
		{
			destroyWindow(iStr);
		}
		else
		{
			imshow(iStr, userRegions[i]);
		}
	}
#endif
}

int HandGestureMultiCursor::togglePointingMode()
{
	pointingControlMode = (pointingControlMode == POINTING_DISABLE ? POINTING_ENABLE : POINTING_DISABLE);
	return pointingControlMode;
}

void HandGestureMultiCursor::calcCursorPos(CvBlobs blobs)
{
	// ��΍��W�w�薳���̏ꍇ
	if (pointingControlMode == POINTING_DISABLE)
	{
		for (size_t blobID = 0; blobID < userData.size(); ++blobID)
		{
			userData[blobID].cursorInfo.isShownCursor = false;
		}
		return;
	}

	for (size_t blobID = 0; blobID < userData.size(); ++blobID)
	{
		userData[blobID].cursorInfo.isShownCursor = false;
		HandInfo* handPointing = &userData[blobID].handInfoR;
		if (handPointing->isTracked)
		{
			Mat handPoint = (cv::Mat_<float>(4, 1) << handPointing->cameraPoint.x * 1000, handPointing->cameraPoint.y * 1000, handPointing->cameraPoint.z * 1000, 1);
			Mat headPoint = (cv::Mat_<float>(4, 1) << userData[blobID].headInfo.cameraPoint.x * 1000, userData[blobID].headInfo.cameraPoint.y * 1000, userData[blobID].headInfo.cameraPoint.z * 1000, 1);

			// ��̈ʒu�␳ [mm]
			//*handPoint.ptr<float>(0, 0) += 100;
			//*handPoint.ptr<float>(1, 0) += 10000;
			//*handPoint.ptr<float>(2, 0) -= 175;
			*headPoint.ptr<float>(0, 0) -= 100;
			*headPoint.ptr<float>(1, 0) -= 100;
			*headPoint.ptr<float>(2, 0) += 150;
			
			for (size_t i = 0; i < TKinect2Display.size(); ++i)
				//for (size_t i = 0; i < TKinect2Display.size(); ++i)
			{
				Mat handPointScreen = TKinect2Display[i] * handPoint;
				Mat headPointScreen = TKinect2Display[i] * headPoint;

				// �G���[�`�F�b�N
				if (*handPointScreen.ptr<float>(2, 0) < *headPointScreen.ptr<float>(2, 0))
				{
					// ���蓪�̕����f�B�X�v���C�ɋ߂��̂͂�������
					continue;
				}

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


				// �J�[�\���ʒu[pixel]�����߂�
				Mat cursor2d = TDisplay2Pixel[i] * cursor3d;
				cursor2d /= *cursor2d.ptr<float>(2, 0);
				//cout << "cursor: " <<  cursor2d << endl;
				// Set cursor position in pixel coordinate if the cursor is in the display
				if (windowOffsetX[i] <= *cursor2d.ptr<float>(0, 0) && *cursor2d.ptr<float>(0, 0) < windowOffsetX[i] + VEC_WIN_WIDTH[i]
					&& 0 <= *cursor2d.ptr<float>(1, 0) && *cursor2d.ptr<float>(1, 0) < VEC_WIN_HEIGHT[i])
				{
					if (userData[blobID].isDataFound
						&& i == preUserData[userData[blobID].preDataID].cursorInfo.displayNum)
					{
						
						//cout << userData[blobID].cursorInfo.preCursor2d.size() << endl;

						// �W�b�^�[�h�~
						// �On�t���[���̕��ς̈ʒu�����߂�i�W�b�^�[�h�~�j
						Point2f cursor2dBuf = Point2f(*cursor2d.ptr<float>(0, 0), *cursor2d.ptr<float>(1, 0));
						userData[blobID].cursorInfo.preCursor2d.push_back(cursor2dBuf);

						if (userData[blobID].cursorInfo.preCursor2d.size() > preCursor2dNumMax)
						{
							userData[blobID].cursorInfo.preCursor2d.pop_front();
						}
						Point2f cursor2dAve;
						list<Point2f>::iterator it = userData[blobID].cursorInfo.preCursor2d.begin(); // �C�e���[�^
						while (it != userData[blobID].cursorInfo.preCursor2d.end())  // list�̖����܂�
						{
							cursor2dAve += *it;
							++it;  // �C�e���[�^���P�i�߂�
						}
						cursor2dAve.x /= userData[blobID].cursorInfo.preCursor2d.size();
						cursor2dAve.y /= userData[blobID].cursorInfo.preCursor2d.size();
						userData[blobID].cursorInfo.position = cursor2dAve;
					}
					else
					{
						userData[blobID].cursorInfo.preCursor2d.clear();
						userData[blobID].cursorInfo.position = Point2f(*cursor2d.ptr<float>(0, 0), *cursor2d.ptr<float>(1, 0));
					}
					userData[blobID].cursorInfo.isShownCursor = true;
					userData[blobID].cursorInfo.displayNum = i;

					//userData[blobID].cursorInfo.position = Point2f(*cursor2d.ptr<float>(0, 0), *cursor2d.ptr<float>(1, 0));
					
					break;
				}

				// ������ƃo�O�𖳗����C�����Ă�
				//if (TKinect2Display.size() > 1)
				//{
				//	if (i == 0)
				//	{
				//		i = 2;
				//	}
				//	else if (i == 1)
				//	{
				//		cout << i << endl;
				//		i = 3;
				//		cout << i << endl;
				//	}
				//	else if (i == 2)
				//	{
				//		i = 1;
				//	}
				//}
				//else
				//{
				//	++i;
				//}
			}

			if (!userData[blobID].cursorInfo.isShownCursor)
			{
				userData[blobID].cursorInfo.preCursor2d.clear();
			}
		}
	}
}

void HandGestureMultiCursor::pointingCursorControl()
{
	timer.stop();
	double addingTime = timer.getTimeSec();

	// ��莞�Ԉȏ�f�B�X�v���C�̓����Ƃ�����w�����ƃJ�[�\���������Ɉړ�
	//for (size_t i = 0; i < 1; ++i)
	for (size_t i = 0; i < userData.size(); ++i)
	{
		HandInfo* handPointing = &userData[i].handInfoR;

		if (!userData[i].cursorInfo.isShownCursor || handPointing->isOnTable || !userData[i].isDataFound)
		{
			userData[i].cursorInfo.isShownCursor = false;
			userData[i].cursorInfo.stayingTime = 0.0;
			continue;
		}

		int distance = sqrt(
			pow(userData[i].cursorInfo.position.x - preUserData[userData[i].preDataID].cursorInfo.position.x, 2)
			+ pow(userData[i].cursorInfo.position.y - preUserData[userData[i].preDataID].cursorInfo.position.y, 2)
			); 

		if (distance < 50)
		{
#ifndef ONLY_POINTIG_GESTURE
			// ��莞��(timeLimit)�����ꏊ���w���������J�[�\�����ړ�
			userData[i].cursorInfo.stayingTime += addingTime;
			if (userData[i].cursorInfo.stayingTime > timeLimit)
			{
				
				if (windowOffsetX[userData[i].cursorInfo.displayNum] < userData[i].cursorInfo.position.x && userData[i].cursorInfo.position.x < windowOffsetX[userData[i].cursorInfo.displayNum] + VEC_WIN_WIDTH[userData[i].cursorInfo.displayNum]
					&& 0 < userData[i].cursorInfo.position.y && userData[i].cursorInfo.position.y < VEC_WIN_HEIGHT[userData[i].cursorInfo.displayNum])
				{
					SetCursor(userData[i].cursorInfo.position.x, userData[i].cursorInfo.position.y);
					userData[i].cursorInfo.stayingTime = 0.0;

					// ���₷�����邽��ctrl�L�[�ŃJ�[�\���ʒu����
					::keybd_event(VK_CONTROL, 0, 0, 0);
					::keybd_event(VK_CONTROL, 0, KEYEVENTF_KEYUP, 0);
				}
			}
#else
			// ��ɃJ�[�\���ړ�����
			//if (windowOffsetX[userData[i].cursorInfo.displayNum] < userData[i].cursorInfo.position.x 
			//	&& userData[i].cursorInfo.position.x < windowOffsetX[userData[i].cursorInfo.displayNum] + VEC_WIN_WIDTH[userData[i].cursorInfo.displayNum]
			//	&& 0 < userData[i].cursorInfo.position.y
			//	&& userData[i].cursorInfo.position.y < VEC_WIN_HEIGHT[userData[i].cursorInfo.displayNum])
			if (userData[i].cursorInfo.isShownCursor)
			{
				if (userData[i].preDataID >= 0)
				{
					int distance = norm(Point(
						userData[i].cursorInfo.position.x - preUserData[userData[i].preDataID].cursorInfo.position.x,
						userData[i].cursorInfo.position.y - preUserData[userData[i].preDataID].cursorInfo.position.y
						));
					if (distance < 20)
					{
						userData[i].cursorInfo.position = preUserData[userData[i].preDataID].cursorInfo.position;
					}
				}
				cvWaitKey(1);
				SetCursor(userData[i].cursorInfo.position.x, userData[i].cursorInfo.position.y);

				userData[i].cursorInfo.stayingTime += addingTime;
			}
#endif
		}
	}

	timer.reset();
	timer.start();
}

// �N���X�錾
static MouseControl mouseControl;
void HandGestureMultiCursor::relativeCursorControl()
{
	const Scalar touchedColor = Scalar(255, 0, 255);
	const Scalar untouchedColor = Scalar(0, 255, 0);
	const float lineLength = 0.25f;

	for (size_t i = 0; i < userData.size(); ++i)
	{
		// �}�E�X�̑��Έʒu�ړ����s��
		switch (relativeControlMode)
		{
		case mc::DISTANCE_BASED_METHOD:
			mouseControl.moveCursorDistance(userData[i], tableParam, tableCorners);
			putText(userAreaMat, "Distance based mode", Point(2, 28), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 200), 1, CV_AA);
			break;
		case mc::TIME_BASED_METHOD:
			mouseControl.moveCursorTime(userData[i], tableParam, tableCorners);
			putText(userAreaMat, "Time based mode", Point(2, 28), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 200), 1, CV_AA);
			break;
		case mc::DISTANCE_TIME_BASED_METHOD:
			mouseControl.moveCursorTimeDistance(userData[i], tableParam, tableCorners);
			putText(userAreaMat, "Distance + Time based mode", Point(2, 28), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 200), 1, CV_AA);
			break;
		default:
			cerr << "Error: relativeControlMode is not selected." << endl;
			break;
		}

#if 1	// Debug: �r�̍��W�n�\��
		if (!userData[i].cursorInfo.TKinect2Table.empty())
		{
			HandInfo handinfo = userData[i].handInfoR;

			// ���݂̎�̈ʒu
			CameraSpacePoint test;
			test.X = handinfo.cameraPoint.x;
			test.Y = handinfo.cameraPoint.y;
			test.Z = handinfo.cameraPoint.z;
			DepthSpacePoint zero1;
			kinectBasics.GetMapper()->MapCameraPointToDepthSpace(test, &zero1);
			circle(userAreaMat, Point(zero1.X, zero1.Y), 3, touchedColor, 2);

			// ���_
			Mat zero = (Mat_<float>(4, 1) << 0, 0, 0, 1);
			zero = userData[i].cursorInfo.TKinect2Table.inv() * zero;
			CameraSpacePoint camera;
			camera.X = zero.at<float>(0, 0);
			camera.Y = zero.at<float>(1, 0);
			camera.Z = zero.at<float>(2, 0);
			DepthSpacePoint depthZero;
			kinectBasics.GetMapper()->MapCameraPointToDepthSpace(camera, &depthZero);

			// X
			Mat move = (Mat_<float>(4, 1) << lineLength, 0, 0, 1);
			move = userData[i].cursorInfo.TKinect2Table.inv() * move;
			camera;
			camera.X = move.at<float>(0, 0);
			camera.Y = move.at<float>(1, 0);
			camera.Z = move.at<float>(2, 0);
			DepthSpacePoint depth;
			kinectBasics.GetMapper()->MapCameraPointToDepthSpace(camera, &depth);
			line(userAreaMat, Point(depthZero.X, depthZero.Y), Point(depth.X, depth.Y), Scalar(0, 0, 255), 2);
			// Y
			move = (Mat_<float>(4, 1) << 0, lineLength, 0, 1);
			move = userData[i].cursorInfo.TKinect2Table.inv() * move;
			camera.X = move.at<float>(0, 0);
			camera.Y = move.at<float>(1, 0);
			camera.Z = move.at<float>(2, 0);
			kinectBasics.GetMapper()->MapCameraPointToDepthSpace(camera, &depth);
			line(userAreaMat, Point(depthZero.X, depthZero.Y), Point(depth.X, depth.Y), Scalar(0, 255, 0), 2);
			// Z
			move = (Mat_<float>(4, 1) << 0, 0, lineLength, 1);
			move = userData[i].cursorInfo.TKinect2Table.inv() * move;
			camera.X = move.at<float>(0, 0);
			camera.Y = move.at<float>(1, 0);
			camera.Z = move.at<float>(2, 0);
			kinectBasics.GetMapper()->MapCameraPointToDepthSpace(camera, &depth);
			line(userAreaMat, Point(depthZero.X, depthZero.Y), Point(depth.X, depth.Y), Scalar(255, 0, 0), 2);
		}
		else
		{
			if (userData[i].handInfoR.isTracked)
				circle(userAreaMat, userData[i].handInfoR.depthPoint, 3, untouchedColor, 2);
		}

		if (userData[i].handInfoL.isTracked)
		{
			Scalar colorL = userData[i].handInfoL.isOnTable ? touchedColor : untouchedColor;
			circle(userAreaMat, userData[i].handInfoL.depthPoint, 3, colorL, 2);
		}
#endif
	}
}

void HandGestureMultiCursor::SetCursor(float x, float y)
{
	const int offsetX = 15;
	const int offsetY = 8;

	// �X�N���[�����W��mouse_event()�p�̍��W�ϊ�
	DWORD dwX = (x + offsetX) * 65535 / ::GetSystemMetrics(SM_CXSCREEN);
	DWORD dwY = (y + offsetY) * 65535 / ::GetSystemMetrics(SM_CYSCREEN);

	// Set mouse cursor position
	::mouse_event(MOUSEEVENTF_ABSOLUTE | MOUSEEVENTF_MOVE, dwX, dwY, NULL, NULL);
}

#pragma region OpenGL

void HandGestureMultiCursor::initGL(int argc, char* argv[])
{
	const int offsetX = 9;

	glutInit(&argc, argv);

	WinIDs = new int[TKinect2Display.size()];
#if 1
	for (size_t i = 0; i < TKinect2Display.size(); ++i)
	{
		glutInitWindowPosition(windowOffsetX[i] + offsetX * i, 0);
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
		//glutMouseFunc(smouse);

		glClearColor(1.0, 1.0, 1.0, 1.0);

		/* Camera setup */
		glViewport(0, 0, kinectBasics.widthDepth, kinectBasics.heightDepth);
		glLoadIdentity();

		//if (i == 0){
		//	/* GL�̃E�B���h�E���t���X�N���[���� */
		//	//GL�̃f�o�C�X�R���e�L�X�g�n���h���擾
		//	glutSetWindow(WinIDs[i]);
		//	HDC glDc = wglGetCurrentDC();
		//	//�E�B���h�E�n���h���擾
		//	HWND hWnd = WindowFromDC(glDc);
		//	//�E�B���h�E�̑����ƈʒu�ύX
		//	SetWindowLong(hWnd, GWL_STYLE, WS_POPUP);
		//	//SetWindowPos(hWnd, HWND_TOP, 0, 0, 50, 50, SWP_SHOWWINDOW);
		//	SetWindowPos(hWnd, HWND_TOP, windowOffsetX[i], 0, VEC_WIN_WIDTH[i], VEC_WIN_HEIGHT[i], SWP_SHOWWINDOW);
		//}
	} 
	cout << "aaa: " << windowOffsetX[2] << endl;
#else
	if (TKinect2Display.size() >= 0)
	{
		glutInitWindowPosition(0, 0);
		glutInitWindowSize(200, 100);

		glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
		WinIDs[0] = glutCreateWindow("00");

		//// Register callback functions
		//glutReshapeFunc(sreshape);
		glutDisplayFunc(sdisplay);
		glutIdleFunc(sidle);
		glutKeyboardFunc(skeyboard);
		//glutMouseFunc(smouse);

		glClearColor(1.0, 1.0, 1.0, 1.0);

		/* Camera setup */
		glViewport(0, 0, kinectBasics.widthDepth, kinectBasics.heightDepth);
		glLoadIdentity();

		/* GL�̃E�B���h�E���t���X�N���[���� */
		//GL�̃f�o�C�X�R���e�L�X�g�n���h���擾
		//glutSetWindow(WinIDs[0]);
		HDC glDc = wglGetCurrentDC();
		//�E�B���h�E�n���h���擾
		HWND hWnd = WindowFromDC(glDc);
		//�E�B���h�E�̑����ƈʒu�ύX
		SetWindowLong(hWnd, GWL_STYLE, WS_POPUP);
		SetWindowPos(hWnd, HWND_TOP, 0, 0, 50, 50, SWP_SHOWWINDOW);
	}
#endif
}

// �J�[�\����`�悷��
void HandGestureMultiCursor::display(void)
{
	const float divisionX = 12;
	float cellLength = VEC_WIN_WIDTH[0] / divisionX;

	// ��ʂ��N���A
	glClearColor(1.0, 1.0, 1.0, 1.0);
	for (size_t i = 0; i < VEC_WIN_WIDTH.size(); ++i)
	{
		glutSetWindow(WinIDs[i]);
		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
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
				Point2f cursorPos(
					(userData[i].cursorInfo.position.x - windowOffsetX[userData[i].cursorInfo.displayNum] - windowWidth / 2) / windowWidth * 2,
					(windowHeight - userData[i].cursorInfo.position.y - windowHeight / 2) / windowHeight * 2
					);
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
	// �I��
	case 'q':	
	case 'Q':
	case '\033':	// ESC
		exit(0);
		break;
	// �f�o�b�O�E�B���h�E�̕\���E��\���؂�ւ�
	case 'd':
	case 'D':
		isShowDebugWindows = !isShowDebugWindows;
		break;
	// ���[�U�̓����ʒu���Z�b�g�D�����ʒu�������������͂����
	case 'r':
	case 'R':
		preUserData.clear();
		break;
	// �N���b�N�̗L�����C������
	case 'c':
	case 'C':
		mouseControl.toggleClickMode() == mc::CLICK_ENABLE ?
			std::cout << "Click enable" << endl :
			std::cout << "Click disable" << endl;
		break;
	// ��΍��W�w��ɂ�鑀��@�؂�ւ�
	case 'p':
	case 'P':
		togglePointingMode() == POINTING_ENABLE ?
			std::cout << "Pointing enable" << std::endl :
			std::cout << "Pointing disable" << std::endl;
		break;
	// ���΍��W�w��ɂ�鑀��@�؂�ւ�
	case '1':	// ��@�P�i�����w��j�ɐ؂�ւ�
		relativeControlMode = mc::DISTANCE_BASED_METHOD;
		cout << "Changed to use Distance based method." << endl;
		break;
	case '2':	// ��@�Q�i���Ԏw��j�ɐ؂�ւ�
		relativeControlMode = mc::TIME_BASED_METHOD;
		cout << "Changed to use Time based method." << endl;
		break;
	case '3':	// ��@�R�i�����{���Ԏw��j�ɐ؂�ւ�
		relativeControlMode = mc::DISTANCE_TIME_BASED_METHOD;
		cout << "Changed to use Distance + Time based method." << endl;
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
		SetCursor(mouseX, mouseY);

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
		// ���E���]�𒼂�
		*TK2D.ptr<float>(1, 0) *= -1;
		*TK2D.ptr<float>(1, 1) *= -1;
		*TK2D.ptr<float>(1, 2) *= -1;
		*TK2D.ptr<float>(1, 3) *= -1;
		read(fn[1], TD2P);	// Load transformation matrix from display plane to display pixel image 

		if (winWidth > 0 && winHeight > 0 && !TK2D.empty() && !TD2P.empty())
		{
			VEC_WIN_WIDTH.push_back(winWidth);
			VEC_WIN_HEIGHT.push_back(winHeight);
			TKinect2Display.push_back(TK2D);
			TDisplay2Pixel.push_back(TD2P);


			int offsetX = windowOffsetX[i] + winWidth;
			windowOffsetX.push_back(offsetX);

			cout << "Succeeded to load display[" << i << "]" << endl;
			cout << "(width, height) = " << winWidth << ", " << winHeight << endl;
			cout << "TKinect2Display: " << endl << TKinect2Display[i] << endl;
			cout << "TDisplay2Pixel: " << endl << TDisplay2Pixel[i] << endl;
		}
		else
		{
			cout << "Failed to load display[" << i << "]" << endl;
			ErrorExit();
		}
	}

	if (VEC_WIN_WIDTH.size() <= 0)
	{
		cout << "Error(loadCalibData): No display information was loaded" << endl;
		ErrorExit();
	}

	// Import table information
	FileStorage cvfs(tableInfo_filename, CV_STORAGE_READ);
	FileNode node(cvfs.fs, NULL);
	FileNode fn = node[string("mat_array")];
	read(fn[0], tableParam);
	cout << "Transformation matrix from kinect to table: " << endl << tableParam << endl;
	// Import table corners
	fn = node[string("table_corners")];
	read(fn[0], tableCorners);
	if (tableCorners.cols != 2 || tableCorners.rows != 4)
	{
		cout << "Error: Table corner infomation is not correct." << endl;
		ErrorExit();
	}
	cout << "Table corners:" << endl << tableCorners << endl;
	
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

//void HandGestureMultiCursor::mouse(int button, int state, int mouse_x, int mouse_y)
//{
//	switch (button) {
//	case GLUT_LEFT_BUTTON:
//		printf("left");
//		break;
//	case GLUT_MIDDLE_BUTTON:
//		printf("middle");
//		break;
//	case GLUT_RIGHT_BUTTON:
//		printf("right");
//		break;
//	default:
//		break;
//	}
//
//	printf(" button is ");
//
//	switch (state) {
//	case GLUT_UP:
//		printf("up");
//		break;
//	case GLUT_DOWN:
//		printf("down");
//		break;
//	default:
//		break;
//	}
//
//	printf(" at (%d, %d)\n", mouse_x, mouse_y);
//}
//
//void smouse(int button, int state, int mouse_x, int mouse_y)
//{
//	app.mouse(button, state, mouse_x, mouse_y);
//}

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
