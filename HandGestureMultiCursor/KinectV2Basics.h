#pragma once

// InfraredSourceValueMaximum is the highest value that can be returned in the InfraredFrame.
// It is cast to a float for readability in the visualization code.
#define InfraredSourceValueMaximum static_cast<float>(USHRT_MAX)

// The InfraredOutputValueMinimum value is used to set the lower limit, post processing, of the
// infrared data that we will render.
// Increasing or decreasing this value sets a brightness "wall" either closer or further away.
#define InfraredOutputValueMinimum 0.01f 

// The InfraredOutputValueMaximum value is the upper limit, post processing, of the
// infrared data that we will render.
#define InfraredOutputValueMaximum 1.0f

// The InfraredSceneValueAverage value specifies the average infrared value of the scene.
// This value was selected by analyzing the average pixel intensity for a given scene.
// Depending on the visualization requirements for a given application, this value can be
// hard coded, as was done here, or calculated by averaging the intensity for each pixel prior
// to rendering.
#define InfraredSceneValueAverage 0.08f

/// The InfraredSceneStandardDeviations value specifies the number of standard deviations
/// to apply to InfraredSceneValueAverage. This value was selected by analyzing data
/// from a given scene.
/// Depending on the visualization requirements for a given application, this value can be
/// hard coded, as was done here, or calculated at runtime.
#define InfraredSceneStandardDeviations 3.0f

class KinectV2Basics
{
	

public:
	KinectV2Basics();
	~KinectV2Basics();

	// 各サイズ
	const static int widthDepth   = 512;
	const static int heightDepth  = 424;
	const static int widthColor  = 1920;
	const static int heightColor = 1080;
	const static int widthInfrared = 512;
	const static int heightInfrared = 424;

	// Select using data (Call before "bool SetupKinectV2()")
	void SelectUsingData(const bool isUseDepthStream, const bool isUseColorStream);

	// Set up KinectV2
	bool SetupKinectV2();

	// Getters
	IKinectSensor*			GetSensor();
	IDepthFrameSource*		GetSourceDepth();
	IDepthFrameReader*		GetReaderDepth();
	IDepthFrame*			GetFrameDepth();
	IColorFrameSource*		GetSourceColor();
	IColorFrameReader*		GetReaderColor();
	IColorFrame*			GetFrameColor();
	IInfraredFrameSource*	GetSourceInfrared();
	IInfraredFrameReader*	GetReaderInfrared();
	IInfraredFrame*			GetFrameInfrared();
	ICoordinateMapper*		GetMapper();
	
	std::vector<UINT16> infraredBuffer;
#ifdef OPENCV
	// 現フレームの深度画像(imshow()表示用)と，ピクセルごとのZ軸距離を格納した行列(imshow()不可)を取得
	bool GetDepthMat(cv::Mat& outDepth8U, cv::Mat& outDepth16S, cv::Mat& outPoints32FC3);
	bool GetDepthMat(cv::Mat& outDepth8U, cv::Mat& outDepth16S);
	bool GetDepthMat(cv::Mat& outDepth16S);

	// 深度カメラ座標系の点群取得
	bool GetPointsMat(cv::Mat& pointsMat);

	// 現フレームのカラー画像(Mat)取得
	bool GetColorMat(cv::Mat& outColor);
	bool GetColorMat(cv::Mat& outColor, float scale);	// 画像サイズが大きいのでスケール調整可能にした

	// 現フレームの赤外光画像(Mat)取得
	bool GetInfraredMat(cv::Mat& outInfrared);
#endif

private:
	// データ取得するまでの各ポインタ
	IKinectSensor*			pSensor;
	// Depth
	IDepthFrameSource*		pDepthSource;
	IDepthFrameReader*		pDepthReader;
	IDepthFrame*			pDepthFrame;
	// Color
	IColorFrameSource*		pColorSource;
	IColorFrameReader*		pColorReader;
	IColorFrame*			pColorFrame;
	// Infrared
	IInfraredFrameSource*	pInfraredSource;
	IInfraredFrameReader*	pInfraredReader;
	IInfraredFrame*			pInfraredFrame;
	// Mapper
	ICoordinateMapper*		pCoordinateMapper;

	bool isUseDepth, isUseColor;

#ifdef OPENCV


#endif
};