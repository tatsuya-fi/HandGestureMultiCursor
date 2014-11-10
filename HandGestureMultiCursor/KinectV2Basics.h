#pragma once
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

	// Select using data (Call before "bool SetupKinectV2()")
	void SelectUsingData(const bool isUseDepthStream, const bool isUseColorStream);

	// Set up KinectV2
	bool SetupKinectV2();

	// Getters
	IKinectSensor*		GetSensor();
	IDepthFrameSource*	GetSourceDepth();
	IDepthFrameReader*	GetReaderDepth();
	IDepthFrame*		GetFrameDepth();
	IColorFrameSource*	GetSourceColor();
	IColorFrameReader*	GetReaderColor();
	IColorFrame*		GetFrameColor();
	ICoordinateMapper*	GetMapper();
	

#ifdef OPENCV
	// outDepth8UC3:   現フレームの深度画像(imshow()表示用)
	// outDepth16S:    ピクセルごとのZ軸距離を格納した行列
	// outPoints32FC3: 深度カメラ座標系における三次元点
	bool GetDepthMat(cv::Mat& outDepth16S);
	bool GetDepthMat(cv::Mat& outDepth8UC3, cv::Mat& outDepth16S);
	bool GetDepthMat(cv::Mat& outDepth8UC3, cv::Mat& outDepth16S, cv::Mat& outPoints32FC3);

	// 深度カメラ座標系の点群取得
	bool GetPointsMat(cv::Mat& pointsMat);

	// 現フレームのカラー画像(Mat)取得
	bool GetColorMat(cv::Mat& outColor);
	bool GetColorMat(cv::Mat& outColor, float scale);	// 画像サイズが大きいのでスケール調整可能にした
#endif

private:
	// データ取得するまでの各ポインタ
	IKinectSensor*		pSensor;
	// Depth
	IDepthFrameSource*	pDepthSource;
	IDepthFrameReader*	pDepthReader;
	IDepthFrame*		pDepthFrame;
	// Color
	IColorFrameSource*	pColorSource;
	IColorFrameReader*	pColorReader;
	IColorFrame*		pColorFrame;
	// Mapper
	ICoordinateMapper*	pCoordinateMapper;

	// 各データのオンオフ
	bool isUseDepth, isUseColor;
	

#ifdef OPENCV


#endif
};