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

	// �e�T�C�Y
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
	// ���t���[���̐[�x�摜(imshow()�\���p)�ƁC�s�N�Z�����Ƃ�Z���������i�[�����s��(imshow()�s��)���擾
	bool GetDepthMat(cv::Mat& outDepth8U, cv::Mat& outDepth16S, cv::Mat& outPoints32FC3);
	bool GetDepthMat(cv::Mat& outDepth8U, cv::Mat& outDepth16S);
	bool GetDepthMat(cv::Mat& outDepth16S);

	// �[�x�J�������W�n�̓_�Q�擾
	bool GetPointsMat(cv::Mat& pointsMat);

	// ���t���[���̃J���[�摜(Mat)�擾
	bool GetColorMat(cv::Mat& outColor);
	bool GetColorMat(cv::Mat& outColor, float scale);	// �摜�T�C�Y���傫���̂ŃX�P�[�������\�ɂ���

	// ���t���[���̐ԊO���摜(Mat)�擾
	bool GetInfraredMat(cv::Mat& outInfrared);
#endif

private:
	// �f�[�^�擾����܂ł̊e�|�C���^
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