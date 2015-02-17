#include "stdafx.h"
#include "KinectV2Basics.h"

using namespace std;
using namespace cv;

KinectV2Basics::KinectV2Basics()
{
}


KinectV2Basics::~KinectV2Basics()
{
	// 各ポインタを開放
	//SafeRelease(pDepthSource);
	//SafeRelease(pDepthReader);
	//SafeRelease(pDepthFrame);
	SafeRelease(pCoordinateMapper);

	if (pSensor)
	{
		pSensor->Close();
	}
	SafeRelease(pSensor);
}

void KinectV2Basics::SelectUsingData(const bool isUseDepthStream, const bool isUseColorStream)
{
	isUseDepth = isUseDepthStream;
	isUseColor = isUseColorStream;
}

bool KinectV2Basics::SetupKinectV2()
{
	HRESULT hResult = S_OK;
	// Sensor
	hResult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hResult))
	{
		cerr << "Error : GetDefaultKinectSensor" << endl;
		return false;
	}
	hResult = pSensor->Open();
	if (FAILED(hResult))
	{
		cerr << "Error : IKinectSensor::Open()" << endl;
		return false;
	}

	if (isUseDepth)
	{
		// Source(Depth)
		hResult = pSensor->get_DepthFrameSource(&pDepthSource);
		if (FAILED(hResult))
		{
			cerr << "Error : IKinectSensor::get_DepthFrameSource()" << endl;
			return false;
		}

		// Reader(Depth)
		hResult = pDepthSource->OpenReader(&pDepthReader);
		if (FAILED(hResult))
		{
			cerr << "Error : IDepthFrameSource::OpenReader()" << endl;
			return false;
		}
	}

	if (isUseColor)
	{
		// Source(Color)
		hResult = pSensor->get_ColorFrameSource(&pColorSource);
		if (FAILED(hResult))
		{
			cerr << "Error : IKinectSensor::get_ColorFrameSource()" << endl;
			return false;
		}

		// Reader(Color)
		hResult = pColorSource->OpenReader(&pColorReader);
		if (FAILED(hResult))
		{
			cerr << "Error : IColorFrameSource::OpenReader()" << endl;
			return false;
		}
	}

	// Source(Infrared)
	hResult = pSensor->get_InfraredFrameSource(&pInfraredSource);
	if (FAILED(hResult))
	{
		cerr << "Error : IKinectSensor::get_ColorFrameSource()" << endl;
		return false;
	}

	// Reader(Infrared)
	hResult = pInfraredSource->OpenReader(&pInfraredReader);
	if (FAILED(hResult))
	{
		cerr << "Error : IColorFrameSource::OpenReader()" << endl;
		return false;
	}

	// バッファーを作成する
	infraredBuffer.resize(widthInfrared * heightInfrared);

	// Mapper
	hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
	if (FAILED(hResult))
	{
		cerr << "Error : IKinectSensor::get_CoordinateMapper()" << endl;
		return false;
	}

	return true;
}

//--------------------------------------------------
// Getter

IKinectSensor* KinectV2Basics::GetSensor()
{
	return pSensor;
}

IDepthFrameSource* KinectV2Basics::GetSourceDepth()
{
	return pDepthSource;
}

IDepthFrameReader* KinectV2Basics::GetReaderDepth()
{
	return pDepthReader;
}

IDepthFrame* KinectV2Basics::GetFrameDepth()
{
	return pDepthFrame;
}

IColorFrameSource* KinectV2Basics::GetSourceColor()
{
	return pColorSource;
}

IColorFrameReader* KinectV2Basics::GetReaderColor()
{
	return pColorReader;
}

IColorFrame* KinectV2Basics::GetFrameColor()
{
	return pColorFrame;
}

ICoordinateMapper* KinectV2Basics::GetMapper()
{
	return pCoordinateMapper;
}

//--------------------------------------------------
// Methods which use OpenCV

#ifdef OPENCV

bool KinectV2Basics::GetDepthMat(cv::Mat& outDepth8U, cv::Mat& outDepth16U, cv::Mat& outPoints32FC3)
{
	if (GetDepthMat(outDepth8U, outDepth16U))
	{
		// Init
		outPoints32FC3 = Mat::zeros(heightDepth, widthDepth, CV_32FC3);
		// 距離情報取得 (pDepthBuffer)
		unsigned int bufferSize = widthDepth * heightDepth * sizeof(unsigned short);
		UINT16* pDepthBuffer = NULL;
		HRESULT hResult = pDepthFrame->AccessUnderlyingBuffer(&bufferSize, &pDepthBuffer);
		if (FAILED(hResult))
		{
			cout << "Error: AccessUnderlyingBuffer(). You need to call GetDepthMat() first" << endl;
			return false;
		}

		// Get 3D points in depth camera coordinate
		UINT pointNum = widthDepth * heightDepth;
		CameraSpacePoint* cameraSpacePoint = new CameraSpacePoint[widthDepth * heightDepth];	// Points in camera coordinate [m]
		hResult = pCoordinateMapper->MapDepthFrameToCameraSpace(pointNum, pDepthBuffer, pointNum, cameraSpacePoint);
		if (FAILED(hResult))
		{
			cout << "Error: MapDepthFrameToCameraSpace()" << endl;
			return false;
		}
		for (int y = 0; y < heightDepth; ++y)
		{
			for (int x = 0; x < widthDepth; ++x)
			{
				int index = y * widthDepth + x;
				outPoints32FC3.ptr<float>(y, x)[0] = cameraSpacePoint[index].X;
				outPoints32FC3.ptr<float>(y, x)[1] = cameraSpacePoint[index].Y;
				outPoints32FC3.ptr<float>(y, x)[2] = cameraSpacePoint[index].Z;
			}
		}
		delete[] cameraSpacePoint;	// メモリ開放必要

		return true;
	}
	else
	{
		return false;
	}
}

bool KinectV2Basics::GetDepthMat(Mat& outDepth8U, Mat& outDepth16S)
{
	if (GetDepthMat(outDepth16S))
	{
		outDepth16S.convertTo(outDepth8U, CV_8U, -225.0f / 4500.0f, 255.0f);
		return true;
	}
	else
	{
		return false;
	}
}

bool KinectV2Basics::GetDepthMat(Mat& outDepth16S)
{
	unsigned int bufferSize = widthDepth * heightDepth * sizeof(unsigned short);
	Mat bufferMat(heightDepth, widthDepth, CV_16SC1);

	HRESULT hResult = S_OK;
	SafeRelease(pDepthFrame);
	hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
	if (SUCCEEDED(hResult))
	{
		hResult = pDepthFrame->AccessUnderlyingBuffer(&bufferSize, reinterpret_cast<UINT16**>(&bufferMat.data));
		// y軸に対して反転...しない！
		Mat bufferMatFlip = bufferMat.clone();
		//flip(bufferMat, bufferMatFlip, 1);
		if (SUCCEEDED(hResult))
		{
			// Z軸距離を格納
			outDepth16S = bufferMatFlip;
		}
		else
		{
			//cerr << "Error : IDepthFrameSource::AccessUnderlyingBuffer()" << endl;
			return false;
		}
	}
	else
	{
		//cerr << "Error : IDepthFrame::AcquireLatestFrame()" << endl;
		return false;
	}

	return true;
}

bool KinectV2Basics::GetPointsMat(Mat& pointsMat)
{
	// Init
	pointsMat = Mat::zeros(heightDepth, widthDepth, CV_32FC3);

	// 距離情報取得 (pDepthBuffer)
	unsigned int bufferSize = widthDepth * heightDepth * sizeof(unsigned short);
	UINT16* pDepthBuffer = NULL;
	HRESULT hResult = pDepthFrame->AccessUnderlyingBuffer(&bufferSize, &pDepthBuffer);
	if (FAILED(hResult))
	{
		cout << "Error: AccessUnderlyingBuffer(). You need to call GetDepthMat() first" << endl;
		return false;
	}

	// 深度カメラ座標系での三次元座標を求める
	UINT pointNum = widthDepth * heightDepth;
	CameraSpacePoint* cameraSpacePoint = new CameraSpacePoint[widthDepth * heightDepth];	// Points in camera coordinate [m]
	hResult = pCoordinateMapper->MapDepthFrameToCameraSpace(pointNum, pDepthBuffer, pointNum, cameraSpacePoint);
	if (FAILED(hResult))
	{
		cout << "Error: MapDepthFrameToCameraSpace()" << endl;
		return false;
	}
	for (int y = 0; y < heightDepth; ++y)
	{
		for (int x = 0; x < widthDepth; ++x)
		{
			int index = y * widthDepth + x;
			pointsMat.ptr<float>(y, x)[0] = cameraSpacePoint[index].X;
			pointsMat.ptr<float>(y, x)[1] = cameraSpacePoint[index].Y;
			pointsMat.ptr<float>(y, x)[2] = cameraSpacePoint[index].Z;
		}
	}
	delete[] cameraSpacePoint;	// メモリ開放必要

	return true;
}

bool KinectV2Basics::GetColorMat(cv::Mat& outColor)
{
	unsigned int bufferSize = widthColor * heightColor * 4 * sizeof(unsigned char);
	Mat colorMat(heightColor, widthColor, CV_8UC4);

	HRESULT hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
	if (SUCCEEDED(hResult))
	{
		hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(colorMat.data), ColorImageFormat_Bgra);
		Mat colorMat2 = colorMat;
		// y軸に対して反転...しない！
		//flip(colorMat, colorMat2, 1);
		if (SUCCEEDED(hResult))
		{
			// カラー画像を格納
			outColor = colorMat2;

			SafeRelease(pColorFrame);
			return true;

		}
		else
		{
			//cerr << "Error: IColorFrame::CopyConvertedFrameDataToArray()" << endl;
			return false;
		}
	}
	else
	{
		//cerr << "Error: IColorFrameReader::AcquireLatestFrame()" << endl;
		return false;
	}
}

bool KinectV2Basics::GetColorMat(cv::Mat& outColor, float scale)
{
	if (GetColorMat(outColor))
	{
		resize(outColor, outColor, Size(), scale, scale);
		return true;
	}
	else
	{
		return false;
	}
}

bool KinectV2Basics::GetInfraredMat(cv::Mat& outInfrared)
{
	unsigned int bufferSize = widthInfrared * heightInfrared * 4 * sizeof(unsigned char);
	Mat outInfraredBuf = Mat::zeros(heightInfrared, widthInfrared, CV_8UC4);
	Mat bufferMat(heightDepth, widthDepth, CV_16SC1);

	UINT nBufferSize = 0;
	UINT16 *pBuffer = NULL;

	HRESULT hResult = pInfraredReader->AcquireLatestFrame(&pInfraredFrame);
	if (SUCCEEDED(hResult))
	{
		hResult = pInfraredFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		//hResult = pInfraredFrame->CopyFrameDataToArray(infraredBuffer.size(), reinterpret_cast<UINT16*>(bufferMat.data));
		//Mat colorMat2 = colorMat;
		// y軸に対して反転...しない！
		//flip(colorMat, colorMat2, 1);
		if (SUCCEEDED(hResult))
		{
			for (int y = 0; y < heightInfrared; ++y)
			{
				for (int x = 0; x < widthInfrared; ++x)
				{
					// normalize the incoming infrared data (ushort) to a float ranging from 
					// [InfraredOutputValueMinimum, InfraredOutputValueMaximum] by
					// 1. dividing the incoming value by the source maximum value
					float intensityRatio = static_cast<float>(*pBuffer) / InfraredSourceValueMaximum;

					// 2. dividing by the (average scene value * standard deviations)
					intensityRatio /= InfraredSceneValueAverage * InfraredSceneStandardDeviations;

					// 3. limiting the value to InfraredOutputValueMaximum
					intensityRatio = min(InfraredOutputValueMaximum, intensityRatio);

					// 4. limiting the lower value InfraredOutputValueMinimym
					intensityRatio = max(InfraredOutputValueMinimum, intensityRatio);

					// 5. converting the normalized value to a byte and using the result
					// as the RGB components required by the image
					byte intensity = static_cast<byte>(intensityRatio * 255.0f);

					int index = (y * widthInfrared + x) * 4;
					UCHAR* dataInfrared = &outInfraredBuf.data[index];
					dataInfrared[0] = intensity;
					dataInfrared[1] = intensity;
					dataInfrared[2] = intensity;
					dataInfrared[3] = 1;

					++pBuffer;
				}
			}

			//flip(outInfrared, outInfrared, 1);
			outInfrared = outInfraredBuf.clone();

			SafeRelease(pInfraredFrame);
			return true;

		}
		else
		{
			//cerr << "Error: IColorFrame::CopyConvertedFrameDataToArray()" << endl;
			return false;
		}
	}
	else
	{
		//cerr << "Error: IColorFrameReader::AcquireLatestFrame()" << endl;
		return false;
	}

}

#endif
