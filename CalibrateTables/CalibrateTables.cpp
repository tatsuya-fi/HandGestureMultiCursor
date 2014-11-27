// CalibrateTables.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"


using namespace std;
using namespace cv;

static int mX;
static int mY;
static int mEvent;
static int mFlag;

const static char* windowName = "CalibrateTables";


void onMouse(int event, int x, int y, int flag, void*)
{
	mEvent = event;
	mX = x;
	mY = y;
	mFlag = flag;

	std::string desc;

	//// マウスイベントを取得
	//switch (event) {
	//case cv::EVENT_MOUSEMOVE:
	//	desc += "MOUSE_MOVE";
	//	break;
	//case cv::EVENT_LBUTTONDOWN:
	//	desc += "LBUTTON_DOWN";
	//	break;
	//case cv::EVENT_RBUTTONDOWN:
	//	desc += "RBUTTON_DOWN";
	//	break;
	//case cv::EVENT_MBUTTONDOWN:
	//	desc += "MBUTTON_DOWN";
	//	break;
	//case cv::EVENT_LBUTTONUP:
	//	desc += "LBUTTON_UP";
	//	break;
	//case cv::EVENT_RBUTTONUP:
	//	desc += "RBUTTON_UP";
	//	break;
	//case cv::EVENT_MBUTTONUP:
	//	desc += "MBUTTON_UP";
	//	break;
	//case cv::EVENT_LBUTTONDBLCLK:
	//	desc += "LBUTTON_DBLCLK";
	//	break;
	//case cv::EVENT_RBUTTONDBLCLK:
	//	desc += "RBUTTON_DBLCLK";
	//	break;
	//case cv::EVENT_MBUTTONDBLCLK:
	//	desc += "MBUTTON_DBLCLK";
	//	break;
	//}

	//// マウスボタン，及び修飾キーを取得
	//if (flag & cv::EVENT_FLAG_LBUTTON)
	//	desc += " + LBUTTON";
	//if (flag & cv::EVENT_FLAG_RBUTTON)
	//	desc += " + RBUTTON";
	//if (flag & cv::EVENT_FLAG_MBUTTON)
	//	desc += " + MBUTTON";
	//if (flag & cv::EVENT_FLAG_CTRLKEY)
	//	desc += " + CTRL";
	//if (flag & cv::EVENT_FLAG_SHIFTKEY)
	//	desc += " + SHIFT";
	//if (flag & cv::EVENT_FLAG_ALTKEY)
	//	desc += " + ALT";

	//std::cout << desc << " (" << x << ", " << y << ")" << std::endl;
}

const cv::Mat calcPlaneParam(const vector<Point3f> planePoints)
{
	cout << planePoints.size() << " points were selected" << endl;
	Mat pointsMat = Mat::zeros(planePoints.size(), 3, CV_32F);
	for (size_t i = 0; i < planePoints.size(); ++i)
	{
		*pointsMat.ptr<float>(i, 0) = planePoints[i].x;
		*pointsMat.ptr<float>(i, 1) = planePoints[i].y;
		*pointsMat.ptr<float>(i, 2) = planePoints[i].z;
	}

	Mat ones = Mat::ones(planePoints.size(), 1, CV_32F);

	// 擬似逆行列計算
	Mat planeParam = (pointsMat.t() * pointsMat).inv() * pointsMat.t() * ones;


	// Save data
	const char* filename = "../HandGestureMultiCursor/calibData/TableInfo1.xml";
	cv::FileStorage   cvfs(filename, CV_STORAGE_WRITE);
	cv::WriteStructContext ws(cvfs, "mat_array", CV_NODE_SEQ);    // create node
	cv::write(cvfs, "", planeParam);

	cout << "Parameter was saved" << endl;
	cout << planeParam << endl;

	return planeParam;
}

// 正常に動作するとは限らない
#if 0
const cv::Mat calcPlaneParamRANSAC(const vector<Point3f> planePoints)
{
	if (planePoints.size() <= 0)
	{
		cout << "No table points were selected. Select table points by clicking." << endl;
		return Mat();
	}

	cout << planePoints.size() << " points were selected" << endl;

	Mat planeParam, minPlaneParam;
	const float thresholdRANSAC = 0.001f;
	float error = 100.0f, minError = error;
	srand((unsigned)time(NULL));		// 乱数初期化
	int count = 0;
	while (minError > thresholdRANSAC && count < 1000)
	{
		// ランダムにn点をサンプリング
		const int n = 3;
		Mat A = Mat::zeros(n, 3, CV_32F);
		Mat B = Mat::ones(n, 1, CV_32F);
		for (int i = 0; i < n; ++i)
		{
			int randID = rand() % planePoints.size();
			*A.ptr<float>(i, 0) = planePoints[randID].x;
			*A.ptr<float>(i, 1) = planePoints[randID].y;
			*A.ptr<float>(i, 2) = planePoints[randID].z;
		}
		// 擬似逆行列を用いてパラメータ推定
		planeParam = (A.t() * A).inv() * A.t() * B;

		error = 0;
		for (size_t i = 0; i < planePoints.size(); ++i)
		{
			error += abs(planePoints[i].x * *planeParam.ptr<float>(0, 0) + planePoints[i].y * *planeParam.ptr<float>(1, 0) + planePoints[i].z * *planeParam.ptr<float>(2, 0) - 1);
		}
		error /= (planePoints.size() - n);

		if (0 >= error && count == 0) { continue; }

		if (0 < error && (error < minError || count == 0))
		{
			minError = error;
			minPlaneParam = planeParam;
		}
		++count;
	}
	cout << "Reprojection: " << minError << endl;

	vector<Point3f> planePointsNoNoise;
	// 誤差の少ない点のみを保存
	for (size_t i = 0; i < planePoints.size(); ++i)
	{
		error = abs(planePoints[i].x * *planeParam.ptr<float>(0, 0) + planePoints[i].y * *planeParam.ptr<float>(1, 0) + planePoints[i].z * *planeParam.ptr<float>(2, 0) - 1);
		if (error < 0.1f)
		{
			planePointsNoNoise.push_back(planePoints[i]);
		}
	}
	cout << planePointsNoNoise.size() << endl;
	cout << planePoints.size() - planePointsNoNoise.size() << "points ejected" << endl;

	// 再度RANSAC
	error = 100.0f;
	minError = error;
	while (minError > thresholdRANSAC && count < 1000)
	{
		// ランダムにn点をサンプリング
		const int n = 3;
		Mat A = Mat::zeros(n, 3, CV_32F);
		Mat B = Mat::ones(n, 1, CV_32F);
		for (int i = 0; i < n; ++i)
		{
			int randID = rand() % planePointsNoNoise.size();
			*A.ptr<float>(i, 0) = planePointsNoNoise[randID].x;
			*A.ptr<float>(i, 1) = planePointsNoNoise[randID].y;
			*A.ptr<float>(i, 2) = planePointsNoNoise[randID].z;
		}
		// 擬似逆行列を用いてパラメータ推定
		planeParam = (A.t() * A).inv() * A.t() * B;

		error = 0;
		for (size_t i = 0; i < planePointsNoNoise.size(); ++i)
		{
			error += abs(planePointsNoNoise[i].x * *planeParam.ptr<float>(0, 0) + planePointsNoNoise[i].y * *planeParam.ptr<float>(1, 0) + planePointsNoNoise[i].z * *planeParam.ptr<float>(2, 0) - 1);
		}
		error /= (planePointsNoNoise.size() - n);

		if (0 >= error && count == 0) { continue; }

		if (0 < error && (error < minError || count == 0))
		{
			minError = error;
			minPlaneParam = planeParam;
		}
		++count;
	}


	cout << "Reprojection error: " << error << endl;

	// Save data
	const char* filename = "savedData/plane.xml";
	cv::FileStorage   cvfs(filename, CV_STORAGE_WRITE);
	cv::WriteStructContext ws(cvfs, "mat_array", CV_NODE_SEQ);    // create node
	cv::write(cvfs, "", minPlaneParam);

	cout << "Parameter was saved" << endl;
	cout << minPlaneParam << endl;

	return minPlaneParam;
}

const cv::Mat calcPlaneParamSVD(const vector<Point3f> planePoints)
{
	cout << planePoints.size() << " points were selected" << endl;

	Mat planeParam, minPlaneParam;
	const float thresholdRANSAC = 0.001f;
	float error = 100.0f, minError = error;
	srand((unsigned)time(NULL));		// 乱数初期化
	int count = 0;
	while (minError > thresholdRANSAC && count < 1000)
	{
		// ランダムにn点をサンプリング
		const int n = 5;
		vector<int> idList;
		Mat A = Mat::ones(n, 4, CV_32F);
		for (int i = 0; i < n; ++i)
		{
			int randID = rand() % planePoints.size();
			idList.push_back(randID);
			*A.ptr<float>(i, 0) = planePoints[randID].x;
			*A.ptr<float>(i, 1) = planePoints[randID].y;
			*A.ptr<float>(i, 2) = planePoints[randID].z;
		}
		// SVDを用いてパラメータ推定
		Mat u, vt;
		SVD::compute(A, planeParam, u, vt);
		//cout << planeParam << endl;
		error = 0;
		for (size_t i = 0; i < planePoints.size(); ++i)
		{
			bool isSelectedPoint = false;
			for (size_t id = 0; id < idList.size(); ++id)
			{
				if (idList[id] == i) { isSelectedPoint = true; }
			}
			if (!isSelectedPoint)
			{
				error += abs(
					planePoints[i].x * *planeParam.ptr<float>(0, 0)
					+ planePoints[i].y * *planeParam.ptr<float>(1, 0)
					+ planePoints[i].z * *planeParam.ptr<float>(2, 0)
					+ *planeParam.ptr<float>(3, 0)
					);
			}
		}
		error /= (planePoints.size() - n);
		//cout << error << endl;


		if (0 >= error && count == 0) { continue; }

		if (0 < error && (error < minError || count == 0))
		{
			minError = error;
			minPlaneParam = planeParam.clone();
			cout << minError << endl;
			cout << minPlaneParam << endl;
		}
		++count;
	}
	cout << "Reprojection: " << minError << endl;

	//vector<Point3f> planePointsNoNoise;
	//// 誤差の少ない点のみを保存
	//for (size_t i = 0; i < planePoints.size(); ++i)
	//{
	//	error = abs(planePoints[i].x * *planeParam.ptr<float>(0, 0) + planePoints[i].y * *planeParam.ptr<float>(1, 0) + planePoints[i].z * *planeParam.ptr<float>(2, 0) - 1);
	//	if (error < 0.1f)
	//	{
	//		planePointsNoNoise.push_back(planePoints[i]);
	//	}
	//}
	//cout << planePointsNoNoise.size() << endl;
	//cout << planePoints.size() - planePointsNoNoise.size() << "points ejected" << endl;

	//// 再度RANSAC
	//error = 100.0f;
	//minError = error;
	//while (minError > thresholdRANSAC && count < 1000)
	//{
	//	// ランダムにn点をサンプリング
	//	const int n = 3;
	//	Mat A = Mat::zeros(n, 3, CV_32F);
	//	Mat B = Mat::ones(n, 1, CV_32F);
	//	for (int i = 0; i < n; ++i)
	//	{
	//		int randID = rand() % planePointsNoNoise.size();
	//		*A.ptr<float>(i, 0) = planePointsNoNoise[randID].x;
	//		*A.ptr<float>(i, 1) = planePointsNoNoise[randID].y;
	//		*A.ptr<float>(i, 2) = planePointsNoNoise[randID].z;
	//	}
	//	// 擬似逆行列を用いてパラメータ推定
	//	planeParam = (A.t() * A).inv() * A.t() * B;

	//	error = 0;
	//	for (size_t i = 0; i < planePointsNoNoise.size(); ++i)
	//	{
	//		error += abs(planePointsNoNoise[i].x * *planeParam.ptr<float>(0, 0) + planePointsNoNoise[i].y * *planeParam.ptr<float>(1, 0) + planePointsNoNoise[i].z * *planeParam.ptr<float>(2, 0) - 1);
	//	}
	//	error /= (planePointsNoNoise.size() - n);

	//	if (0 >= error && count == 0) { continue; }

	//	if (0 < error && (error < minError || count == 0))
	//	{
	//		minError = error;
	//		minPlaneParam = planeParam;
	//	}
	//	++count;
	//}

	//Mat pointsMat = Mat::ones(planePoints.size(), 4, CV_32F);
	//for (size_t i = 0; i < planePoints.size(); ++i)
	//{
	//	*pointsMat.ptr<float>(i, 0) = planePoints[i].x;
	//	*pointsMat.ptr<float>(i, 1) = planePoints[i].y;
	//	*pointsMat.ptr<float>(i, 2) = planePoints[i].z;
	//}

	//Mat w, u, vt;
	//SVD::compute(pointsMat, w, u, vt);

	//cout << w << endl;

	//// 擬似逆行列計算
	//Mat planeParam = (pointsMat.t() * pointsMat).inv() * pointsMat.t() * ones;

	//// Save data
	//const char* filename = "savedData/plane.xml";
	//cv::FileStorage   cvfs(filename, CV_STORAGE_WRITE);
	//cv::WriteStructContext ws(cvfs, "mat_array", CV_NODE_SEQ);    // create node
	//cv::write(cvfs, "", planeParam);

	//cout << "Parameter was saved" << endl;
	//cout << planeParam << endl;

	return minPlaneParam;
}
#endif

void drawPlane(cv::Mat& showImgColor, cv::Mat& planeParam, cv::Mat& pointCloud)
{
	// 描画する平面からの距離のしきい値[m]
	const float thresh = 0.03;
	for (int y = 0; y < showImgColor.rows; ++y)
	{
		for (int x = 0; x < showImgColor.cols; ++x)
		{
			float distance = abs(
				*planeParam.ptr<float>(0, 0) * pointCloud.ptr<float>(y, x)[0]
				+ *planeParam.ptr<float>(1, 0) * pointCloud.ptr<float>(y, x)[1]
				+ *planeParam.ptr<float>(2, 0) * pointCloud.ptr<float>(y, x)[2] - 1)
				/ sqrt(*planeParam.ptr<float>(0, 0) * *planeParam.ptr<float>(0, 0)
				+ *planeParam.ptr<float>(1, 0) * *planeParam.ptr<float>(1, 0)
				+ *planeParam.ptr<float>(2, 0) * *planeParam.ptr<float>(2, 0));
			if (0 < distance && distance < thresh)
			{
				//cout << distance << endl;
				showImgColor.at<Vec3b>(y, x)[0] = 0;
				showImgColor.at<Vec3b>(y, x)[1] = 255;
				showImgColor.at<Vec3b>(y, x)[2] = 0;
			}
		}
	}
	cout << "Draw plane" << endl;
}

void calibTable(const KinectV2Basics kinect, cv::Mat& showImg, cv::Mat& pointCloud)
{
	if (pointCloud.empty() || pointCloud.channels() != 3)
	{
		cout << pointCloud.channels() << endl;
		cout << "Error: calibTable(): Input Mat is not correct." << endl;
		return;
	}

	Mat showImgColor = Mat::zeros(showImg.rows, showImg.cols, CV_8UC3);
	for (int y = 0; y < showImg.rows; ++y)
	{
		for (int x = 0; x < showImg.cols; ++x)
		{
			showImgColor.at<Vec3b>(y, x)[0] = showImg.at<unsigned char>(y, x);
			showImgColor.at<Vec3b>(y, x)[1] = showImg.at<unsigned char>(y, x);
			showImgColor.at<Vec3b>(y, x)[2] = showImg.at<unsigned char>(y, x);
		}
	}

	cout << "Click table area and press c to calcurate plane parameters." << endl;
	vector<Point3f> tablePoints;
	const int offset = 5;	// [pixel]
	while (1)
	{
		// <TODO>
		// このゲットしてきた座標ほんとにあってる？確認
		if (mFlag & cv::EVENT_FLAG_LBUTTON)
		{
			//for (int y = mY - offset; y < mY + offset; ++y)
			for (int y = mY; y < mY + 1; ++y)
			{
				if (y < 0) { continue; }
				else if (showImg.rows < y) { break; }
				//for (int x = mX - offset; x < mX + offset; ++x)
				for (int x = mX; x < mX + 1; ++x)
				{
					if (x < 0) { continue; }
					else if (showImg.cols < x) { break; }
					if (showImgColor.ptr<unsigned char>(y, x)[0] == 0
						&& showImgColor.ptr<unsigned char>(y, x)[1] == 0
						&& showImgColor.ptr<unsigned char>(y, x)[2] == 255
						|| pointCloud.ptr<float>(y, x)[2] > 1000 || pointCloud.ptr<float>(y, x)[2] < 0.05)
					{
						continue;
					}
					// 色変える
					showImgColor.ptr<unsigned char>(y, x)[0] = 0;
					showImgColor.ptr<unsigned char>(y, x)[1] = 0;
					showImgColor.ptr<unsigned char>(y, x)[2] = 255;

					// [mm]
					Point3f addPoint;
					addPoint.x = pointCloud.ptr<float>(y, x)[0];
					addPoint.y = pointCloud.ptr<float>(y, x)[1];
					addPoint.z = pointCloud.ptr<float>(y, x)[2];

					tablePoints.push_back(addPoint);
				}
			}
			imshow(windowName, showImgColor);
		}

		switch (waitKey(20))
		{
		case 'c':
		case 'C':
		{
			Mat planeParam = calcPlaneParam(tablePoints);
			cout << planeParam << endl;
			if (planeParam.empty()) { continue; }
			drawPlane(showImgColor, planeParam, pointCloud);
			imshow(windowName, showImgColor);
			tablePoints.clear();
			showImgColor = Mat::zeros(showImg.rows, showImg.cols, CV_8UC3);
			for (int y = 0; y < showImg.rows; ++y)
			{
				for (int x = 0; x < showImg.cols; ++x)
				{
					showImgColor.at<Vec3b>(y, x)[0] = showImg.at<unsigned char>(y, x);
					showImgColor.at<Vec3b>(y, x)[1] = showImg.at<unsigned char>(y, x);
					showImgColor.at<Vec3b>(y, x)[2] = showImg.at<unsigned char>(y, x);
				}
			}
		}
			break;
		case KEY_ESC:
			return;
		default:
			break;
		}
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	KinectV2Basics kinectV2Basics;

	// 使用するカメラの設定
	kinectV2Basics.SelectUsingData(true, false);
	// 初期化
	kinectV2Basics.SetupKinectV2();

	// ウィンドウの作成
	namedWindow(windowName, WINDOW_AUTOSIZE);
	// マウス用コールバック関数の設定
	setMouseCallback(windowName, onMouse, 0);

	Mat a = (Mat_<double>(3, 3) <<
		101, 0, -5,
		0, 10, -99 / 20,
		11, 5, -3
		);
	Mat b = Mat::ones(3, 1, CV_64F);

	Mat c1 = a.inv() * b;
	Mat c2 = (a.t() * a).inv() * a.t() * b;
	cout << c1 << endl;
	cout << c2 << endl;
	cout << a.at<double>(0, 0) * c1.at<double>(0, 0) + a.at<double>(0, 1) * c1.at<double>(1, 0) + a.at<double>(0, 2) * c1.at<double>(2, 0) << endl;
	cout << a.at<double>(1, 0) * c1.at<double>(0, 0) + a.at<double>(1, 1) * c1.at<double>(1, 0) + a.at<double>(1, 2) * c1.at<double>(2, 0) << endl;
	cout << a.at<double>(2, 0) * c1.at<double>(0, 0) + a.at<double>(2, 1) * c1.at<double>(1, 0) + a.at<double>(2, 2) * c1.at<double>(2, 0) << endl;

	// 机の情報を取得するフレームを決定する
	cout << "Press Space key for deciding frame." << endl;
	while (1)
	{
		Mat showImg, depthMat, pointCloud;
		kinectV2Basics.GetDepthMat(showImg, depthMat, pointCloud);
		if (!showImg.empty())
		{
			imshow(windowName, showImg);
		}

		switch (waitKey(20))
		{
		case ' ':
			calibTable(kinectV2Basics, showImg, pointCloud);
			return 0;
			break;
		case KEY_ESC:
			return 0;
			break;
		default:
			break;
		}
	}

	return 0;
}
