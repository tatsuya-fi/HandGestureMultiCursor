#pragma once
class MouseControl
{
public:
	MouseControl();
	~MouseControl();

	enum
	{
		ARM_COORDINATE,		// 腕方向をy座標においた座標系
	};

	void moveCursorDistance(hgmc::UserData userData, const cv::Mat& tableParam);
	void moveCursorTime(hgmc::UserData userData, const cv::Mat& tableParam);
	void moveCursorDistanceTime(hgmc::UserData userData, const cv::Mat& tableParam);
private:
	// ドラッグ開始位置
	POINT sPt;
	cv::Point3f sPtCamera;

	// ドラッグ中か否か
	bool isDragging;

	// 座標系の定義
	unsigned int coordinateMode;

	// 机平面への座標変換行列
	cv::Mat TKinect2Table;

	void checkCursor(hgmc::UserData userData, const cv::Mat& tableParam);
	void MoveMouse(UINT nX, UINT nY);
};

