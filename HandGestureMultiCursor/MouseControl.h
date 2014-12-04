#pragma once
class MouseControl
{
public:
	MouseControl();
	~MouseControl();

	enum
	{
		ARM_COORDINATE,		// 腕方向をy座標においた座標系
		TABLE_COORDINATE	// テーブルの向きを基準においた座標系[未実装]
	};

	// 相対操作の手法三種
	// 距離指定(マウス)
	void moveCursorDistance(hgmc::UserData& userData, const cv::Mat& tableParam);
	// 時間指定(レバー)
	void moveCursorTime(hgmc::UserData userData, const cv::Mat& tableParam);
	// ハイブリッド(傾き考慮したレバー)
	void moveCursorDistanceTime(hgmc::UserData userData, const cv::Mat& tableParam);

private:
	// 手と机との接触判定のためのしきい値
	const float threshold = 0.048;		// [m]

	// ドラッグ開始位置
	//POINT sPt;
	cv::Point3f sPtCamera;

	//// 現在のマウスの相対移動量
	//cv:: Point2i cursorMove;

	//// ドラッグ中か否か(右手)
	//bool isDragging;

	//// クリック中か否か(左手)
	//bool isClicking;

	// 座標系の定義
	unsigned int coordinateMode;

	//// 机平面への座標変換行列
	//cv::Mat TKinect2Table;


	// 関数定義
	void checkDragging(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const cv::Mat& tableParam);
	DWORD checkClicking(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const cv::Mat& tableParam);
	void MoveMouse(const hgmc::CursorInfo& cursorInfo, DWORD mouseFlag);
};

