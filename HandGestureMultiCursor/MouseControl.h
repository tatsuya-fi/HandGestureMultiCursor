#pragma once
namespace mc
{
	enum
	{
		ARM_COORDINATE,		// 腕方向をy座標においた座標系
		TABLE_COORDINATE	// テーブルの向きを基準においた座標系[未実装]
	};

	// 机の上で手を動かすことによるカーソル移動に使用する手法
	enum
	{
		DISTANCE_BASED_METHOD,
		TIME_BASED_METHOD,
		DISTANCE_TIME_BASED_METHOD,
		RELATIVE_MODE_DISABLE
	};

	// 指差しによるカーソル移動のon/off
	enum
	{
		POINTING_MODE_ON,
		POINTING_MODE_OFF
	};

	// クリック機能のon/off
	enum
	{
		CLICK_ENABLE,
		CLICK_DISABLE
	};
}


class MouseControl
{
public:
	MouseControl();
	~MouseControl();


	// 相対操作の手法三種
	// 距離指定(マウス)
	void moveCursorDistance(hgmc::UserData& userData, const cv::Mat& tableParam, const cv::Mat& tableCorners);
	// 時間指定(レバー)
	void moveCursorTime(hgmc::UserData& userData, const cv::Mat& tableParam, const cv::Mat& tableCorners);
	// ハイブリッド(傾き考慮したレバー)
	void moveCursorTimeDistance(hgmc::UserData& userData, const cv::Mat& tableParam, const cv::Mat& tableCorners);

	int toggleClickMode();

private:
	// 手と机との接触判定のためのしきい値
	const float thresholdDrag = 0.04;		// [m]
	const float thresholdClick = 0.045;		// [m]

	// threshcursorMoveDistより大きく動いたらカーソル位置更新（ぷるぷる防止）
	const int threshCursorMoveDist = 6;			// [pix]
	// threshcursorMoveTimeより大きく動いたらカーソル位置更新（ぷるぷる防止）
	const float threshCursorMoveTime = 0.02;	    // [m]

	// ドラッグ開始位置
	//POINT sPt;
	cv::Point3f sPtCamera;

	// 座標系の定義
	unsigned int coordinateMode;

	// 座標系更新をチェックするタイマー
	cv::TickMeter timerCoordinate;
	// 机上に手があった場合の座標系を更新する頻度[s]
	const float updateCoordinateRate = 5;

	// カーソル位置を取る際，前回の何フレームとの平均求めるか
	const int preCursorMoveNumMax = 5;
	// 前フレームのカーソル位置情報格納用
	std::list<cv::Point2i> preCursorMove;

	// クリック有効か無効かのフラグ
	int clickMode = mc::CLICK_ENABLE;

	// 関数定義
	void resetTransMatrix(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const cv::Mat& tableParam);
	void checkDraggingForDistance(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const cv::Mat& tableParam, const cv::Mat& tableCorners);
	void checkDraggingForTime(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const cv::Mat& tableParam, const cv::Mat& tableCorners);
	DWORD checkClicking(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const cv::Mat& tableParam, const cv::Mat& tableCorners);
	void MoveMouseAbsolute(const hgmc::CursorInfo& cursorInfo, DWORD mouseFlag);
	void MoveMouseRelative(const hgmc::CursorInfo& cursorInfo, DWORD mouseFlag);


	// 外積を使って手が机内か外どちらにあるか計算する
	// http://www.sousakuba.com/Programming/gs_hittest_point_triangle.html
	inline bool isHandInTableArea(const cv::Point2i hand, const cv::Mat& tableCorners)
	{
		// 机を三角形2つに分割して考える
		// Z成分だけチェック
		double c1, c2, c3;
		bool isInTable[2];
		for (int i = 0; i < 2; ++i)
		{
			int ii = 2 * i;
			c1 = (*tableCorners.ptr<int>(ii+1, 0) - *tableCorners.ptr<int>(ii, 0)) * (hand.y - *tableCorners.ptr<int>(ii+1, 1))
				- (*tableCorners.ptr<int>(ii+1, 1) - *tableCorners.ptr<int>(ii, 1)) * (hand.x - *tableCorners.ptr<int>(ii+1, 0));
			c2 = (*tableCorners.ptr<int>((ii+2)%4, 0) - *tableCorners.ptr<int>(ii+1, 0)) * (hand.y - *tableCorners.ptr<int>((ii+2)%4, 1))
				- (*tableCorners.ptr<int>((ii+2)%4, 1) - *tableCorners.ptr<int>(ii+1, 1)) * (hand.x - *tableCorners.ptr<int>((ii+2)%4, 0));
			c3 = (*tableCorners.ptr<int>(ii, 0) - *tableCorners.ptr<int>((ii+2)%4, 0)) * (hand.y - *tableCorners.ptr<int>(ii, 1))
				- (*tableCorners.ptr<int>(ii, 1) - *tableCorners.ptr<int>((ii+2)%4, 1)) * (hand.x - *tableCorners.ptr<int>(ii, 0));
			// 外積がすべて同じ方向を向いていれば三角形内にある
			isInTable[i] = (c1 >= 0 && c2 >= 0 && c3 >= 0) || (c1 <= 0 && c2 <= 0 && c3 <= 0);
		}

		return (isInTable[0] || isInTable[1]);
	}

};

