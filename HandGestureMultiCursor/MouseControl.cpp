#include "stdafx.h"
#include "MouseControl.h"

using namespace std;
using namespace cv;
using namespace hgmc;
using namespace mc;

MouseControl::MouseControl() :
//isDragging(false),
//isClicking(false),
coordinateMode(ARM_COORDINATE)
{
	timerCoordinate.reset();
}

MouseControl::~MouseControl()
{
}

// 手法１
void MouseControl::moveCursorDistance(hgmc::UserData& userData, const Mat& tableParam, const cv::Mat& tableCorners)
{
	///
	/// <Param>
	const float scale = 3000;	// 腕の動きを[m] -> [pixel]へ変換するスケール
	/// </Param>
	///


	// カーソル移動用
	HandInfo& handMove = userData.handInfoR;
	// クリック用
	HandInfo& handClick = userData.handInfoL;

	// マウスが移動中かチェック
	checkDraggingForDistance(userData.cursorInfo, handMove, tableParam, tableCorners);

	// クリック状態のチェック
	DWORD mouseEvent = checkClicking(userData.cursorInfo, handClick, tableParam, tableCorners);



	// 腕を使ってカーソルを動かす
	if (userData.cursorInfo.isDragging)
	{
		// 机平面における始点からの変位を求める
		Mat move = (Mat_<float>(4, 1) <<
			handMove.cameraPoint.x,
			handMove.cameraPoint.y,
			handMove.cameraPoint.z,
			1);
		//cout << userData.cursorInfo.preTKinect2Table << endl;
#if 1
		if (!userData.cursorInfo.preTKinect2Table.empty())
		{
			Mat comp = userData.cursorInfo.TKinect2Table == userData.cursorInfo.preTKinect2Table;
			if (*comp.ptr<float>(0,0) == 0)
			{
				move = userData.cursorInfo.preTKinect2Table * move;
			}
			else
			{
				move = userData.cursorInfo.TKinect2Table * move;
			}
		}
		else
		{
			move = userData.cursorInfo.TKinect2Table * move;
		}
#else
		move = userData.cursorInfo.TKinect2Table * move;
#endif
		// 現在の相対移動量を計算
		Point2i cursorMoveBuf;
		cursorMoveBuf.x = (int)(*move.ptr<float>(0, 0) * scale);
		cursorMoveBuf.y = (int)(*move.ptr<float>(1, 0) * scale);
		//cout << *move.ptr<float>(0, 0) * scale << endl;

		// エラー回避
		if (abs(cursorMoveBuf.x > 1000000) || abs(cursorMoveBuf.y > 1000000))
		{
			cursorMoveBuf = Point2i(0, 0);
		}


		if (norm(cursorMoveBuf - userData.cursorInfo.cursorMove) > threshCursorMoveDist)
		{
			userData.cursorInfo.cursorMove = cursorMoveBuf;
#ifndef ONLY_POINTIG_GESTURE
			// カーソルを操作
			//cvWaitKey(1);	// マウスイベントを頻繁に呼びすぎると動作が不安定になる
			MoveMouseAbsolute(userData.cursorInfo, mouseEvent | MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE);
#else
			MoveMouseAbsolute(userData.cursorInfo, mouseEvent |  MOUSEEVENTF_ABSOLUTE);
#endif
		}

	}
	else
	{
		// カーソルを操作
		if (mouseEvent != NULL)
		{
			//cvWaitKey(1);	// マウスイベントを頻繁に呼びすぎると動作が不安定になる
			MoveMouseAbsolute(userData.cursorInfo, mouseEvent);
		}

		if (!preCursorMove.empty())
		{
			preCursorMove.clear();
		}
		//return;
	}

	
#if 0 // Debug: Show table area
	Mat a = Mat::zeros(kinectBasics.heightDepth, kinectBasics.widthDepth, CV_8UC3);
	for (int y = 0; y < kinectBasics.heightDepth; y++)
	{
		for (int x = 0; x < kinectBasics.widthDepth; x++)
		{
			if (isHandInTableArea(Point2i(x, y), tableCorners))
				circle(a, Point(x, y), 1, Scalar(0, 255, 0), 1);
		}
	}
	imshow("a", a);
#endif
}

// 手法２
void MouseControl::moveCursorTime(hgmc::UserData& userData, const Mat& tableParam, const cv::Mat& tableCorners)
{
	///
	/// <Param>
	///
	// カーソルのジッター防止用のマージン．この距離以上手を動かした時にカーソルは動き出す．
	const float threshMoveDist = 0.01;	// [m]
	// カーソルのスピード．腕の動きを[m] -> [pixel]へ変換するスケール
	const float cursorSpeed = 2;
	///
	/// </Param>
	///


	// カーソル移動用
	HandInfo& handMove = userData.handInfoR;
	// クリック用
	HandInfo& handClick = userData.handInfoL;

	// マウスが移動中かチェック
	checkDraggingForTime(userData.cursorInfo, handMove, tableParam, tableCorners);

	// クリック状態のチェック
	DWORD mouseEvent = checkClicking(userData.cursorInfo, handClick, tableParam, tableCorners);


	// 腕を使ってカーソルを動かす
	if (userData.cursorInfo.isDragging)
	{
		// 机平面における始点からの変位を求める
		Mat move = (Mat_<float>(4, 1) <<
			handMove.cameraPoint.x,
			handMove.cameraPoint.y,
			handMove.cameraPoint.z,
			1);

		move = userData.cursorInfo.TKinect2Table * move;
		
		float moveDist = sqrt(
			*move.ptr<float>(0, 0) * *move.ptr<float>(0, 0)
			+ *move.ptr<float>(1, 0) * *move.ptr<float>(1, 0)
			);
		
		if (moveDist > threshMoveDist
			&& threshCursorMoveTime < moveDist && moveDist < 10000000)
		{
			// カーソルの位置更新を行う
			POINT currentPt;
			::GetCursorPos(&currentPt);

			Point2i cursorMoveBuf = Point2i((LONG)(*move.ptr<float>(0, 0) / moveDist * cursorSpeed) + currentPt.x, (LONG)(-*move.ptr<float>(1, 0) / moveDist * cursorSpeed) + currentPt.y);
			
			// エラー回避
			if (abs(cursorMoveBuf.x > 1000000) || abs(cursorMoveBuf.y > 1000000))
			{
				cursorMoveBuf = Point2i(0, 0);
			}

			userData.cursorInfo.cursorMove = cursorMoveBuf;
#ifndef ONLY_POINTIG_GESTURE
			// カーソルを操作
			//cvWaitKey(1);	// マウスイベントを頻繁に呼びすぎると動作が不安定になる
			MoveMouseRelative(userData.cursorInfo, mouseEvent | MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE);
#endif
		}

	}
	else
	{
		// カーソルを操作
		if (mouseEvent != NULL)
		{
			//cvWaitKey(1);	// マウスイベントを頻繁に呼びすぎると動作が不安定になる
			MoveMouseAbsolute(userData.cursorInfo, mouseEvent);
		}
		//return;
	}


#if 0 // Debug: Show table area
	Mat a = Mat::zeros(kinectBasics.heightDepth, kinectBasics.widthDepth, CV_8UC3);
	for (int y = 0; y < kinectBasics.heightDepth; y++)
	{
		for (int x = 0; x < kinectBasics.widthDepth; x++)
		{
			if (isHandInTableArea(Point2i(x, y), tableCorners))
				circle(a, Point(x, y), 1, Scalar(0, 255, 0), 1);
		}
	}
	imshow("a", a);
#endif
}

// 手法３
void MouseControl::moveCursorTimeDistance(hgmc::UserData& userData, const Mat& tableParam, const cv::Mat& tableCorners)
{
	///
	/// <Param>
	const float cursorSpeed = 5;	// 腕の動きを[m] -> [pixel]へ変換するスケール
	//const float 
	/// </Param>
	///


	// カーソル移動用
	HandInfo& handMove = userData.handInfoR;
	// クリック用
	HandInfo& handClick = userData.handInfoL;

	// マウスが移動中かチェック
	checkDraggingForTime(userData.cursorInfo, handMove, tableParam, tableCorners);

	// クリック状態のチェック
	DWORD mouseEvent = checkClicking(userData.cursorInfo, handClick, tableParam, tableCorners);



	// 腕を使ってカーソルを動かす
	if (userData.cursorInfo.isDragging)
	{
		// 机平面における始点からの変位を求める
		Mat move = (Mat_<float>(4, 1) <<
			handMove.cameraPoint.x,
			handMove.cameraPoint.y,
			handMove.cameraPoint.z,
			1);
		//cout << userData.cursorInfo.preTKinect2Table << endl;

		move = userData.cursorInfo.TKinect2Table * move;
		float moveDist = sqrt(
			*move.ptr<float>(0, 0) * *move.ptr<float>(0, 0)
			+ *move.ptr<float>(1, 0) * *move.ptr<float>(1, 0)
			);
		const float threshMoveDist = 0.1;	// [m]
		if (moveDist > threshMoveDist
			&& threshCursorMoveTime < moveDist && moveDist < 10000000)
		{
			// 位置更新
			POINT currentPt;
			::GetCursorPos(&currentPt);

			Point2i cursorMoveBuf = Point2i((LONG)(*move.ptr<float>(0, 0) / moveDist * cursorSpeed) + currentPt.x, (LONG)(-*move.ptr<float>(1, 0) / moveDist * cursorSpeed) + currentPt.y);

			// エラー回避
			if (abs(cursorMoveBuf.x > 1000000) || abs(cursorMoveBuf.y > 1000000))
			{
				cursorMoveBuf = Point2i(0, 0);
			}

			userData.cursorInfo.cursorMove = cursorMoveBuf;
#ifndef ONLY_POINTIG_GESTURE
			// カーソルを操作
			//cvWaitKey(1);	// マウスイベントを頻繁に呼びすぎると動作が不安定になる
			if (mouseEvent == MOUSEEVENTF_LEFTDOWN)
			{
				MoveMouseRelative(userData.cursorInfo, mouseEvent | MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE);
				MoveMouseRelative(userData.cursorInfo, MOUSEEVENTF_LEFTUP);
			}
			else
			{
				MoveMouseRelative(userData.cursorInfo, mouseEvent | MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE);
			}
#endif
		}

	}
	else
	{
		userData.cursorInfo.cursorMove = Point2i(0, 0);
		// カーソルを操作
		if (mouseEvent != NULL)
		{
			//cvWaitKey(1);	// マウスイベントを頻繁に呼びすぎると動作が不安定になる
			MoveMouseAbsolute(userData.cursorInfo, mouseEvent);
		}
		//return;
	}


#if 0 // Debug: Show table area
	Mat a = Mat::zeros(kinectBasics.heightDepth, kinectBasics.widthDepth, CV_8UC3);
	for (int y = 0; y < kinectBasics.heightDepth; y++)
	{
		for (int x = 0; x < kinectBasics.widthDepth; x++)
		{
			if (isHandInTableArea(Point2i(x, y), tableCorners))
				circle(a, Point(x, y), 1, Scalar(0, 255, 0), 1);
		}
	}
	imshow("a", a);
#endif
}

void MouseControl::resetTransMatrix(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const cv::Mat& tableParam)
{
	cursorInfo.cursorMove.x = 0;
	cursorInfo.cursorMove.y = 0;

	// 現在のマウス位置と手の位置を初期位置として記憶
	::GetCursorPos(&cursorInfo.sPt);
	sPtCamera.x = handInfo.cameraPoint.x;
	sPtCamera.y = handInfo.cameraPoint.y;
	sPtCamera.z = handInfo.cameraPoint.z;

	// 平面への座標変換行列計算
	if (coordinateMode == ARM_COORDINATE)
	{
		cursorInfo.TKinect2Table = Mat::eye(4, 4, CV_32F);
		// 並進成分
		*cursorInfo.TKinect2Table.ptr<float>(0, 3) = handInfo.cameraPoint.x;
		*cursorInfo.TKinect2Table.ptr<float>(1, 3) = handInfo.cameraPoint.y;
		*cursorInfo.TKinect2Table.ptr<float>(2, 3) = handInfo.cameraPoint.z;

		// 回転成分
		// 法線をZ方向に
		Mat Z;
		normalize(tableParam, Z);
		// 手と腕の重心を結ぶベクトルを平面に射影した方向をY方向に
		Mat Y = Mat::zeros(3, 1, CV_32F);
		*Y.ptr<float>(0, 0) = handInfo.cameraPoint.x - handInfo.centroid3f.x;
		*Y.ptr<float>(1, 0) = handInfo.cameraPoint.y - handInfo.centroid3f.y;
		*Y.ptr<float>(2, 0) = handInfo.cameraPoint.z - handInfo.centroid3f.z;
		Y = (1 - Y.dot(Z)) * Y;
		normalize(Y, Y);
		// Y軸とZ軸の外積をX軸に
		Mat X = -1 * Y.cross(Z);
		cv::normalize(X, X);

		*cursorInfo.TKinect2Table.ptr<float>(0, 0) = *X.ptr<float>(0, 0);
		*cursorInfo.TKinect2Table.ptr<float>(1, 0) = *X.ptr<float>(1, 0);
		*cursorInfo.TKinect2Table.ptr<float>(2, 0) = *X.ptr<float>(2, 0);

		*cursorInfo.TKinect2Table.ptr<float>(0, 1) = *Y.ptr<float>(0, 0);
		*cursorInfo.TKinect2Table.ptr<float>(1, 1) = *Y.ptr<float>(1, 0);
		*cursorInfo.TKinect2Table.ptr<float>(2, 1) = *Y.ptr<float>(2, 0);

		*cursorInfo.TKinect2Table.ptr<float>(0, 2) = *Z.ptr<float>(0, 0);
		*cursorInfo.TKinect2Table.ptr<float>(1, 2) = *Z.ptr<float>(1, 0);
		*cursorInfo.TKinect2Table.ptr<float>(2, 2) = *Z.ptr<float>(2, 0);

		cursorInfo.TKinect2Table = cursorInfo.TKinect2Table.inv();
	}
}

void MouseControl::checkDraggingForDistance(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const cv::Mat& tableParam, const cv::Mat& tableCorners)
{
	handInfo.isOnTable = false;

	// 手の状況に合わせて処理
	if (!handInfo.isTracked)
	{
		// ドラッグ終了
		cursorInfo.isDragging = false;
	}
	else
	{
		// テーブルとの距離でドラッグか判定
		float distance = -(
			*tableParam.ptr<float>(0, 0) * handInfo.cameraPoint.x
			+ *tableParam.ptr<float>(1, 0) * handInfo.cameraPoint.y
			+ *tableParam.ptr<float>(2, 0) * handInfo.cameraPoint.z - 1)
			/ sqrt(*tableParam.ptr<float>(0, 0) * *tableParam.ptr<float>(0, 0)
			+ *tableParam.ptr<float>(1, 0) * *tableParam.ptr<float>(1, 0)
			+ *tableParam.ptr<float>(2, 0) * *tableParam.ptr<float>(2, 0));
		// 手がテーブル領域内かチェック
		bool isInTable = isHandInTableArea(handInfo.depthPoint, tableCorners);

		//cout << distance << endl;
		// ドラッグ終了
		if (distance > thresholdDrag || !isInTable)
		{
			cursorInfo.isDragging = false;
		}
		// ドラッグ中
		else if (distance > 0)
		{
			if (!cursorInfo.TKinect2Table.empty())
			{
				cursorInfo.preTKinect2Table = cursorInfo.TKinect2Table.clone();
			}

			if (cursorInfo.isDragging)
			{
				timerCoordinate.stop();
				if (timerCoordinate.getTimeSec() < updateCoordinateRate)
				{
					timerCoordinate.start();
					handInfo.isOnTable = true;
					cursorInfo.isDragging = true;
					return;
				}
			}

			resetTransMatrix(cursorInfo, handInfo, tableParam);

			handInfo.isOnTable = true;
			cursorInfo.isDragging = true;

			timerCoordinate.reset();
			timerCoordinate.start();
		}
		else
		{
			cout << "eee" << endl;
			//handInfo.isOnTable = true;
		}
	}
	return;
}

void MouseControl::checkDraggingForTime(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const cv::Mat& tableParam, const cv::Mat& tableCorners)
{
	handInfo.isOnTable = false;

	// 手の状況に合わせて処理
	if (!handInfo.isTracked)
	{
		// ドラッグ終了
		cursorInfo.isDragging = false;
	}
	else
	{
		// テーブルとの距離でドラッグか判定
		float distance = -(
			*tableParam.ptr<float>(0, 0) * handInfo.cameraPoint.x
			+ *tableParam.ptr<float>(1, 0) * handInfo.cameraPoint.y
			+ *tableParam.ptr<float>(2, 0) * handInfo.cameraPoint.z - 1)
			/ sqrt(*tableParam.ptr<float>(0, 0) * *tableParam.ptr<float>(0, 0)
			+ *tableParam.ptr<float>(1, 0) * *tableParam.ptr<float>(1, 0)
			+ *tableParam.ptr<float>(2, 0) * *tableParam.ptr<float>(2, 0));
		// 手がテーブル領域内かチェック
		bool isInTable = isHandInTableArea(handInfo.depthPoint, tableCorners);

		//cout << distance << endl;
		// ドラッグ終了
		if (distance > thresholdDrag || !isInTable)
		{
			cursorInfo.isDragging = false;
		}
		// ドラッグ開始
		else if (distance > 0 && !cursorInfo.isDragging)
		{
			resetTransMatrix(cursorInfo, handInfo, tableParam);

			handInfo.isOnTable = true;
			cursorInfo.isDragging = true;

			timerCoordinate.reset();
			timerCoordinate.start();
		}
		// ドラッグ中
		else
		{
			handInfo.isOnTable = true;
			cursorInfo.isDragging = true;
		}
	}
	return;
}

DWORD MouseControl::checkClicking(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const cv::Mat& tableParam, const cv::Mat& tableCorners)
{
	// クリック無効の場合
	if (clickMode == CLICK_DISABLE)
	{
		cursorInfo.isClicking = false;
		handInfo.isOnTable = false;
		return NULL;
	}

	if (!handInfo.isTracked)
	{
		DWORD flag;
		//cursorInfo.isClicking ? flag = MOUSEEVENTF_LEFTUP : flag = NULL;
		cursorInfo.isClicking ? flag = MOUSEEVENTF_LEFTUP : flag = MOUSEEVENTF_LEFTUP; // ずっとドラッグ状態になる時はこっち
		cursorInfo.isClicking = false;
		handInfo.isOnTable = false;
		return flag;
	}

	float distance = -(
		*tableParam.ptr<float>(0, 0) * handInfo.cameraPoint.x
		+ *tableParam.ptr<float>(1, 0) * handInfo.cameraPoint.y
		+ *tableParam.ptr<float>(2, 0) * handInfo.cameraPoint.z - 1)
		/ sqrt(*tableParam.ptr<float>(0, 0) * *tableParam.ptr<float>(0, 0)
		+ *tableParam.ptr<float>(1, 0) * *tableParam.ptr<float>(1, 0)
		+ *tableParam.ptr<float>(2, 0) * *tableParam.ptr<float>(2, 0));
	// 手がテーブル領域内かチェック
	bool isInTable = isHandInTableArea(handInfo.depthPoint, tableCorners);

	// テーブルの上に手がある
	if (0 < distance && distance < thresholdClick && isInTable)
	{
		handInfo.isOnTable = true;

		if (!cursorInfo.isClicking)
		{
			cursorInfo.isClicking = true;

			// クリック開始
			return MOUSEEVENTF_LEFTDOWN;
			//return NULL;
		}
	}
	else
	{
		cursorInfo.isClicking = false;
		handInfo.isOnTable = false;
		return MOUSEEVENTF_LEFTUP;
	}
}

void MouseControl::MoveMouseAbsolute(const hgmc::CursorInfo& cursorInfo, DWORD mouseFlag)
{


	// 前nフレームの平均の位置を求める（ジッター防止）
	Point2i cursorMoveBuf = Point2i(cursorInfo.sPt.x + cursorInfo.cursorMove.x, cursorInfo.sPt.y - cursorInfo.cursorMove.y);
	preCursorMove.push_back(cursorMoveBuf);
	if (preCursorMove.size() > preCursorMoveNumMax)
	{
		preCursorMove.pop_front();
	}
	Point2i cursorMoveAve;
	list<Point2i>::iterator it = preCursorMove.begin(); // イテレータ
	while (it != preCursorMove.end())  // listの末尾まで
	{
		cursorMoveAve += *it;
		++it;  // イテレータを１つ進める
	}
	cursorMoveAve.x /= preCursorMove.size();
	cursorMoveAve.y /= preCursorMove.size();

	cout << cursorMoveAve << endl;
	if (norm(cursorMoveAve) > 100000) return;

	//スクリーン座標をmouse_event()用の座標に変換
	DWORD dwX = (cursorMoveAve.x) * 65535 / ::GetSystemMetrics(SM_CXSCREEN);
	DWORD dwY = (cursorMoveAve.y) * 65535 / ::GetSystemMetrics(SM_CYSCREEN);

	//マウスカーソルを移動
	::mouse_event(mouseFlag, dwX, dwY, NULL, NULL);
}

void MouseControl::MoveMouseRelative(const hgmc::CursorInfo& cursorInfo, DWORD mouseFlag)
{


	DWORD	dwX;
	DWORD	dwY;

	//スクリーン座標をmouse_event()用の座標に変換
	dwX = (cursorInfo.cursorMove.x) * 65535 / ::GetSystemMetrics(SM_CXSCREEN);
	dwY = (cursorInfo.cursorMove.y) * 65535 / ::GetSystemMetrics(SM_CYSCREEN);

	//マウスカーソルを移動
	::mouse_event(mouseFlag, dwX, dwY, NULL, NULL);
}

int MouseControl::toggleClickMode()
{
	clickMode = (clickMode == CLICK_DISABLE ? CLICK_ENABLE : CLICK_DISABLE);
	
	return clickMode;
}