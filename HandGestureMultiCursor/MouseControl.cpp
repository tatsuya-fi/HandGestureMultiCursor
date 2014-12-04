#include "stdafx.h"
#include "MouseControl.h"

using namespace std;
using namespace cv;
using namespace hgmc;

MouseControl::MouseControl() :
//isDragging(false),
//isClicking(false),
coordinateMode(ARM_COORDINATE)
{
}

MouseControl::~MouseControl()
{
}


void MouseControl::moveCursorDistance(hgmc::UserData& userData, const Mat& tableParam)
{
	// カーソル移動用
	HandInfo& handMove = userData.handInfoR;
	// クリック用
	HandInfo& handClick = userData.handInfoL;

	// マウスが移動中かチェック
	checkDragging(userData.cursorInfo, handMove, tableParam);

	// クリック状態のチェック
	DWORD mouseEvent = checkClicking(userData.cursorInfo, handClick, tableParam);



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
		//cout << userData.cursorInfo.TKinect2Table << endl;
		// 現在の相対移動量を計算
		const int scale = 2000;	// 腕の動きを[m] -> [pixel]へ変換するスケール
		Point2i cursorMoveBuf;
		cursorMoveBuf.x = (int)(*move.ptr<float>(0, 0) * scale);
		cursorMoveBuf.y = (int)(*move.ptr<float>(1, 0) * scale);

		// threshcursorMoveより大きく動いたらカーソル位置更新（ぷるぷる防止）
		const int threshcursorMove = 10;
		if (norm(cursorMoveBuf - userData.cursorInfo.cursorMove) > threshcursorMove)
		{
			userData.cursorInfo.cursorMove = cursorMoveBuf;
		}
#ifndef ONLY_POINTIG_GESTURE
		// カーソルを操作
		cvWaitKey(3);	// マウスイベントを頻繁に呼びすぎると動作が不安定になる
		MoveMouse(userData.cursorInfo, mouseEvent | MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE);
#endif

		//return TKinect2Table;
	}
	else
	{
		// カーソルを操作
		cvWaitKey(3);	// マウスイベントを頻繁に呼びすぎると動作が不安定になる
		MoveMouse(userData.cursorInfo, mouseEvent);
		//return;
	}

}

void MouseControl::checkDragging(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const Mat& tableParam)
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
		float distance = abs(
			*tableParam.ptr<float>(0, 0) * handInfo.cameraPoint.x
			+ *tableParam.ptr<float>(1, 0) * handInfo.cameraPoint.y
			+ *tableParam.ptr<float>(2, 0) * handInfo.cameraPoint.z - 1)
			/ sqrt(*tableParam.ptr<float>(0, 0) * *tableParam.ptr<float>(0, 0)
			+ *tableParam.ptr<float>(1, 0) * *tableParam.ptr<float>(1, 0)
			+ *tableParam.ptr<float>(2, 0) * *tableParam.ptr<float>(2, 0));
		//cout << distance << endl;
		if (distance > threshold)
		{
			// ドラッグ終了
			cursorInfo.isDragging = false;
		}
		// ドラッグ開始
		else if (!cursorInfo.isDragging)
		{
			handInfo.isOnTable = true;
			cursorInfo.isDragging = true;

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
		// ドラッグ中
		else
		{
			handInfo.isOnTable = true;
		}
	}
	return;
}



DWORD MouseControl::checkClicking(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const cv::Mat& tableParam)
{
	if (!handInfo.isTracked)
	{
		DWORD flag;
		cursorInfo.isClicking ? flag = MOUSEEVENTF_LEFTUP : flag = 0;
		cursorInfo.isClicking = false;
		handInfo.isOnTable = false;
		return 0;
	}

	float distance = abs(
		*tableParam.ptr<float>(0, 0) * handInfo.cameraPoint.x
		+ *tableParam.ptr<float>(1, 0) * handInfo.cameraPoint.y
		+ *tableParam.ptr<float>(2, 0) * handInfo.cameraPoint.z - 1)
		/ sqrt(*tableParam.ptr<float>(0, 0) * *tableParam.ptr<float>(0, 0)
		+ *tableParam.ptr<float>(1, 0) * *tableParam.ptr<float>(1, 0)
		+ *tableParam.ptr<float>(2, 0) * *tableParam.ptr<float>(2, 0));
	// テーブルの上に手がある
	if (distance < threshold)
	{
		handInfo.isOnTable = true;

		if (!cursorInfo.isClicking)
		{
			cursorInfo.isClicking = true;

			// クリック開始
			return MOUSEEVENTF_LEFTDOWN;
		}
	}
	else
	{
		cursorInfo.isClicking = false;
		handInfo.isOnTable = false;
		return MOUSEEVENTF_LEFTUP;
	}
}

void MouseControl::MoveMouse(const hgmc::CursorInfo& cursorInfo, DWORD mouseFlag)
{
	DWORD	dwX;
	DWORD	dwY;

	//スクリーン座標をmouse_event()用の座標に変換
	dwX = (cursorInfo.sPt.x + cursorInfo.cursorMove.x) * 65535 / ::GetSystemMetrics(SM_CXSCREEN);
	dwY = (cursorInfo.sPt.y - cursorInfo.cursorMove.y) * 65535 / ::GetSystemMetrics(SM_CYSCREEN);


	//マウスカーソルを移動
	::mouse_event(mouseFlag, dwX, dwY, NULL, NULL);
}