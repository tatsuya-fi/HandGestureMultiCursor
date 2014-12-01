#include "stdafx.h"
#include "MouseControl.h"

using namespace std;
using namespace cv;
using namespace hgmc;

MouseControl::MouseControl() :
isDragging(false),
coordinateMode(ARM_COORDINATE)
{
}


MouseControl::~MouseControl()
{
}


void MouseControl::moveCursorDistance(hgmc::UserData userData, const Mat& tableParam)
{
	// ドラッグ状態のチェック
	checkCursor(userData, tableParam);

	// 机平面における始点からのベクトルを求める
	if (isDragging)
	{
		Mat move = (Mat_<float>(4, 1) <<
			userData.handInfoR.cameraPoint.x,
			userData.handInfoR.cameraPoint.y,
			userData.handInfoR.cameraPoint.z,
			1);
		move = TKinect2Table * move;
		
		cout << move << endl;
	}
}

void MouseControl::checkCursor(hgmc::UserData userData, const Mat& tableParam)
{
	// 手の状況に合わせて処理
	if (!userData.handInfoR.isTracked)
	{
		// ドラッグ終了5
		isDragging = false;
	}
	else
	{
		// テーブルとの距離でドラッグか判定
		const float threshold = 0.15;		// [m]
		float distance = abs(
			*tableParam.ptr<float>(0, 0) * userData.handInfoR.cameraPoint.x
			+ *tableParam.ptr<float>(1, 0) * userData.handInfoR.cameraPoint.y
			+ *tableParam.ptr<float>(2, 0) * userData.handInfoR.cameraPoint.z - 1)
			/ sqrt(*tableParam.ptr<float>(0, 0) * *tableParam.ptr<float>(0, 0)
			+ *tableParam.ptr<float>(1, 0) * *tableParam.ptr<float>(1, 0)
			+ *tableParam.ptr<float>(2, 0) * *tableParam.ptr<float>(2, 0));
		cout << distance << endl;
		if (distance > threshold)
		{
			// ドラッグ終了
			isDragging = false;
			return;
		}
		// ドラッグ開始
		else if (!isDragging)
		{
			isDragging = true;

			// 現在のマウス位置と手の位置を初期位置として記憶
			::GetCursorPos(&sPt);
			sPtCamera.x = userData.handInfoR.cameraPoint.x;
			sPtCamera.y = userData.handInfoR.cameraPoint.y;
			sPtCamera.z = userData.handInfoR.cameraPoint.z;

			// 平面への座標変換行列計算
			if (coordinateMode == ARM_COORDINATE)
			{
				TKinect2Table = Mat::eye(4, 4, CV_32F);
				// 並進成分
				*TKinect2Table.ptr<float>(0, 3) = -sPtCamera.x;
				*TKinect2Table.ptr<float>(1, 3) = -sPtCamera.y;
				*TKinect2Table.ptr<float>(2, 3) = -sPtCamera.z;

				// 回転成分
				Mat Y = Mat::zeros(3, 1, CV_32F);
				*Y.ptr<float>(0, 0) = userData.handInfoR.cameraPoint.x - userData.handInfoR.centroid3f.x;
				*Y.ptr<float>(1, 0) = userData.handInfoR.cameraPoint.y - userData.handInfoR.centroid3f.y;
				*Y.ptr<float>(2, 0) = userData.handInfoR.cameraPoint.z - userData.handInfoR.centroid3f.z;
				normalize(Y, Y);

				Mat Z;
				normalize(tableParam, Z);

				Mat X = Y.cross(Z);
				normalize(X, X);

				*TKinect2Table.ptr<float>(0, 0) = *X.ptr<float>(0, 0);
				*TKinect2Table.ptr<float>(1, 0) = *X.ptr<float>(1, 0);
				*TKinect2Table.ptr<float>(2, 0) = *X.ptr<float>(2, 0);

				*TKinect2Table.ptr<float>(0, 1) = *Y.ptr<float>(0, 0);
				*TKinect2Table.ptr<float>(1, 1) = *Y.ptr<float>(1, 0);
				*TKinect2Table.ptr<float>(2, 1) = *Y.ptr<float>(2, 0);

				*TKinect2Table.ptr<float>(0, 2) = *Z.ptr<float>(0, 0);
				*TKinect2Table.ptr<float>(1, 2) = *Z.ptr<float>(1, 0);
				*TKinect2Table.ptr<float>(2, 2) = *Z.ptr<float>(2, 0);
			}
		}
	}
}

void MouseControl::MoveMouse(UINT nX, UINT nY)
{
	DWORD	dwX;
	DWORD	dwY;

	//スクリーン座標をmouse_event()用の座標に変換
	dwX = nX * 65535 / ::GetSystemMetrics(SM_CXSCREEN);
	dwY = nY * 65535 / ::GetSystemMetrics(SM_CYSCREEN);

	//マウスカーソルを移動
	::mouse_event(MOUSEEVENTF_ABSOLUTE | MOUSEEVENTF_MOVE, dwX, dwY, NULL, NULL);
}