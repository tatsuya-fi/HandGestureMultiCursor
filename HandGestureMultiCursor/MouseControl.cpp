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
	// �h���b�O��Ԃ̃`�F�b�N
	checkCursor(userData, tableParam);

	// �����ʂɂ�����n�_����̃x�N�g�������߂�
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
	// ��̏󋵂ɍ��킹�ď���
	if (!userData.handInfoR.isTracked)
	{
		// �h���b�O�I��5
		isDragging = false;
	}
	else
	{
		// �e�[�u���Ƃ̋����Ńh���b�O������
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
			// �h���b�O�I��
			isDragging = false;
			return;
		}
		// �h���b�O�J�n
		else if (!isDragging)
		{
			isDragging = true;

			// ���݂̃}�E�X�ʒu�Ǝ�̈ʒu�������ʒu�Ƃ��ċL��
			::GetCursorPos(&sPt);
			sPtCamera.x = userData.handInfoR.cameraPoint.x;
			sPtCamera.y = userData.handInfoR.cameraPoint.y;
			sPtCamera.z = userData.handInfoR.cameraPoint.z;

			// ���ʂւ̍��W�ϊ��s��v�Z
			if (coordinateMode == ARM_COORDINATE)
			{
				TKinect2Table = Mat::eye(4, 4, CV_32F);
				// ���i����
				*TKinect2Table.ptr<float>(0, 3) = -sPtCamera.x;
				*TKinect2Table.ptr<float>(1, 3) = -sPtCamera.y;
				*TKinect2Table.ptr<float>(2, 3) = -sPtCamera.z;

				// ��]����
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

	//�X�N���[�����W��mouse_event()�p�̍��W�ɕϊ�
	dwX = nX * 65535 / ::GetSystemMetrics(SM_CXSCREEN);
	dwY = nY * 65535 / ::GetSystemMetrics(SM_CYSCREEN);

	//�}�E�X�J�[�\�����ړ�
	::mouse_event(MOUSEEVENTF_ABSOLUTE | MOUSEEVENTF_MOVE, dwX, dwY, NULL, NULL);
}