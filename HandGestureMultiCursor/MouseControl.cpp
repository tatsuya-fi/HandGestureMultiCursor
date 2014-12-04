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
	// �J�[�\���ړ��p
	HandInfo& handMove = userData.handInfoR;
	// �N���b�N�p
	HandInfo& handClick = userData.handInfoL;

	// �}�E�X���ړ������`�F�b�N
	checkDragging(userData.cursorInfo, handMove, tableParam);

	// �N���b�N��Ԃ̃`�F�b�N
	DWORD mouseEvent = checkClicking(userData.cursorInfo, handClick, tableParam);



	// �r���g���ăJ�[�\���𓮂���
	if (userData.cursorInfo.isDragging)
	{
		// �����ʂɂ�����n�_����̕ψʂ����߂�
		Mat move = (Mat_<float>(4, 1) <<
			handMove.cameraPoint.x,
			handMove.cameraPoint.y,
			handMove.cameraPoint.z,
			1);
		move = userData.cursorInfo.TKinect2Table * move;
		//cout << userData.cursorInfo.TKinect2Table << endl;
		// ���݂̑��Έړ��ʂ��v�Z
		const int scale = 2000;	// �r�̓�����[m] -> [pixel]�֕ϊ�����X�P�[��
		Point2i cursorMoveBuf;
		cursorMoveBuf.x = (int)(*move.ptr<float>(0, 0) * scale);
		cursorMoveBuf.y = (int)(*move.ptr<float>(1, 0) * scale);

		// threshcursorMove���傫����������J�[�\���ʒu�X�V�i�Ղ�Ղ�h�~�j
		const int threshcursorMove = 10;
		if (norm(cursorMoveBuf - userData.cursorInfo.cursorMove) > threshcursorMove)
		{
			userData.cursorInfo.cursorMove = cursorMoveBuf;
		}
#ifndef ONLY_POINTIG_GESTURE
		// �J�[�\���𑀍�
		cvWaitKey(3);	// �}�E�X�C�x���g��p�ɂɌĂт�����Ɠ��삪�s����ɂȂ�
		MoveMouse(userData.cursorInfo, mouseEvent | MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE);
#endif

		//return TKinect2Table;
	}
	else
	{
		// �J�[�\���𑀍�
		cvWaitKey(3);	// �}�E�X�C�x���g��p�ɂɌĂт�����Ɠ��삪�s����ɂȂ�
		MoveMouse(userData.cursorInfo, mouseEvent);
		//return;
	}

}

void MouseControl::checkDragging(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const Mat& tableParam)
{
	handInfo.isOnTable = false;

	// ��̏󋵂ɍ��킹�ď���
	if (!handInfo.isTracked)
	{
		// �h���b�O�I��
		cursorInfo.isDragging = false;
	}
	else
	{
		// �e�[�u���Ƃ̋����Ńh���b�O������
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
			// �h���b�O�I��
			cursorInfo.isDragging = false;
		}
		// �h���b�O�J�n
		else if (!cursorInfo.isDragging)
		{
			handInfo.isOnTable = true;
			cursorInfo.isDragging = true;

			cursorInfo.cursorMove.x = 0;
			cursorInfo.cursorMove.y = 0;

			// ���݂̃}�E�X�ʒu�Ǝ�̈ʒu�������ʒu�Ƃ��ċL��
			::GetCursorPos(&cursorInfo.sPt);
			sPtCamera.x = handInfo.cameraPoint.x;
			sPtCamera.y = handInfo.cameraPoint.y;
			sPtCamera.z = handInfo.cameraPoint.z;

			// ���ʂւ̍��W�ϊ��s��v�Z
			if (coordinateMode == ARM_COORDINATE)
			{
				cursorInfo.TKinect2Table = Mat::eye(4, 4, CV_32F);
				// ���i����
				*cursorInfo.TKinect2Table.ptr<float>(0, 3) = handInfo.cameraPoint.x;
				*cursorInfo.TKinect2Table.ptr<float>(1, 3) = handInfo.cameraPoint.y;
				*cursorInfo.TKinect2Table.ptr<float>(2, 3) = handInfo.cameraPoint.z;

				// ��]����
				// �@����Z������
				Mat Z;
				normalize(tableParam, Z);
				// ��Ƙr�̏d�S�����ԃx�N�g���𕽖ʂɎˉe����������Y������
				Mat Y = Mat::zeros(3, 1, CV_32F);
				*Y.ptr<float>(0, 0) = handInfo.cameraPoint.x - handInfo.centroid3f.x;
				*Y.ptr<float>(1, 0) = handInfo.cameraPoint.y - handInfo.centroid3f.y;
				*Y.ptr<float>(2, 0) = handInfo.cameraPoint.z - handInfo.centroid3f.z;
				Y = (1 - Y.dot(Z)) * Y;
				normalize(Y, Y);
				// Y����Z���̊O�ς�X����
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
		// �h���b�O��
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
	// �e�[�u���̏�Ɏ肪����
	if (distance < threshold)
	{
		handInfo.isOnTable = true;

		if (!cursorInfo.isClicking)
		{
			cursorInfo.isClicking = true;

			// �N���b�N�J�n
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

	//�X�N���[�����W��mouse_event()�p�̍��W�ɕϊ�
	dwX = (cursorInfo.sPt.x + cursorInfo.cursorMove.x) * 65535 / ::GetSystemMetrics(SM_CXSCREEN);
	dwY = (cursorInfo.sPt.y - cursorInfo.cursorMove.y) * 65535 / ::GetSystemMetrics(SM_CYSCREEN);


	//�}�E�X�J�[�\�����ړ�
	::mouse_event(mouseFlag, dwX, dwY, NULL, NULL);
}