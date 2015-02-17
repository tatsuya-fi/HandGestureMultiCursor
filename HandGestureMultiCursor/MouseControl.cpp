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

// ��@�P
void MouseControl::moveCursorDistance(hgmc::UserData& userData, const Mat& tableParam, const cv::Mat& tableCorners)
{
	///
	/// <Param>
	const float scale = 3000;	// �r�̓�����[m] -> [pixel]�֕ϊ�����X�P�[��
	/// </Param>
	///


	// �J�[�\���ړ��p
	HandInfo& handMove = userData.handInfoR;
	// �N���b�N�p
	HandInfo& handClick = userData.handInfoL;

	// �}�E�X���ړ������`�F�b�N
	checkDraggingForDistance(userData.cursorInfo, handMove, tableParam, tableCorners);

	// �N���b�N��Ԃ̃`�F�b�N
	DWORD mouseEvent = checkClicking(userData.cursorInfo, handClick, tableParam, tableCorners);



	// �r���g���ăJ�[�\���𓮂���
	if (userData.cursorInfo.isDragging)
	{
		// �����ʂɂ�����n�_����̕ψʂ����߂�
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
		// ���݂̑��Έړ��ʂ��v�Z
		Point2i cursorMoveBuf;
		cursorMoveBuf.x = (int)(*move.ptr<float>(0, 0) * scale);
		cursorMoveBuf.y = (int)(*move.ptr<float>(1, 0) * scale);
		//cout << *move.ptr<float>(0, 0) * scale << endl;

		// �G���[���
		if (abs(cursorMoveBuf.x > 1000000) || abs(cursorMoveBuf.y > 1000000))
		{
			cursorMoveBuf = Point2i(0, 0);
		}


		if (norm(cursorMoveBuf - userData.cursorInfo.cursorMove) > threshCursorMoveDist)
		{
			userData.cursorInfo.cursorMove = cursorMoveBuf;
#ifndef ONLY_POINTIG_GESTURE
			// �J�[�\���𑀍�
			//cvWaitKey(1);	// �}�E�X�C�x���g��p�ɂɌĂт�����Ɠ��삪�s����ɂȂ�
			MoveMouseAbsolute(userData.cursorInfo, mouseEvent | MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE);
#else
			MoveMouseAbsolute(userData.cursorInfo, mouseEvent |  MOUSEEVENTF_ABSOLUTE);
#endif
		}

	}
	else
	{
		// �J�[�\���𑀍�
		if (mouseEvent != NULL)
		{
			//cvWaitKey(1);	// �}�E�X�C�x���g��p�ɂɌĂт�����Ɠ��삪�s����ɂȂ�
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

// ��@�Q
void MouseControl::moveCursorTime(hgmc::UserData& userData, const Mat& tableParam, const cv::Mat& tableCorners)
{
	///
	/// <Param>
	///
	// �J�[�\���̃W�b�^�[�h�~�p�̃}�[�W���D���̋����ȏ��𓮂��������ɃJ�[�\���͓����o���D
	const float threshMoveDist = 0.01;	// [m]
	// �J�[�\���̃X�s�[�h�D�r�̓�����[m] -> [pixel]�֕ϊ�����X�P�[��
	const float cursorSpeed = 2;
	///
	/// </Param>
	///


	// �J�[�\���ړ��p
	HandInfo& handMove = userData.handInfoR;
	// �N���b�N�p
	HandInfo& handClick = userData.handInfoL;

	// �}�E�X���ړ������`�F�b�N
	checkDraggingForTime(userData.cursorInfo, handMove, tableParam, tableCorners);

	// �N���b�N��Ԃ̃`�F�b�N
	DWORD mouseEvent = checkClicking(userData.cursorInfo, handClick, tableParam, tableCorners);


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
		
		float moveDist = sqrt(
			*move.ptr<float>(0, 0) * *move.ptr<float>(0, 0)
			+ *move.ptr<float>(1, 0) * *move.ptr<float>(1, 0)
			);
		
		if (moveDist > threshMoveDist
			&& threshCursorMoveTime < moveDist && moveDist < 10000000)
		{
			// �J�[�\���̈ʒu�X�V���s��
			POINT currentPt;
			::GetCursorPos(&currentPt);

			Point2i cursorMoveBuf = Point2i((LONG)(*move.ptr<float>(0, 0) / moveDist * cursorSpeed) + currentPt.x, (LONG)(-*move.ptr<float>(1, 0) / moveDist * cursorSpeed) + currentPt.y);
			
			// �G���[���
			if (abs(cursorMoveBuf.x > 1000000) || abs(cursorMoveBuf.y > 1000000))
			{
				cursorMoveBuf = Point2i(0, 0);
			}

			userData.cursorInfo.cursorMove = cursorMoveBuf;
#ifndef ONLY_POINTIG_GESTURE
			// �J�[�\���𑀍�
			//cvWaitKey(1);	// �}�E�X�C�x���g��p�ɂɌĂт�����Ɠ��삪�s����ɂȂ�
			MoveMouseRelative(userData.cursorInfo, mouseEvent | MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE);
#endif
		}

	}
	else
	{
		// �J�[�\���𑀍�
		if (mouseEvent != NULL)
		{
			//cvWaitKey(1);	// �}�E�X�C�x���g��p�ɂɌĂт�����Ɠ��삪�s����ɂȂ�
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

// ��@�R
void MouseControl::moveCursorTimeDistance(hgmc::UserData& userData, const Mat& tableParam, const cv::Mat& tableCorners)
{
	///
	/// <Param>
	const float cursorSpeed = 5;	// �r�̓�����[m] -> [pixel]�֕ϊ�����X�P�[��
	//const float 
	/// </Param>
	///


	// �J�[�\���ړ��p
	HandInfo& handMove = userData.handInfoR;
	// �N���b�N�p
	HandInfo& handClick = userData.handInfoL;

	// �}�E�X���ړ������`�F�b�N
	checkDraggingForTime(userData.cursorInfo, handMove, tableParam, tableCorners);

	// �N���b�N��Ԃ̃`�F�b�N
	DWORD mouseEvent = checkClicking(userData.cursorInfo, handClick, tableParam, tableCorners);



	// �r���g���ăJ�[�\���𓮂���
	if (userData.cursorInfo.isDragging)
	{
		// �����ʂɂ�����n�_����̕ψʂ����߂�
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
			// �ʒu�X�V
			POINT currentPt;
			::GetCursorPos(&currentPt);

			Point2i cursorMoveBuf = Point2i((LONG)(*move.ptr<float>(0, 0) / moveDist * cursorSpeed) + currentPt.x, (LONG)(-*move.ptr<float>(1, 0) / moveDist * cursorSpeed) + currentPt.y);

			// �G���[���
			if (abs(cursorMoveBuf.x > 1000000) || abs(cursorMoveBuf.y > 1000000))
			{
				cursorMoveBuf = Point2i(0, 0);
			}

			userData.cursorInfo.cursorMove = cursorMoveBuf;
#ifndef ONLY_POINTIG_GESTURE
			// �J�[�\���𑀍�
			//cvWaitKey(1);	// �}�E�X�C�x���g��p�ɂɌĂт�����Ɠ��삪�s����ɂȂ�
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
		// �J�[�\���𑀍�
		if (mouseEvent != NULL)
		{
			//cvWaitKey(1);	// �}�E�X�C�x���g��p�ɂɌĂт�����Ɠ��삪�s����ɂȂ�
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

void MouseControl::checkDraggingForDistance(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const cv::Mat& tableParam, const cv::Mat& tableCorners)
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
		float distance = -(
			*tableParam.ptr<float>(0, 0) * handInfo.cameraPoint.x
			+ *tableParam.ptr<float>(1, 0) * handInfo.cameraPoint.y
			+ *tableParam.ptr<float>(2, 0) * handInfo.cameraPoint.z - 1)
			/ sqrt(*tableParam.ptr<float>(0, 0) * *tableParam.ptr<float>(0, 0)
			+ *tableParam.ptr<float>(1, 0) * *tableParam.ptr<float>(1, 0)
			+ *tableParam.ptr<float>(2, 0) * *tableParam.ptr<float>(2, 0));
		// �肪�e�[�u���̈�����`�F�b�N
		bool isInTable = isHandInTableArea(handInfo.depthPoint, tableCorners);

		//cout << distance << endl;
		// �h���b�O�I��
		if (distance > thresholdDrag || !isInTable)
		{
			cursorInfo.isDragging = false;
		}
		// �h���b�O��
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

	// ��̏󋵂ɍ��킹�ď���
	if (!handInfo.isTracked)
	{
		// �h���b�O�I��
		cursorInfo.isDragging = false;
	}
	else
	{
		// �e�[�u���Ƃ̋����Ńh���b�O������
		float distance = -(
			*tableParam.ptr<float>(0, 0) * handInfo.cameraPoint.x
			+ *tableParam.ptr<float>(1, 0) * handInfo.cameraPoint.y
			+ *tableParam.ptr<float>(2, 0) * handInfo.cameraPoint.z - 1)
			/ sqrt(*tableParam.ptr<float>(0, 0) * *tableParam.ptr<float>(0, 0)
			+ *tableParam.ptr<float>(1, 0) * *tableParam.ptr<float>(1, 0)
			+ *tableParam.ptr<float>(2, 0) * *tableParam.ptr<float>(2, 0));
		// �肪�e�[�u���̈�����`�F�b�N
		bool isInTable = isHandInTableArea(handInfo.depthPoint, tableCorners);

		//cout << distance << endl;
		// �h���b�O�I��
		if (distance > thresholdDrag || !isInTable)
		{
			cursorInfo.isDragging = false;
		}
		// �h���b�O�J�n
		else if (distance > 0 && !cursorInfo.isDragging)
		{
			resetTransMatrix(cursorInfo, handInfo, tableParam);

			handInfo.isOnTable = true;
			cursorInfo.isDragging = true;

			timerCoordinate.reset();
			timerCoordinate.start();
		}
		// �h���b�O��
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
	// �N���b�N�����̏ꍇ
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
		cursorInfo.isClicking ? flag = MOUSEEVENTF_LEFTUP : flag = MOUSEEVENTF_LEFTUP; // �����ƃh���b�O��ԂɂȂ鎞�͂�����
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
	// �肪�e�[�u���̈�����`�F�b�N
	bool isInTable = isHandInTableArea(handInfo.depthPoint, tableCorners);

	// �e�[�u���̏�Ɏ肪����
	if (0 < distance && distance < thresholdClick && isInTable)
	{
		handInfo.isOnTable = true;

		if (!cursorInfo.isClicking)
		{
			cursorInfo.isClicking = true;

			// �N���b�N�J�n
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


	// �On�t���[���̕��ς̈ʒu�����߂�i�W�b�^�[�h�~�j
	Point2i cursorMoveBuf = Point2i(cursorInfo.sPt.x + cursorInfo.cursorMove.x, cursorInfo.sPt.y - cursorInfo.cursorMove.y);
	preCursorMove.push_back(cursorMoveBuf);
	if (preCursorMove.size() > preCursorMoveNumMax)
	{
		preCursorMove.pop_front();
	}
	Point2i cursorMoveAve;
	list<Point2i>::iterator it = preCursorMove.begin(); // �C�e���[�^
	while (it != preCursorMove.end())  // list�̖����܂�
	{
		cursorMoveAve += *it;
		++it;  // �C�e���[�^���P�i�߂�
	}
	cursorMoveAve.x /= preCursorMove.size();
	cursorMoveAve.y /= preCursorMove.size();

	cout << cursorMoveAve << endl;
	if (norm(cursorMoveAve) > 100000) return;

	//�X�N���[�����W��mouse_event()�p�̍��W�ɕϊ�
	DWORD dwX = (cursorMoveAve.x) * 65535 / ::GetSystemMetrics(SM_CXSCREEN);
	DWORD dwY = (cursorMoveAve.y) * 65535 / ::GetSystemMetrics(SM_CYSCREEN);

	//�}�E�X�J�[�\�����ړ�
	::mouse_event(mouseFlag, dwX, dwY, NULL, NULL);
}

void MouseControl::MoveMouseRelative(const hgmc::CursorInfo& cursorInfo, DWORD mouseFlag)
{


	DWORD	dwX;
	DWORD	dwY;

	//�X�N���[�����W��mouse_event()�p�̍��W�ɕϊ�
	dwX = (cursorInfo.cursorMove.x) * 65535 / ::GetSystemMetrics(SM_CXSCREEN);
	dwY = (cursorInfo.cursorMove.y) * 65535 / ::GetSystemMetrics(SM_CYSCREEN);

	//�}�E�X�J�[�\�����ړ�
	::mouse_event(mouseFlag, dwX, dwY, NULL, NULL);
}

int MouseControl::toggleClickMode()
{
	clickMode = (clickMode == CLICK_DISABLE ? CLICK_ENABLE : CLICK_DISABLE);
	
	return clickMode;
}