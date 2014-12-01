#pragma once
class MouseControl
{
public:
	MouseControl();
	~MouseControl();

	enum
	{
		ARM_COORDINATE,		// �r������y���W�ɂ��������W�n
	};

	void moveCursorDistance(hgmc::UserData userData, const cv::Mat& tableParam);
	void moveCursorTime(hgmc::UserData userData, const cv::Mat& tableParam);
	void moveCursorDistanceTime(hgmc::UserData userData, const cv::Mat& tableParam);
private:
	// �h���b�O�J�n�ʒu
	POINT sPt;
	cv::Point3f sPtCamera;

	// �h���b�O�����ۂ�
	bool isDragging;

	// ���W�n�̒�`
	unsigned int coordinateMode;

	// �����ʂւ̍��W�ϊ��s��
	cv::Mat TKinect2Table;

	void checkCursor(hgmc::UserData userData, const cv::Mat& tableParam);
	void MoveMouse(UINT nX, UINT nY);
};

