#pragma once
class MouseControl
{
public:
	MouseControl();
	~MouseControl();

	enum
	{
		ARM_COORDINATE,		// �r������y���W�ɂ��������W�n
		TABLE_COORDINATE	// �e�[�u���̌�������ɂ��������W�n[������]
	};

	// ���Α���̎�@�O��
	// �����w��(�}�E�X)
	void moveCursorDistance(hgmc::UserData& userData, const cv::Mat& tableParam);
	// ���Ԏw��(���o�[)
	void moveCursorTime(hgmc::UserData userData, const cv::Mat& tableParam);
	// �n�C�u���b�h(�X���l���������o�[)
	void moveCursorDistanceTime(hgmc::UserData userData, const cv::Mat& tableParam);

private:
	// ��Ɗ��Ƃ̐ڐG����̂��߂̂������l
	const float threshold = 0.045;		// [m]

	// �h���b�O�J�n�ʒu
	//POINT sPt;
	cv::Point3f sPtCamera;

	//// ���݂̃}�E�X�̑��Έړ���
	//cv:: Point2i cursorMove;

	//// �h���b�O�����ۂ�(�E��)
	//bool isDragging;

	//// �N���b�N�����ۂ�(����)
	//bool isClicking;

	// ���W�n�̒�`
	unsigned int coordinateMode;

	//// �����ʂւ̍��W�ϊ��s��
	//cv::Mat TKinect2Table;


	// �֐���`
	void checkDragging(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const cv::Mat& tableParam);
	DWORD checkClicking(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const cv::Mat& tableParam);
	void MoveMouse(const hgmc::CursorInfo& cursorInfo, DWORD mouseFlag);
};

