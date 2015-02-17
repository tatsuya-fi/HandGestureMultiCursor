#pragma once
namespace mc
{
	enum
	{
		ARM_COORDINATE,		// �r������y���W�ɂ��������W�n
		TABLE_COORDINATE	// �e�[�u���̌�������ɂ��������W�n[������]
	};

	// ���̏�Ŏ�𓮂������Ƃɂ��J�[�\���ړ��Ɏg�p�����@
	enum
	{
		DISTANCE_BASED_METHOD,
		TIME_BASED_METHOD,
		DISTANCE_TIME_BASED_METHOD,
		RELATIVE_MODE_DISABLE
	};

	// �w�����ɂ��J�[�\���ړ���on/off
	enum
	{
		POINTING_MODE_ON,
		POINTING_MODE_OFF
	};

	// �N���b�N�@�\��on/off
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


	// ���Α���̎�@�O��
	// �����w��(�}�E�X)
	void moveCursorDistance(hgmc::UserData& userData, const cv::Mat& tableParam, const cv::Mat& tableCorners);
	// ���Ԏw��(���o�[)
	void moveCursorTime(hgmc::UserData& userData, const cv::Mat& tableParam, const cv::Mat& tableCorners);
	// �n�C�u���b�h(�X���l���������o�[)
	void moveCursorTimeDistance(hgmc::UserData& userData, const cv::Mat& tableParam, const cv::Mat& tableCorners);

	int toggleClickMode();

private:
	// ��Ɗ��Ƃ̐ڐG����̂��߂̂������l
	const float thresholdDrag = 0.04;		// [m]
	const float thresholdClick = 0.045;		// [m]

	// threshcursorMoveDist���傫����������J�[�\���ʒu�X�V�i�Ղ�Ղ�h�~�j
	const int threshCursorMoveDist = 6;			// [pix]
	// threshcursorMoveTime���傫����������J�[�\���ʒu�X�V�i�Ղ�Ղ�h�~�j
	const float threshCursorMoveTime = 0.02;	    // [m]

	// �h���b�O�J�n�ʒu
	//POINT sPt;
	cv::Point3f sPtCamera;

	// ���W�n�̒�`
	unsigned int coordinateMode;

	// ���W�n�X�V���`�F�b�N����^�C�}�[
	cv::TickMeter timerCoordinate;
	// ����Ɏ肪�������ꍇ�̍��W�n���X�V����p�x[s]
	const float updateCoordinateRate = 5;

	// �J�[�\���ʒu�����ہC�O��̉��t���[���Ƃ̕��ϋ��߂邩
	const int preCursorMoveNumMax = 5;
	// �O�t���[���̃J�[�\���ʒu���i�[�p
	std::list<cv::Point2i> preCursorMove;

	// �N���b�N�L�����������̃t���O
	int clickMode = mc::CLICK_ENABLE;

	// �֐���`
	void resetTransMatrix(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const cv::Mat& tableParam);
	void checkDraggingForDistance(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const cv::Mat& tableParam, const cv::Mat& tableCorners);
	void checkDraggingForTime(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const cv::Mat& tableParam, const cv::Mat& tableCorners);
	DWORD checkClicking(hgmc::CursorInfo& cursorInfo, hgmc::HandInfo& handInfo, const cv::Mat& tableParam, const cv::Mat& tableCorners);
	void MoveMouseAbsolute(const hgmc::CursorInfo& cursorInfo, DWORD mouseFlag);
	void MoveMouseRelative(const hgmc::CursorInfo& cursorInfo, DWORD mouseFlag);


	// �O�ς��g���Ď肪�������O�ǂ���ɂ��邩�v�Z����
	// http://www.sousakuba.com/Programming/gs_hittest_point_triangle.html
	inline bool isHandInTableArea(const cv::Point2i hand, const cv::Mat& tableCorners)
	{
		// �����O�p�`2�ɕ������čl����
		// Z���������`�F�b�N
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
			// �O�ς����ׂē��������������Ă���ΎO�p�`���ɂ���
			isInTable[i] = (c1 >= 0 && c2 >= 0 && c3 >= 0) || (c1 <= 0 && c2 <= 0 && c3 <= 0);
		}

		return (isInTable[0] || isInTable[1]);
	}

};

