#pragma once

using namespace std;
using namespace cv;
using namespace cvb;

///////////////////////////////////////////////////////////////////
/// <Settings> 調整可能なパラメータ
/////////////////////////////////////////////////////////////////// 

//#define USE_KINECT_V1		// Kinect v1を用いる場合はコメントを外す
#define NEAR_MODE		// nearモードを使う場合はコメントを外す(Kinect v1のみ)

#define TRACK_GESTURE_BY_AREA	// これをdefineした時は手領域の正規化した面積に応じてジェスチャ判定する

// #define USE_COLOR_V2		// カラー画像を使用する．コメントアウトすると少し早くなる．多分．


// The height of Kinect which is set on the celling
//const static int KINECT_HEIGHT = 2700;		// デスクトップPC
const static int KINECT_HEIGHT = 1800;		// mac

// The threshold which detect users in the first step (The distance from the floor)
// 最初にユーザを検出する時のしきい値(地面からの高さ)[mm]
//const static int USER_HEIGHT_THRESHOLD = 900;	// デスクトップPC
const static int USER_HEIGHT_THRESHOLD = 800;	// mac

// The height of the desk (This separate objects and users)
//const static int DESK_HEIGHT = KINECT_HEIGHT - 1.9944671684835250e+003;	// デスクトップPC
const static int DESK_HEIGHT = 550;		// mac

// 各座標変換行列
const static char* disp_file[] = {
	"calibData/DispInfo1.xml"
	//,"calibData/DispInfo2.xml"
};
const static vector<String> DISP_INFO_FILENAMES(begin(disp_file), end(disp_file));

// 手を検出するための, 頭を中心とした球の半径 [mm]
const static float SENCIG_CIRCLE_RADIUS = 0.45;


///////////////////////////////////////////////////////////////////
/// </Settings>
///////////////////////////////////////////////////////////////////




// Maximum height of the users
const static int HEAD_HEIGHT_MAX = 2400;

// The length of the user's shoulder [mm]
const static int SHOULDER_LENGTH = 300;

// The lenth of the user's head [mm]
const static int HEAD_LENGTH = 150;


#ifdef USE_KINECT_V1
// The resolution of the kinect depth camera
const NUI_IMAGE_RESOLUTION KINECT_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;

const static Mat T_KinectCameraToWorld = (cv::Mat_<float>(4,4) <<  
	0, 1, 0, 3.4,
	0, 0, 1, -0.30,
	1, 0, 0, -2.4,
	0, 0, 0, 1);
const static Mat T_WorldToScreen = (cv::Mat_<float>(4, 4) << 
	1, 0, 0, 0,
	0, 1, 0, 0,
	0, 0, 1, 0,
	0, 0, 0, 1);
#endif

#define COLOR_IMAGE_WINDOW_NAME "RBG Image"
#define DEPTH_IMAGE_WINDOW_NAME "Depth Image"

#ifdef NEAR_MODE
const int SENCEING_MIN = 400;		// 深度画像に表示する最小距離[mm]
const int SENCEING_MAX = 3000;		// 深度画像に表示する最大距離[mm]
#else
const int SENCEING_MIN = 800;		// 深度画像に表示する最小距離[mm]
const int SENCEING_MAX = 4000;		// 深度画像に表示する最大距離[mm]
#endif

#define ERROR_CHECK( ret )										\
  if ( ret != S_OK ) {											\
    std::stringstream ss;										\
    ss << "failed " #ret " " << std::hex << ret << std::endl;	\
    throw std::runtime_error( ss.str().c_str() );				\
  }

class HandGestureMultiCursor
{
public:
	HandGestureMultiCursor();
	~HandGestureMultiCursor();

	// メインループ関数
	void run();

	// 各Kinect初期化関数
	void initKinectV1();
	void initKinectV2();

	// 座標変換行列の読み込み
	void loadCalibData();

	// GL初期化
	void initGL(int argc, char* argv[]);

	// OpenGL callback function
	void display(void);
	void reshape(int w, int h);
	void idle(void);
	void keyboard(unsigned char key, int x, int y);
	void mouse(int button, int state, int mouse_x, int mouse_y);

	void showHelp();

private:
	// Screen resolution
	vector<int> VEC_WIN_WIDTH;
	vector<int> VEC_WIN_HEIGHT;

#ifdef USE_KINECT_V1
	// Window size (depth)
	int CAMERA_WIDTH;
	int CAMERA_HEIGHT;
#endif
	// Each pixel or 3D point data
	Mat	userAreaMat;	// Areas of each users
	Mat point3fMatrix;	// 3D points of the observed points
	Mat heightMatrix;	// Heights of each pixel from the floor
	Mat labelMat;		// Label of each pixels
	Mat preLabelMat;	// Label of each pixels in pre-frame

	Mat depthImage;		// Image from kinect depth camera
	Mat rgbImage;		// Image from kinect color camera

	vector<Mat> handRegions;

	// 座標変換行列
	vector<Mat> TKinect2Display;
	vector<Mat> TDisplay2Pixel;
	vector<int> windowOffsetX;	// マルチディスプレイ表示の際，他のディスプレイを考慮した座標値を求めるために使う

	// 頭に関する情報
	typedef struct {
		Point2i depthPoint;
		Point3f cameraPoint;

		int height;
	} HeadInfo;

	// 手に関する情報
	typedef struct {
		Point3f cameraPoint;
		float area;
		bool isTracked;
	} HandInfo;

	// カーソルに関する情報
	typedef struct {
		int displayNum;
		Point2f position;
		bool isShownCursor;
		bool isClicking;
	} CursorInfo;
	

	// Data for each user
	typedef struct {
		
		bool isDataFound;	// 前フレームのデータとして参照するとき対応するblobが見つかったかどうか

		HeadInfo headInfo;

		HandInfo handInfo;

		// 重心
		Point2i centroid;
		unsigned long labelID;

		CursorInfo cursorInfo;

	} UserData;

	//UserData userData;
	vector<UserData> userData;
	vector<UserData> preUserData;
#ifdef USE_KINECT_V1
	/* Handles for kinect v1 */
	INuiSensor* kinect;
	HANDLE imageStreamHandle;
	HANDLE depthStreamHandle;

	HANDLE streamEvent;
	/* Functions for kinect v1 */
	void createInstance();
	bool getDepthImageV1();
	void getRgbImageV1();
#endif
	/* Functions for kinect v2*/
	bool getDepthImageV2();
	void getRgbImageV2();

	/* Other Functions */
	bool getFrameData();
	CvBlobs labelingUserArea(Mat& mat);
	void detectHeadPosition(CvBlobs blobs);
	void detectHandPosition(CvBlobs blobs);
	void setCursor(CvBlobs blobs);
	void detectHandGesture(CvBlobs blobs);
	void detectFingerTips(vector<Mat> vecSrc);


	/* For showing results */
	bool isShowDebugWindows;
	void showDebugWindows();


	/* OpenGL */
	int* WinIDs;

	void MouseControl(float x, float y);

};

// クラス宣言
extern HandGestureMultiCursor app;
extern KinectV2Basics kinectBasics;

/* Kinect_v2 */
// Sensor
//static IKinectSensor* pSensor;
//// Reader: Depth dataを保管するストリーム
//static IDepthFrameReader* pDepthReader;
//// CoordinateMapper
//static ICoordinateMapper* pCoordinateMapper;

#ifdef USE_COLOR_V2
// Reader: Color dataを保管するストリーム
static IColorFrameReader* pColorReader;
#endif

void sdisplay();
void sreshape(int w, int h);
void sidle(void);
void skeyboard(unsigned char key, int x, int y);
void smouse(int button, int state, int mouse_x, int mouse_y);