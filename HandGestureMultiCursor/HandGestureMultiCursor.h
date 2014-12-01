#pragma once



namespace hgmc	// Start namespace hgmc
{

///////////////////////////////////////////////////////////////////
/// <Settings> 調整可能なパラメータ
/////////////////////////////////////////////////////////////////// 

//#define USE_KINECT_V1		// Kinect v1を用いる場合はコメントを外す
#define NEAR_MODE		// nearモードを使う場合はコメントを外す(Kinect v1のみ)

// #define USE_COLOR_V2		// カラー画像を使用する．コメントアウトすると少し早くなる．多分．

// The height of Kinect which is set on the celling [mm]
const static float KINECT_HEIGHT = 2000;		// デスクトップPC
//const static float KINECT_HEIGHT = 1800;		// mac

// The threshold which detect users in the first step (The distance from the floor)
// 最初にユーザを検出する時のしきい値(地面からの高さ)[mm]
const static int USER_HEIGHT_THRESHOLD = 600;	// デスクトップPC
//const static int USER_HEIGHT_THRESHOLD = 800;	// mac

// The height of the desk (This separate objects and users)
const static float DESK_HEIGHT = KINECT_HEIGHT -  1.9944671684835250e+003;	// デスクトップPC
//const static int DESK_HEIGHT = 550;		// mac

// 各座標変換行列
const static char* dispInfo_filenames[] = {
	"calibData/DispInfo1.xml"
	//,"calibData/DispInfo2.xml"
};
const static std::vector<std::string> DISP_INFO_FILENAMES(std::begin(dispInfo_filenames), std::end(dispInfo_filenames));

//const static char* tableInfo_filenames[] = {
//
//};
const static char* tableInfo_filename = { "calibData/TableInfo1.xml" };

// 手を検出するための, 頭を中心とした球の半径 [m]
const static float SENCIG_CIRCLE_RADIUS = 0.3;


///////////////////////////////////////////////////////////////////
/// </Settings>
///////////////////////////////////////////////////////////////////


// Threshold for separating table and user [mm]
const static float TABLE_THRESHOLD = 15;

// Maximum height of the users [mm]
const static float HEAD_HEIGHT_MAX = 2400;

// The length of the user's shoulder [mm]
const static float SHOULDER_LENGTH = 300;

// The lenth of the user's head [mm]
const static float HEAD_LENGTH = 150;


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

//
// Structures
//
// 頭に関する情報
typedef struct {
	cv::Point2i depthPoint;
	cv::Point3f cameraPoint;

	int height;	// Height from table
} HeadInfo;

// 手に関する情報
typedef struct {
	cv::Point2i depthPoint;
	cv::Point3f cameraPoint;
	cv::Point3f centroid3f;
	float area;
	bool isTracked;
} HandInfo;

// カーソルに関する情報
typedef struct {
	int displayNum;
	cv::Point2f position;
	bool isShownCursor;
	bool isClicking;
	float clickingTime; // [sec]
} CursorInfo;

//
// Main user data stracture
//
typedef struct {

	bool isDataFound;	// 前フレームのデータとして参照するとき対応するblobが見つかったかどうか

	HeadInfo headInfo;

	// Will use this
	HandInfo handInfoR;
	HandInfo handInfoL;
	// Will delete
	HandInfo handInfo;

	// 重心
	cv::Point2i centroid;
	unsigned long labelID;

	CursorInfo cursorInfo;

	int preDataID;	// For Accessing pre data

} UserData;



}	// End of namespace hgmc

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

	// Initialise OpenGL
	void initGL(int argc, char* argv[]);

	// OpenGL callback function
	void display(void);		// Draw cursors
	void reshape(int w, int h);
	void idle(void);
	void keyboard(unsigned char key, int x, int y);
	void mouse(int button, int state, int mouse_x, int mouse_y);

	void showHelp();

private:
	// Screen resolution
	std::vector<int> VEC_WIN_WIDTH;
	std::vector<int> VEC_WIN_HEIGHT;

#ifdef USE_KINECT_V1
	// Window size (depth)
	int CAMERA_WIDTH;
	int CAMERA_HEIGHT;
#endif
	// Each pixel or 3D point data
	cv::Mat	userAreaMat;	// Areas of each users
	cv::Mat point3fMatrix;	// 3D points of the observed points
	cv::Mat heightMatrix;	// Heights of each pixel from the floor
	cv::Mat labelMat;		// Label of each pixels
	cv::Mat preLabelMat;	// Label of each pixels in pre-frame
	cv::Mat heightFromTable;// Heights of each pixel from table

	cv::Mat depthImage;		// Image from kinect depth camera
	cv::Mat rgbImage;		// Image from kinect color camera

	std::vector<cv::Mat> handRegions;
	std::vector<cv::Mat> headRegions;

	// 座標変換行列
	std::vector<cv::Mat> TKinect2Display;
	std::vector<cv::Mat> TDisplay2Pixel;
	std::vector<int> windowOffsetX;	// マルチディスプレイ表示の際，他のディスプレイを考慮した座標値を求めるために使う

	cv::Mat tableParam;	// テーブル平面を表すパラメータ

	// Informations of each users
	//UserData userData;
	std::vector<hgmc::UserData> userData;
	std::vector<hgmc::UserData> preUserData;

	cv::TickMeter timer;
	cv::TickMeter fpsTimer;
	double fps;
	std::string fpsStr;
	int fpsCount = 0;

	bool isCursorMoving = true;

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
	cvb::CvBlobs labelingUserArea(cv::Mat& mat);
	void detectHeadPosition(cvb::CvBlobs blobs);
	void detectArm(cvb::CvBlobs blobs);
	void detectHand(std::vector<cv::Mat> handRegions, std::vector<cv::Mat> headRegions);
	void calcCursorPos(cvb::CvBlobs blobs);
	void pointingCursorControl();
	void relativeCursorControl();
	void updatePreData();


	/* For showing results */
	bool isShowDebugWindows;
	void showDebugWindows();


	/* OpenGL */
	int* WinIDs;

	void SetCursor(float x, float y);

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