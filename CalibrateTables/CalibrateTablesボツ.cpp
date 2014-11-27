#include "stdafx.h"

using namespace std;
using namespace cv;

#pragma warning(disable:4819)

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>

#define _USE_MATH_DEFINES	// math.hのM_PIを使うため
#include <math.h>			// 角度計算用

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <AR/ar.h>
#include <AR/param.h>
#include <AR/video.h>
#include <AR/gsub.h>

// グローバル変数
/* カメラ構成 */
char *vconf_name = "Data/WDM_camera_flipV.xml";	// ビデオデバイスの設定ファイル
int  xsize;											// ウィンドウサイズ
int  ysize;											// ウィンドウサイズ
int  thresh = 100;									// 2値化の閾値
int  countT = 0;										// 処理フレーム数

static KinectV2Basics kinectApp;
static Mat gARTImageMat;

/* カメラパラメータ */
char *cparam_name = "Data/intrinsicParamKinect.xml";			// カメラパラメータファイル
//char *cparam_name = "Data/camera_para.dat";			// カメラパラメータファイル
ARParam cparam;										// カメラパラメータ

static ARGL_CONTEXT_SETTINGS_REF gArglSettings = NULL;

/* カメラからの画像格納用(OpenCV) */
static Mat imageMat;
static VideoCapture cap(0);

/* パターンファイル */
#define MARK1_MARK_ID	1						// マーカーID
#define MARK1_PATT_NAME	"Data\\patt.sample1"		// パターンファイル名
#define MARK1_SIZE		188.0					// パターンの幅（mm）

//#define MARK1_PATT_NAME	"Data\\patt.sample2"		// パターンファイル名
//#define MARK1_SIZE		676.0					// パターンの幅（mm）


typedef struct {
	char   *patt_name;			// パターンファイル
	int    patt_id;				// パターンのID
	int    mark_id;				// マーカーID
	int    visible;				// 検出フラグ
	double patt_width;			// パターンのサイズ（単位：ｍｍ）
	double patt_center[2];		// パターンの中心座標
	double patt_trans[3][4];	// 座標変換行列
} MARK_T;

MARK_T   marker = { MARK1_PATT_NAME, -1, MARK1_MARK_ID, 0, MARK1_SIZE, { 0.0, 0.0 } };

// プロトタイプ宣言
void Init(void);
void MainLoop(void);
void SetupLighting1(void);
void SetupLighting2(void);
void SetupMaterial1(void);
void SetupMaterial2(void);
void KeyEvent(unsigned char key, int x, int y);
void MouseEvent(int button, int state, int x, int y);
void DrawObject(int mark_id, double patt_trans[3][4]);
void saveTransMat();
ARUint8* getKinectImage();

//=======================================================
// main関数
//=======================================================
int main(int argc, char **argv)
{
	// GLUTの初期化
	glutInit(&argc, argv);

	// ARアプリケーションの初期化
	Init();

	// ビデオキャプチャの開始
	arVideoCapStart();

	// メインループの開始
	argMainLoop(MouseEvent, KeyEvent, MainLoop);

	return 0;
}
static Mat cameraPara, distCoeffs;
int arParamLoadCV(const char* cparam_name, ARParam* wparam)
{


	FileStorage cvfs(cparam_name, CV_STORAGE_READ);
	FileNode node(cvfs.fs, NULL);
	FileNode fn = node[string("mat_array")];
	read(fn[0], cameraPara);
	read(fn[1], distCoeffs);

	wparam->mat[0][0] = cameraPara.at<double>(0, 0);
	wparam->mat[0][1] = cameraPara.at<double>(0, 1);
	wparam->mat[0][2] = cameraPara.at<double>(0, 2);
	wparam->mat[0][3] = 0;
	wparam->mat[1][0] = cameraPara.at<double>(1, 0);
	wparam->mat[1][1] = cameraPara.at<double>(1, 1);
	wparam->mat[1][2] = cameraPara.at<double>(1, 2);
	wparam->mat[1][3] = 0;
	wparam->mat[2][0] = cameraPara.at<double>(2, 0);
	wparam->mat[2][1] = cameraPara.at<double>(2, 1);
	wparam->mat[2][2] = cameraPara.at<double>(2, 2);
	wparam->mat[2][3] = 0;

	wparam->dist_factor[0] = cameraPara.at<double>(0, 2);
	wparam->dist_factor[1] = cameraPara.at<double>(1, 2);
	wparam->dist_factor[2] = 1;// distCoeffs.at<double>(0, 0);
	wparam->dist_factor[3] = 1;

	wparam->xsize = xsize;
	wparam->ysize = ysize;

	return 0;
}

//=======================================================
// 初期化関数
//=======================================================
void Init(void)
{
	ARParam wparam;		// カメラパラメータ

	// ビデオデバイスの設定
	if (!kinectApp.SetupKinectV2()) {
		cout << "error video" << endl;
	}

	// ウィンドウサイズの取得
	xsize = kinectApp.widthColor;
	ysize = kinectApp.heightColor;
	printf("Image size (x,y) = (%d,%d)\n", xsize, ysize);

	// カメラパラメータの設定
	if (arParamLoadCV(cparam_name, &wparam) < 0){
		printf("カメラパラメータの読み込みに失敗しました\n");
		exit(0);
	}

	// カメラパラメータのサイズ調整
	arParamChangeSize(&wparam, xsize, ysize, &cparam);
	// カメラパラメータの初期化
	arInitCparam(&cparam);
	printf("*** Camera Parameter ***\n");
	arParamDisp(&cparam);

	// パターンファイルのロード
	if ((marker.patt_id = arLoadPatt(marker.patt_name)) < 0){
		printf("パターンファイルの読み込みに失敗しました\n");
		printf("%s\n", marker.patt_name);
		exit(0);
	}


	// gsubライブラリの初期化
	argInit(&cparam, 1.0, 0, 0, 0, 0);

	// Setup argl library for current context.

	if ((gArglSettings = arglSetupForCurrentContext()) == NULL) {
		fprintf(stderr, "main(): arglSetupForCurrentContext() returned error.\n");
		exit(-1);
	}


	// ウィンドウタイトルの設定
	glutSetWindowTitle("Calc trans matrix");

	cout << "Press 's' key for saving transformation matrix." << endl;
	cout << "Esc: quit" << endl;
}

ARUint8* getKinectImage()
{
	// KinectからMat形式のカラー画像を取得
	Mat matCV;
	if (!kinectApp.GetColorMat(matCV))
	{
		return NULL;
	}
	Mat unmatCV;
	undistort(matCV, unmatCV, cameraPara, distCoeffs);
	resize(matCV, matCV, Size(matCV.cols / 2, matCV.rows / 2));
	imshow("CV Image", matCV);

	// メモリを勝手に開放されないようにstatic Matに格納
	gARTImageMat = unmatCV;
	// ARUint8にキャスト
	//ARUint8 *imageBuffer = reinterpret_cast<ARUint8*>(gARTImageMat.data);
	ARUint8 *imageBuffer = (ARUint8*)(gARTImageMat.data);

	return imageBuffer;
}

//=======================================================
// メインループ関数
//=======================================================
void MainLoop(void)
{
	ARUint8          *image;			// カメラキャプチャ画像
	ARMarkerInfo     *marker_info;		// マーカ情報
	int              marker_num;		// 検出されたマーカの数
	int              j, k;

	// カメラ画像の取得
	if ((image = getKinectImage()) == NULL){
		//if ((image = (ARUint8 *)arVideoGetImage()) == NULL){
		arUtilSleep(2);
		return;
	}
	if (countT == 0) arUtilTimerReset();
	countT++;

	// カメラ画像の描画
	arglDistortionCompensationSet(gArglSettings, FALSE);	// ARToolKit歪み補正をオフに
	argDrawMode2D();
	argDispImage(image, 0, 0);

	// マーカの検出と認識
	if (arDetectMarker(image, thresh, &marker_info, &marker_num) < 0){
		exit(0);
	}

	// 次の画像のキャプチャ指示
	arVideoCapNext();

	// 3Dオブジェクトを描画するための準備
	argDrawMode3D();
	argDraw3dCamera(0, 0);
	glClearDepth(1.0);					// デプスバッファの消去値
	glClear(GL_DEPTH_BUFFER_BIT);		// デプスバッファの初期化

	// マーカの一致度の比較
	k = -1;
	for (j = 0; j<marker_num; j++){
		if (marker.patt_id == marker_info[j].id){
			if (k == -1) k = j;
			else if (marker_info[k].cf < marker_info[j].cf) k = j;
		}
	}

	// マーカーが見つからなかったとき
	if (k == -1){
		marker.visible = 0;
	}

	// Show marker's confidence
	cout << "confidence:" << marker_info[k].cf << endl;

	// 座標変換行列を取得
	if (marker.visible == 0) {
		// 1フレームを使ってマーカの位置・姿勢（座標変換行列）の計算
		arGetTransMat(&marker_info[k], marker.patt_center, marker.patt_width, marker.patt_trans);
	}
	else {
		// 前のフレームを使ってマーカの位置・姿勢（座標変換行列）の計算
		arGetTransMatCont(&marker_info[k], marker.patt_trans, marker.patt_center, marker.patt_width, marker.patt_trans);
	}
	marker.visible = 1;

	// 3Dオブジェクトの描画
	DrawObject(marker.mark_id, marker.patt_trans);


	// バッファの内容を画面に表示
	argSwapBuffers();

#if 0
	// 2つのマーカ間の角度の差を表示（マーカ1[Hiro]とマーカ3[Kanji]を認識した場合）
	if (marker[0].visible > 0 && marker[2].visible > 0){
		double wmat1[3][4], wmat2[3][4];
		double yaw, pitch, roll;

		// ビュー→マーカ行列（カメラ座標系を基準に考えたマーカの位置・姿勢）を取得
		arUtilMatInv(marker[0].patt_trans, wmat1);
		// マーカ1座標系を基準に考えたマーカ3の位置（＝マーカ1とマーカ3の距離・姿勢の差）を取得
		arUtilMatMul(wmat1, marker[2].patt_trans, wmat2);

		// 姿勢の差を表示
		//for( i=0; i<3; i++ ) {
		//    for( j=0; j< 3; j++ ) printf("%5.4f ", wmat2[i][j]);
		//    printf("\n");
		//}
		//printf("\n");

		// 角度の差を表示（-180°～180°）
		yaw = atan2(wmat2[1][0], wmat2[0][0]);
		pitch = atan2(wmat2[2][1], wmat2[2][2]);
		roll = atan2(wmat2[2][0], sqrt(wmat2[2][1] * wmat2[2][1] + wmat2[2][2] * wmat2[2][2]));

		printf("yaw = %4.4lf pitch = %4.4lf roll = %4.4lf\n", 180.0*yaw / M_PI, 180.0*pitch / M_PI, 180.0*roll / M_PI);
	}
#endif
}


//=======================================================
// 3Dオブジェクトの描画を行う関数
//=======================================================
void DrawObject(int mark_id, double patt_trans[3][4])
{
	double gl_para[16];	// ARToolKit->OpenGL変換行列

	// 陰面消去
	glEnable(GL_DEPTH_TEST);			// 陰面消去・有効
	glDepthFunc(GL_LEQUAL);			// デプステスト

	// 変換行列の適用
	argConvGlpara(patt_trans, gl_para);	// ARToolKitからOpenGLの行列に変換
	glMatrixMode(GL_MODELVIEW);			// 行列変換モード・モデルビュー
	glLoadMatrixd(gl_para);				// 読み込む行列を指定

	switch (mark_id){
	case MARK1_MARK_ID:
		// ライティング
		SetupLighting1();			// ライトの定義
		glEnable(GL_LIGHTING);	// ライティング・有効
		glEnable(GL_LIGHT0);		// ライト0・オン
		// オブジェクトの材質
		SetupMaterial1();

		// 3Dオブジェクトの描画
		glTranslatef(0.0, 0.0, 25.0);	// マーカの上に載せるためにZ方向（マーカ上方）に25.0[mm]移動
		//glutSolidCube(30.0);			// ソリッドキューブを描画（1辺のサイズ[mm]）
		glutSolidTeapot(90.0);
		break;
	}


	// 終了処理
	glDisable(GL_LIGHTING);		// ライティング・無効
	glDisable(GL_DEPTH_TEST);		// デプステスト・無効
}


//=======================================================
// ライティング
//=======================================================
void SetupLighting1(void)
{
	// ライトの定義
	GLfloat lt0_position[] = { 100.0, -200.0, 200.0, 0.0 };	// ライト0の位置
	GLfloat lt0_ambient[] = { 0.1, 0.1, 0.1, 1.0 };			// 　　　　 環境光
	GLfloat lt0_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };			// 　　　　 拡散光

	// ライトの設定
	glLightfv(GL_LIGHT0, GL_POSITION, lt0_position);
	glLightfv(GL_LIGHT0, GL_AMBIENT, lt0_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lt0_diffuse);
}

void SetupLighting2(void)
{
	// ライトの定義
	GLfloat lt0_position[] = { 100.0, 200.0, 200.0, 0.0 };	// ライト0の位置
	GLfloat lt0_ambient[] = { 0.2, 0.2, 0.2, 1.0 };			// 　　　　 環境光
	GLfloat lt0_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };			// 　　　　 拡散光

	// ライトの設定
	glLightfv(GL_LIGHT0, GL_POSITION, lt0_position);
	glLightfv(GL_LIGHT0, GL_AMBIENT, lt0_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lt0_diffuse);
}


//=======================================================
// マテリアルの設定
//=======================================================
void SetupMaterial1(void)
{
	// オブジェクトの材質
	GLfloat mat_ambient[] = { 0.0, 1.0, 1.0, 1.0 };	// 材質の環境光
	GLfloat mat_specular[] = { 0.0, 0.0, 1.0, 1.0 };	// 鏡面光
	GLfloat mat_shininess[] = { 50.0 };				// 鏡面係数

	// マテリアルの設定
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
}

void SetupMaterial2(void)
{
	// オブジェクトの材質
	GLfloat mat_ambient[] = { 0.0, 0.0, 1.0, 1.0 };	// 材質の環境光
	GLfloat mat_specular[] = { 0.0, 0.0, 1.0, 1.0 };	// 鏡面光
	GLfloat mat_shininess[] = { 50.0 };				// 鏡面係数

	// マテリアルの設定
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
}

//=======================================================
// 座標変換行列保存関数
//=======================================================
void saveTransMat()
{
	const char* transMatName = "..\\HandGestureMultiCursor\\calibData\\TableInfo1.xml";
	Mat T_Marker2KinectCCamera = (Mat_<double>(4, 4) <<
		marker.patt_trans[0][0], marker.patt_trans[0][1], marker.patt_trans[0][2], marker.patt_trans[0][3],
		marker.patt_trans[1][0], marker.patt_trans[1][1], marker.patt_trans[1][2], marker.patt_trans[1][3],
		marker.patt_trans[2][0], marker.patt_trans[2][1], marker.patt_trans[2][2], marker.patt_trans[2][3],
		0, 0, 0, 1
		);
	FileStorage cvfs(transMatName, CV_STORAGE_WRITE);
	WriteStructContext ws(cvfs, "mat_array", CV_NODE_SEQ);
	write(cvfs, "", T_Marker2KinectCCamera.inv());

	cout << "Transformation matrix (Kinect color camera to Marker) saved.\n" << T_Marker2KinectCCamera.inv() << endl;

}

//=======================================================
// キーボード入力処理関数
//=======================================================
void KeyEvent(unsigned char key, int x, int y)
{
	// ESCキーを入力したらアプリケーション終了
	if (key == 0x1b){
		printf("*** %f (frame/sec)\n", (double)countT / arUtilTimer());
		exit(0);
	}
	// マーカ同士の座標変換行列を保存
	else if (key == 's' || key == 'S'){
		if (marker.visible > 0){
			saveTransMat();

			cout << "Press any key for quit" << endl;
			getchar();
			exit(0);
		}
		else {
			printf("Couldn't find all markers, try again.\n");
		}
	}
}


//=======================================================
// マウス入力処理関数
//=======================================================
void MouseEvent(int button, int state, int x, int y)
{
	// 入力状態を表示
	printf("ボタン：%d 状態：%d 座標：(x,y)=(%d,%d) \n", button, state, x, y);
}

