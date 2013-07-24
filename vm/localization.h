/**********************************************************************************************	*/
//	name:			localization.h
//	Description:	自己位置推定用変数及び関数
//	Details:		現在地は(localization_x, localizaton_y)、車体の向きはlocalization_theta
//	Auther:			Kaoru　Beppu
//	update:			2013.07.12
/**********************************************************************************************	*/

#ifndef _LOCALIZATION_
#define _LOCALIZATION_

/* マクロ定義 */
#define WHEEL_R 40.0			// 車輪の半径
#define WHEEL_D 81.0			// ライトセンサ(中心)から車輪までの距離
#define PI 3.1415926535			// π
#define IN_START_X 350.0
#define IN_START_Y 496.0
#define OUT_START_X 0.0
#define OUT_START_Y	0.0

/* グローバル変数 */
F32 pre_phi_R = 0.0;			// 内部用変数
F32 pre_phi_L = 0.0;			// 内部用変数
F32 localization_x = 0.0;		// 自己位置推定x[mm]
F32 localization_y = 0.0;		// 自己位置推定y[mm]
F32 localization_theta = 0.0;	// 自己位置推定θ[rad]
//U8 line = 0;

/* 関数 */
extern void localization(F32 left_moter, F32 right_moter);
extern void init_localization();
extern void correct_localization(F32 x, F32 y, F32 theta, F32 mode);	//　アクションに追加したい

#endif
