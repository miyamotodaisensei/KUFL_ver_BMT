<<<<<<< HEAD
#include "opos.h"
#include <math.h>

/******************************************************************************	*/
//	name:			opos
//	Description:	目的値指定型推進機構
//	Details:		x:移動したいx / y:移動したいy
//					mode:0or1ならばマップ上の座標(x,y)へ移動、2or3ならばその場を原点とした(x,y)へ移動
//	Attention:		modeの2or3は現状正しく動かないため修正の必要あり
//	Auther:			Kaoru　Beppu & Yuya Miyamoto
//	update:			2013.07.12
/******************************************************************************	*/
extern void opos(F32 x, F32 y, U16 mode) {
	F32 target_theta;
	F32 opos_theta;
	F32 opos_dif;
	//F32 opos_differential;
	F32 opos_P_gain = -1;
	//F32 opos_I_gain = -2;
	//F32 opos_D_gain = -20;

	if(mode == 2 || mode == 3) {
		x += localization_x;
		y += localization_y;
	}
	target_theta = atan2(y-localization_y, x-localization_x);	// 目標地への角度を算出(atan2は-180<=θ<=180の値をとる)
	if(mode == 2 || mode == 3) {
		target_theta += localization_theta;
	}
	target_theta = target_theta * 180 / PI;						// rad -> deg

	opos_theta = localization_theta;							// 現在の角度を取得
	opos_theta = opos_theta * 180 / PI;							// rad -> deg
	opos_theta = (S32)opos_theta % 360;							// 360度を超えないようにする

	if(opos_theta > 90 && opos_theta < 360 && target_theta < 0) {
		target_theta += 360;									// target_thetaが180度以上の値をとるように補正
	}
	else if(opos_theta < -90 && opos_theta > -360 && target_theta > 0) {
		target_theta = 360 - target_theta;						// target_thetaが-180度以下の値をとるように補正
	}

	opos_dif = target_theta - opos_theta;
	//opos_differential = opos_dif - opos_pre_dif;
	//opos_integral += (opos_dif + opos_pre_dif) / 2.0 * 0.004;
	//opos_turn = opos_P_gain * opos_dif + opos_I_gain * opos_integral + opos_D_gain * opos_differential;
	opos_turn = opos_P_gain * opos_dif;							// とりあえずP制御のみ

	//opos_pre_dif = opos_dif;

	if(opos_turn > 100) {
		opos_turn = 100;
	}
	else if(opos_turn < -100) {
		opos_turn = -100;
	}

	/*
	display_clear(1);
	display_goto_xy(0, 0);
	display_string("opos:");
	display_int((int)opos_theta, 0);
	display_goto_xy(0, 1);
	display_string("target:");
	display_int((int)target_theta, 0);
	display_goto_xy(0,2);
	display_string("turn:");
	display_int((int)opos_turn, 0);
	*/
	return;
}

/******************************************************************************	*/
//	name:			init_opos
//	Description:	目的値指定型推進機構の初期化関数
//	Details:		opos関数を使用し始める前に起動
//	Attention:		今のところあまり意味なし(これから必要になるかも・・・)
//	Auther:			Kaoru　Beppu
//	update:			2013.07.12
/******************************************************************************	*/
void init_opos() {
	opos_turn = 0.0;
}
=======
#include "opos.h"
#include <math.h>

/******************************************************************************	*/
//	name:			opos
//	Description:	目的値指定型推進機構
//	Details:		x:移動したいx / y:移動したいy
//					mode:0or1ならばマップ上の座標(x,y)へ移動、2or3ならばその場を原点とした(x,y)へ移動
//	Attention:		modeの2or3は現状正しく動かないため修正の必要あり
//	Auther:			Kaoru　Beppu & Yuya Miyamoto
//	update:			2013.07.12
/******************************************************************************	*/
extern void opos(F32 x, F32 y, U16 mode) {
	F32 target_theta;
	F32 opos_theta;
	F32 opos_dif;
	//F32 opos_differential;
	F32 opos_P_gain = -1;
	//F32 opos_I_gain = -2;
	//F32 opos_D_gain = -20;

	if(mode == 2 || mode == 3) {
		x += localization_x;
		y += localization_y;
	}
	target_theta = atan2(y-localization_y, x-localization_x);	// 目標地への角度を算出(atan2は-180<=θ<=180の値をとる)
	if(mode == 2 || mode == 3) {
		target_theta += localization_theta;
	}
	target_theta = target_theta * 180 / PI;						// rad -> deg

	opos_theta = localization_theta;							// 現在の角度を取得
	opos_theta = opos_theta * 180 / PI;							// rad -> deg
	opos_theta = (S32)opos_theta % 360;							// 360度を超えないようにする

	if(opos_theta > 90 && opos_theta < 360 && target_theta < 0) {
		target_theta += 360;									// target_thetaが180度以上の値をとるように補正
	}
	else if(opos_theta < -90 && opos_theta > -360 && target_theta > 0) {
		target_theta = 360 - target_theta;						// target_thetaが-180度以下の値をとるように補正
	}

	opos_dif = target_theta - opos_theta;
	//opos_differential = opos_dif - opos_pre_dif;
	//opos_integral += (opos_dif + opos_pre_dif) / 2.0 * 0.004;
	//opos_turn = opos_P_gain * opos_dif + opos_I_gain * opos_integral + opos_D_gain * opos_differential;
	opos_turn = opos_P_gain * opos_dif;							// とりあえずP制御のみ

	//opos_pre_dif = opos_dif;

	if(opos_turn > 100) {
		opos_turn = 100;
	}
	else if(opos_turn < -100) {
		opos_turn = -100;
	}

	/*
	display_clear(1);
	display_goto_xy(0, 0);
	display_string("opos:");
	display_int((int)opos_theta, 0);
	display_goto_xy(0, 1);
	display_string("target:");
	display_int((int)target_theta, 0);
	display_goto_xy(0,2);
	display_string("turn:");
	display_int((int)opos_turn, 0);
	*/
	return;
}

/******************************************************************************	*/
//	name:			init_opos
//	Description:	目的値指定型推進機構の初期化関数
//	Details:		opos関数を使用し始める前に起動
//	Attention:		今のところあまり意味なし(これから必要になるかも・・・)
//	Auther:			Kaoru　Beppu
//	update:			2013.07.12
/******************************************************************************	*/
void init_opos() {
	opos_turn = 0.0;
}
>>>>>>> c3ff83e59237bc9c1ec50b0d96c01ba8335f6f5d
