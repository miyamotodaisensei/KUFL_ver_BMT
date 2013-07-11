#include "opos.h"
#include <math.h>

/******************************************************************************
** 関数	|	unsigned long num2str(long num, char str[], unsigned long offset)
** 概要	|	long型を文字列に変換する関数
** 引数	|	num		:	変換する数字(型が違う場合long型に変換する)
**		|	str		:	変換した文字列を格納する配列
**		|	offset	:	文字列のオフセット(通常0)
** 戻値	|	変換されて出来た文字列の文字数
*******************************************************************************	*/
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
	target_theta = atan2(y-localization_y, x-localization_x);
	if(mode == 2 || mode == 3) {
		target_theta += localization_theta;
	}
	target_theta = target_theta * 180 / PI;

	opos_theta = localization_theta;
	opos_theta = opos_theta * 180 / PI;
	opos_theta = (S32)opos_theta % 360;

	opos_dif = target_theta - opos_theta;
	//opos_differential = opos_dif - opos_pre_dif;
	//opos_integral += (opos_dif + opos_pre_dif) / 2.0 * 0.004;
	//opos_turn = opos_P_gain * opos_dif + opos_I_gain * opos_integral + opos_D_gain * opos_differential;
	opos_turn = opos_P_gain * opos_dif;

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

void init_opos() {
	opos_turn = 0.0;
}
