
/******************************************************************************	*/
//	name:			opos.h
//	Description:	目的値指定型推進機構用変数及び関数
//	Details:		opos_turn:旋回値
//	Auther:			Kaoru　Beppu & Yuya Miyamoto
//	update:			2013.07.12
/******************************************************************************	*/

#ifndef _OPOS_
#define _OPOS_

#include "Common.h"
#include "localization.h"

F32 opos_turn = 0;
//F32 opos_pre_dif = 0;
//F32 opos_integral = 0;

extern void opos(F32 x, F32 y, U16 mode);
extern void init_opos();

/******************************************************************************	*/
//	name:			opos.h
//	Description:	目的値指定型推進機構用変数及び関数
//	Details:		opos_turn:旋回値
//	Auther:			Kaoru　Beppu & Yuya Miyamoto
//	update:			2013.07.12
/******************************************************************************	*/

#ifndef _OPOS_
#define _OPOS_

#include "Common.h"
#include "localization.h"

F32 opos_turn = 0;
//F32 opos_pre_dif = 0;
//F32 opos_integral = 0;

extern void opos(F32 x, F32 y, U16 mode);
extern void init_opos();

#endif
