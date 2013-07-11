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
