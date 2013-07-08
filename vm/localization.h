#ifndef _LOCALIZATION_
#define _LOCALIZATION_

#define WHEEL_R 39.0
#define WHEEL_D 80.0
#define PI 3.1415926535

F32 pre_phi_R = 0.0;
F32 pre_phi_L = 0.0;
F32 localization_x = 0.0;
F32 localization_y = 0.0;
F32 localization_theta = 0.0;
U8 line = 0;

extern void localization(F32 left_moter, F32 right_moter);

#endif
