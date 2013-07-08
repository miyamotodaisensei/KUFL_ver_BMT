#include <math.h>
#include "Common.h"
#include "localization.h"

extern void localization(F32 left_moter, F32 right_moter) {
	F32 phi_R;
	F32 phi_L;
	F32 delta_L_R;
	F32 delta_L_L;
	F32 delta_L;
	F32 delta_theta;
	F32 rho;

	phi_R = right_moter - pre_phi_R;
	phi_L = left_moter - pre_phi_L;
	pre_phi_R = right_moter;
	pre_phi_L = left_moter;
	delta_L_R = 2 * PI * WHEEL_R * phi_R / 360;
	delta_L_L = 2 * PI * WHEEL_R * phi_L / 360;
	delta_L = (delta_L_R + delta_L_L) / 2;
	delta_theta = (delta_L_R - delta_L_L) / (2 * WHEEL_D);
	//delta_theta = (delta_L_R) / (2 * WHEEL_D);
	rho = delta_L / delta_theta;

	if(delta_theta < 0.0001) {
		localization_x = localization_x + delta_L * cos(localization_theta + delta_theta);
		localization_y = localization_y + delta_L * sin(localization_theta + delta_theta);
		line = 0;
	}
	else {
		delta_L = 2 * rho * sin(delta_theta / 2);
		localization_x = localization_x + delta_L * cos(localization_theta + delta_theta / 2);
		localization_y = localization_y + delta_L * sin(localization_theta + delta_theta / 2);
		line = 1;
	}
	localization_theta = localization_theta + delta_theta;
	/*
	display_clear(1);
	display_goto_xy(0, 0);
	display_string("x:");
	display_int((int)localization_x, 0);
	display_goto_xy(0, 1);
	display_string("y:");
	display_int((int)localization_y, 0);
	display_goto_xy(0,2);
	display_string("theta:");
	display_int((int)localization_theta, 0);
	display_goto_xy(0, 3);
	display_string("localization");
	display_goto_xy(0, 4);
	display_string("L_L:");
	display_int((int)delta_L_L, 0);
	display_goto_xy(0, 5);
	display_string("L_R:");
	display_int((int)delta_L_R, 0);
	display_goto_xy(0, 6);
	display_string("L:");
	display_int((int)delta_L, 0);
	display_goto_xy(0, 7);
	display_string("theta:");
	display_int((int)delta_theta, 0);
	*/
	return;
}
