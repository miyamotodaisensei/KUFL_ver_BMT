#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "Common.h"

#define ROTATE_BUFFER_LENGTH_MAX 250

typedef struct tag_Sensor
{
	U16 light;
	U16 light_ave;

	U16 gyro;

	U8 touch;
	S32 distance;
	int count_left;
	int count_right;
	U16 rotate_left[ROTATE_BUFFER_LENGTH_MAX];
	U16 rotate_right[ROTATE_BUFFER_LENGTH_MAX];
	U16 rotate_left_ave;
	U16 rotate_right_ave;
	U16 count_tail;
	U16 battery;
	U16 realblack;
	U16 realwhite;
	/*
	U8 bottle_is_left;
	U8 bottle_is_right;
	*/
	U8 BTstart;
}Sensor_t;

#endif

