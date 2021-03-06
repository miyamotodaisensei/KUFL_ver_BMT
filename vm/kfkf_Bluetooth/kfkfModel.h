/*
####################################################################################################
	name: kfkfModel.h
	Description: "ETロボコンkfkf"用プログラム
	---
	update: 2013.06.22
####################################################################################################
*/
#ifndef _KFKFMODEL_H_
#define _KFKFMODEL_H_

#include "../Common.h"


/*
===============================================================================================
	列挙型の定義
===============================================================================================
*/
/*--------------------------*/
/* 	ETロボコンkfkfのaction	*/
/*--------------------------*/
typedef enum ActType
{
	NO_TRANSITION = -1,
	DO_NOTHING = 0,
	BALANCE_STOP = 1,
	BALANCE_LINETRACE = 2,
	BALANCE_MOUSETRACE = 4,
	TAIL_RUN_FREEDOM = 5,
	/*追加機能*/
	OPOS = 6,
	OPOS_SET = 7,
	TIMER_SET = 8,
	MOTOR_SET = 9,
	PID_SET = 10,
	OPOS_TAIL = 11,
	STEP_OFFSET_SET = 12,
	RAISE_TAIL = 13,
	TAIL_LINETRACE = 14,
	PIVOT_TURN = 15,
	SELECT_LOGTYPE = 16,
	GRAY_MARKER_OFFSET = 17,
	BALANCE_RUN_FREEDOM = 18,
	SONAR_SET = 20,
	CORRECT_LOCALIZATION = 21,
	SEARCH_BOTTLE_LEFT = 22,
	SEARCH_BOTTLE_END = 23,
	SEARCH_BOTTLE_JUDGE = 24
}ActType_e;

/*--------------------------*/
/* 	ETロボコンkfkfのevent	*/
/*--------------------------*/
typedef enum EvtType
{
	AUTO = 0,
	TOUCH = 1,
	WHITE = 2,
	BLACK = 3,
	GRAY_MARKER = 4,
	STEP = 5,

	/*追加機能*/
	OPOS_END = 6,

	SONAR = 8,
	TIMER = 9,
	MOTOR_COUNT = 10,
	BT_START = 11,
	PIVOT_TURN_END = 12,
	STRAIGHT = 13,
	CURB = 14,
	H_MOUSE_CHANGE = 15,
	SONAR_255 = 7
}EvtType_e;

/*
===============================================================================================
	構造体の定義
===============================================================================================
*/
/*--------------------------*/
/* ETロボコンkfkf用構造体	*/
/*--------------------------*/
typedef struct tag_State
{
	S16 state_no;
	ActType_e action_no;
	S16 value0;
	S16 value1;
	S16 value2;
	S16 value3;
}State_t;


/*--------------------------*/
/* 	*/
/*--------------------------*/
typedef struct tag_Controller
{
	U8 touch_status;

	/* 光の状態 */
	/* 0:不明 / 1:白色上 / 2:黒色上 */
	U8 light_status;
	//S32 gray_marker_count;

	U8 target_distance;

	U8 timer_flag;
	U32 start_time;
	U32 target_time;

	U8 motor_counter_flag;
	int start_motor_count;
	int target_motor_count;

	U8 BTstart_status;

	U8 pivot_turn_flag;
	int start_pivot_turn_encoder_R;
	int target_pivot_turn_angle_R;

	/*追加機能　opos用変数　ここから*/
	U8 opos_end_flag;
	F32 opos_target_x;
	F32 opos_target_y;
	U8 opos_mode;
	S8 opos_speed;
	U8 opos_flag;
	/*ここまで*/

	/*
	U8 bottle_left_flag;
	U8 bottle_right_flag;
	U8 bottle_left_length;
	U8 bottle_right_length;
	U8 bottle_judge;
	*/
	U8 start_flag;
	U8 curb_flag;
	U8 curb_judge;
	U8 gray_flag;
	U8 dif_Light;

}Controller_t;

//--------------------------------------------------------------------
//	functions
//--------------------------------------------------------------------
//
void InitKFKF(void);
// Receive kfkf model data.
U8 ReceiveBT(void);
// Return current kfkf model state.
S16 getCurrentStateNum(void);
//
State_t getCurrentState(void);
//
void setEvent(EvtType_e event_id);
//
void clearEvent(void);
// ??
void setNextState(void);
//
S8 BluetoothStart(void);

#endif
