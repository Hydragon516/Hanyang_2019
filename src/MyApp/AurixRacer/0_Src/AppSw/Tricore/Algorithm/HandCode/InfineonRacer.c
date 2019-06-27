/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include "InfineonRacer.h"
#include "Basic.h"

/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/
#define WIDTH (8)
#define STANDARD (64)
#define OFFSET_MAX (20)
#define SUM_RATIO (0.65)

/******************************************************************************/
/*--------------------------------Enumerations--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/

InfineonRacer_t IR_Ctrl  /**< \brief  global data */
		={64, 64, FALSE  };

boolean isLaneValid = TRUE;
/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/******************************************************************************/
static int lane;
static float offset;
static float angle;

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/
void InfineonRacer_init(void){
	;
}

void InfineonRacer_detectLane(void){
	/* IR_LineScan.adcResult 의 정보를 읽어들여서
	 * IR_Ctrl.Ls0Margin, IR_Ctrl.Ls1Margin 정보를 계산한다
	 */

	/*****************lane index 계산************************/
	// lane : 구간합 최소일 때 index, 구간에서 가장 큰 index로 설정함 (WIDTH-1 ~ 127)
	int i;
	int sum = 0;		// WIDTH 구간만큼의 LineScan Result 부분합
	int min_sum;		// 최소 구간합

	for(i = 0; i < WIDTH; i++){
		sum += IR_LineScan.adcResult[0][i];
	}
	min_sum = sum;
	lane = WIDTH - 1;

	for(i = WIDTH; i < 128; i++){
		sum += IR_LineScan.adcResult[0][i];
		sum -= IR_LineScan.adcResult[0][i - WIDTH];
		// 구간 합이 최소가 되는 곳이 lane
		if(sum < min_sum){
			min_sum = sum;
			lane = i;
		}
	}
	// isLaneValid 계산
	int average = 0;
	for(i = 0; i < lane - WIDTH; i++){
		average += IR_LineScan.adcResult[0][i];
	}
	for(i = lane; i < 127; i++){
		average += IR_LineScan.adcResult[0][i];
	}
	average = average * 8 / 120;
	if(min_sum < (average * SUM_RATIO)){
		isLaneValid = TRUE;
	}
	else{
		isLaneValid = FALSE;
	}

	/************** offset 계산 ******************/
	offset = STANDARD - lane; // 양수 : 차가 왼쪽에 있음, 음수 : 차가 오른쪽에 있음
	if(offset > OFFSET_MAX){
		offset = OFFSET_MAX;
	}
	else if(offset < -(OFFSET_MAX))
	{
		offset = -(OFFSET_MAX);
	}

}

void InfineonRacer_control(void){
	// angle이 offset에 linear 비례
	if(isLaneValid){
		angle = 0.5 * (offset / OFFSET_MAX);
		IR_setSrvAngle(angle);
	}
}

int get_lane(void){
	return lane;
}
