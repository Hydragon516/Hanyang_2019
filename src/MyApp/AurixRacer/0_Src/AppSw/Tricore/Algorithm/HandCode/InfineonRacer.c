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
#define VALID_RATIO (0.65)

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
boolean isLaneValid = FALSE;
/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/******************************************************************************/
static int lane;			// 차선 (0 ~ 127)
static float offset;		// 차가 기준점에서 떨어진 정도. 양수(왼쪽) 음수(오른쪽)
static float angle;			// 서보모터 각도 (-0.5 ~ 0.5)
static boolean StartLaneChange = FALSE;
static int count = 0;

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
	// lane 구간 평균이 lane 제외구간 평균의 RATIO배 이상이어야 TRUE
	int average = 0;
	for(i = 0; i < lane - WIDTH; i++){
		average += IR_LineScan.adcResult[0][i];
	}
	for(i = lane; i < 127; i++){
		average += IR_LineScan.adcResult[0][i];
	}
	average = average * 8 / 120;
	if(min_sum < (average * VALID_RATIO)){
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
	/*if(isLaneValid){
		angle = 0.5 * (offset / OFFSET_MAX);
		IR_setSrvAngle(angle);
	}*/
	if(!StartLaneChange){
		if(isLaneValid){
			angle = 0.5 * (offset / OFFSET_MAX);
			IR_setSrvAngle(angle);
		}
		if((IR_AdcResult[0] * 5) > 1.5) {
			StartLaneChange = TRUE;
			IR_setSrvAngle(-0.5);
		}
	}
	else if(StartLaneChange) {
		if((IR_AdcResult[0] * 5) < 1.5) {
			/*if(isLaneValid) {
				IR_setSrvAngle(0.5);
			}*/
			count++;
			if(count > 30) {
				IR_setSrvAngle(0.5);
				if(count > 70){
					IR_setSrvAngle(0.0);
					StartLaneChange = FALSE;
					count = 0;
				}
			}
		}
	}

}

int get_lane(void){
	return lane;
}
