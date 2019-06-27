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
	/* IR_LineScan.adcResult �� ������ �о�鿩��
	 * IR_Ctrl.Ls0Margin, IR_Ctrl.Ls1Margin ������ ����Ѵ�
	 */

	/*****************lane index ���************************/
	// lane : ������ �ּ��� �� index, �������� ���� ū index�� ������ (WIDTH-1 ~ 127)
	int i;
	int sum = 0;		// WIDTH ������ŭ�� LineScan Result �κ���
	int min_sum;		// �ּ� ������

	for(i = 0; i < WIDTH; i++){
		sum += IR_LineScan.adcResult[0][i];
	}
	min_sum = sum;
	lane = WIDTH - 1;

	for(i = WIDTH; i < 128; i++){
		sum += IR_LineScan.adcResult[0][i];
		sum -= IR_LineScan.adcResult[0][i - WIDTH];
		// ���� ���� �ּҰ� �Ǵ� ���� lane
		if(sum < min_sum){
			min_sum = sum;
			lane = i;
		}
	}
	// isLaneValid ���
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

	/************** offset ��� ******************/
	offset = STANDARD - lane; // ��� : ���� ���ʿ� ����, ���� : ���� �����ʿ� ����
	if(offset > OFFSET_MAX){
		offset = OFFSET_MAX;
	}
	else if(offset < -(OFFSET_MAX))
	{
		offset = -(OFFSET_MAX);
	}

}

void InfineonRacer_control(void){
	// angle�� offset�� linear ���
	if(isLaneValid){
		angle = 0.5 * (offset / OFFSET_MAX);
		IR_setSrvAngle(angle);
	}
}

int get_lane(void){
	return lane;
}
