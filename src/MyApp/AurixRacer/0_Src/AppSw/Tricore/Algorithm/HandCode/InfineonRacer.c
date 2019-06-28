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
static int lane;			// ���� (0 ~ 127)
static float offset;		// ���� ���������� ������ ����. ���(����) ����(������)
static float angle;			// �������� ���� (-0.5 ~ 0.5)
static boolean StartLaneChange = FALSE;
static int count = 0;

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
	// lane ���� ����� lane ���ܱ��� ����� RATIO�� �̻��̾�� TRUE
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
