/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include "InfineonRacer.h"
#include "Basic.h"

/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/
#define WIDTH (7)
#define STANDARD (64)
#define OFFSET_MAX (20)
#define VALID_RATIO (0.4)

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
static int invalid_cnt = 0;
static int cnt = 0;
//static boolean NewLane = FALSE;
static boolean isFullLane = TRUE;

static boolean LaneDetected[50] = { 0, };
static int LaneDetectedSum = 0;

static float32 AdcResults[10] = { 0, };
static boolean ObstacleDetected = FALSE;
static float32 AdcResultSum = 0;

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/
void InfineonRacer_init(void){
	IR_setMotor0Vol(-0.18);
}

void InfineonRacer_detectLane(void){
	/* Lane Index ���
	 * WIDTH ������ LineScan.adc ���� ���ϰ� (������)
	 * �� ���� ���� ���� ������ Lane
	 * Lane : 0 ~ 127
	 */
	int i;
	int sum = 0;		// WIDTH ����ŭ�� LineScan.adc ������
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
		// ���� �� ���� ū ������ ������ (10~16�� �ּұ����̶�� lane = 16)
		if(sum < min_sum){
			min_sum = sum;
			lane = i;
		}
	}
	/* isLaneValid ���
	 * lane ���� ����� lane ���ܱ��� ����� VALID_RATIO�� �̻��̾�� TRUE
	 */
	int average = 0;
	for(i = 0; i < lane - WIDTH; i++){
		average += IR_LineScan.adcResult[0][i];
	}
	for(i = lane; i < 128; i++){
		average += IR_LineScan.adcResult[0][i];
	}
	average = average * WIDTH / (128 - WIDTH);
	//printf("min_sum : %d\n", min_sum);
	//printf("average : %d\n", average);
	if(min_sum < (average * VALID_RATIO)){
		isLaneValid = TRUE;
		//printf("Lane : Valid\n");
	}
	else{
		isLaneValid = FALSE;
		//printf("Lane : inValid\n");
	}


	/* offset ���
	 * �߽� STANDARD�� ���� ���� ������ ����
	 * OFFSET_MAX�� �ʰ��� �� ����
	 */
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
	/* �Ϲ����� ��Ȳ
	 * angle�� offset�� linear�ϰ� ����Ѵ�
	 */
	if(!StartLaneChange){
		if(isLaneValid){
			angle = 0.45 * (offset / OFFSET_MAX);
			angle = 0.1 + angle;
			IR_setSrvAngle(angle);
		}
		// ���� �� ��ֹ��� ������
		// �ʱ� ������ �ϰ� StartLaneChange ���� �����Ѵ�
		if(ObstacleDetected) {
			StartLaneChange = TRUE;
			invalid_cnt = 0;
			cnt = 0;
			IR_setMotor0Vol(0);
			// ���������ϴ� �������� angle�� �ִ��� �����ش�
			if(isFullLane) {
				IR_setSrvAngle(-0.35);
			}
			else {
				IR_setSrvAngle(0.55);
			}
		}
	}
	/* ���������ϴ� ��Ȳ
	 * ��� ����ٰ� ����Ѵ� (ȸ���ݰ��� �ִ��� �۰� �ϱ� ����)
	 * �����ϴ� ������ ������ �ٽ� �ݴ� angle�� �ִ�� �־��ٰ�
	 * �����ð��� ������ ���� ���� ���ư���
	 */
	else if(StartLaneChange) {
		cnt++;
		// ��� ����ٰ� ���
		if(IR_Motor.Motor0Vol == 0) {
			if(cnt > 10) {
				IR_setMotor0Vol(-0.18);
			}
		}
		/*
		else {
			// ������ �������� �ʴ� ��츦 count
			if(!isLaneValid) {
				invalid_cnt++;
			}
			// ������ ������ ���
			// ������ �������� ���� ��Ȳ�� ����� �� (���������� �ƴ� ���ο� ����)
			// ������ �����Ǵ� ���� �ٽ� �ݴ� angle�� �ְ�
			else {
				if(invalid_cnt > 20) {
					if(!NewLane) {
						cnt = 0;
						NewLane = TRUE;
					}
					if(isFullLane) {
						IR_setSrvAngle(0.55);
					}
					else {
						IR_setSrvAngle(-0.35);
					}
				}
			}
			// ���ο� ������ ����(�ݴ� angle�� �ְ�) �����ð��� ������ ���� ���� ���ư���
			if(NewLane && cnt > 20) {
				IR_setSrvAngle(0);
				StartLaneChange = FALSE;
				NewLane = FALSE;
				ObstacleDetected = FALSE;
			}
		}
		*/
		else {
			if(cnt > 120) {
				IR_setSrvAngle(0);
				StartLaneChange = FALSE;
				ObstacleDetected = FALSE;
			}
			else if(cnt > 65) {
				if(isFullLane) {
					IR_setSrvAngle(0.55);
				}
				else {
					IR_setSrvAngle(-0.35);
				}
			}
		}
	}
}

int get_lane(void){
	return lane;
}

/* ���� �Ǽ��� �����ؼ� isFullLane�� ����
 * ���� �ð����� LaneDetected ������ ���� �̻��̸� �Ǽ�
 */
void InfineonRacer_DotFullLane(sint32 task_cnt) {
	if(!StartLaneChange) {
		LaneDetectedSum -= LaneDetected[task_cnt];
		LaneDetected[task_cnt] = isLaneValid;
		LaneDetectedSum += LaneDetected[task_cnt];

		// LaneDetectedSum : 0 ~ 50
		// 90% �̻� ������ �Ǽ����� ����
		if(LaneDetectedSum > 40) {
			isFullLane = TRUE;
		}
		else {
			isFullLane = FALSE;
		}
	}
}

void InfineonRacer_detectObstacle(sint32 task_cnt) {
	float32 AdcResult = IR_AdcResult[0];
	AdcResultSum -= AdcResults[task_cnt];
	AdcResultSum += AdcResult;
	AdcResults[task_cnt] = IR_AdcResult[0];
	if(AdcResultSum > 2.5) {
		ObstacleDetected = TRUE;
	}
//	if(task_cnt == 0) {
//		printf("AdcResultSum : %f\n", AdcResultSum);
//	}
}
