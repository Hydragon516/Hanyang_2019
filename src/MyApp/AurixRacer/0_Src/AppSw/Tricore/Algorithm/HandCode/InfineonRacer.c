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
boolean SpeedControlZone = FALSE;
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

static boolean ObstacleCount = FALSE;

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/
void InfineonRacer_init(void){
	IR_setMotor0Vol(-0.18);
    printf("Adc[3] : %f\n", IR_AdcResult[3]);
}

void InfineonRacer_detectLane(sint32 task_cnt_10m){
	/* Lane Index ���
	 * WIDTH ������ LineScan.adc ���� ���ϰ� (������)
	 * �� ���� ���� ���� ������ Lane
	 * Lane : 0 ~ 127
	 */
	int i;
	int sum = 0;		// WIDTH ����ŭ�� LineScan.adc ������
	int sums[128];
	int min_sum;		// �ּ� ������
	int max_sum;		// �ִ� ������

	for(i = 0; i < WIDTH; i++){
		sum += IR_LineScan.adcResult[0][i];
	}
	min_sum = sum;
	max_sum = sum;
	lane = WIDTH - 1;

	for(i = WIDTH; i < 128; i++){
		sum += IR_LineScan.adcResult[0][i];
		sum -= IR_LineScan.adcResult[0][i - WIDTH];
		sums[i] = sum;
		// ���� ���� �ּҰ� �Ǵ� ���� lane
		// ���� �� ���� ū ������ ������ (10~16�� �ּұ����̶�� lane = 16)
		if(sum < min_sum){
			min_sum = sum;
			lane = i;
		}
		if(sum > max_sum) {
			max_sum = sum;
		}
	}
	/* isLaneValid ���
	 * min���� ����� max���� ����� VALID_RATIO�� �̸��̾�� Valid
	 */
	if(min_sum < (max_sum * VALID_RATIO)) {
		isLaneValid = TRUE;
	}
	else {
		isLaneValid = FALSE;
	}

	// ���� �Ǽ� ����
	InfineonRacer_DotFullLane((task_cnt_10m / 2) % 50);

	/*	Speed Control Zone ���
	 *  valid�� �ٸ� ������ �� �����ϴ� �� �˻�
	 *  �� ������ min������ ����� ������ ������ lane�� �� �� �̻���
	 */
	int valid_sum = max_sum * VALID_RATIO;
	for(i = WIDTH; i < 128; i++) {
		if(sums[i] < valid_sum) {
			// valid�� ������ min������ 15�̻� ������ ������ Ⱦ�ܺ���
			if((lane - i) > 15 || (lane - i) < -15) {
				// ������ ��ֹ��� �־��ٸ� SCZŻ��
				if(ObstacleCount) {
					SpeedControlZone = FALSE;
				}
				// ������ ��ֹ��� �� ���� �����ٸ� SCZ ����
				else {
					SpeedControlZone = TRUE;
				}
				// lane�� �� �� �̻� ������ inValid (Ⱦ�ܺ��� ����� ���������� �����ϱ� ����)
				isLaneValid = FALSE;
				break;
			}
		}
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
	 * angle : -0.35 ~ 0.55 (�ϵ����� 0.1�� ����)
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
			// SCZ�� �ƴ϶�� �������
			if(!SpeedControlZone) {
				IR_setMotor0Vol(0);
				IR_setMotor0En(0);
			}
			else {
				ObstacleCount = TRUE;
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
			if(cnt > 110) {
				IR_setSrvAngle(0);
				StartLaneChange = FALSE;
				ObstacleDetected = FALSE;
			}
			else if(cnt > 60) {
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
	if(AdcResultSum > 5.8) {
		ObstacleDetected = TRUE;
	}
//	if(task_cnt == 0) {
//		printf("AdcResultSum : %f\n", AdcResultSum);
//	}
}

boolean get_StartLaneChange(void) {
	return StartLaneChange;
}

boolean get_ObstacleCount(void) {
	return ObstacleCount;
}

int get_lane(void){
	return lane;
}
