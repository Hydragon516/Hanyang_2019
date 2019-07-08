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
#define OFFSET_MAX (25)
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
boolean SpeedControlZone = TRUE;
boolean FINALMODE;
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

static float32 AdcResults0[10] = { 0, };
static float32 AdcResult0Sum = 0;
static float32 AdcResults1[10] = { 0, };
static float32 AdcResult1Sum = 0;
static boolean ObstacleDetected = FALSE;

static boolean EmergencyStop = FALSE;

static boolean ObstacleCount = FALSE;

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/
void InfineonRacer_init(void){
	IR_setMotor0Vol(-0.2);
	IR_setSrvAngle(0.12);

    BasicVadcBgScan_run();
    printf("<InfineonRacer_init> Adc[3] : %f\n", IR_AdcResult[3]);
#ifndef __RACEMODE__
#define __RACEMODE__
    if(IR_AdcResult[3] > 0.5) {
    	FINALMODE = FALSE;
    }
    else {
    	FINALMODE = TRUE;
    }
#endif
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

	for(i = 2; i < WIDTH + 2; i++){
		sum += IR_LineScan.adcResult[0][i];
	}
	min_sum = sum;
	max_sum = sum;
	lane = WIDTH + 1;

	for(i = WIDTH; i < 126; i++){
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
	if(!StartLaneChange) {
		int valid_sum = max_sum * VALID_RATIO;
		for(i = WIDTH + 2; i < 126; i++) {
			if(sums[i] < valid_sum) {
				// valid�� ������ min������ 15�̻� ������ ������ Ⱦ�ܺ���
				if((lane - i) > 15 || (lane - i) < -15) {
					// ������ ��ֹ��� �־��ٸ� SCZŻ��
					if(ObstacleCount) {
						SpeedControlZone = FALSE;
						if(!EmergencyStop) {
							IR_setMotor0Vol(-0.22);
						}
					}
					// lane�� �� �� �̻� ������ inValid (Ⱦ�ܺ��� ����� ���������� �����ϱ� ����)
					isLaneValid = FALSE;
					break;
				}
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
	 * angle : -0.35 ~ 0.6 (�ϵ����� 0.125�� ����)
	 */
	if(!StartLaneChange){
		if(isLaneValid){
			angle = 0.475 * (offset / OFFSET_MAX);
			angle = 0.125 + angle;
			IR_setSrvAngle(angle);
		}
		// SpeedControlZone ���� �� ��ֹ��� ������
		// �ʱ� ������ �ϰ� StartLaneChange ���� �����Ѵ�
		if(SpeedControlZone) {
			if(ObstacleDetected) {
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
					IR_setSrvAngle(0.6);
				}
			}
		}
		// SpeedControlZone�� �ƴѰ��
		// ��ֹ��� ������ �������
		else {
			if(EmergencyStop) {
				IR_setMotor0Vol(0);
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
			if(cnt > 20) {
				IR_setMotor0Vol(-0.2);
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
			//������������
			if(isFullLane) {
				if(cnt > 135) {
					IR_setSrvAngle(0);
					StartLaneChange = FALSE;
					ObstacleDetected = FALSE;
				}
				else if(cnt > 90) {
					IR_setMotor0Vol(-0.2);
				}
				else if(cnt > 70) {
					IR_setSrvAngle(0.6);
					IR_setMotor0Vol(0);
				}
			}
			//��������������
			else {
				if(cnt > 145) {
					IR_setSrvAngle(0);
					StartLaneChange = FALSE;
					ObstacleDetected = FALSE;
				}
				else if(cnt > 105) {
					IR_setMotor0Vol(-0.2);
				}
				else if(cnt > 85) {
					IR_setSrvAngle(-0.35);
					IR_setMotor0Vol(0);
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
	float32 AdcResult0 = IR_AdcResult[0];
	float32 AdcResult1 = IR_AdcResult[1];

	AdcResult0Sum -= AdcResults0[task_cnt];
	AdcResult0Sum += AdcResult0;
	AdcResults0[task_cnt] = AdcResult0;

	AdcResult1Sum -= AdcResults1[task_cnt];
	AdcResult1Sum += AdcResult1;
	AdcResults1[task_cnt] = AdcResult1;

	if(AdcResult0Sum > 5.4) {
		ObstacleDetected = TRUE;
	}

	if(!SpeedControlZone && AdcResult1Sum > 2.0) {
		EmergencyStop = TRUE;
	}
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

void InfineonRacer_detectLane_trial(void) {
	int i;
	int sum = 0;		// WIDTH ����ŭ�� LineScan.adc ������
	int min_sum;		// �ּ� ������
	int max_sum;		// �ִ� ������

	for(i = 2; i < WIDTH + 2; i++){
		sum += IR_LineScan.adcResult[0][i];
	}
	min_sum = sum;
	max_sum = sum;
	lane = WIDTH + 1;

	for(i = WIDTH; i < 126; i++){
		sum += IR_LineScan.adcResult[0][i];
		sum -= IR_LineScan.adcResult[0][i - WIDTH];
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

void InfineonRacer_control_trial(void) {
	if(isLaneValid) {
		angle = 0.475 * (offset / OFFSET_MAX);
		angle = 0.125 + angle;
		IR_setSrvAngle(angle);
	}
}
