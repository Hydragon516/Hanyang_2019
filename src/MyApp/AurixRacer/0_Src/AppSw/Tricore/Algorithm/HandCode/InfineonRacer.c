/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include "InfineonRacer.h"
#include "Basic.h"

/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/
#define WIDTH (7)
#define STANDARD_RIGHT (64)
#define STANDARD_LEFT (70)
#define OFFSET_MAX (25)
#define VALID_RATIO_RIGHT (0.4)
#define VALID_RATIO_LEFT (0.5)

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
boolean RightLaneValid = FALSE;
boolean LeftLaneValid = FALSE;
boolean SpeedControlZone = TRUE;
boolean FINALMODE;
/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/******************************************************************************/
static int lane_right;			// ���� (0 ~ 127)
static int lane_left;
static float offset_right;		// ���� ���������� ������ ����. ���(����) ����(������)
static float offset_left;
static float angle;			// �������� ���� (-0.5 ~ 0.5)

static boolean StartLaneChange = FALSE;
static int invalid_cnt = 0;
static int cnt = 0;
//static boolean NewLane = FALSE;
static boolean isRightLane = TRUE;

static boolean RightLaneDetected[40] = { 0, };
static boolean LeftLaneDetected[40] = { 0, };
static int RightLaneDetectedSum = 0;
static int LeftLaneDetectedSum = 0;

static float32 AdcResults0[5] = { 0, };
static float32 AdcResult0Sum = 0;
static float32 AdcResults1[5] = { 0, };
static float32 AdcResult1Sum = 0;
static boolean ObstacleDetected = FALSE;

static boolean EmergencyStop = FALSE;

static boolean ObstacleCount = FALSE;

static boolean LeftCameraLock = FALSE;
static boolean RightCameraLock = FALSE;

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/
void InfineonRacer_init(void){
	IR_setMotor0Vol(-0.2);
	IR_setSrvAngle(0.2);

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
	int sum_right = 0;		// WIDTH ����ŭ�� LineScan.adc ������
	int sum_left = 0;
	int sums_right[128];
//	int sums_left[128];
	int min_sum_right;		// �ּ� ������
	int min_sum_left;
	int max_sum_right;		// �ִ� ������
	int max_sum_left;

	for(i = 2; i < WIDTH + 2; i++){
		sum_right += IR_LineScan.adcResult[0][i];
	}
	min_sum_right = sum_right;
	max_sum_right = sum_right;
	lane_right = WIDTH + 1;
	for(i = 20; i < WIDTH + 20; i++) {
		sum_left += IR_LineScan.adcResult[1][i];
	}
	min_sum_left = sum_left;
	max_sum_left = sum_left;
	lane_left = WIDTH + 19;

	for(i = WIDTH + 2; i < 126; i++){
		sum_right += IR_LineScan.adcResult[0][i];
		sum_right -= IR_LineScan.adcResult[0][i - WIDTH];
		sums_right[i] = sum_right;
		// ���� ���� �ּҰ� �Ǵ� ���� lane
		// ���� �� ���� ū ������ ������ (10~16�� �ּұ����̶�� lane = 16)
		if(sum_right < min_sum_right){
			min_sum_right = sum_right;
			lane_right = i;
		}
		else if(sum_right > max_sum_right) {
			max_sum_right = sum_right;
		}
	}
	for(i = WIDTH + 20; i < 120; i++) {
		sum_left += IR_LineScan.adcResult[1][i];
		sum_left -= IR_LineScan.adcResult[1][i - WIDTH];
//		sums_left[i] = sum_left;

		if(sum_left < min_sum_left) {
			min_sum_left = sum_left;
			lane_left = i;
		}
		else if(sum_left > max_sum_right) {
			max_sum_left = sum_left;
		}
	}
	/* RightLaneValid ���
	 * min���� ����� max���� ����� VALID_RATIO�� �̸��̾�� Valid
	 */
	if(min_sum_right < (max_sum_right * VALID_RATIO_RIGHT)) {
		RightLaneValid = TRUE;
	}
	else {
		RightLaneValid = FALSE;
	}
	if(min_sum_left < (max_sum_left * VALID_RATIO_LEFT)) {
		LeftLaneValid = TRUE;
	}
	else {
		LeftLaneValid = FALSE;
	}

	// ���� �Ǽ� ����
	InfineonRacer_DotFullLane((task_cnt_10m / 2) % 40);

	/*	Speed Control Zone ���
	 *  valid�� �ٸ� ������ �� �����ϴ� �� �˻�
	 *  �� ������ min������ ����� ������ ������ lane�� �� �� �̻���
	 */
	if(!StartLaneChange) {
		int valid_sum_right = max_sum_right * VALID_RATIO_RIGHT;
		for(i = WIDTH + 2; i < 126; i++) {
			if(sums_right[i] < valid_sum_right) {
				// valid�� ������ min������ 15�̻� ������ ������ Ⱦ�ܺ���
				if((lane_right - i) > 15 || (lane_right - i) < -15) {
					// ������ ��ֹ��� �־��ٸ� SCZŻ��
					if(ObstacleCount) {
						SpeedControlZone = FALSE;
						if(!EmergencyStop) {
							IR_setMotor0Vol(-0.22);
						}
					}
					// lane�� �� �� �̻� ������ inValid (Ⱦ�ܺ��� ����� ���������� �����ϱ� ����)
					RightLaneValid = FALSE;
					LeftLaneValid = FALSE;
					break;
				}
			}
		}
	}

	/* offset ���
	 * �߽� STANDARD�� ���� ���� ������ ����
	 * OFFSET_MAX�� �ʰ��� �� ����
	 */
	offset_right = STANDARD_RIGHT - lane_right; // ��� : ���� ���ʿ� ����, ���� : ���� �����ʿ� ����
	offset_left = lane_left - STANDARD_LEFT;
	if(offset_right > OFFSET_MAX){
		offset_right = OFFSET_MAX;
	}
	else if(offset_right < -(OFFSET_MAX))
	{
		offset_right = -(OFFSET_MAX);
	}
	if(offset_left > OFFSET_MAX) {
		offset_left = OFFSET_MAX;
	}
	else if(offset_left < -(OFFSET_MAX))
	{
		offset_left = -(OFFSET_MAX);
	}
}

void InfineonRacer_control(void){
	/* �Ϲ����� ��Ȳ
	 * angle�� offset�� linear�ϰ� ����Ѵ�
	 * angle : -0.35 ~ 0.7 (�ϵ����� 0.175�� ����)
	 */
	if(!StartLaneChange){
		if(RightLaneValid && !RightCameraLock) {
			angle = 0.525 * (offset_right / OFFSET_MAX);
			angle = 0.175 + angle;
			IR_setSrvAngle(angle);
			LeftCameraLock = FALSE;
		}
		else if(LeftLaneValid && !LeftCameraLock) {
			angle = 0.525 * (offset_left / OFFSET_MAX);
			angle = 0.175 + angle;
			IR_setSrvAngle(angle);
			RightCameraLock = FALSE;
		}
		if(angle < -0.33 && !isRightLane) {
			LeftCameraLock = TRUE;
		}
		else if(angle > 0.68 && isRightLane) {
			RightCameraLock = TRUE;
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
				if(isRightLane) {
					IR_setSrvAngle(-0.35);
				}
				else {
					IR_setSrvAngle(0.7);
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
			// �������� ��������
			// ������ �������� �ʴ� ��츦 count
			if(isRightLane) {
				if(!RightLaneValid) {
					invalid_cnt++;
				}
				// �ݴ��� ������ �����Ǵ� ���� �ٽ� �ݴ� angle�� �ְ�
				if(LeftLaneValid && (invalid_cnt > 10)) {
					if(!NewLane) {
						cnt = 0;
						NewLane = TRUE;
						IR_setSrvAngle(0.6);
					}
				}
				// �����ð��� ������ ���� ���� ���ư���
				if(NewLane && cnt > 20) {
					StartLaneChange = FALSE;
					NewLane = FALSE;
					ObstacleDetected = FALSE;
					IR_setSrvAngle(0.125);
				}
			}
			// ���������� ��������
			else {
				if(!LeftLaneValid) {
					invalid_cnt++;
				}
				if(RightLaneValid && (invalid_cnt > 10)) {
					if(!NewLane) {
						cnt = 0;
						NewLane = TRUE;
						IR_setSrvAngle(-0.35);
					}
				}
				if(NewLane && cnt > 30) {
					StartLaneChange = FALSE;
					NewLane = FALSE;
					ObstacleDetected = FALSE;
					IR_setSrvAngle(0.125);
				}
			}
		}
		*/

		else {
			//����������������
			if(isRightLane) {
				if(cnt > 140) {
					IR_setSrvAngle(0);
					StartLaneChange = FALSE;
					ObstacleDetected = FALSE;
				}
				else if(cnt > 95) {
					IR_setMotor0Vol(-0.2);
				}
				else if(cnt > 75) {
					IR_setSrvAngle(0.7);
					IR_setMotor0Vol(0);
				}
			}
			//������������������
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

	if(LeftCameraLock || RightCameraLock) {
		IR_setLed0(TRUE);
	}
	else {
		IR_setLed0(FALSE);
	}
}

/* ���� �Ǽ��� �����ؼ� isRightLane�� ����
 * ���� �ð����� LaneDetected ������ ���� �̻��̸� �Ǽ�
 */
void InfineonRacer_DotFullLane(sint32 task_cnt) {
	if(!StartLaneChange) {
		RightLaneDetectedSum -= RightLaneDetected[task_cnt];
		RightLaneDetected[task_cnt] = RightLaneValid;
		RightLaneDetectedSum += RightLaneDetected[task_cnt];

		LeftLaneDetectedSum -= LeftLaneDetected[task_cnt];
		LeftLaneDetected[task_cnt] = LeftLaneValid;
		LeftLaneDetectedSum += LeftLaneDetected[task_cnt];
		// RightLaneDetectedSum : 0 ~ 40
		// �� 90% �̻� ������ �Ǽ����� ����
		if(RightLaneDetectedSum > 35) {
			isRightLane = TRUE;
		}
		else {
			isRightLane = FALSE;
		}
//		// ���ʴ� ������ ������ isRightLane ����
//		else {
//			;
//		}
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

	if(AdcResult0Sum > 2.7) {
		ObstacleDetected = TRUE;
	}

	if(!SpeedControlZone && AdcResult1Sum > 1.0) {
		EmergencyStop = TRUE;
	}

		printf("Adc0 : %f\n", AdcResult0Sum);
}

boolean get_StartLaneChange(void) {
	return StartLaneChange;
}

boolean get_ObstacleCount(void) {
	return ObstacleCount;
}

int get_lane(void){
	return lane_right;
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
	lane_right = WIDTH + 1;

	for(i = WIDTH; i < 126; i++){
		sum += IR_LineScan.adcResult[0][i];
		sum -= IR_LineScan.adcResult[0][i - WIDTH];
		// ���� ���� �ּҰ� �Ǵ� ���� lane
		// ���� �� ���� ū ������ ������ (10~16�� �ּұ����̶�� lane = 16)
		if(sum < min_sum){
			min_sum = sum;
			lane_right = i;
		}
		if(sum > max_sum) {
			max_sum = sum;
		}
	}
	/* isLaneValid ���
	 * min���� ����� max���� ����� VALID_RATIO�� �̸��̾�� Valid
	 */
	if(min_sum < (max_sum * VALID_RATIO_RIGHT)) {
		RightLaneValid = TRUE;
	}
	else {
		RightLaneValid = FALSE;
	}

	/* offset ���
	 * �߽� STANDARD�� ���� ���� ������ ����
	 * OFFSET_MAX�� �ʰ��� �� ����
	 */
	offset_right = STANDARD_RIGHT - lane_right; // ��� : ���� ���ʿ� ����, ���� : ���� �����ʿ� ����
	if(offset_right > OFFSET_MAX){
		offset_right = OFFSET_MAX;
	}
	else if(offset_right < -(OFFSET_MAX))
	{
		offset_right = -(OFFSET_MAX);
	}
}

void InfineonRacer_control_trial(void) {
	if(RightLaneValid) {
		angle = 0.525 * (offset_right / OFFSET_MAX);
		angle = 0.175 + angle;
		IR_setSrvAngle(angle);
	}
}
