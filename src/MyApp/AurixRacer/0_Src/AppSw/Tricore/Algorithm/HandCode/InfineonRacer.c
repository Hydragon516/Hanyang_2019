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
static int lane_right;			// 차선 (0 ~ 127)
static int lane_left;
static float offset_right;		// 차가 기준점에서 떨어진 정도. 양수(왼쪽) 음수(오른쪽)
static float offset_left;
static float angle;			// 서보모터 각도 (-0.5 ~ 0.5)

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
	/* Lane Index 계산
	 * WIDTH 개수의 LineScan.adc 합을 구하고 (구간합)
	 * 그 합이 가장 작은 구간이 Lane
	 * Lane : 0 ~ 127
	 */
	int i;
	int sum_right = 0;		// WIDTH 수만큼의 LineScan.adc 구간합
	int sum_left = 0;
	int sums_right[128];
//	int sums_left[128];
	int min_sum_right;		// 최소 구간합
	int min_sum_left;
	int max_sum_right;		// 최대 구간합
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
		// 구간 합이 최소가 되는 곳이 lane
		// 구간 중 가장 큰 값으로 설정함 (10~16이 최소구간이라면 lane = 16)
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
	/* RightLaneValid 계산
	 * min구간 평균이 max구간 평균의 VALID_RATIO배 미만이어야 Valid
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

	// 점선 실선 구분
	InfineonRacer_DotFullLane((task_cnt_10m / 2) % 40);

	/*	Speed Control Zone 계산
	 *  valid한 다른 구간이 더 존재하는 지 검사
	 *  그 구간이 min구간과 충분히 떨어져 있으면 lane이 두 개 이상임
	 */
	if(!StartLaneChange) {
		int valid_sum_right = max_sum_right * VALID_RATIO_RIGHT;
		for(i = WIDTH + 2; i < 126; i++) {
			if(sums_right[i] < valid_sum_right) {
				// valid한 구간이 min구간과 15이상 떨어져 있으면 횡단보도
				if((lane_right - i) > 15 || (lane_right - i) < -15) {
					// 이전에 장애물이 있었다면 SCZ탈출
					if(ObstacleCount) {
						SpeedControlZone = FALSE;
						if(!EmergencyStop) {
							IR_setMotor0Vol(-0.22);
						}
					}
					// lane이 두 개 이상 잡히면 inValid (횡단보도 통과시 안정적으로 주행하기 위함)
					RightLaneValid = FALSE;
					LeftLaneValid = FALSE;
					break;
				}
			}
		}
	}

	/* offset 계산
	 * 중심 STANDARD로 부터 차가 떨어진 정도
	 * OFFSET_MAX를 초과할 수 없음
	 */
	offset_right = STANDARD_RIGHT - lane_right; // 양수 : 차가 왼쪽에 있음, 음수 : 차가 오른쪽에 있음
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
	/* 일반적인 상황
	 * angle이 offset에 linear하게 비례한다
	 * angle : -0.35 ~ 0.7 (하드웨어상 0.175가 센터)
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
		// SpeedControlZone 주행 중 장애물을 만나면
		// 초기 세팅을 하고 StartLaneChange 모드로 진입한다
		if(SpeedControlZone) {
			if(ObstacleDetected) {
				ObstacleCount = TRUE;
				StartLaneChange = TRUE;
				invalid_cnt = 0;
				cnt = 0;
				IR_setMotor0Vol(0);
				// 차선변경하는 방향으로 angle을 최대한 꺾어준다
				if(isRightLane) {
					IR_setSrvAngle(-0.35);
				}
				else {
					IR_setSrvAngle(0.7);
				}
			}
		}
		// SpeedControlZone이 아닌경우
		// 장애물을 만나면 비상제동
		else {
			if(EmergencyStop) {
				IR_setMotor0Vol(0);
			}
		}
	}
	/* 차선변경하는 상황
	 * 잠시 멈췄다가 출발한다 (회전반경을 최대한 작게 하기 위함)
	 * 변경하는 차선을 읽으면 다시 반대 angle을 최대로 주었다가
	 * 일정시간이 지나면 원래 모드로 돌아간다
	 */
	else if(StartLaneChange) {
		cnt++;
		// 잠시 멈췄다가 출발
		if(IR_Motor.Motor0Vol == 0) {
			if(cnt > 20) {
				IR_setMotor0Vol(-0.2);
			}
		}
		/*
		else {
			// 왼쪽으로 차선변경
			// 차선이 감지되지 않는 경우를 count
			if(isRightLane) {
				if(!RightLaneValid) {
					invalid_cnt++;
				}
				// 반대쪽 차선이 감지되는 순간 다시 반대 angle을 주고
				if(LeftLaneValid && (invalid_cnt > 10)) {
					if(!NewLane) {
						cnt = 0;
						NewLane = TRUE;
						IR_setSrvAngle(0.6);
					}
				}
				// 일정시간이 지나면 원래 모드로 돌아간다
				if(NewLane && cnt > 20) {
					StartLaneChange = FALSE;
					NewLane = FALSE;
					ObstacleDetected = FALSE;
					IR_setSrvAngle(0.125);
				}
			}
			// 오른쪽으로 차선변경
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
			//왼쪽으로차선변경
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
			//오른쪽으로차선변경
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

/* 점선 실선을 구분해서 isRightLane에 저장
 * 일정 시간동안 LaneDetected 비율이 기준 이상이면 실선
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
		// 약 90% 이상 감지를 실선으로 간주
		if(RightLaneDetectedSum > 35) {
			isRightLane = TRUE;
		}
		else {
			isRightLane = FALSE;
		}
//		// 양쪽다 잡히지 않으면 isRightLane 유지
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
	int sum = 0;		// WIDTH 수만큼의 LineScan.adc 구간합
	int min_sum;		// 최소 구간합
	int max_sum;		// 최대 구간합

	for(i = 2; i < WIDTH + 2; i++){
		sum += IR_LineScan.adcResult[0][i];
	}
	min_sum = sum;
	max_sum = sum;
	lane_right = WIDTH + 1;

	for(i = WIDTH; i < 126; i++){
		sum += IR_LineScan.adcResult[0][i];
		sum -= IR_LineScan.adcResult[0][i - WIDTH];
		// 구간 합이 최소가 되는 곳이 lane
		// 구간 중 가장 큰 값으로 설정함 (10~16이 최소구간이라면 lane = 16)
		if(sum < min_sum){
			min_sum = sum;
			lane_right = i;
		}
		if(sum > max_sum) {
			max_sum = sum;
		}
	}
	/* isLaneValid 계산
	 * min구간 평균이 max구간 평균의 VALID_RATIO배 미만이어야 Valid
	 */
	if(min_sum < (max_sum * VALID_RATIO_RIGHT)) {
		RightLaneValid = TRUE;
	}
	else {
		RightLaneValid = FALSE;
	}

	/* offset 계산
	 * 중심 STANDARD로 부터 차가 떨어진 정도
	 * OFFSET_MAX를 초과할 수 없음
	 */
	offset_right = STANDARD_RIGHT - lane_right; // 양수 : 차가 왼쪽에 있음, 음수 : 차가 오른쪽에 있음
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
