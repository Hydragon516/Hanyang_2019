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
static int lane;			// 차선 (0 ~ 127)
static float offset;		// 차가 기준점에서 떨어진 정도. 양수(왼쪽) 음수(오른쪽)
static float angle;			// 서보모터 각도 (-0.5 ~ 0.5)

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
	/* Lane Index 계산
	 * WIDTH 개수의 LineScan.adc 합을 구하고 (구간합)
	 * 그 합이 가장 작은 구간이 Lane
	 * Lane : 0 ~ 127
	 */
	int i;
	int sum = 0;		// WIDTH 수만큼의 LineScan.adc 구간합
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
		// 구간 중 가장 큰 값으로 설정함 (10~16이 최소구간이라면 lane = 16)
		if(sum < min_sum){
			min_sum = sum;
			lane = i;
		}
	}
	/* isLaneValid 계산
	 * lane 구간 평균이 lane 제외구간 평균의 VALID_RATIO배 이상이어야 TRUE
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


	/* offset 계산
	 * 중심 STANDARD로 부터 차가 떨어진 정도
	 * OFFSET_MAX를 초과할 수 없음
	 */
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
	/* 일반적인 상황
	 * angle이 offset에 linear하게 비례한다
	 */
	if(!StartLaneChange){
		if(isLaneValid){
			angle = 0.45 * (offset / OFFSET_MAX);
			angle = 0.1 + angle;
			IR_setSrvAngle(angle);
		}
		// 주행 중 장애물을 만나면
		// 초기 세팅을 하고 StartLaneChange 모드로 진입한다
		if(ObstacleDetected) {
			StartLaneChange = TRUE;
			invalid_cnt = 0;
			cnt = 0;
			IR_setMotor0Vol(0);
			// 차선변경하는 방향으로 angle을 최대한 꺾어준다
			if(isFullLane) {
				IR_setSrvAngle(-0.35);
			}
			else {
				IR_setSrvAngle(0.55);
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
			if(cnt > 10) {
				IR_setMotor0Vol(-0.18);
			}
		}
		/*
		else {
			// 차선이 감지되지 않는 경우를 count
			if(!isLaneValid) {
				invalid_cnt++;
			}
			// 차선이 감지된 경우
			// 차선이 감지되지 않은 상황이 충분할 때 (기존차선이 아닌 새로운 차선)
			// 차선이 감지되는 순간 다시 반대 angle을 주고
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
			// 새로운 차선을 감지(반대 angle을 주고) 일정시간이 지나면 원래 모드로 돌아간다
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

/* 점선 실선을 구분해서 isFullLane에 저장
 * 일정 시간동안 LaneDetected 비율이 기준 이상이면 실선
 */
void InfineonRacer_DotFullLane(sint32 task_cnt) {
	if(!StartLaneChange) {
		LaneDetectedSum -= LaneDetected[task_cnt];
		LaneDetected[task_cnt] = isLaneValid;
		LaneDetectedSum += LaneDetected[task_cnt];

		// LaneDetectedSum : 0 ~ 50
		// 90% 이상 감지를 실선으로 간주
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
