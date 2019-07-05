#include "AppTaskFu.h"
#include "InfineonRacer.h"

static sint32 task_cnt_1m = 0;
static sint32 task_cnt_10m = 0;
static sint32 task_cnt_100m = 0;
static sint32 task_cnt_1000m = 0;

boolean task_flag_1m = FALSE;
boolean task_flag_10m = FALSE;
boolean task_flag_100m = FALSE;
boolean task_flag_1000m = FALSE;

void appTaskfu_init(void){
	BasicLineScan_init();
	BasicPort_init();
    BasicGtmTom_init();
    BasicVadcBgScan_init();
    BasicGpt12Enc_init();
    AsclinShellInterface_init();

#if BOARD == APPLICATION_KIT_TC237
    tft_app_init(1);
    perf_meas_init();
#elif BOARD == SHIELD_BUDDY

#endif

#if CODE == CODE_HAND
    InfineonRacer_init();
#elif CODE == CODE_ERT
    IR_Controller_initialize();
#else

#endif
    /* 버튼 값 받아오기
     * 예선 결선 모드
     */
}

void appTaskfu_1ms(void)
{
	task_cnt_1m++;
	if(task_cnt_1m == 1000){
		task_cnt_1m = 0;
	}
	InfineonRacer_detectObstacle(task_cnt_1m % 10);
}


void appTaskfu_10ms(void)
{
	task_cnt_10m++;
	if(task_cnt_10m == 1000){
		task_cnt_10m = 0;
	}

	if(task_cnt_10m%2 == 0){
		BasicLineScan_run();
		InfineonRacer_detectLane(task_cnt_10m);
		BasicPort_run();
		BasicGtmTom_run();
		BasicVadcBgScan_run();

		if(IR_Ctrl.basicTest == FALSE){
			#if CODE == CODE_HAND
				InfineonRacer_control();
			#elif CODE == CODE_ERT
				IR_Controller_step();
			#else

			#endif
		}
		AsclinShellInterface_runLineScan();

		if(SpeedControlZone) {
			IR_setLed0(TRUE);
		}
		else {
			IR_setLed0(FALSE);
		}
	}

}

void appTaskfu_100ms(void)
{
	task_cnt_100m++;
	if(task_cnt_100m == 1000){
		task_cnt_100m = 0;
	}
#if BOARD == APPLICATION_KIT_TC237
	if(task_cnt_100m % REFRESH_TFT == 0){
		tft_app_run();
	}

#elif BOARD == SHIELD_BUDDY

#endif

	if(task_cnt_100m % 2 == 0) {
		int i;
		printf("SpeedControlZone : ");
		SpeedControlZone ? printf("TRUE\n") : printf("FALSE\n");
		printf("ObstacleCount : ");
		i = get_ObstacleCount();
		i ? printf("TRUE\n") : printf("FALSE\n");
		printf("StartLaneChange : ");
		i = get_StartLaneChange();
		i ? printf("TRUE\n") : printf("FALSE\n");
		printf("\n");
	}

}

void appTaskfu_1000ms(void)
{
	task_cnt_1000m++;
	if(task_cnt_1000m == 1000){
		task_cnt_1000m = 0;
	}
	//printf("Led0 : %d\n", IR_Port.led0);
	//printf("Adc[3] : %f\n", IR_AdcResult[3]);
    //printf("Adc[3] : %f\n", IR_AdcResult[3]);
}

void appTaskfu_1ms_trial(void)
{
	task_cnt_1m++;
	if(task_cnt_1m == 1000){
		task_cnt_1m = 0;
	}
}


void appTaskfu_10ms_trial(void)
{
	task_cnt_10m++;
	if(task_cnt_10m == 1000){
		task_cnt_10m = 0;
	}

	if(task_cnt_10m%2 == 0){
		BasicLineScan_run();
		InfineonRacer_detectLane_trial();
		BasicPort_run();
		BasicGtmTom_run();
		BasicVadcBgScan_run();

		if(IR_Ctrl.basicTest == FALSE){
			#if CODE == CODE_HAND
				InfineonRacer_control_trial();
			#elif CODE == CODE_ERT
				IR_Controller_step();
			#else

			#endif
		}
		AsclinShellInterface_runLineScan();
	}

}

void appTaskfu_100ms_trial(void)
{
	task_cnt_100m++;
	if(task_cnt_100m == 1000){
		task_cnt_100m = 0;
	}
#if BOARD == APPLICATION_KIT_TC237
	if(task_cnt_100m % REFRESH_TFT == 0){
		tft_app_run();
	}

#elif BOARD == SHIELD_BUDDY

#endif

	if(!FINALMODE) {
		IR_setLed0(FALSE);
	}
}

void appTaskfu_1000ms_trial(void)
{
	task_cnt_1000m++;
	if(task_cnt_1000m == 1000){
		task_cnt_1000m = 0;
	}
}

void appTaskfu_idle(void){
	AsclinShellInterface_run();
#if BOARD == APPLICATION_KIT_TC237
	perf_meas_idle();
#elif BOARD == SHIELD_BUDDY

#endif

}

void appIsrCb_1ms(void){
	BasicGpt12Enc_run();
}

