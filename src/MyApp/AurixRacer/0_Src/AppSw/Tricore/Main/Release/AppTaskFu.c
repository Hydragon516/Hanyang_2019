#include "AppTaskFu.h"

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
}

void appTaskfu_1ms(void)
{
	task_cnt_1m++;
	if(task_cnt_1m == 1000){
		task_cnt_1m = 0;
	}

}


void appTaskfu_10ms(void)
{
	task_cnt_10m++;
	if(task_cnt_10m == 1000){
		task_cnt_10m = 0;
	}

	if(task_cnt_10m%2 == 0){
		BasicLineScan_run();
		InfineonRacer_detectLane();
		BasicPort_run();
		BasicGtmTom_run();
		BasicVadcBgScan_run();
		printf("lane : %d\n", get_lane());

		if(IR_Ctrl.basicTest == FALSE){
			#if CODE == CODE_HAND
				InfineonRacer_control();
			#elif CODE == CODE_ERT
				IR_Controller_step();
			#else

			#endif
		}
		AsclinShellInterface_runLineScan();
	}

	//printf("lane : %d\n", get_lane());

	//printf("AdcResult1: \t");
	//printf("%f \n", IR_AdcResult[1]);

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

	/*
	printf("lineScan : ");
	printf("%d ", IR_LineScan.adcResult[0][0]);
	printf("%d ", IR_LineScan.adcResult[0][1]);
	printf("%d / ", IR_LineScan.adcResult[0][2]);
	printf("%d ", IR_LineScan.adcResult[0][63]);
	printf("%d ", IR_LineScan.adcResult[0][64]);
	printf("%d / ", IR_LineScan.adcResult[0][65]);
	printf("%d ", IR_LineScan.adcResult[0][125]);
	printf("%d ", IR_LineScan.adcResult[0][126]);
	printf("%d ", IR_LineScan.adcResult[0][127]);

	printf("\n");
	*/

}

void appTaskfu_1000ms(void)
{
	if(task_cnt_1000m % 2 == 0){
		//IR_setSrvAngle(0.5);
		IR_setMotor0Vol(-0.2);
	}
	else{
		//IR_setSrvAngle(-0.5);
		//IR_setMotor0Vol(0.3);
	}
	task_cnt_1000m++;
	if(task_cnt_1000m == 1000){
		task_cnt_1000m = 0;
	}

	printf("appTaskfu_1000ms : %d\n", task_cnt_1000m);

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

