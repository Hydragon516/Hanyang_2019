#ifndef INFINEONRACER_H_
#define INFINEONRACER_H_


/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include <Ifx_Types.h>
#include "Configuration.h"

/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/
#define IR_getLs0Margin()		IR_Ctrl.Ls0Margin
#define IR_getLs1Margin()		IR_Ctrl.Ls1Margin

/******************************************************************************/
/*--------------------------------Enumerations--------------------------------*/
/******************************************************************************/



/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/
typedef struct{
	sint32 Ls0Margin;
	sint32 Ls1Margin;
	boolean basicTest;
}InfineonRacer_t;

/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/
IFX_EXTERN InfineonRacer_t IR_Ctrl;
IFX_EXTERN boolean isLaneValid;
IFX_EXTERN boolean SpeedControlZone;
/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/
IFX_EXTERN void InfineonRacer_init(void);
IFX_EXTERN void InfineonRacer_detectLane(sint32);
IFX_EXTERN void InfineonRacer_control(void);
IFX_EXTERN int get_lane(void);
IFX_EXTERN void InfineonRacer_DotFullLane(sint32);
IFX_EXTERN void InfineonRacer_detectObstacle(sint32);
IFX_EXTERN boolean get_StartLaneChange(void);
IFX_EXTERN boolean get_ObstacleCount(void);

#endif
