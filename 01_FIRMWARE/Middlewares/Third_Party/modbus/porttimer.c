#include "port.h"
#include "mb.h"
#include "mbport.h"
#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim3;

BOOL xMBPortTimersInit( USHORT usTim1Timerout50us ) {
    // Nạp giới hạn đếm (ARR)
    __HAL_TIM_SET_AUTORELOAD(&htim3, usTim1Timerout50us - 1);
    return TRUE;
}

inline void vMBPortTimersEnable( void ) {
    // Bật ngắt Timer
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Base_Start_IT(&htim3);
}

inline void vMBPortTimersDisable( void ) {
    // Tắt ngắt Timer
    HAL_TIM_Base_Stop_IT(&htim3);
}
void vMBPortTimerClose( void ) { }
