/*
 * button_task.c
 *
 *  Created on: Mar 31, 2026
 *      Author: 84373
 */


#include "button_task.h"

uint8_t Read_Button_Task(void)
{
    static uint8_t last_state = 0;
    uint8_t current_state = 0;
    uint8_t button_event = 0;

    if(LL_GPIO_IsInputPinSet(BUTTON_SETUP_PORT, BUTTON_SETUP_PIN) == 0)
        current_state |= SETUP_MSK;

    if(LL_GPIO_IsInputPinSet(BUTTON_UP_PORT, BUTTON_UP_PIN) == 0)
        current_state |= UP_MSK;

    if(LL_GPIO_IsInputPinSet(BUTTON_DOWN_PORT, BUTTON_DOWN_PIN) == 0)
        current_state |= DOWN_MSK;

    if(LL_GPIO_IsInputPinSet(BUTTON_ESC_PORT, BUTTON_ESC_PIN) == 0)
        current_state |= ESC_MSK;

    button_event = current_state & (~last_state);

    if(button_event != 0)
    {
        osMessageQueuePut(buttonQueueHandle, &button_event, 0, 0);
    }

    last_state = current_state;
    return button_event;
}
