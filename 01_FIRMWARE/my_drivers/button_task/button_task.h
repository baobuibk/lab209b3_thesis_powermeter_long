/*
 * button_task.h
 *
 *  Created on: Mar 31, 2026
 *      Author: 84373
 */

#ifndef BUTTON_TASK_H_
#define BUTTON_TASK_H_

#include "main.h"
#include "cmsis_os.h"

extern osMessageQueueId_t buttonQueueHandle;

// define PIN for button
//SW2 = UP = KEY0 =PA11
//SW3 =ĐWN = KEY1 =PA12
//SW4 = ESC =KEY2 = PB1
//SW5 = SET = KEY3 = PB0

#define BUTTON_UP_PORT          GPIOA
#define BUTTON_UP_PIN           LL_GPIO_PIN_11

#define BUTTON_DOWN_PORT        GPIOA
#define BUTTON_DOWN_PIN         LL_GPIO_PIN_12

#define BUTTON_ESC_PORT         GPIOB
#define BUTTON_ESC_PIN          LL_GPIO_PIN_0

#define BUTTON_SETUP_PORT       GPIOB
#define BUTTON_SETUP_PIN        LL_GPIO_PIN_1




// define PIN_MSK for button

#define UP_MSK					(1U << 0)
#define DOWN_MSK				(1U << 1)
#define ESC_MSK					(1U << 2)
#define SETUP_MSK 				(1U << 3)

uint8_t Read_Button_Task(void);
#endif /* BUTTON_TASK_H_ */
