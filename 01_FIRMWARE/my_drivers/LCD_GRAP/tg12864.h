/*
 * tg12864.h
 *
 *  Created on: Jan 26, 2026
 *      Author: 84373
 */

#ifndef TG12864_H_
#define TG12864_H_
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_utils.h"
#include <stdint.h>
#include <stddef.h>
#include "font5x7.h"
#include "font8x8.h"
extern const TG12864_Font Font5x7;
extern const TG12864_Font Font8x8;

typedef struct {
    GPIO_TypeDef* port;
    uint32_t      pin;
} TG12864_Pin;

typedef struct {
    // Dữ liệu DB0–DB7
    TG12864_Pin db[8];

    // Chân điều khiển
    TG12864_Pin di;
    TG12864_Pin rw;
    TG12864_Pin e;
    TG12864_Pin cs1;
    TG12864_Pin cs2;
    TG12864_Pin rst;
} TG12864_Handle;




// ==== PUBLIC API ====
void TG12864_Init(TG12864_Handle* lcd);
void TG12864_DrawByte(TG12864_Handle* lcd, uint8_t x, uint8_t page, uint8_t data);
void TG12864_Clear(TG12864_Handle* lcd);
void TG12864_TestPattern(TG12864_Handle* lcd);
void TG12864_DrawChar(TG12864_Handle* lcd, uint8_t x, uint8_t page, char c);
void TG12864_DrawString(TG12864_Handle* lcd, uint8_t x, uint8_t page, const char* str);
void TG12864_DrawBitmap(TG12864_Handle* lcd, const uint8_t* bitmap);
void TG12864_DrawChar_Font(TG12864_Handle* lcd, uint8_t x, uint8_t page, char c, const TG12864_Font* font);
void TG12864_DrawString_Font(TG12864_Handle* lcd, uint8_t x, uint8_t page, const char* str, const TG12864_Font* font);

// ==== ROTATION CONTROL FUNCTIONS ====
void TG12864_SetRotate180(uint8_t enable);
uint8_t TG12864_GetRotate180(void);

// ==== DEBUG FUNCTIONS ====
void TG12864_TestRotation(TG12864_Handle* lcd);


#endif /* TG12864_H_ */
