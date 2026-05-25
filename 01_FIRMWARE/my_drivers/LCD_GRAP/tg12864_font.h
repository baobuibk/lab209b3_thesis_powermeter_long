/*
 * tg12864_font.h
 *
 *  Created on: Jan 26, 2026
 *      Author: 84373
 */

#ifndef TG12864_FONT_H_
#define TG12864_FONT_H_

#include <stdint.h>

typedef struct {
    const uint8_t* data;      // pointer to the font bitmap array
    uint8_t width;            // chiều rộng mỗi ký tự
    uint8_t height;           // chiều cao mỗi ký tự
    uint8_t first_char;       // mã ASCII ký tự đầu tiên hỗ trợ
    uint8_t last_char;        // mã ASCII ký tự cuối cùng hỗ trợ
} TG12864_Font;

// Cho phép mở rộng hỗ trợ nhiều font
extern const TG12864_Font Font5x7;
extern const TG12864_Font Font8x8;

#endif /* TG12864_FONT_H_ */
