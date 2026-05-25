#include "tg12864.h"
#include "font5x7.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_utils.h"

#define TG_PIN_RESET 0
#define TG_PIN_SET   1
extern const TG12864_Font Font5x7;  // Thêm dòng này vào đầu file nếu chưa có
extern const TG12864_Font Font8x8;

// ==== ROTATION CONFIGURATION ====
static uint8_t g_rotate_180 = 1;


// ==== TÁC ĐỘNG CHÂN ====
static void write_pin(TG12864_Pin pin, uint8_t state) {
    if (state)
        LL_GPIO_SetOutputPin(pin.port, pin.pin);
    else
        LL_GPIO_ResetOutputPin(pin.port, pin.pin);
}

static void lcd_set_data(TG12864_Handle* lcd, uint8_t data) {
    for (int i = 0; i < 8; i++) {
        write_pin(lcd->db[i], (data & (1 << i)) ? TG_PIN_SET : TG_PIN_RESET);
    }
}

static void lcd_enable_pulse(TG12864_Handle* lcd) {
    write_pin(lcd->e, TG_PIN_SET);
    for (volatile uint32_t i = 0; i < 10; i++) {
            // Không làm gì cả, chỉ tốn thời gian
        }    write_pin(lcd->e, TG_PIN_RESET);
}

static void lcd_write_instruction(TG12864_Handle* lcd, uint8_t cmd, uint8_t chip) {
    write_pin(lcd->di, TG_PIN_RESET);
    if (lcd->rw.port != NULL)
        write_pin(lcd->rw, TG_PIN_RESET);  // RW pin không sử dụng trong chế độ ghi

    write_pin(lcd->cs1, (chip == 1) ? TG_PIN_SET : TG_PIN_RESET);
    write_pin(lcd->cs2, (chip == 2) ? TG_PIN_SET : TG_PIN_RESET);

    lcd_set_data(lcd, cmd);
    lcd_enable_pulse(lcd);
}

static void lcd_write_data(TG12864_Handle* lcd, uint8_t data, uint8_t chip) {
    write_pin(lcd->di, TG_PIN_SET);
    if (lcd->rw.port != NULL)
        write_pin(lcd->rw, TG_PIN_RESET);  // RW pin không sử dụng trong chế độ ghi

    write_pin(lcd->cs1, (chip == 1) ? TG_PIN_SET : TG_PIN_RESET);
    write_pin(lcd->cs2, (chip == 2) ? TG_PIN_SET : TG_PIN_RESET);

    lcd_set_data(lcd, data);
    lcd_enable_pulse(lcd);
}

// ==== PUBLIC API ====
void TG12864_Init(TG12864_Handle* lcd) {
    if (lcd->rst.port != NULL) {
        write_pin(lcd->rst, TG_PIN_RESET);
        HAL_Delay(10);
        write_pin(lcd->rst, TG_PIN_SET);
    }

    for (int chip = 1; chip <= 2; chip++) {
        lcd_write_instruction(lcd, 0x3F, chip);  // Display ON
    }
}

static void lcd_goto(TG12864_Handle* lcd, uint8_t x, uint8_t page) {
    // Simple goto without rotation (rotation handled in DrawByte)
    uint8_t chip = (x < 64) ? 1 : 2;
    uint8_t col  = (x < 64) ? x : (x - 64);

    lcd_write_instruction(lcd, 0xB8 | page, chip);
    lcd_write_instruction(lcd, 0x40 | col, chip);
}

void TG12864_DrawByte(TG12864_Handle* lcd, uint8_t x, uint8_t page, uint8_t data) {
    // For 180-degree rotation, we need to transform coordinates AND flip bits
    uint8_t actual_x = x;
    uint8_t actual_page = page;

    if (g_rotate_180) {
        // Transform coordinates for 180-degree rotation
        actual_x = 127 - x;
        actual_page = 7 - page;

        // Flip bits vertically within the byte
        uint8_t flipped = 0;
        for (int i = 0; i < 8; i++) {
            if (data & (1 << i)) {
                flipped |= (1 << (7 - i));
            }
        }
        data = flipped;
    }

    // Use direct addressing instead of lcd_goto to avoid double transformation
    uint8_t chip = (actual_x < 64) ? 1 : 2;
    uint8_t col  = (actual_x < 64) ? actual_x : (actual_x - 64);

    lcd_write_instruction(lcd, 0xB8 | actual_page, chip);
    lcd_write_instruction(lcd, 0x40 | col, chip);
    lcd_write_data(lcd, data, chip);
}

void TG12864_Clear(TG12864_Handle* lcd) {
    for (uint8_t page = 0; page < 8; page++) {
        for (uint8_t x = 0; x < 128; x++) {
            TG12864_DrawByte(lcd, x, page, 0x00);
        }
    }
}

void TG12864_TestPattern(TG12864_Handle* lcd) {
    for (uint8_t page = 0; page < 8; page++) {
        for (uint8_t x = 0; x < 128; x++) {
            TG12864_DrawByte(lcd, x, page, (x % 2 == 0) ? 0xFF : 0x00);
        }
    }
}

void TG12864_DrawChar(TG12864_Handle* lcd, uint8_t x, uint8_t page, char c) {
    const TG12864_Font* font = &Font5x7;
    TG12864_DrawChar_Font(lcd, x, page, c, font);  // Use the improved function
}

void TG12864_DrawString(TG12864_Handle* lcd, uint8_t x, uint8_t page, const char* str) {
    const TG12864_Font* font = &Font5x7;
    TG12864_DrawString_Font(lcd, x, page, str, font);  // Use the improved function
}

void TG12864_DrawBitmap(TG12864_Handle* lcd, const uint8_t* bitmap) {
    for (uint8_t page = 0; page < 8; page++) {
        for (uint8_t x = 0; x < 128; x++) {
            uint16_t index = page * 128 + x;
            TG12864_DrawByte(lcd, x, page, bitmap[index]);
        }
    }
}

//void TG12864_DrawChar_Font(TG12864_Handle* lcd, uint8_t x, uint8_t page, char c, const TG12864_Font* font) {
//    if (c < font->first_char || c > font->last_char) return;
//
//    uint16_t offset = (c - font->first_char) * font->width;
//
//    for (uint8_t i = 0; i < font->width; i++) {
//        uint8_t data = font->data[offset + i];
//        TG12864_DrawByte(lcd, x + i, page, data);
//    }
//}

void TG12864_DrawChar_Font(TG12864_Handle* lcd, uint8_t x, uint8_t page, char c, const TG12864_Font* font) {
    if (c < font->first_char || c > font->last_char) return;

    uint16_t offset = (c - font->first_char) * font->width;

    for (uint8_t i = 0; i < font->width; i++) {
        uint8_t target_x = x + i;
        if (target_x >= 128) break; // Tránh tràn màn hình

        // Ép buộc cập nhật lại địa chỉ X/Page mỗi khi vẽ 1 cột để đảm bảo
        // khi vượt qua ranh giới x=63, chip 2 sẽ nhận được lệnh Goto Page đúng.
        TG12864_DrawByte(lcd, target_x, page, font->data[offset + i]);
    }
}

void TG12864_DrawString_Font(TG12864_Handle* lcd, uint8_t x, uint8_t page, const char* str, const TG12864_Font* font) {
    while (*str) {
        TG12864_DrawChar_Font(lcd, x, page, *str, font);
        x += font->width + 1;  // +1 for character spacing
        str++;
    }
}

// ==== ROTATION CONTROL FUNCTIONS ====
void TG12864_SetRotate180(uint8_t enable) {
    g_rotate_180 = enable;
}

uint8_t TG12864_GetRotate180(void) {
    return g_rotate_180;
}

// ==== DEBUG FUNCTIONS ====
void TG12864_TestRotation(TG12864_Handle* lcd) {
    TG12864_Clear(lcd);

    // Test normal orientation
    TG12864_SetRotate180(0);
    TG12864_DrawString_Font(lcd, 0, 0, "Normal", &Font5x7);
    TG12864_DrawString_Font(lcd, 0, 1, "Top-Left", &Font5x7);
    TG12864_DrawString_Font(lcd, 0, 7, "Bottom", &Font5x7);

    HAL_Delay(2000);  // Wait 2 seconds

    // Test 180-degree rotation
    TG12864_Clear(lcd);
    TG12864_SetRotate180(1);
    TG12864_DrawString_Font(lcd, 0, 0, "Rotated", &Font5x7);
    TG12864_DrawString_Font(lcd, 0, 1, "180deg", &Font5x7);
    TG12864_DrawString_Font(lcd, 0, 7, "Check", &Font5x7);

    // Reset to normal after test
    // TG12864_SetRotate180(0);
}

