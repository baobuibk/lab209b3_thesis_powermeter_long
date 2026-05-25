#include "uart_debug.h"

// Biến tĩnh để lưu bộ UART sẽ dùng
static UART_HandleTypeDef *g_huart;

void UART_Debug_Init(UART_HandleTypeDef *huart) {
    g_huart = huart;

    // Tắt buffering để printf in ra ngay lập tức
    setvbuf(stdout, NULL, _IONBF, 0);
}

// Ghi đè hàm hệ thống để kết nối printf với UART
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  if (g_huart != NULL) {
      HAL_UART_Transmit(g_huart, (uint8_t *)&ch, 1, 10);
      return ch;
  }
  return -1;
}
