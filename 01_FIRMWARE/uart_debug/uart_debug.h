#ifndef __UART_DEBUG_H
#define __UART_DEBUG_H

#include "main.h" // Để lấy cấu hình UART_HandleTypeDef
#include <stdio.h>

// Truyền vào con trỏ UART bạn muốn dùng để in (ví dụ: &huart1)
void UART_Debug_Init(UART_HandleTypeDef *huart);

#endif
