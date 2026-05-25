#include "port.h"
#include "mb.h"
#include "mbport.h"
#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef huart1;
// Chân DE/RE nối vào PA8 (sửa lại theo phần cứng của ông)
#define RS485_TX_EN() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define RS485_RX_EN() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)

void vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable ) {
    if( xRxEnable ) {
        // --- FIX LỖI CỤT FRAME RS485 TẠI ĐÂY ---
        if (!xTxEnable) {
            // Đợi cờ TC (Truyền hoàn tất) bật lên rồi mới hạ chân DE
            while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);
        }
        RS485_RX_EN(); // Hạ chân DE/RE xuống để nghe
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    } else {
        __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
    }

    if( xTxEnable ) {
        RS485_TX_EN(); // Nâng chân DE/RE lên để nói
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
    } else {
        __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
    }
}

BOOL xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity ,UCHAR ucStopBits) {
    // Code HAL Init UART đã nằm bên CubeMX sinh ra rồi, chỗ này cứ trả về TRUE
    return TRUE;
}

BOOL xMBPortSerialPutByte( CHAR ucByte ) {
    huart1.Instance->DR = ucByte;
    return TRUE;
}

BOOL xMBPortSerialGetByte( CHAR * pucByte ) {
    *pucByte = (uint8_t)(huart1.Instance->DR & (uint8_t)0x00FF);
    return TRUE;
}
