#include "port.h"
#include "mb.h"
#include "mbport.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "stm32f1xx_hal.h"
// Hàng đợi sự kiện cho Modbus
static QueueHandle_t xQueueHdl;

BOOL xMBPortEventInit( void )
{
    // Tạo Queue chứa tối đa 5 sự kiện
    xQueueHdl = xQueueCreate( 5, sizeof( eMBEventType ) );
    return ( xQueueHdl != NULL ) ? TRUE : FALSE;
}

BOOL xMBPortEventPost( eMBEventType eEvent )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Kiểm tra xem lệnh gọi này đang ở trong Ngắt (ISR) hay ở Task bình thường
    if( ( SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk ) != 0 )
    {
        // Nếu ở trong Ngắt (như từ UART hoặc Timer)
        xQueueSendFromISR( xQueueHdl, ( const void * )&eEvent, &xHigherPriorityTaskWoken );
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
    else
    {
        // Nếu ở trong Task
        xQueueSend( xQueueHdl, ( const void * )&eEvent, 0 );
    }
    return TRUE;
}

BOOL xMBPortEventGet( eMBEventType * peEvent )
{
    // Task Modbus sẽ bị Block (ngủ) ở đây cho đến khi có sự kiện đẩy vào Queue
    if( xQueueReceive( xQueueHdl, peEvent, portMAX_DELAY ) == pdTRUE )
    {
        return TRUE;
    }
    return FALSE;
}
