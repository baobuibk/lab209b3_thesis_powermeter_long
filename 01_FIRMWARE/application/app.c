/**
 * @file app.c
 * @brief Application logic for 3-Phase Monitor using M90E32AS and TG12864 LCD.
 * @date Mar 18, 2026
 */

#include "app.h"
#include "tg12864.h"
#include "stm32f1xx_ll_gpio.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "button_task.h"
#include "M90E32AS_driver.h"
#include "uart_debug.h"

extern void Modbus_Update_Data(void);

/* ================= APP CONFIG ================= */

#define LCD_LINE_MAX             22
#define GRID_FREQ_DEFAULT        50
#define LOG_TAG                  "APP"

#define FILTER_ALPHA             0.15f
#define DEMAND_PERIOD_SEC        900
#define SAMPLE_INTERVAL_MS       1000


/* ================= LCD DISPLAY FORMAT HELPERS =================
 * Driver lưu RAW.
 * App/LCD đổi RAW sang đơn vị hiển thị.
 *
 * U     : raw / RAW_VOLTAGE_SCALE = V
 * I     : raw * RAW_CURRENT_SCALE_MA = mA
 * P/Q/S : raw * RAW_POWER_SCALE_uW = uW
 * Freq  : raw / RAW_FREQ_SCALE = Hz
 * Angle : raw / RAW_ANGLE_SCALE = degree
 * PF    : raw / RAW_PF_SCALE
 */

#define DISP_U_INT(x) \
    ((int32_t)(x) / RAW_VOLTAGE_SCALE)

#define DISP_U_DEC1(x) \
    (labs((long)((int32_t)(x) % RAW_VOLTAGE_SCALE)) / 10)


#define DISP_I_mA(x) \
    ((int32_t)(x) * RAW_CURRENT_SCALE_MA)

#define DISP_I_INT(x) \
    (DISP_I_mA(x) / 1000)

#define DISP_I_DEC3(x) \
    (labs((long)(DISP_I_mA(x) % 1000)))


#define DISP_FREQ_INT(x) \
    ((int32_t)(x) / RAW_FREQ_SCALE)

#define DISP_FREQ_DEC2(x) \
    (labs((long)((int32_t)(x) % RAW_FREQ_SCALE)))


#define DISP_ANGLE_INT(x) \
    ((int32_t)(x) / RAW_ANGLE_SCALE)

#define DISP_ANGLE_DEC1(x) \
    (labs((long)((int32_t)(x) % RAW_ANGLE_SCALE)))


#define DISP_PF_INT(x) \
    ((int32_t)(x) / RAW_PF_SCALE)

#define DISP_PF_DEC3(x) \
    (labs((long)((int32_t)(x) % RAW_PF_SCALE)))


/*
 * Power display:
 * raw power -> uW = raw * RAW_POWER_SCALE_uW
 * uW -> mW  = uW / 1000
 * kW display with 3 decimals:
 *   kW integer = mW / 1,000,000
 *   decimals   = (mW % 1,000,000) / 1000
 */
#define DISP_PWR_mW(x) \
    ((int32_t)(((int64_t)(x) * RAW_POWER_SCALE_uW) / 1000))

#define DISP_PWR_INT(x) \
    (DISP_PWR_mW(x) / 1000000L)

#define DISP_PWR_DEC3(x) \
    ((labs((long)(DISP_PWR_mW(x))) % 1000000L) / 1000L)


/* ================= DISPLAY DEADBAND HELPERS =================
 * Chỉ lọc khi hiển thị / truyền thông.
 * Không làm thay đổi dữ liệu RAW trong driver.
 */



/* --- External Handles --- */
extern SPI_HandleTypeDef  hspi2;
extern UART_HandleTypeDef huart3;
extern TG12864_Handle     myLCD;
extern osMessageQueueId_t buttonQueueHandle;
uint16_t sys_status0 = 0;

/* --- Static Variables --- */
M90E32_t m90_device = {
    .hspi    = &hspi2,
    .PORT    = GPIOB,
    .CS_PIN  = GPIO_PIN_12,
    .PM_PORT = GPIOA,
    .PM0_PIN = GPIO_PIN_11,
    .PM1_PIN = GPIO_PIN_12
};
Meter_Stats_t meter_stats;
//M90E32_Measurements_t m90_results;
M90E32_Measurements_t m90_results;
static uint16_t button_counters[4] = {0};
/* --- Private Function Prototypes --- */
static void App_Update_Button_Stats(void);
static void App_Reset_Energy(void)
{
    m90_results.ActiveEnergy_Forward_T = 0;
    m90_results.ActiveEnergy_Forward_A = 0;
    m90_results.ActiveEnergy_Forward_B = 0;
    m90_results.ActiveEnergy_Forward_C = 0;

    m90_results.ActiveEnergy_Reverse_T = 0;
    m90_results.ActiveEnergy_Reverse_A = 0;
    m90_results.ActiveEnergy_Reverse_B = 0;
    m90_results.ActiveEnergy_Reverse_C = 0;
}
void App_Init_Stats(void) {
    // Max khởi tạo bằng 0
    meter_stats.V1_Max = 0; meter_stats.V2_Max = 0; meter_stats.V3_Max = 0;
    meter_stats.I1_Max = 0; meter_stats.I2_Max = 0; meter_stats.I3_Max = 0;
    meter_stats.P1_Max = 0; meter_stats.P2_Max = 0; meter_stats.P3_Max = 0;

    // Min PHẢI khởi tạo bằng một số rất lớn
    meter_stats.V1_Min = 9999;
    meter_stats.V2_Min = 9999;
    meter_stats.V3_Min = 9999;
}

static void App_Process_Measurements(void) {
    // Deadband điện áp pha
    if (m90_results.UrmsA < DEADBAND_VOLTAGE_RAW) m90_results.UrmsA = 0;
    if (m90_results.UrmsB < DEADBAND_VOLTAGE_RAW) m90_results.UrmsB = 0;
    if (m90_results.UrmsC < DEADBAND_VOLTAGE_RAW) m90_results.UrmsC = 0;

    // Deadband dòng
    if (m90_results.IrmsA < DEADBAND_CURRENT_RAW) m90_results.IrmsA = 0;
    if (m90_results.IrmsB < DEADBAND_CURRENT_RAW) m90_results.IrmsB = 0;
    if (m90_results.IrmsC < DEADBAND_CURRENT_RAW) m90_results.IrmsC = 0;
    if (m90_results.IrmsN < DEADBAND_CURRENT_RAW) m90_results.IrmsN = 0;

    // Tính lại áp dây từ điện áp pha đã deadband
    m90_results.UrmsAB = (int16_t)(M90E32_Cal_Phase(
        (float)m90_results.UrmsA / RAW_VOLTAGE_SCALE,
        (float)m90_results.UrmsB / RAW_VOLTAGE_SCALE,
        (float)(m90_results.UAngleA - m90_results.UAngleB) / RAW_ANGLE_SCALE
    ) * RAW_VOLTAGE_SCALE);

    m90_results.UrmsBC = (int16_t)(M90E32_Cal_Phase(
        (float)m90_results.UrmsB / RAW_VOLTAGE_SCALE,
        (float)m90_results.UrmsC / RAW_VOLTAGE_SCALE,
        (float)(m90_results.UAngleB - m90_results.UAngleC) / RAW_ANGLE_SCALE
    ) * RAW_VOLTAGE_SCALE);

    m90_results.UrmsCA = (int16_t)(M90E32_Cal_Phase(
        (float)m90_results.UrmsC / RAW_VOLTAGE_SCALE,
        (float)m90_results.UrmsA / RAW_VOLTAGE_SCALE,
        (float)(m90_results.UAngleC - m90_results.UAngleA) / RAW_ANGLE_SCALE
    ) * RAW_VOLTAGE_SCALE);

    // Deadband công suất
    if (m90_results.PmeanA_Forward < DEADBAND_POWER_RAW) m90_results.PmeanA_Forward = 0;
    if (m90_results.PmeanB_Forward < DEADBAND_POWER_RAW) m90_results.PmeanB_Forward = 0;
    if (m90_results.PmeanC_Forward < DEADBAND_POWER_RAW) m90_results.PmeanC_Forward = 0;
    if (m90_results.PmeanT_Forward < DEADBAND_POWER_RAW) m90_results.PmeanT_Forward = 0;

    if (m90_results.PmeanA_Reverse < DEADBAND_POWER_RAW) m90_results.PmeanA_Reverse = 0;
    if (m90_results.PmeanB_Reverse < DEADBAND_POWER_RAW) m90_results.PmeanB_Reverse = 0;
    if (m90_results.PmeanC_Reverse < DEADBAND_POWER_RAW) m90_results.PmeanC_Reverse = 0;
    if (m90_results.PmeanT_Reverse < DEADBAND_POWER_RAW) m90_results.PmeanT_Reverse = 0;

    if (labs(m90_results.QmeanA) < DEADBAND_POWER_RAW) m90_results.QmeanA = 0;
    if (labs(m90_results.QmeanB) < DEADBAND_POWER_RAW) m90_results.QmeanB = 0;
    if (labs(m90_results.QmeanC) < DEADBAND_POWER_RAW) m90_results.QmeanC = 0;
    if (labs(m90_results.QmeanT) < DEADBAND_POWER_RAW) m90_results.QmeanT = 0;

    if (m90_results.SmeanA < DEADBAND_POWER_RAW) m90_results.SmeanA = 0;
    if (m90_results.SmeanB < DEADBAND_POWER_RAW) m90_results.SmeanB = 0;
    if (m90_results.SmeanC < DEADBAND_POWER_RAW) m90_results.SmeanC = 0;
    if (m90_results.SAmeanT < DEADBAND_POWER_RAW) m90_results.SAmeanT = 0;
}
void App_Update_MinMax(void) {
    // --- PHA A ---
    if (m90_results.UrmsA > meter_stats.V1_Max) meter_stats.V1_Max = m90_results.UrmsA;
    // Bắt Min (Lọc nhiễu: Chỉ bắt Min khi có điện thật sự, ví dụ áp > 50V)
    if (m90_results.UrmsA < meter_stats.V1_Min && m90_results.UrmsA > DEADBAND_VOLTAGE_RAW) {
        meter_stats.V1_Min = m90_results.UrmsA;
    }
    if (m90_results.IrmsA > meter_stats.I1_Max) meter_stats.I1_Max = m90_results.IrmsA;
    if (m90_results.PmeanA_Forward > meter_stats.P1_Max) meter_stats.P1_Max = m90_results.PmeanA_Forward;

    // --- PHA B ---
    if (m90_results.UrmsB > meter_stats.V2_Max) meter_stats.V2_Max = m90_results.UrmsB;
    if (m90_results.UrmsB < meter_stats.V2_Min && m90_results.UrmsB > DEADBAND_VOLTAGE_RAW) {
        meter_stats.V2_Min = m90_results.UrmsB;
    }
    if (m90_results.IrmsB > meter_stats.I2_Max) meter_stats.I2_Max = m90_results.IrmsB;
    if (m90_results.PmeanB_Forward > meter_stats.P2_Max) meter_stats.P2_Max = m90_results.PmeanB_Forward;

    // --- PHA C ---
    if (m90_results.UrmsC > meter_stats.V3_Max) meter_stats.V3_Max = m90_results.UrmsC;
    if (m90_results.UrmsC < meter_stats.V3_Min && m90_results.UrmsC > DEADBAND_VOLTAGE_RAW) {
        meter_stats.V3_Min = m90_results.UrmsC;
    }
    if (m90_results.IrmsC > meter_stats.I3_Max) meter_stats.I3_Max = m90_results.IrmsC;
    if (m90_results.PmeanC_Forward > meter_stats.P3_Max) meter_stats.P3_Max = m90_results.PmeanC_Forward;
}

void App_Calculate_Demand_And_Avg(void) {
    static uint32_t last_sample_tick = 0;
    static uint16_t sample_count = 0;
    static uint8_t is_first_run = 1;
	static uint8_t skip_samples = 10;

	if (skip_samples > 0) {
		meter_stats.V1_Avg = m90_results.UrmsA; meter_stats.V2_Avg = m90_results.UrmsB; meter_stats.V3_Avg = m90_results.UrmsC;
		meter_stats.P1_Avg = m90_results.PmeanA_Forward; meter_stats.P2_Avg = m90_results.PmeanB_Forward; meter_stats.P3_Avg = m90_results.PmeanC_Forward;
		meter_stats.S1_Avg = m90_results.SmeanA; meter_stats.S2_Avg = m90_results.SmeanB; meter_stats.S3_Avg = m90_results.SmeanC;
		skip_samples--;
		return;
	}

    // =================================================================
    // 1. TÍNH AVG TỨC THỜI (LỌC NHIỄU EMA) - PURE INTEGER (Alpha 0.15)
    // =================================================================
    if (is_first_run) {
        meter_stats.V1_Avg = m90_results.UrmsA; meter_stats.V2_Avg = m90_results.UrmsB; meter_stats.V3_Avg = m90_results.UrmsC;
        meter_stats.Vab_Avg = m90_results.UrmsAB; meter_stats.Vbc_Avg = m90_results.UrmsBC; meter_stats.Vca_Avg = m90_results.UrmsCA;
        meter_stats.I1_Avg = m90_results.IrmsA; meter_stats.I2_Avg = m90_results.IrmsB; meter_stats.I3_Avg = m90_results.IrmsC;
        meter_stats.P1_Avg = m90_results.PmeanA_Forward; meter_stats.P2_Avg = m90_results.PmeanB_Forward; meter_stats.P3_Avg = m90_results.PmeanC_Forward;
        meter_stats.S1_Avg = m90_results.SmeanA; meter_stats.S2_Avg = m90_results.SmeanB; meter_stats.S3_Avg = m90_results.SmeanC;
        is_first_run = 0;
    } else {
        // Avg_mới = (15 * Mới + 85 * Cũ) / 100
        meter_stats.V1_Avg = (15 * m90_results.UrmsA + 85 * meter_stats.V1_Avg) / 100;
        meter_stats.V2_Avg = (15 * m90_results.UrmsB + 85 * meter_stats.V2_Avg) / 100;
        meter_stats.V3_Avg = (15 * m90_results.UrmsC + 85 * meter_stats.V3_Avg) / 100;

        meter_stats.Vab_Avg = (15 * m90_results.UrmsAB + 85 * meter_stats.Vab_Avg) / 100;
        meter_stats.Vbc_Avg = (15 * m90_results.UrmsBC + 85 * meter_stats.Vbc_Avg) / 100;
        meter_stats.Vca_Avg = (15 * m90_results.UrmsCA + 85 * meter_stats.Vca_Avg) / 100;

        meter_stats.I1_Avg = (15 * m90_results.IrmsA + 85 * meter_stats.I1_Avg) / 100;
        meter_stats.I2_Avg = (15 * m90_results.IrmsB + 85 * meter_stats.I2_Avg) / 100;
        meter_stats.I3_Avg = (15 * m90_results.IrmsC + 85 * meter_stats.I3_Avg) / 100;

        meter_stats.P1_Avg = (15 * m90_results.PmeanA_Forward + 85 * meter_stats.P1_Avg) / 100;
        meter_stats.P2_Avg = (15 * m90_results.PmeanB_Forward + 85 * meter_stats.P2_Avg) / 100;
        meter_stats.P3_Avg = (15 * m90_results.PmeanC_Forward + 85 * meter_stats.P3_Avg) / 100;

        meter_stats.S1_Avg = (15 * m90_results.SmeanA + 85 * meter_stats.S1_Avg) / 100;
        meter_stats.S2_Avg = (15 * m90_results.SmeanB + 85 * meter_stats.S2_Avg) / 100;
        meter_stats.S3_Avg = (15 * m90_results.SmeanC + 85 * meter_stats.S3_Avg) / 100;
    }

    // =================================================================
    // 2. TÍNH DEMAND (CHU KỲ 15 PHÚT)
    // =================================================================
    uint32_t current_tick = HAL_GetTick();

    // Dùng biến int32_t để chống gọi thư viện Float, dư sức chứa 900 vòng
    static int32_t sum_V1 = 0, sum_V2 = 0, sum_V3 = 0;
    static int32_t sum_Vab = 0, sum_Vbc = 0, sum_Vca = 0;
    static int32_t sum_P1 = 0, sum_P2 = 0, sum_P3 = 0;
    static int32_t sum_S1 = 0, sum_S2 = 0, sum_S3 = 0;

    if (current_tick - last_sample_tick >= SAMPLE_INTERVAL_MS) {
        last_sample_tick = current_tick;

        sum_V1 += m90_results.UrmsA; sum_V2 += m90_results.UrmsB; sum_V3 += m90_results.UrmsC;
        sum_Vab += m90_results.UrmsAB; sum_Vbc += m90_results.UrmsBC; sum_Vca += m90_results.UrmsCA;
        sum_P1 += m90_results.PmeanA_Forward; sum_P2 += m90_results.PmeanB_Forward; sum_P3 += m90_results.PmeanC_Forward;
        sum_S1 += m90_results.SmeanA; sum_S2 += m90_results.SmeanB; sum_S3 += m90_results.SmeanC;

        sample_count++;

        if (sample_count > 0) {
            meter_stats.V1_Dmd = sum_V1 / sample_count; meter_stats.V2_Dmd = sum_V2 / sample_count; meter_stats.V3_Dmd = sum_V3 / sample_count;
            meter_stats.Vab_Dmd = sum_Vab / sample_count; meter_stats.Vbc_Dmd = sum_Vbc / sample_count; meter_stats.Vca_Dmd = sum_Vca / sample_count;
            meter_stats.P1_Dmd = sum_P1 / sample_count; meter_stats.P2_Dmd = sum_P2 / sample_count; meter_stats.P3_Dmd = sum_P3 / sample_count;
            meter_stats.S1_Dmd = sum_S1 / sample_count; meter_stats.S2_Dmd = sum_S2 / sample_count; meter_stats.S3_Dmd = sum_S3 / sample_count;
        }

        if (sample_count >= DEMAND_PERIOD_SEC) {
            sum_V1 = 0; sum_V2 = 0; sum_V3 = 0;
            sum_Vab = 0; sum_Vbc = 0; sum_Vca = 0;
            sum_P1 = 0; sum_P2 = 0; sum_P3 = 0;
            sum_S1 = 0; sum_S2 = 0; sum_S3 = 0;
            sample_count = 0;
        }
    }
}
/**
 * @brief Initialize application modules
 */
void App_Init(void) {
    UART_Debug_Init(&huart3);

    if (M90E32_Init(&m90_device) != M90E32_OK) {
        printf("[%s] M90E32AS Connection Failed!\r\n", LOG_TAG);
        return;
    }

    printf("[%s] M90E32AS Connected!\r\n", LOG_TAG);
    App_Init_Stats();

    M90E32_Calibration(&m90_device);
    M90E32_Config_Grid(&m90_device, GRID_FREQ_DEFAULT, 0, 0, 0);
    M90E32_MeterEn(&m90_device);
}

/**
 * @brief Display electrical parameters on LCD
 */
//void App_Display_Grid_Data(void) {
//    char line[LCD_LINE_MAX];
//
//    TG12864_DrawString(&myLCD, 0, 0, "--- 3P MONITOR ---");
//
//    // Line 2: Voltage (V)
//    snprintf(line, sizeof(line), "V:%4.0f %4.0f %4.0f",
//             m90_results.UrmsA, m90_results.UrmsB, m90_results.UrmsC);
//    TG12864_DrawString(&myLCD, 0, 2, line);
//
//    // Line 4: Current (I)
//    snprintf(line, sizeof(line), "I:%5.2f %5.2f %5.2f",
//             m90_results.IrmsA, m90_results.IrmsB, m90_results.IrmsC);
//    TG12864_DrawString(&myLCD, 0, 4, line);
//
//    // Line 6: Power (P) and Temperature (T)
//    snprintf(line, sizeof(line), "P:%-6.1fW    T:%3.1fC ",
//             m90_results.PmeanT_Forward, m90_results.Temp);
//    TG12864_DrawString(&myLCD, 0, 6, line);
//}

/**
 * @brief Internal logic to update button press statistics
 */
static void App_Update_Button_Stats(void) {
    uint8_t event;
    if (osMessageQueueGet(buttonQueueHandle, &event, 0, 0) == osOK) {
        if (event & SETUP_MSK) button_counters[0]++;
        if (event & UP_MSK)    button_counters[1]++;
        if (event & DOWN_MSK)  button_counters[2]++;
        if (event & ESC_MSK)   button_counters[3]++;
    }
}

/**
 * @brief Display button counters (Optional/Debug screen)
 */
void App_Display_Button_Count(void) {
    char buff[LCD_LINE_MAX];

    App_Update_Button_Stats();

    snprintf(buff, sizeof(buff), "SET:%-4u UP:%-4u", button_counters[0], button_counters[1]);
    TG12864_DrawString(&myLCD, 0, 2, buff);

    snprintf(buff, sizeof(buff), "DWN:%-4u ESC:%-4u", button_counters[2], button_counters[3]);
    TG12864_DrawString(&myLCD, 0, 3, buff);
}

void App_LCD_Display(Menu_Page_t current_page) {
    char line[30];

    int32_t pA, pB, pC, pTot;
    int32_t qTot;
    int32_t pf_val;

    char signQ, signPF;

    switch(current_page) {

        case PAGE_VOLTAGE_CURRENT:
            TG12864_DrawString(&myLCD, 25, 0, "[   V   I   ]");

            snprintf(line, sizeof(line), "L1 %3ld.%01ldV %2ld.%03ldA",
                     DISP_U_INT(m90_results.UrmsA),
                     DISP_U_DEC1(m90_results.UrmsA),
                     DISP_I_INT(m90_results.IrmsA),
                     DISP_I_DEC3(m90_results.IrmsA));
            TG12864_DrawString(&myLCD, 0, 2, line);

            snprintf(line, sizeof(line), "L2 %3ld.%01ldV %2ld.%03ldA",
                     DISP_U_INT(m90_results.UrmsB),
                     DISP_U_DEC1(m90_results.UrmsB),
                     DISP_I_INT(m90_results.IrmsB),
                     DISP_I_DEC3(m90_results.IrmsB));
            TG12864_DrawString(&myLCD, 0, 4, line);

            snprintf(line, sizeof(line), "L3 %3ld.%01ldV %2ld.%03ldA",
                     DISP_U_INT(m90_results.UrmsC),
                     DISP_U_DEC1(m90_results.UrmsC),
                     DISP_I_INT(m90_results.IrmsC),
                     DISP_I_DEC3(m90_results.IrmsC));
            TG12864_DrawString(&myLCD, 0, 6, line);
            break;

        case PAGE_VOLT_NEUTRAL:
            TG12864_DrawString(&myLCD, 25, 0, "[    L-N    ]");

            snprintf(line, sizeof(line), "L1-N %3ld.%01ld V",
                     DISP_U_INT(m90_results.UrmsA),
                     DISP_U_DEC1(m90_results.UrmsA));
            TG12864_DrawString(&myLCD, 0, 2, line);

            snprintf(line, sizeof(line), "L2-N %3ld.%01ld V",
                     DISP_U_INT(m90_results.UrmsB),
                     DISP_U_DEC1(m90_results.UrmsB));
            TG12864_DrawString(&myLCD, 0, 4, line);

            snprintf(line, sizeof(line), "L3-N %3ld.%01ld V",
                     DISP_U_INT(m90_results.UrmsC),
                     DISP_U_DEC1(m90_results.UrmsC));
            TG12864_DrawString(&myLCD, 0, 6, line);
            break;

        /* ================= NHÓM ĐIỆN ÁP PHA CHI TIẾT ================= */
        case PAGE_VOLT_HIRES_MIN:
            TG12864_DrawString(&myLCD, 25, 0, "[    L-N    ]");
            snprintf(line, sizeof(line), "L1 %3ld.%01ld V", DISP_U_INT(meter_stats.V1_Min), DISP_U_DEC1(meter_stats.V1_Min));
            TG12864_DrawString(&myLCD, 0, 2, line);
            snprintf(line, sizeof(line), "L2 %3ld.%01ld V", DISP_U_INT(meter_stats.V2_Min), DISP_U_DEC1(meter_stats.V2_Min));
            TG12864_DrawString(&myLCD, 0, 4, line);
            snprintf(line, sizeof(line), "L3 %3ld.%01ld V", DISP_U_INT(meter_stats.V3_Min), DISP_U_DEC1(meter_stats.V3_Min));
            TG12864_DrawString(&myLCD, 0, 6, line);
            TG12864_DrawString(&myLCD, 100, 1, "MIN");
            break;

        case PAGE_VOLT_HIRES_MAX:
            TG12864_DrawString(&myLCD, 25, 0, "[    L-N    ]");
            snprintf(line, sizeof(line), "L1 %3ld.%01ld V", DISP_U_INT(meter_stats.V1_Max), DISP_U_DEC1(meter_stats.V1_Max));
            TG12864_DrawString(&myLCD, 0, 2, line);
            snprintf(line, sizeof(line), "L2 %3ld.%01ld V", DISP_U_INT(meter_stats.V2_Max), DISP_U_DEC1(meter_stats.V2_Max));
            TG12864_DrawString(&myLCD, 0, 4, line);
            snprintf(line, sizeof(line), "L3 %3ld.%01ld V", DISP_U_INT(meter_stats.V3_Max), DISP_U_DEC1(meter_stats.V3_Max));
            TG12864_DrawString(&myLCD, 0, 6, line);
            TG12864_DrawString(&myLCD, 100, 3, "MAX");
            break;

        case PAGE_VOLT_HIRES_AVG:
            TG12864_DrawString(&myLCD, 25, 0, "[    L-N    ]");
            snprintf(line, sizeof(line), "L1 %3ld.%01ld V", DISP_U_INT(meter_stats.V1_Avg), DISP_U_DEC1(meter_stats.V1_Avg));
            TG12864_DrawString(&myLCD, 0, 2, line);
            snprintf(line, sizeof(line), "L2 %3ld.%01ld V", DISP_U_INT(meter_stats.V2_Avg), DISP_U_DEC1(meter_stats.V2_Avg));
            TG12864_DrawString(&myLCD, 0, 4, line);
            snprintf(line, sizeof(line), "L3 %3ld.%01ld V", DISP_U_INT(meter_stats.V3_Avg), DISP_U_DEC1(meter_stats.V3_Avg));
            TG12864_DrawString(&myLCD, 0, 6, line);
            TG12864_DrawString(&myLCD, 100, 5, "AVG");
            break;

        case PAGE_VOLT_HIRES_DMD:
            TG12864_DrawString(&myLCD, 25, 0, "[    L-N    ]");
            snprintf(line, sizeof(line), "L1 %3ld.%01ld V", DISP_U_INT(meter_stats.V1_Dmd), DISP_U_DEC1(meter_stats.V1_Dmd));
            TG12864_DrawString(&myLCD, 0, 2, line);
            snprintf(line, sizeof(line), "L2 %3ld.%01ld V", DISP_U_INT(meter_stats.V2_Dmd), DISP_U_DEC1(meter_stats.V2_Dmd));
            TG12864_DrawString(&myLCD, 0, 4, line);
            snprintf(line, sizeof(line), "L3 %3ld.%01ld V", DISP_U_INT(meter_stats.V3_Dmd), DISP_U_DEC1(meter_stats.V3_Dmd));
            TG12864_DrawString(&myLCD, 0, 6, line);
            TG12864_DrawString(&myLCD, 100, 7, "DMD");
            break;

        case PAGE_VOLT_LINE:
            TG12864_DrawString(&myLCD, 25, 0, "[    L-L    ]");

            snprintf(line, sizeof(line), "L12 %3ld.%01ld V",
                     DISP_U_INT(m90_results.UrmsAB),
                     DISP_U_DEC1(m90_results.UrmsAB));
            TG12864_DrawString(&myLCD, 0, 2, line);

            snprintf(line, sizeof(line), "L23 %3ld.%01ld V",
                     DISP_U_INT(m90_results.UrmsBC),
                     DISP_U_DEC1(m90_results.UrmsBC));
            TG12864_DrawString(&myLCD, 0, 4, line);

            snprintf(line, sizeof(line), "L31 %3ld.%01ld V",
                     DISP_U_INT(m90_results.UrmsCA),
                     DISP_U_DEC1(m90_results.UrmsCA));
            TG12864_DrawString(&myLCD, 0, 6, line);
            break;

        /* ================= NHÓM ĐIỆN ÁP DÂY CHI TIẾT ================= */
        case PAGE_VOLT_LINE_MIN:
            TG12864_DrawString(&myLCD, 25, 0, "[    L-L    ]");
            snprintf(line, sizeof(line), "L12 %3ld.%01ld V", DISP_U_INT(meter_stats.Vab_Min), DISP_U_DEC1(meter_stats.Vab_Min));
            TG12864_DrawString(&myLCD, 0, 2, line);
            snprintf(line, sizeof(line), "L23 %3ld.%01ld V", DISP_U_INT(meter_stats.Vbc_Min), DISP_U_DEC1(meter_stats.Vbc_Min));
            TG12864_DrawString(&myLCD, 0, 4, line);
            snprintf(line, sizeof(line), "L31 %3ld.%01ld V", DISP_U_INT(meter_stats.Vca_Min), DISP_U_DEC1(meter_stats.Vca_Min));
            TG12864_DrawString(&myLCD, 0, 6, line);
            TG12864_DrawString(&myLCD, 100, 1, "MIN");
            break;

        case PAGE_VOLT_LINE_MAX:
            TG12864_DrawString(&myLCD, 25, 0, "[    L-L    ]");
            snprintf(line, sizeof(line), "L12 %3ld.%01ld V", DISP_U_INT(meter_stats.Vab_Max), DISP_U_DEC1(meter_stats.Vab_Max));
            TG12864_DrawString(&myLCD, 0, 2, line);
            snprintf(line, sizeof(line), "L23 %3ld.%01ld V", DISP_U_INT(meter_stats.Vbc_Max), DISP_U_DEC1(meter_stats.Vbc_Max));
            TG12864_DrawString(&myLCD, 0, 4, line);
            snprintf(line, sizeof(line), "L31 %3ld.%01ld V", DISP_U_INT(meter_stats.Vca_Max), DISP_U_DEC1(meter_stats.Vca_Max));
            TG12864_DrawString(&myLCD, 0, 6, line);
            TG12864_DrawString(&myLCD, 100, 3, "MAX");
            break;

        case PAGE_VOLT_LINE_AVG:
            TG12864_DrawString(&myLCD, 25, 0, "[    L-L    ]");
            snprintf(line, sizeof(line), "L12 %3ld.%01ld V", DISP_U_INT(meter_stats.Vab_Avg), DISP_U_DEC1(meter_stats.Vab_Avg));
            TG12864_DrawString(&myLCD, 0, 2, line);
            snprintf(line, sizeof(line), "L23 %3ld.%01ld V", DISP_U_INT(meter_stats.Vbc_Avg), DISP_U_DEC1(meter_stats.Vbc_Avg));
            TG12864_DrawString(&myLCD, 0, 4, line);
            snprintf(line, sizeof(line), "L31 %3ld.%01ld V", DISP_U_INT(meter_stats.Vca_Avg), DISP_U_DEC1(meter_stats.Vca_Avg));
            TG12864_DrawString(&myLCD, 0, 6, line);
            TG12864_DrawString(&myLCD, 100, 5, "AVG");
            break;

        case PAGE_VOLT_LINE_DMD:
            TG12864_DrawString(&myLCD, 25, 0, "[    L-L    ]");
            snprintf(line, sizeof(line), "L12 %3ld.%01ld V", DISP_U_INT(meter_stats.Vab_Dmd), DISP_U_DEC1(meter_stats.Vab_Dmd));
            TG12864_DrawString(&myLCD, 0, 2, line);
            snprintf(line, sizeof(line), "L23 %3ld.%01ld V", DISP_U_INT(meter_stats.Vbc_Dmd), DISP_U_DEC1(meter_stats.Vbc_Dmd));
            TG12864_DrawString(&myLCD, 0, 4, line);
            snprintf(line, sizeof(line), "L31 %3ld.%01ld V", DISP_U_INT(meter_stats.Vca_Dmd), DISP_U_DEC1(meter_stats.Vca_Dmd));
            TG12864_DrawString(&myLCD, 0, 6, line);
            TG12864_DrawString(&myLCD, 100, 7, "DMD");
            break;

        case PAGE_CURRENT:
            TG12864_DrawString(&myLCD, 25, 0, "[     I     ]");

            snprintf(line, sizeof(line), "L1 %2ld.%03ld A",
                     DISP_I_INT(m90_results.IrmsA),
                     DISP_I_DEC3(m90_results.IrmsA));
            TG12864_DrawString(&myLCD, 0, 2, line);

            snprintf(line, sizeof(line), "L2 %2ld.%03ld A",
                     DISP_I_INT(m90_results.IrmsB),
                     DISP_I_DEC3(m90_results.IrmsB));
            TG12864_DrawString(&myLCD, 0, 4, line);

            snprintf(line, sizeof(line), "L3 %2ld.%03ld A",
                     DISP_I_INT(m90_results.IrmsC),
                     DISP_I_DEC3(m90_results.IrmsC));
            TG12864_DrawString(&myLCD, 0, 6, line);
            break;

        /* ================= NHÓM DÒNG ĐIỆN CHI TIẾT ================= */
        case PAGE_CURR_MAX:
            TG12864_DrawString(&myLCD, 25, 0, "[     I     ]");
            snprintf(line, sizeof(line), "L1 %2ld.%03ld A", DISP_I_INT(meter_stats.I1_Max), DISP_I_DEC3(meter_stats.I1_Max));
            TG12864_DrawString(&myLCD, 0, 2, line);
            snprintf(line, sizeof(line), "L2 %2ld.%03ld A", DISP_I_INT(meter_stats.I2_Max), DISP_I_DEC3(meter_stats.I2_Max));
            TG12864_DrawString(&myLCD, 0, 4, line);
            snprintf(line, sizeof(line), "L3 %2ld.%03ld A", DISP_I_INT(meter_stats.I3_Max), DISP_I_DEC3(meter_stats.I3_Max));
            TG12864_DrawString(&myLCD, 0, 6, line);
            TG12864_DrawString(&myLCD, 100, 3, "MAX");
            break;

        case PAGE_CURR_AVG:
            TG12864_DrawString(&myLCD, 25, 0, "[     I     ]");
            snprintf(line, sizeof(line), "L1 %2ld.%03ld A", DISP_I_INT(meter_stats.I1_Avg), DISP_I_DEC3(meter_stats.I1_Avg));
            TG12864_DrawString(&myLCD, 0, 2, line);
            snprintf(line, sizeof(line), "L2 %2ld.%03ld A", DISP_I_INT(meter_stats.I2_Avg), DISP_I_DEC3(meter_stats.I2_Avg));
            TG12864_DrawString(&myLCD, 0, 4, line);
            snprintf(line, sizeof(line), "L3 %2ld.%03ld A", DISP_I_INT(meter_stats.I3_Avg), DISP_I_DEC3(meter_stats.I3_Avg));
            TG12864_DrawString(&myLCD, 0, 6, line);
            TG12864_DrawString(&myLCD, 100, 5, "AVG");
            break;

        case PAGE_POWER:
            TG12864_DrawString(&myLCD, 25, 0, "[  P  (kW)  ]");

            snprintf(line, sizeof(line), "L1 %2ld.%03ld kW",
                     DISP_PWR_INT(m90_results.PmeanA_Forward),
                     DISP_PWR_DEC3(m90_results.PmeanA_Forward));
            TG12864_DrawString(&myLCD, 0, 2, line);

            snprintf(line, sizeof(line), "L2 %2ld.%03ld kW",
                     DISP_PWR_INT(m90_results.PmeanB_Forward),
                     DISP_PWR_DEC3(m90_results.PmeanB_Forward));
            TG12864_DrawString(&myLCD, 0, 4, line);

            snprintf(line, sizeof(line), "L3 %2ld.%03ld kW",
                     DISP_PWR_INT(m90_results.PmeanC_Forward),
                     DISP_PWR_DEC3(m90_results.PmeanC_Forward));
            TG12864_DrawString(&myLCD, 0, 6, line);
            break;

        /* ================= NHÓM CÔNG SUẤT P CHI TIẾT ================= */
        case PAGE_POWER_MAX:
            TG12864_DrawString(&myLCD, 25, 0, "[  P  (kW)  ]");
            snprintf(line, sizeof(line), "L1 %2ld.%03ld kW", DISP_PWR_INT(meter_stats.P1_Max), DISP_PWR_DEC3(meter_stats.P1_Max));
            TG12864_DrawString(&myLCD, 0, 2, line);
            snprintf(line, sizeof(line), "L2 %2ld.%03ld kW", DISP_PWR_INT(meter_stats.P2_Max), DISP_PWR_DEC3(meter_stats.P2_Max));
            TG12864_DrawString(&myLCD, 0, 4, line);
            snprintf(line, sizeof(line), "L3 %2ld.%03ld kW", DISP_PWR_INT(meter_stats.P3_Max), DISP_PWR_DEC3(meter_stats.P3_Max));
            TG12864_DrawString(&myLCD, 0, 6, line);
            TG12864_DrawString(&myLCD, 100, 3, "MAX");
            break;

        case PAGE_POWER_AVG:
            TG12864_DrawString(&myLCD, 25, 0, "[  P  (kW)  ]");
            snprintf(line, sizeof(line), "L1 %2ld.%03ld kW", DISP_PWR_INT(meter_stats.P1_Avg), DISP_PWR_DEC3(meter_stats.P1_Avg));
            TG12864_DrawString(&myLCD, 0, 2, line);
            snprintf(line, sizeof(line), "L2 %2ld.%03ld kW", DISP_PWR_INT(meter_stats.P2_Avg), DISP_PWR_DEC3(meter_stats.P2_Avg));
            TG12864_DrawString(&myLCD, 0, 4, line);
            snprintf(line, sizeof(line), "L3 %2ld.%03ld kW", DISP_PWR_INT(meter_stats.P3_Avg), DISP_PWR_DEC3(meter_stats.P3_Avg));
            TG12864_DrawString(&myLCD, 0, 6, line);
            TG12864_DrawString(&myLCD, 100, 5, "AVG");
            break;

        case PAGE_POWER_DMD:
            TG12864_DrawString(&myLCD, 25, 0, "[  P  (kW)  ]");
            snprintf(line, sizeof(line), "L1 %2ld.%03ld kW", DISP_PWR_INT(meter_stats.P1_Dmd), DISP_PWR_DEC3(meter_stats.P1_Dmd));
            TG12864_DrawString(&myLCD, 0, 2, line);
            snprintf(line, sizeof(line), "L2 %2ld.%03ld kW", DISP_PWR_INT(meter_stats.P2_Dmd), DISP_PWR_DEC3(meter_stats.P2_Dmd));
            TG12864_DrawString(&myLCD, 0, 4, line);
            snprintf(line, sizeof(line), "L3 %2ld.%03ld kW", DISP_PWR_INT(meter_stats.P3_Dmd), DISP_PWR_DEC3(meter_stats.P3_Dmd));
            TG12864_DrawString(&myLCD, 0, 6, line);
            TG12864_DrawString(&myLCD, 100, 7, "DMD");
            break;

        case PAGE_S:
            TG12864_DrawString(&myLCD, 25, 0, "[  S (kVA)  ]");

            snprintf(line, sizeof(line), "L1 %2ld.%03ld kVA",
                     DISP_PWR_INT(m90_results.SmeanA),
                     DISP_PWR_DEC3(m90_results.SmeanA));
            TG12864_DrawString(&myLCD, 0, 2, line);

            snprintf(line, sizeof(line), "L2 %2ld.%03ld kVA",
                     DISP_PWR_INT(m90_results.SmeanB),
                     DISP_PWR_DEC3(m90_results.SmeanB));
            TG12864_DrawString(&myLCD, 0, 4, line);

            snprintf(line, sizeof(line), "L3 %2ld.%03ld kVA",
                     DISP_PWR_INT(m90_results.SmeanC),
                     DISP_PWR_DEC3(m90_results.SmeanC));
            TG12864_DrawString(&myLCD, 0, 6, line);
            break;

        /* ================= NHÓM CÔNG SUẤT S CHI TIẾT ================= */
        case PAGE_S_MAX:
            TG12864_DrawString(&myLCD, 25, 0, "[  S (kVA)  ]");
            snprintf(line, sizeof(line), "L1 %2ld.%03ld kVA", DISP_PWR_INT(meter_stats.S1_Max), DISP_PWR_DEC3(meter_stats.S1_Max));
            TG12864_DrawString(&myLCD, 0, 2, line);
            snprintf(line, sizeof(line), "L2 %2ld.%03ld kVA", DISP_PWR_INT(meter_stats.S2_Max), DISP_PWR_DEC3(meter_stats.S2_Max));
            TG12864_DrawString(&myLCD, 0, 4, line);
            snprintf(line, sizeof(line), "L3 %2ld.%03ld kVA", DISP_PWR_INT(meter_stats.S3_Max), DISP_PWR_DEC3(meter_stats.S3_Max));
            TG12864_DrawString(&myLCD, 0, 6, line);
            TG12864_DrawString(&myLCD, 100, 3, "MAX");
            break;

        case PAGE_S_AVG:
            TG12864_DrawString(&myLCD, 25, 0, "[  S (kVA)  ]");
            snprintf(line, sizeof(line), "L1 %2ld.%03ld kVA", DISP_PWR_INT(meter_stats.S1_Avg), DISP_PWR_DEC3(meter_stats.S1_Avg));
            TG12864_DrawString(&myLCD, 0, 2, line);
            snprintf(line, sizeof(line), "L2 %2ld.%03ld kVA", DISP_PWR_INT(meter_stats.S2_Avg), DISP_PWR_DEC3(meter_stats.S2_Avg));
            TG12864_DrawString(&myLCD, 0, 4, line);
            snprintf(line, sizeof(line), "L3 %2ld.%03ld kVA", DISP_PWR_INT(meter_stats.S3_Avg), DISP_PWR_DEC3(meter_stats.S3_Avg));
            TG12864_DrawString(&myLCD, 0, 6, line);
            TG12864_DrawString(&myLCD, 100, 5, "AVG");
            break;

        case PAGE_S_DMD:
            TG12864_DrawString(&myLCD, 25, 0, "[  S (kVA)  ]");
            snprintf(line, sizeof(line), "L1 %2ld.%03ld kVA", DISP_PWR_INT(meter_stats.S1_Dmd), DISP_PWR_DEC3(meter_stats.S1_Dmd));
            TG12864_DrawString(&myLCD, 0, 2, line);
            snprintf(line, sizeof(line), "L2 %2ld.%03ld kVA", DISP_PWR_INT(meter_stats.S2_Dmd), DISP_PWR_DEC3(meter_stats.S2_Dmd));
            TG12864_DrawString(&myLCD, 0, 4, line);
            snprintf(line, sizeof(line), "L3 %2ld.%03ld kVA", DISP_PWR_INT(meter_stats.S3_Dmd), DISP_PWR_DEC3(meter_stats.S3_Dmd));
            TG12864_DrawString(&myLCD, 0, 6, line);
            TG12864_DrawString(&myLCD, 100, 7, "DMD");
            break;

        case PAGE_FREQUENCY:
            TG12864_DrawString(&myLCD, 25, 0, "[   FREQ    ]");

            snprintf(line, sizeof(line), "Freq: %2ld.%02ld Hz",
                     DISP_FREQ_INT(m90_results.Freg),
                     DISP_FREQ_DEC2(m90_results.Freg));
            TG12864_DrawString(&myLCD, 0, 3, line);
            break;

        case PAGE_TOTAL_P_PHASE_DIR:
            TG12864_DrawString(&myLCD, 25, 0, "[   P DIR   ]");

            pA = (m90_results.PmeanA_Forward > 0) ?
                 m90_results.PmeanA_Forward : m90_results.PmeanA_Reverse;

            snprintf(line, sizeof(line), "L1 %2ld.%03ld %s",
                     DISP_PWR_INT(pA),
                     DISP_PWR_DEC3(pA),
                     (m90_results.PmeanA_Reverse > 0) ? "<-" : "->");
            TG12864_DrawString(&myLCD, 0, 2, line);

            pB = (m90_results.PmeanB_Forward > 0) ?
                 m90_results.PmeanB_Forward : m90_results.PmeanB_Reverse;

            snprintf(line, sizeof(line), "L2 %2ld.%03ld %s",
                     DISP_PWR_INT(pB),
                     DISP_PWR_DEC3(pB),
                     (m90_results.PmeanB_Reverse > 0) ? "<-" : "->");
            TG12864_DrawString(&myLCD, 0, 4, line);

            pC = (m90_results.PmeanC_Forward > 0) ?
                 m90_results.PmeanC_Forward : m90_results.PmeanC_Reverse;

            snprintf(line, sizeof(line), "L3 %2ld.%03ld %s",
                     DISP_PWR_INT(pC),
                     DISP_PWR_DEC3(pC),
                     (m90_results.PmeanC_Reverse > 0) ? "<-" : "->");
            TG12864_DrawString(&myLCD, 0, 6, line);
            break;

        case PAGE_TOTAL_P_3PHASE:
            TG12864_DrawString(&myLCD, 25, 0, "[  P TOTAL  ]");

            pTot = (m90_results.PmeanT_Forward > 0) ?
                   m90_results.PmeanT_Forward : m90_results.PmeanT_Reverse;

            snprintf(line, sizeof(line), "P: %2ld.%03ld kW",
                     DISP_PWR_INT(pTot),
                     DISP_PWR_DEC3(pTot));
            TG12864_DrawString(&myLCD, 0, 2, line);

            signQ = (m90_results.QmeanT < 0) ? '-' : ' ';
            qTot = labs((long)m90_results.QmeanT);

            snprintf(line, sizeof(line), "Q: %c%ld.%03ld kVAr",
                     signQ,
                     DISP_PWR_INT(qTot),
                     DISP_PWR_DEC3(qTot));
            TG12864_DrawString(&myLCD, 0, 4, line);

            snprintf(line, sizeof(line), "S: %2ld.%03ld kVA",
                     DISP_PWR_INT(m90_results.SAmeanT),
                     DISP_PWR_DEC3(m90_results.SAmeanT));
            TG12864_DrawString(&myLCD, 0, 6, line);
            break;

        case PAGE_SYS_INFO:
            TG12864_DrawString(&myLCD, 25, 0, "[ SYS INFO  ]");

            snprintf(line, sizeof(line), "Freq: %2ld.%02ld Hz",
                     DISP_FREQ_INT(m90_results.Freg),
                     DISP_FREQ_DEC2(m90_results.Freg));
            TG12864_DrawString(&myLCD, 0, 2, line);

            snprintf(line, sizeof(line), "Temp: %3d C", m90_results.Temp);
            TG12864_DrawString(&myLCD, 0, 4, line);

            signPF = (m90_results.PF_Total < 0) ? '-' : ' ';
            pf_val = labs((long)m90_results.PF_Total);

            snprintf(line, sizeof(line), "PF Tot:%c%ld.%03ld",
                     signPF,
                     DISP_PF_INT(pf_val),
                     DISP_PF_DEC3(pf_val));
            TG12864_DrawString(&myLCD, 0, 6, line);
            break;

        case PAGE_COSPHI:
            TG12864_DrawString(&myLCD, 25, 0, "[    PF     ]");

            signPF = (m90_results.PF_A < 0) ? '-' : ' ';
            pf_val = labs((long)m90_results.PF_A);

            snprintf(line, sizeof(line), "PF L1: %c%ld.%03ld",
                     signPF,
                     DISP_PF_INT(pf_val),
                     DISP_PF_DEC3(pf_val));
            TG12864_DrawString(&myLCD, 0, 2, line);

            signPF = (m90_results.PF_B < 0) ? '-' : ' ';
            pf_val = labs((long)m90_results.PF_B);

            snprintf(line, sizeof(line), "PF L2: %c%ld.%03ld",
                     signPF,
                     DISP_PF_INT(pf_val),
                     DISP_PF_DEC3(pf_val));
            TG12864_DrawString(&myLCD, 0, 4, line);

            signPF = (m90_results.PF_C < 0) ? '-' : ' ';
            pf_val = labs((long)m90_results.PF_C);

            snprintf(line, sizeof(line), "PF L3: %c%ld.%03ld",
                     signPF,
                     DISP_PF_INT(pf_val),
                     DISP_PF_DEC3(pf_val));
            TG12864_DrawString(&myLCD, 0, 6, line);
            break;

        case PAGE_ENERGY:
        {
            TG12864_DrawString(&myLCD, 25, 0, "[  ENERGY   ]");

            uint32_t imp = m90_results.ActiveEnergy_Forward_T * RAW_ENERGY_SCALE;
            uint32_t exp = m90_results.ActiveEnergy_Reverse_T * RAW_ENERGY_SCALE;

            uint32_t imp_int  = imp / ENERGY_PULSE_PER_KWH;
            uint32_t imp_dec4 = (imp % ENERGY_PULSE_PER_KWH) / 32;

            uint32_t exp_int  = exp / ENERGY_PULSE_PER_KWH;
            uint32_t exp_dec4 = (exp % ENERGY_PULSE_PER_KWH) / 32;

            snprintf(line, sizeof(line), "IMP:%3lu.%04lu kWh", imp_int, imp_dec4);
            TG12864_DrawString(&myLCD, 0, 2, line);

            snprintf(line, sizeof(line), "EXP:%3lu.%04lu kWh", exp_int, exp_dec4);
            TG12864_DrawString(&myLCD, 0, 4, line);
            break;
        }

        default:
            break;
    }
}



void App_LCD_Task(void) {
    static Menu_Page_t current_page = PAGE_VOLTAGE_CURRENT;
    Button_Event_t receivedBtn;
    osStatus_t status = osMessageQueueGet(buttonQueueHandle, &receivedBtn, NULL, 100);

    if(status == osOK){
        TG12864_Clear(&myLCD);
        switch(receivedBtn){

            case BTN_DOWN: {
                // =========================================================
                // NÚT DOWN: CUỘN TRANG CHI TIẾT TRONG CÙNG 1 NHÓM
                // =========================================================
                // Nhóm 1: Điện áp pha (từ 0 đến 5)
                if (current_page >= PAGE_VOLTAGE_CURRENT && current_page < PAGE_VOLT_HIRES_DMD) current_page++;
                else if (current_page == PAGE_VOLT_HIRES_DMD) current_page = PAGE_VOLTAGE_CURRENT;

                // Nhóm 2: Điện áp dây (từ 6 đến 10)
                else if (current_page >= PAGE_VOLT_LINE && current_page < PAGE_VOLT_LINE_DMD) current_page++;
                else if (current_page == PAGE_VOLT_LINE_DMD) current_page = PAGE_VOLT_LINE;

                // Nhóm 3: Dòng điện (từ 11 đến 13)
                else if (current_page >= PAGE_CURRENT && current_page < PAGE_CURR_AVG) current_page++;
                else if (current_page == PAGE_CURR_AVG) current_page = PAGE_CURRENT;

                // Nhóm 4: Công suất P (từ 14 đến 17)
                else if (current_page >= PAGE_POWER && current_page < PAGE_POWER_DMD) current_page++;
                else if (current_page == PAGE_POWER_DMD) current_page = PAGE_POWER;

                // Nhóm 5: Công suất S (từ 18 đến 21)
                else if (current_page >= PAGE_S && current_page < PAGE_S_DMD) current_page++;
                else if (current_page == PAGE_S_DMD) current_page = PAGE_S;

                // Nhóm Công suất tổng (từ 23 đến 24)
                else if (current_page == PAGE_TOTAL_P_PHASE_DIR) current_page = PAGE_TOTAL_P_3PHASE;
                else if (current_page == PAGE_TOTAL_P_3PHASE) current_page = PAGE_TOTAL_P_PHASE_DIR;

                // Các page đơn lẻ (Freq, Sys Info, Cosphi, Energy) thì nút DOWN đứng im hoặc vòng lại chính nó
                break;
            }

            case BTN_UP: {
                // =========================================================
                // NÚT UP: BƯỚC NHẢY TỚI NHÓM CHÍNH TIẾP THEO
                // =========================================================
                if (current_page >= PAGE_VOLTAGE_CURRENT && current_page <= PAGE_VOLT_HIRES_DMD)
                    current_page = PAGE_VOLT_LINE;

                else if (current_page >= PAGE_VOLT_LINE && current_page <= PAGE_VOLT_LINE_DMD)
                    current_page = PAGE_CURRENT;

                else if (current_page >= PAGE_CURRENT && current_page <= PAGE_CURR_AVG)
                    current_page = PAGE_POWER;

                else if (current_page >= PAGE_POWER && current_page <= PAGE_POWER_DMD)
                    current_page = PAGE_S;

                else if (current_page >= PAGE_S && current_page <= PAGE_S_DMD)
                    current_page = PAGE_FREQUENCY;

                else if (current_page == PAGE_FREQUENCY)
                    current_page = PAGE_TOTAL_P_PHASE_DIR;

                else if (current_page >= PAGE_TOTAL_P_PHASE_DIR && current_page <= PAGE_TOTAL_P_3PHASE)
                    current_page = PAGE_SYS_INFO;

                else if (current_page == PAGE_SYS_INFO)
                    current_page = PAGE_COSPHI;

                else if (current_page == PAGE_COSPHI)
                    current_page = PAGE_ENERGY;

                else if (current_page == PAGE_ENERGY)
                    current_page = PAGE_VOLTAGE_CURRENT; // Vòng lại đầu tiên

                break;
            }

            case BTN_ESC: {
                if (current_page == PAGE_ENERGY) {
                    // Logic reset điện năng
                	App_Reset_Energy();
                    TG12864_Clear(&myLCD);
                    TG12864_DrawString(&myLCD, 15, 3, " ENERGY RESET! ");
                    osDelay(500);
                    TG12864_Clear(&myLCD);
                    current_page = PAGE_ENERGY;
                } else {
                    current_page = PAGE_VOLTAGE_CURRENT;
                }
                break;
            }

            case BTN_SET: break;
            case BTN_NONE: break;
        }
    }
    App_LCD_Display(current_page);
}
void App_ReadMeasurements_Task(void) {
//    M90E32_Measurements_t raw_data;

    // Đọc dữ liệu thô từ IC
    if (M90E32_ReadAllMeasurements(&m90_device, &m90_results) != M90E32_OK) {
        printf("[%s] Read Error!\r\n", LOG_TAG);
        return;
    }
    // Cập nhật Min/Max.
    App_Process_Measurements();

    App_Update_MinMax();
    App_Calculate_Demand_And_Avg();
    M90E32_Read_EmmState0(&m90_device, &sys_status0);
    Modbus_Update_Data();
}

void App_ReadButton_Task(void) {
    Read_Button_Task();
}
