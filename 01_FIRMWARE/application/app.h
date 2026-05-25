/*
 * app.h
 *
 *  Created on: Mar 18, 2026
 *      Author: 84373
 */

#ifndef APP_H_
#define APP_H_

#include <stdio.h>
#include <string.h>
#include "main.h"

typedef enum {
    PAGE_VOLTAGE_CURRENT= 0,
	PAGE_VOLT_NEUTRAL,
	PAGE_VOLT_HIRES_MIN,
	PAGE_VOLT_HIRES_MAX,
	PAGE_VOLT_HIRES_AVG,
	PAGE_VOLT_HIRES_DMD,
    // --- NHÓM ĐIỆN ÁP ---
    PAGE_VOLT_LINE,
	PAGE_VOLT_LINE_MIN,      // <--- THÊM DÒNG NÀY (Hiển thị điện áp dây L-L)
	PAGE_VOLT_LINE_MAX,
    PAGE_VOLT_LINE_AVG,
	PAGE_VOLT_LINE_DMD,

    // --- NHÓM DÒNG ĐIỆN ---
    PAGE_CURRENT,
    PAGE_CURR_MAX,
    PAGE_CURR_AVG,

    // CÔNG SUẤT P
    PAGE_POWER,
    PAGE_POWER_MAX,
	PAGE_POWER_AVG,
	PAGE_POWER_DMD,
	// NHÓM 5: CÔNG SUẤT S (kVA)
	PAGE_S,   // [MỚI] Trang Công suất biểu kiến (S)
	PAGE_S_MAX,
	PAGE_S_AVG,
//	PAGE_S_MIN,
	PAGE_S_DMD,
	PAGE_FREQUENCY,

	PAGE_TOTAL_P_PHASE_DIR,
	PAGE_TOTAL_P_3PHASE,

	// --- NHOM TAN SO ---
	PAGE_SYS_INFO,         // [MỚI] Trang thông số tổng hợp (Tần số, Nhiệt độ chip)
	PAGE_COSPHI,
    // --- NHÓM ĐIỆN NĂNG ---
    PAGE_ENERGY
} Menu_Page_t;

typedef enum{
	BTN_NONE = 0,
	BTN_UP,
	BTN_DOWN,
	BTN_SET,
	BTN_ESC
} Button_Event_t;
// Thêm vào app.h (Bên trên phần khai báo hàm)
typedef struct {
    // ---- 32-BIT: CÔNG SUẤT (P, S) ----
    int32_t P1_Max, P2_Max, P3_Max;
    int32_t P1_Avg, P2_Avg, P3_Avg;
    int32_t P1_Dmd, P2_Dmd, P3_Dmd;
    int32_t S1_Max, S2_Max, S3_Max;
    int32_t S1_Avg, S2_Avg, S3_Avg;
    int32_t S1_Dmd, S2_Dmd, S3_Dmd;

    // ---- 16-BIT UN-SIGNED: ĐIỆN ÁP & DÒNG ĐIỆN ----
    uint16_t V1_Max, V2_Max, V3_Max;
    uint16_t V1_Min, V2_Min, V3_Min;
    uint16_t V1_Avg, V2_Avg, V3_Avg;
    uint16_t V1_Dmd, V2_Dmd, V3_Dmd;

    uint16_t Vab_Max, Vbc_Max, Vca_Max;
    uint16_t Vab_Min, Vbc_Min, Vca_Min;
    uint16_t Vab_Avg, Vbc_Avg, Vca_Avg;
    uint16_t Vab_Dmd, Vbc_Dmd, Vca_Dmd;

    uint16_t I1_Max, I2_Max, I3_Max;
    uint16_t I1_Avg, I2_Avg, I3_Avg;
} Meter_Stats_t;

//M90E32_Measurements_t m90_results;

void App_Init(void);
void App_Loop(void);
void App_LCD_Task(void);
void App_ReadMeasurements_Task(void);
void App_ReadButton_Task(void);

#endif /* APP_H_ */
