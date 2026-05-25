/*
 * M90E32AS_driver.h
 *
 * Driver cho hệ thống đo lường điện năng 3 pha 2 chiều.
 * Quy ước:
 * - Driver đọc và lưu giá trị RAW từ IC.
 * - App / LCD / Modbus chịu trách nhiệm đổi RAW sang đơn vị hiển thị.
 */

#ifndef M90E32AS_DRIVER_H_
#define M90E32AS_DRIVER_H_

#include "main.h"

/* ================= DEVICE / REGISTER DEFINES ================= */

#define M90E32_PLCONSTH_REG          0x31
#define M90E32_PLCONSTL_REG          0x32

/**
 * @brief M90E32AS Device Management Structure
 */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *PORT;
    uint16_t CS_PIN;
    GPIO_TypeDef *PM_PORT;
    uint16_t PM0_PIN;
    uint16_t PM1_PIN;
} M90E32_t;

/**
 * @brief Driver Return Status Enumeration
 */
typedef enum {
    M90E32_OK      = 0x00,
    M90E32_ERROR   = 0x01,
    M90E32_TIMEOUT = 0x02,
    M90E32_BUSY    = 0x03
} M90E32_Status_t;

/**
 * @brief Struct lưu toàn bộ dữ liệu đo lường dạng RAW.
 */
typedef struct {
    // --- 32-bit measurement ---
    int32_t PmeanT_Forward, PmeanA_Forward, PmeanB_Forward, PmeanC_Forward;
    int32_t PmeanT_Reverse, PmeanA_Reverse, PmeanB_Reverse, PmeanC_Reverse;
    int32_t QmeanT, QmeanA, QmeanB, QmeanC;
    int32_t SAmeanT, SmeanA, SmeanB, SmeanC;

    // --- Energy pulse accumulator ---
    uint32_t ActiveEnergy_Forward_T, ActiveEnergy_Forward_A, ActiveEnergy_Forward_B, ActiveEnergy_Forward_C;
    uint32_t ActiveEnergy_Reverse_T, ActiveEnergy_Reverse_A, ActiveEnergy_Reverse_B, ActiveEnergy_Reverse_C;

    // --- 16-bit measurement ---
    int16_t UrmsA, UrmsB, UrmsC;
    int16_t UrmsAB, UrmsBC, UrmsCA;
    int16_t IrmsA, IrmsB, IrmsC, IrmsN;
    int16_t PAngleA, PAngleB, PAngleC;
    int16_t UAngleA, UAngleB, UAngleC;
    int16_t PF_A, PF_B, PF_C, PF_Total;
    int16_t Freg;
    int16_t Temp;
    int16_t T0;
} M90E32_Measurements_t;


/* ================= SOFTWARE SCALE =================
 * K_I_SOFTWARE dùng để scale dòng/công suất/điện năng ở tầng hiển thị.
 * Không dùng để ghi trực tiếp vào thanh ghi gain của IC.
 */
#define K_I_SOFTWARE                 5.0f
#define K_I_SOFTWARE_INT             5

//#define ENERGY_PULSE_PER_WH        320000ULL
#define ENERGY_PULSE_PER_KWH       320000ULL



/* ================= RAW LSB DEFINES =================
 * Đây là LSB gốc/logic để đổi RAW sang đơn vị vật lý.
 */
#define M90E32_VOLTAGE_LSB           0.01f       // V / count
#define M90E32_CURRENT_LSB           0.001f      // A / count trước software scale
#define M90E32_POWER_LSB             0.00032f    // W / count trước software scale
#define M90E32_ENERGY_LSB            0.000003125f// Wh / pulse trước software scale
#define M90E32_CELSIUS_LSB           1.0f        // °C / count
#define M90E32_DEGREE_LSB            0.1f        // degree / count
#define M90E32_FREQ_LSB              0.01f       // Hz / count
#define M90E32_PF_LSB                0.001f      // PF / count


/* ================= INTEGER DISPLAY SCALE =================
 * Dùng cho app.c để format LCD bằng integer.
 */
#define RAW_VOLTAGE_SCALE            100         // raw / 100 = V
#define RAW_CURRENT_SCALE_MA         K_I_SOFTWARE_INT
#define RAW_FREQ_SCALE               100         // raw / 100 = Hz
#define RAW_ANGLE_SCALE              10          // raw / 10 = degree
#define RAW_PF_SCALE                 1000        // raw / 1000 = PF

// Power raw -> micro-Watt sau software scale:
// 0.00032 W = 320 uW
#define RAW_POWER_SCALE_uW           (320 * K_I_SOFTWARE_INT)

// Energy raw pulse software multiplier
#define RAW_ENERGY_SCALE             1.0f


/* ================= DEADBAND ENGINEERING VALUES ================= */

#define DEADBAND_VOLTAGE_V           5.0f
#define DEADBAND_CURRENT_A           0.03f
#define DEADBAND_POWER_W             2.0f


/* ================= DEADBAND RAW VALUES =================
 * Dùng khi cần lọc trên dữ liệu RAW.
 */
#define DEADBAND_VOLTAGE_RAW         ((int32_t)(DEADBAND_VOLTAGE_V / M90E32_VOLTAGE_LSB))
#define DEADBAND_CURRENT_RAW         ((int32_t)((DEADBAND_CURRENT_A * 1000.0f) / RAW_CURRENT_SCALE_MA))
#define DEADBAND_POWER_RAW           ((int32_t)((DEADBAND_POWER_W * 1000000.0f) / RAW_POWER_SCALE_uW))

/*----------------- Cấu hình & Trạng thái -------------------------*/
#define M90E32_METEREN_REG          ((uint16_t)0x00) // Metering Enable
#define M90E32_CHANNELMAPI_REG      ((uint16_t)0x01) // Current Channel Mapping
#define M90E32_CHANNELMAPU_REG      ((uint16_t)0x02) // Voltage Channel Mapping
#define M90E32_T0_REG               ((uint16_t)0x23) // Nominal temperature (T0)

#define M90E32_MMODE0_REG           ((uint16_t)0x33) // Metering Method Configuration
#define M90E32_MMODE1_REG           ((uint16_t)0x34) // PGA Gain Configuration

#define M90E32_SOFTRESET_REG        ((uint16_t)0x70) // Software Reset
#define M90E32_EMMSTATE0_REG        ((uint16_t)0x71) // EMMSTATE0
#define M90E32_EMMSTATE1_REG        ((uint16_t)0x72) // EMMSTATE1

#define M90E32_SOFTRESET_VALUE      ((uint16_t)0x789A)
#define M90E32_CFGREGACCEN_REG      ((uint16_t)0x7F) // Register Access Enable
#define M90E32_CFGREGACCEN_UNLOCK   ((uint16_t)0x55AA)
#define M90E32_CFGREGACCEN_LOCK     ((uint16_t)0x0000)
#define M90E32_LASTSPIDATA_REG      ((uint16_t)0x78U)

/*----------------- MMode0 Bit Definitions ------------------------*/
#define M90E32_MMODE0_ENPC_Pos      (0U)
#define M90E32_MMODE0_ENPC          (0x1U << M90E32_MMODE0_ENPC_Pos)
#define M90E32_MMODE0_ENPB_Pos      (1U)
#define M90E32_MMODE0_ENPB          (0x1U << M90E32_MMODE0_ENPB_Pos)
#define M90E32_MMODE0_ENPA_Pos      (2U)
#define M90E32_MMODE0_ENPA          (0x1U << M90E32_MMODE0_ENPA_Pos)

#define M90E32_MMODE0_ABSENP_POS    (3U)
#define M90E32_MMODE0_ABSENP        (0x1U << M90E32_MMODE0_ABSENP_POS)
#define M90E32_MMODE0_ABSENQ_POS    (4U)
#define M90E32_MMODE0_ABSENQ        (0x1U << M90E32_MMODE0_ABSENQ_POS)

#define M90E32_MMODE0_MODE3P3W_Pos  (8U)
#define M90E32_MMODE0_MODE3P3W      (0x1U << M90E32_MMODE0_MODE3P3W_Pos)
#define M90E32_MMODE0_FREQ60HZ_Pos  (12U)
#define M90E32_MMODE0_FREQ60HZ      (0x1U << M90E32_MMODE0_FREQ60HZ_Pos)

/*----------------- Thanh ghi Điện năng (Energy) -----------------*/
#define M90E32_APENERGYT_REG        ((uint16_t)0x80U) // Total Forward Active Energy
#define M90E32_APENERGYA_REG        ((uint16_t)0x81U) // Phase A Forward Active Energy
#define M90E32_APENERGYB_REG        ((uint16_t)0x82U) // Phase B Forward Active Energy
#define M90E32_APENERGYC_REG        ((uint16_t)0x83U) // Phase C Forward Active Energy

#define M90E32_ANENERGYT_REG        ((uint16_t)0x84U) // Total Reverse Active Energy
#define M90E32_ANENERGYA_REG        ((uint16_t)0x85U) // Phase A Reverse Active Energy
#define M90E32_ANENERGYB_REG        ((uint16_t)0x86U) // Phase B Reverse Active Energy
#define M90E32_ANENERGYC_REG        ((uint16_t)0x87U) // Phase C Reverse Active Energy
//=================================================

#define M90E32_PF_A_REG            ((uint16_t)0xBD)
#define M90E32_PF_B_REG            ((uint16_t)0xBE)
#define M90E32_PF_C_REG            ((uint16_t)0xBF)
#define M90E32_PF_TOTAL_REG        ((uint16_t)0xBC)
/*----------------- Thanh ghi Công suất Thực (Active Power - P) ---*/
#define M90E32_PMEANT_REG           ((uint16_t)0xB0U) // Total Active Power
#define M90E32_PMEANA_REG           ((uint16_t)0xB1U) // Phase A Active Power
#define M90E32_PMEANB_REG           ((uint16_t)0xB2U) // Phase B Active Power
#define M90E32_PMEANC_REG           ((uint16_t)0xB3U) // Phase C Active Power
// Low Words cho P
#define M90E32_PMEANTLSB_REG        ((uint16_t)0xC0U)
#define M90E32_PMEANALSB_REG        ((uint16_t)0xC1U)
#define M90E32_PMEANBLSB_REG        ((uint16_t)0xC2U)
#define M90E32_PMEANCLSB_REG        ((uint16_t)0xC3U)

/*----------------- Thanh ghi Công suất Phản kháng (Reactive - Q) -*/
#define M90E32_QMEANT_REG           ((uint16_t)0xB4U) // Total Reactive Power
#define M90E32_QMEANA_REG           ((uint16_t)0xB5U) // Phase A Reactive Power
#define M90E32_QMEANB_REG           ((uint16_t)0xB6U) // Phase B Reactive Power
#define M90E32_QMEANC_REG           ((uint16_t)0xB7U) // Phase C Reactive Power
// Low Words cho Q
#define M90E32_QMEANTLSB_REG        ((uint16_t)0xC4U)
#define M90E32_QMEANALSB_REG        ((uint16_t)0xC5U)
#define M90E32_QMEANBLSB_REG        ((uint16_t)0xC6U)
#define M90E32_QMEANCLSB_REG        ((uint16_t)0xC7U)

/*----------------- Thanh ghi Công suất Biểu kiến (Apparent - S) --*/
#define M90E32_SMEANT_REG           ((uint16_t)0xB8U) // Total Apparent Power
#define M90E32_SMEANA_REG           ((uint16_t)0xB9U) // Phase A Apparent Power
#define M90E32_SMEANB_REG           ((uint16_t)0xBAU) // Phase B Apparent Power
#define M90E32_SMEANC_REG           ((uint16_t)0xBBU) // Phase C Apparent Power
// Low Words cho S
#define M90E32_SMEANTLSB_REG        ((uint16_t)0xC8U)
#define M90E32_SMEANALSB_REG        ((uint16_t)0xC9U)
#define M90E32_SMEANBLSB_REG        ((uint16_t)0xCAU)
#define M90E32_SMEANCLSB_REG        ((uint16_t)0xCBU)

/*----------------- Thanh ghi Điện áp & Dòng điện (RMS) -----------*/
#define M90E32_URMSA_REG            ((uint16_t)0xD9U) // Phase A Voltage RMS
#define M90E32_URMSB_REG            ((uint16_t)0xDAU) // Phase B Voltage RMS
#define M90E32_URMSC_REG            ((uint16_t)0xDBU) // Phase C Voltage RMS
#define M90E32_IRMSN_REG            ((uint16_t)0xDCU) // N Line Current RMS
#define M90E32_IRMSA_REG            ((uint16_t)0xDDU) // Phase A Current RMS
#define M90E32_IRMSB_REG            ((uint16_t)0xDEU) // Phase B Current RMS
#define M90E32_IRMSC_REG            ((uint16_t)0xDFU) // Phase C Current RMS

// Low Words cho U và I (Để tăng độ phân giải nếu cần)
#define M90E32_URMSALSB_REG         ((uint16_t)0xE9U)
#define M90E32_URMSBLSB_REG         ((uint16_t)0xEAU)
#define M90E32_URMSCLSB_REG         ((uint16_t)0xEBU)
#define M90E32_IRMSALSB_REG         ((uint16_t)0xEDU)
#define M90E32_IRMSBLSB_REG         ((uint16_t)0xEEU)
#define M90E32_IRMSCLSB_REG         ((uint16_t)0xEFU)

/*----------------- EMM State 0 (Cảnh báo & Hệ thống) -------------*/
// Over Current Status (Bit 15-13)
#define M90E32_EMM0_OI_PHASE_A      ((uint16_t)0x8000U)
#define M90E32_EMM0_OI_PHASE_B      ((uint16_t)0x4000U)
#define M90E32_EMM0_OI_PHASE_C      ((uint16_t)0x2000U)

// Over Voltage Status (Bit 12-10)
#define M90E32_EMM0_OV_PHASE_A      ((uint16_t)0x1000U)
#define M90E32_EMM0_OV_PHASE_B      ((uint16_t)0x0800U)
#define M90E32_EMM0_OV_PHASE_C      ((uint16_t)0x0400U)

// Phase Sequence Error (Bit 9-8)
#define M90E32_EMM0_U_REV_WINST     ((uint16_t)0x0200U)
#define M90E32_EMM0_I_REV_WINST     ((uint16_t)0x0100U)
// Neutral Line Status (Bit 7)
#define M90E32_EMM0_IN_OV0ST            ((uint16_t)0x0080U) // Bit 7: Calculated N line over current

// No-Load Condition Status (Bit 6-4)
#define M90E32_EMM0_TQ_NOLOAD           ((uint16_t)0x0040U) // Bit 6: All phase sum reactive power no-load
#define M90E32_EMM0_TP_NOLOAD           ((uint16_t)0x0020U) // Bit 5: All phase sum active power no-load
#define M90E32_EMM0_TAS_NOLOAD          ((uint16_t)0x0010U) // Bit 4: All phase sum apparent power no-load

// CF Forward/Reverse Status (Bit 3-0)
#define M90E32_EMM0_CF1_REV         ((uint16_t)0x0008U) // Bit 3 (Tổng công suất phát ngược)
#define M90E32_EMM0_CF2_REV         ((uint16_t)0x0004U) // Bit 2
#define M90E32_EMM0_CF3_REV         ((uint16_t)0x0002U) // Bit 1
#define M90E32_EMM0_CF4_REV         ((uint16_t)0x0001U) // Bit 0

/*----------------- Tần số, Góc pha và Nhiệt độ -------------------*/
#define M90E32_FREQ_REG             ((uint16_t)0xF8U) // 1 LSB = 0.01Hz

#define M90E32_P_ANGLE_A_REG        ((uint16_t)0xF9U) // Góc dòng điện Pha A
#define M90E32_P_ANGLE_B_REG        ((uint16_t)0xFAU)
#define M90E32_P_ANGLE_C_REG        ((uint16_t)0xFBU)

#define M90E32_TEMP_REG             ((uint16_t)0xFCU) // Nhiệt độ chip

#define M90E32_U_ANGLE_A_REG        ((uint16_t)0xFDU) // Góc điện áp Pha A
#define M90E32_U_ANGLE_B_REG        ((uint16_t)0xFEU)
#define M90E32_U_ANGLE_C_REG        ((uint16_t)0xFFU)
/* ================= THÔNG SỐ HIỆU CHUẨN (CALIBRATION) ================= */
/* Measurement Calibration Registers (Table-9, Page 59) */
#define M90E32_UGAIN_A_REG          ((uint16_t)0x61U)
#define M90E32_IGAIN_A_REG          ((uint16_t)0x62U)
#define M90E32_UOFFSET_A_REG        ((uint16_t)0x63U)
#define M90E32_IOFFSET_A_REG        ((uint16_t)0x64U)

#define M90E32_UGAIN_B_REG          ((uint16_t)0x65U)
#define M90E32_IGAIN_B_REG          ((uint16_t)0x66U)
#define M90E32_UOFFSET_B_REG        ((uint16_t)0x67U)
#define M90E32_IOFFSET_B_REG        ((uint16_t)0x68U)

#define M90E32_UGAIN_C_REG          ((uint16_t)0x69U)
#define M90E32_IGAIN_C_REG          ((uint16_t)0x6AU)
#define M90E32_UOFFSET_C_REG        ((uint16_t)0x6BU)
#define M90E32_IOFFSET_C_REG        ((uint16_t)0x6CU)

// 2. Hằng số Gain mặc định của IC (0x8000 = 32768)
#define M90E32_DEFAULT_GAIN         32768.0f

// 3. Hệ số hiệu chuẩn Điện áp (Tính bằng: V_Thực_Tế / V_Trên_LCD)
#define CALIB_V_RATIO_A             209.4f/138.1f*(211.3f/210.5f)*(209.4f/210.0f)
#define CALIB_V_RATIO_B             209.4f/138.1f*(211.3f/210.5f)*(209.4f/210.0f)
#define CALIB_V_RATIO_C             209.4f/138.1f*(211.3f/210.5f)*(209.4f/210.0f)

// 4. Hệ số hiệu chuẩn Dòng điện (Tính bằng: I_Thực_Tế / I_Trên_LCD)
#define K_I_SOFTWARE                5.0f


#define CALIB_I_RATIO_A  ((0.26f)/(0.315f))*(6.31f/6.015f)
#define CALIB_I_RATIO_B  ((0.26f)/(0.315f))*(6.31f/6.015f)
#define CALIB_I_RATIO_C  ((0.26f)/(0.315f))*(6.31f/6.015f)
/*=================== FUNCTION PROTOTYPES ==========================*/
M90E32_Status_t M90E32_Init(M90E32_t *device);
M90E32_Status_t M90E32_Read(uint16_t reg, uint16_t *pData, M90E32_t *device);
M90E32_Status_t M90E32_Write(uint16_t reg, uint16_t data, M90E32_t *device);
M90E32_Status_t M90E32_Config_Grid(M90E32_t *device, uint16_t freq, uint16_t mode3P3W, uint16_t calcMethod, uint16_t cfSrc);
void M90E32_Read_EmmState0(M90E32_t *device, uint16_t *pData);
void M90E32_MeterEn(M90E32_t *device);
M90E32_Status_t M90E32_ReadAllMeasurements(M90E32_t *device, M90E32_Measurements_t *results);
void M90E32_PrintMeasurements(M90E32_Measurements_t *results);
void M90E32_CheckSystemStatus(M90E32_t *device, uint16_t *pData);
void M90E32_Calibration(M90E32_t *device);
float M90E32_Cal_Phase(float Phase1_Value, float Phase2_Value, float phase_deg);
#endif /* M90E32AS_DRIVER_H_ */
