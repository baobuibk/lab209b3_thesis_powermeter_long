#include "M90E32AS_driver.h"
#include "stdint.h"
#include "stdio.h"
#include "math.h"

#define M90E32_Delay(mdelay) HAL_Delay(mdelay)

/**
 * @brief Internal SPI transfer abstraction.
 */
static M90E32_Status_t SPI_Transfer(M90E32_t *device, uint8_t *tx, uint8_t *rx, uint16_t len) {
    HAL_StatusTypeDef status;

    if (rx == NULL) {
        status = HAL_SPI_Transmit(device->hspi, tx, len, 100);
    } else {
        status = HAL_SPI_TransmitReceive(device->hspi, tx, rx, len, 100);
    }

    if (status == HAL_OK)
        return M90E32_OK;
    return M90E32_ERROR;
}

/**
 * @brief GPIO wrapper for hardware control pins.
 */
void M90E32_WritePin(GPIO_TypeDef *port, uint16_t pin, uint8_t state) {
    HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void M90E32_Reset(M90E32_t *device){
    M90E32_Write(M90E32_SOFTRESET_REG, M90E32_SOFTRESET_VALUE, device);
}

/**
 * @brief Initializes the M90E32AS by setting power modes and verifying SPI connection.
 */
M90E32_Status_t M90E32_Init(M90E32_t *device) {
    M90E32_WritePin(device->PM_PORT, device->PM0_PIN, 1);
    M90E32_WritePin(device->PM_PORT, device->PM1_PIN, 1);
    M90E32_WritePin(device->PORT, device->CS_PIN, 1);

    M90E32_Delay(100); // Wait for POR timing
    M90E32_Reset(device);
    M90E32_Delay(5);

    uint16_t temp_val = 0;
    M90E32_Status_t status = M90E32_Read(M90E32_T0_REG, &temp_val, device);

    if (status == M90E32_OK) {
        if (temp_val != 0x0000 && temp_val != 0xFFFF) {
            return M90E32_OK;
        }
    }
    return M90E32_ERROR;
}

void M90E32_MeterEn(M90E32_t *device){
		M90E32_Write(M90E32_CFGREGACCEN_REG, M90E32_CFGREGACCEN_UNLOCK, device);

	    // 2. Bật Metering
	    M90E32_Write(M90E32_METEREN_REG, 0x0F, device);

	    // 3. Khóa lại cho an toàn
	    M90E32_Write(M90E32_CFGREGACCEN_REG, M90E32_CFGREGACCEN_LOCK, device);
}

/**
 * @brief Reads a 16-bit value from a device register.
 */
M90E32_Status_t M90E32_Read(uint16_t reg, uint16_t *pData, M90E32_t *device) {
    uint8_t tx[4] = { 0 }, rx[4] = { 0 };
    uint16_t cmd = 0x8000 | (reg & 0x03FF); // Bit 15 = 1 for Read

    tx[0] = (uint8_t) (cmd >> 8);
    tx[1] = (uint8_t) (cmd & 0xFF);
    tx[2] = 0x00; // Dummy bytes
    tx[3] = 0x00;

    M90E32_WritePin(device->PORT, device->CS_PIN, 0);
    M90E32_Status_t status = SPI_Transfer(device, tx, rx, 4);
    M90E32_WritePin(device->PORT, device->CS_PIN, 1);

    if (status == M90E32_OK) {
        *pData = (uint16_t) ((rx[2] << 8) | rx[3]);
    }
    return status;
}

/**
 * @brief Writes a 16-bit value to a device register.
 */
M90E32_Status_t M90E32_Write(uint16_t reg, uint16_t data, M90E32_t *device) {
    uint8_t tx[4];
    uint16_t cmd = (reg & 0x03FF); // Bit 15 = 0 for Write
    uint16_t spi_check_value = 0;

    tx[0] = (uint8_t) (cmd >> 8);
    tx[1] = (uint8_t) (cmd & 0xFF);
    tx[2] = (uint8_t) (data >> 8);
    tx[3] = (uint8_t) (data & 0xFF);

    M90E32_WritePin(device->PORT, device->CS_PIN, 0);
    SPI_Transfer(device, tx, NULL, 4);
    M90E32_WritePin(device->PORT, device->CS_PIN, 1);

    M90E32_Read(M90E32_LASTSPIDATA_REG, &spi_check_value, device);

    if(spi_check_value != data){
        return M90E32_ERROR;
    }
    return M90E32_OK;
}

M90E32_Status_t M90E32_Config_Grid(M90E32_t *device ,uint16_t freq, uint16_t mode3P3W, uint16_t calcMethod, uint16_t cfSrc){
    uint16_t mmode0_val;
    M90E32_Status_t status;

    status = M90E32_Read(M90E32_MMODE0_REG, &mmode0_val, device);
    if(status != M90E32_OK) return status;
// them vao
//    mmode0_val |= (M90E32_MMODE0_ENPA | M90E32_MMODE0_ENPB | M90E32_MMODE0_ENPC);

    if(freq == 60) mmode0_val |= M90E32_MMODE0_FREQ60HZ;
    else           mmode0_val &= ~M90E32_MMODE0_FREQ60HZ;

    if(mode3P3W)   mmode0_val |= M90E32_MMODE0_MODE3P3W;
    else           mmode0_val &= ~M90E32_MMODE0_MODE3P3W;

    if(calcMethod){
        mmode0_val |= M90E32_MMODE0_ABSENP;
        mmode0_val |= M90E32_MMODE0_ABSENQ;
    } else {
        mmode0_val &= ~M90E32_MMODE0_ABSENP;
        mmode0_val &= ~M90E32_MMODE0_ABSENQ;
    }

    M90E32_Write(M90E32_CFGREGACCEN_REG, M90E32_CFGREGACCEN_UNLOCK, device);
    status = M90E32_Write(M90E32_MMODE0_REG, mmode0_val, device);

    M90E32_Write(M90E32_CFGREGACCEN_REG, M90E32_CFGREGACCEN_LOCK, device);

    return status;
}

float M90E32_Cal_Phase(float Phase1_Value, float Phase2_Value, float phase_deg){
    while(phase_deg < 0) {phase_deg += 360.0f;}
    phase_deg = phase_deg * (M_PI / 180.0f);
    return (sqrt(Phase1_Value*Phase1_Value + Phase2_Value*Phase2_Value - 2*Phase1_Value*Phase2_Value*cosf(phase_deg)));
}

void M90E32_Calibration(M90E32_t *device) {
    // 1. Mở khóa quyền ghi thanh ghi (Mã 0x55AA vào thanh ghi 0x7F)
    M90E32_Write(M90E32_CFGREGACCEN_REG, M90E32_CFGREGACCEN_UNLOCK, device);

    // =========================================================
    // 2. GHI HỆ SỐ HIỆU CHUẨN ĐIỆN ÁP (GAIN)
    // =========================================================
    uint32_t pl_constant_val = 29504609;

	M90E32_Write(M90E32_PLCONSTH_REG, (uint16_t)(pl_constant_val >> 16), device);
	M90E32_Write(M90E32_PLCONSTL_REG, (uint16_t)(pl_constant_val & 0xFFFF), device);
    uint16_t UgainA = (uint16_t)(CALIB_V_RATIO_A * M90E32_DEFAULT_GAIN + 0.5f);
    uint16_t UgainB = (uint16_t)(CALIB_V_RATIO_B * M90E32_DEFAULT_GAIN + 0.5f);
    uint16_t UgainC = (uint16_t)(CALIB_V_RATIO_C * M90E32_DEFAULT_GAIN + 0.5f);

    M90E32_Write(M90E32_UGAIN_A_REG, UgainA, device);
    M90E32_Write(M90E32_UGAIN_B_REG, UgainB, device);
    M90E32_Write(M90E32_UGAIN_C_REG, UgainC, device);

    // =========================================================
    // 3. GHI HỆ SỐ HIỆU CHUẨN DÒNG ĐIỆN (GAIN)
    // =========================================================
    uint16_t IgainA = (uint16_t)(CALIB_I_RATIO_A * M90E32_DEFAULT_GAIN);
    uint16_t IgainB = (uint16_t)(CALIB_I_RATIO_B * M90E32_DEFAULT_GAIN);
    uint16_t IgainC = (uint16_t)(CALIB_I_RATIO_C * M90E32_DEFAULT_GAIN);

    M90E32_Write(M90E32_IGAIN_A_REG, IgainA, device);
    M90E32_Write(M90E32_IGAIN_B_REG, IgainB, device);
    M90E32_Write(M90E32_IGAIN_C_REG, IgainC, device);

    // =========================================================
    // 4. GHI HỆ SỐ BÙ NHIỄU KHÔNG TẢI (OFFSET)
    // Lưu ý: Đây là số bù 2 (Signed 16-bit). Phải mò giá trị âm để triệt tiêu nhiễu.
    // =========================================================
    int16_t U_Offset_Val = -300; // Thay đổi số này (VD: -50, -100...) để ép U về 0
    int16_t I_Offset_Val = -300; // Thay đổi số này (VD: -100, -200...) để ép I về 0

    // Ép kiểu uint16_t để hàm Write không báo lỗi type casting
    M90E32_Write(M90E32_UOFFSET_A_REG, (uint16_t)U_Offset_Val, device);
    M90E32_Write(M90E32_UOFFSET_B_REG, (uint16_t)U_Offset_Val, device);
    M90E32_Write(M90E32_UOFFSET_C_REG, (uint16_t)U_Offset_Val, device);

    M90E32_Write(M90E32_IOFFSET_A_REG, (uint16_t)I_Offset_Val, device);
    M90E32_Write(M90E32_IOFFSET_B_REG, (uint16_t)I_Offset_Val, device);
    M90E32_Write(M90E32_IOFFSET_C_REG, (uint16_t)I_Offset_Val, device);

    // 5. Khóa thanh ghi cấu hình lại để bảo vệ dữ liệu
    M90E32_Write(M90E32_CFGREGACCEN_REG, M90E32_CFGREGACCEN_LOCK, device);
}
M90E32_Status_t M90E32_ReadAllMeasurements(M90E32_t *device, M90E32_Measurements_t *results) {
    uint16_t low, high;
    M90E32_Status_t status = M90E32_OK;
    int32_t raw_32;

    // ====================================================================
    // 1. CÔNG SUẤT TÁC DỤNG P - RAW 32-bit có dấu
    // ====================================================================
    M90E32_Read(M90E32_PMEANT_REG, &high, device);
    M90E32_Read(M90E32_PMEANTLSB_REG, &low, device);
    raw_32 = (int32_t)(((uint32_t)high << 16) | low);
    if (raw_32 >= 0) {
        results->PmeanT_Forward = raw_32;
        results->PmeanT_Reverse = 0;
    } else {
        results->PmeanT_Forward = 0;
        results->PmeanT_Reverse = -raw_32;
    }

    M90E32_Read(M90E32_PMEANA_REG, &high, device);
    M90E32_Read(M90E32_PMEANALSB_REG, &low, device);
    raw_32 = (int32_t)(((uint32_t)high << 16) | low);
    if (raw_32 >= 0) {
        results->PmeanA_Forward = raw_32;
        results->PmeanA_Reverse = 0;
    } else {
        results->PmeanA_Forward = 0;
        results->PmeanA_Reverse = -raw_32;
    }

    M90E32_Read(M90E32_PMEANB_REG, &high, device);
    M90E32_Read(M90E32_PMEANBLSB_REG, &low, device);
    raw_32 = (int32_t)(((uint32_t)high << 16) | low);
    if (raw_32 >= 0) {
        results->PmeanB_Forward = raw_32;
        results->PmeanB_Reverse = 0;
    } else {
        results->PmeanB_Forward = 0;
        results->PmeanB_Reverse = -raw_32;
    }

    M90E32_Read(M90E32_PMEANC_REG, &high, device);
    M90E32_Read(M90E32_PMEANCLSB_REG, &low, device);
    raw_32 = (int32_t)(((uint32_t)high << 16) | low);
    if (raw_32 >= 0) {
        results->PmeanC_Forward = raw_32;
        results->PmeanC_Reverse = 0;
    } else {
        results->PmeanC_Forward = 0;
        results->PmeanC_Reverse = -raw_32;
    }

    // ====================================================================
    // 2. CÔNG SUẤT PHẢN KHÁNG Q - RAW 32-bit có dấu
    // ====================================================================
    M90E32_Read(M90E32_QMEANT_REG, &high, device);
    M90E32_Read(M90E32_QMEANTLSB_REG, &low, device);
    results->QmeanT = (int32_t)(((uint32_t)high << 16) | low);

    M90E32_Read(M90E32_QMEANA_REG, &high, device);
    M90E32_Read(M90E32_QMEANALSB_REG, &low, device);
    results->QmeanA = (int32_t)(((uint32_t)high << 16) | low);

    M90E32_Read(M90E32_QMEANB_REG, &high, device);
    M90E32_Read(M90E32_QMEANBLSB_REG, &low, device);
    results->QmeanB = (int32_t)(((uint32_t)high << 16) | low);

    M90E32_Read(M90E32_QMEANC_REG, &high, device);
    M90E32_Read(M90E32_QMEANCLSB_REG, &low, device);
    results->QmeanC = (int32_t)(((uint32_t)high << 16) | low);

    // ====================================================================
    // 3. CÔNG SUẤT BIỂU KIẾN S - RAW 32-bit
    // ====================================================================
    M90E32_Read(M90E32_SMEANT_REG, &high, device);
    M90E32_Read(M90E32_SMEANTLSB_REG, &low, device);
    results->SAmeanT = (int32_t)(((uint32_t)high << 16) | low);

    M90E32_Read(M90E32_SMEANA_REG, &high, device);
    M90E32_Read(M90E32_SMEANALSB_REG, &low, device);
    results->SmeanA = (int32_t)(((uint32_t)high << 16) | low);

    M90E32_Read(M90E32_SMEANB_REG, &high, device);
    M90E32_Read(M90E32_SMEANBLSB_REG, &low, device);
    results->SmeanB = (int32_t)(((uint32_t)high << 16) | low);

    M90E32_Read(M90E32_SMEANC_REG, &high, device);
    M90E32_Read(M90E32_SMEANCLSB_REG, &low, device);
    results->SmeanC = (int32_t)(((uint32_t)high << 16) | low);

    // ====================================================================
    // 4. ĐIỆN ÁP RMS - RAW 16-bit
    // 1 count = 0.01V
    // ====================================================================
    M90E32_Read(M90E32_URMSA_REG, &low, device);
    results->UrmsA = (int16_t)low;

    M90E32_Read(M90E32_URMSB_REG, &low, device);
    results->UrmsB = (int16_t)low;

    M90E32_Read(M90E32_URMSC_REG, &low, device);
    results->UrmsC = (int16_t)low;

    // ====================================================================
    // 5. DÒNG ĐIỆN RMS - RAW 16-bit
    // Scale hiển thị xử lý ở app/modbus
    // ====================================================================
    M90E32_Read(M90E32_IRMSA_REG, &low, device);
    results->IrmsA = (int16_t)low;

    M90E32_Read(M90E32_IRMSB_REG, &low, device);
    results->IrmsB = (int16_t)low;

    M90E32_Read(M90E32_IRMSC_REG, &low, device);
    results->IrmsC = (int16_t)low;

    M90E32_Read(M90E32_IRMSN_REG, &low, device);
    results->IrmsN = (int16_t)low;

    // ====================================================================
    // 6. GÓC PHA - RAW 16-bit
    // 1 count = 0.1 degree
    // ====================================================================
    M90E32_Read(M90E32_U_ANGLE_A_REG, &low, device);
    results->UAngleA = (int16_t)low;

    M90E32_Read(M90E32_U_ANGLE_B_REG, &low, device);
    results->UAngleB = (int16_t)low;

    M90E32_Read(M90E32_U_ANGLE_C_REG, &low, device);
    results->UAngleC = (int16_t)low;

    M90E32_Read(M90E32_P_ANGLE_A_REG, &low, device);
    results->PAngleA = (int16_t)low;

    M90E32_Read(M90E32_P_ANGLE_B_REG, &low, device);
    results->PAngleB = (int16_t)low;

    M90E32_Read(M90E32_P_ANGLE_C_REG, &low, device);
    results->PAngleC = (int16_t)low;

    // ====================================================================
    // 7. TẦN SỐ, NHIỆT ĐỘ, T0 - RAW 16-bit
    // Freq: 1 count = 0.01Hz
    // Temp: raw IC, không nhân 10 nữa
    // ====================================================================
    M90E32_Read(M90E32_FREQ_REG, &low, device);
    results->Freg = (int16_t)low;

    M90E32_Read(M90E32_TEMP_REG, &low, device);
    results->Temp = (int16_t)low;

    M90E32_Read(M90E32_T0_REG, &low, device);
    results->T0 = (int16_t)low;

    // ====================================================================
    // 8. POWER FACTOR - RAW 16-bit có dấu
    // Scale thường là x1000, xử lý hiển thị ở app
    // ====================================================================
    M90E32_Read(M90E32_PF_A_REG, &low, device);
    results->PF_A = (int16_t)low;

    M90E32_Read(M90E32_PF_B_REG, &low, device);
    results->PF_B = (int16_t)low;

    M90E32_Read(M90E32_PF_C_REG, &low, device);
    results->PF_C = (int16_t)low;

    M90E32_Read(M90E32_PF_TOTAL_REG, &low, device);
    results->PF_Total = (int16_t)low;

    // ====================================================================
    // 9. ĐIỆN ÁP DÂY - KHÔNG CÓ THANH GHI RAW RIÊNG
    // Đây là giá trị tính toán từ raw Urms + raw UAngle.
    // Lưu theo cùng scale với điện áp: 1 count = 0.01V
    // ====================================================================
    results->UrmsAB = (int16_t)(M90E32_Cal_Phase(
        (float)results->UrmsA / 100.0f,
        (float)results->UrmsB / 100.0f,
        (float)(results->UAngleA - results->UAngleB) / 10.0f
    ) * 100.0f);

    results->UrmsBC = (int16_t)(M90E32_Cal_Phase(
        (float)results->UrmsB / 100.0f,
        (float)results->UrmsC / 100.0f,
        (float)(results->UAngleB - results->UAngleC) / 10.0f
    ) * 100.0f);

    results->UrmsCA = (int16_t)(M90E32_Cal_Phase(
        (float)results->UrmsC / 100.0f,
        (float)results->UrmsA / 100.0f,
        (float)(results->UAngleC - results->UAngleA) / 10.0f
    ) * 100.0f);

    // ====================================================================
    // 10. ĐIỆN NĂNG - RAW 16-bit cộng dồn
    // Không scale sang Wh/kWh trong driver
    // ====================================================================
    M90E32_Read(M90E32_APENERGYT_REG, &low, device);
    results->ActiveEnergy_Forward_T += (uint32_t)low;

    M90E32_Read(M90E32_APENERGYA_REG, &low, device);
    results->ActiveEnergy_Forward_A += (uint32_t)low;

    M90E32_Read(M90E32_APENERGYB_REG, &low, device);
    results->ActiveEnergy_Forward_B += (uint32_t)low;

    M90E32_Read(M90E32_APENERGYC_REG, &low, device);
    results->ActiveEnergy_Forward_C += (uint32_t)low;

    M90E32_Read(M90E32_ANENERGYT_REG, &low, device);
    results->ActiveEnergy_Reverse_T += (uint32_t)low;

    M90E32_Read(M90E32_ANENERGYA_REG, &low, device);
    results->ActiveEnergy_Reverse_A += (uint32_t)low;

    M90E32_Read(M90E32_ANENERGYB_REG, &low, device);
    results->ActiveEnergy_Reverse_B += (uint32_t)low;

    M90E32_Read(M90E32_ANENERGYC_REG, &low, device);
    results->ActiveEnergy_Reverse_C += (uint32_t)low;

    return status;
}

void M90E32_Read_EmmState0(M90E32_t *device, uint16_t *pData){
    M90E32_Read(M90E32_EMMSTATE0_REG, pData , device);
}

//void M90E32_PrintMeasurements(M90E32_Measurements_t *results) {
//    printf("\r\n--- M90E32AS FULL MEASUREMENT REPORT ---\r\n");
//
//    printf("VOLTAGE (V):\r\n");
//    printf("  L-N: A:%6.1f | B:%6.1f | C:%6.1f\r\n", results->UrmsA, results->UrmsB, results->UrmsC);
//    printf("  L-L: AB:%5.1f | BC:%5.1f | CA:%5.1f\r\n", results->UrmsAB, results->UrmsBC, results->UrmsCA);
//
//    printf("CURRENT (A):\r\n");
//    printf("  RMS: A:%7.3f | B:%7.3f | C:%7.3f\r\n", results->IrmsA, results->IrmsB, results->IrmsC);
//    printf("  Neutral: %7.3f\r\n", results->IrmsN);
//
//    printf("POWER:\r\n");
//    printf("  Forward(W) : Tot:%7.1f | A:%7.1f | B:%7.1f | C:%7.1f\r\n",
//            results->PmeanT_Forward, results->PmeanA_Forward, results->PmeanB_Forward, results->PmeanC_Forward);
//    printf("  Reverse(W) : Tot:%7.1f | A:%7.1f | B:%7.1f | C:%7.1f\r\n",
//            results->PmeanT_Reverse, results->PmeanA_Reverse, results->PmeanB_Reverse, results->PmeanC_Reverse);
//    printf("  React (var): Tot:%7.1f | A:%7.1f | B:%7.1f | C:%7.1f\r\n",
//            results->QmeanT, results->QmeanA, results->QmeanB, results->QmeanC);
//    printf("  Appar (VA) : Tot:%7.1f | A:%7.1f | B:%7.1f | C:%7.1f\r\n",
//            results->SAmeanT, results->SmeanA, results->SmeanB, results->SmeanC);
//
//    printf("ENERGY (kWh):\r\n");
//    printf("  Import : Tot:%8.2f | A:%8.2f | B:%8.2f | C:%8.2f\r\n",
//            results->ActiveEnergy_Forward_T, results->ActiveEnergy_Forward_A, results->ActiveEnergy_Forward_B, results->ActiveEnergy_Forward_C);
//    printf("  Export :vf Tot:%8.2f | A:%8.2f | B:%8.2f | C:%8.2f\r\n",
//            results->ActiveEnergy_Reverse_T, results->ActiveEnergy_Reverse_A, results->ActiveEnergy_Reverse_B, results->ActiveEnergy_Reverse_C);
//
//    printf("PHASE ANGLES (deg):\r\n");
//    printf("  Voltage: A:%6.1f | B:%6.1f | C:%6.1f\r\n", results->UAngleA, results->UAngleB, results->UAngleC);
//    printf("  Current: A:%6.1f | B:%6.1f | C:%6.1f\r\n", results->PAngleA, results->PAngleB, results->PAngleC);
//
//    printf("SYSTEM:\r\n");
//    printf("  Frequency: %5.2f Hz | Temp: %4.1f C | T0: %04X\r\n", results->Freg, results->Temp, (uint16_t)results->T0);
//    printf("----------------------------------------\r\n");
//}

void M90E32_CheckSystemStatus(M90E32_t *device, uint16_t *pData) {
    M90E32_Read_EmmState0(device, pData);

    printf("\r\n--- SYSTEM STATUS REPORT ---\r\n");
    printf("Raw EMMState0: 0x%04X\r\n", *pData);

    if (*pData & M90E32_EMM0_OI_PHASE_A) printf("[!] ALERT: Over-Current Phase A\r\n");
    if (*pData & M90E32_EMM0_OI_PHASE_B) printf("[!] ALERT: Over-Current Phase B\r\n");
    if (*pData & M90E32_EMM0_OI_PHASE_C) printf("[!] ALERT: Over-Current Phase C\r\n");

    if (*pData & M90E32_EMM0_OV_PHASE_A) printf("[!] ALERT: Over-Voltage Phase A\r\n");
    if (*pData & M90E32_EMM0_OV_PHASE_B) printf("[!] ALERT: Over-Voltage Phase B\r\n");
    if (*pData & M90E32_EMM0_OV_PHASE_C) printf("[!] ALERT: Over-Voltage Phase C\r\n");

    if (*pData & M90E32_EMM0_U_REV_WINST) printf("[X] ERROR: Voltage Phase Sequence Error\r\n");
    if (*pData & M90E32_EMM0_I_REV_WINST) printf("[X] ERROR: Current Phase Sequence Error\r\n");

    if (*pData & M90E32_EMM0_TP_NOLOAD) printf("[i] Info: System in No-Load condition\r\n");
    if (*pData & M90E32_EMM0_CF1_REV)    printf("[i] Direction: REVERSE Power (Exporting)\r\n");
    else                                printf("[i] Direction: FORWARD Power (Importing)\r\n");

    printf("----------------------------\r\n");
}


