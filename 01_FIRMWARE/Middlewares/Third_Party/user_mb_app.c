#include "mb.h"
#include "mbport.h"
#include <stdint.h>
#include "M90E32AS_driver.h"
#include "app.h"

// TỔNG THANH GHI 136: Từ địa chỉ 0 đến 135
#define REG_HOLDING_START   0
#define REG_HOLDING_NREGS   150 // Dư một chút cho an toàn

// Mảng đệm chứa dữ liệu Modbus
uint16_t usRegHoldingBuf[REG_HOLDING_NREGS];

// Kéo dữ liệu thực tế từ app.c sang
extern M90E32_Measurements_t m90_results;
extern Meter_Stats_t meter_stats;

/* =======================================================================
 * MACRO QUY ĐỔI GIÁ TRỊ RAW SANG FLOAT (DÀNH CHO 32-BIT)
 * ======================================================================= */
#define MODBUS_SCALE_P(raw)       (((float)(raw) * (float)RAW_POWER_SCALE_uW) / 1000000000.0f)
#define MODBUS_SCALE_ENERGY(raw)  (((float)(raw) * (float)RAW_ENERGY_SCALE) / (float)ENERGY_PULSE_PER_KWH)

/**
 * @brief Hàm chuyển đổi Float (32-bit) sang 2 thanh ghi (16-bit) chuẩn CDAB (Word Swap)
 */
static void FloatToModbusRegisters(float value, uint16_t *regBuffer, uint16_t offset) {
    union {
        float f;
        uint16_t u16[2];
    } converter;
    converter.f = value;
    regBuffer[offset]     = converter.u16[1]; // High Word
    regBuffer[offset + 1] = converter.u16[0]; // Low Word
}

/**
 * @brief Đồng bộ toàn bộ thông số đo lường sang mảng đệm Modbus (Mixed Data Type)
 */
void Modbus_Update_Data(void) {
    int idx = 0; // Tự động tăng con trỏ

    /* =========================================================
     * PHẦN 1: DỮ LIỆU 16-BIT SỐ NGUYÊN (INT16 / UINT16)
     * Master/Gateway sẽ tự chia Scale Factor (10, 100, 1000)
     * ========================================================= */

    // 1. Điện áp L-N & L-L (Scale x10 -> Chia 10) (0 - 5)
    // VD: raw = 22050 (V*100) -> Chia 10 = 2205 (Đẩy lên SCADA) -> SCADA chia 10 = 220.5V
    usRegHoldingBuf[idx++] = (uint16_t)(m90_results.UrmsA / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(m90_results.UrmsB / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(m90_results.UrmsC / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(m90_results.UrmsAB / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(m90_results.UrmsBC / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(m90_results.UrmsCA / 10);

    // 2. Dòng điện (Scale x100 -> Chia 100) (6 - 9)
    // VD: raw*5 = I(mA). Nếu I = 50.25A -> I(mA) = 50250 -> Cần gửi số 5025 (Cho vừa uint16) -> Chia 10
    usRegHoldingBuf[idx++] = (uint16_t)((m90_results.IrmsA * RAW_CURRENT_SCALE_MA) / 10);
    usRegHoldingBuf[idx++] = (uint16_t)((m90_results.IrmsB * RAW_CURRENT_SCALE_MA) / 10);
    usRegHoldingBuf[idx++] = (uint16_t)((m90_results.IrmsC * RAW_CURRENT_SCALE_MA) / 10);
    usRegHoldingBuf[idx++] = (uint16_t)((m90_results.IrmsN * RAW_CURRENT_SCALE_MA) / 10);

    // 3. Freq, Temp, PF, Angles (Giữ nguyên Raw, SCADA tự chia theo Datasheet IC) (10 - 21)
    usRegHoldingBuf[idx++] = (uint16_t)m90_results.Freg;        // Chia 100 -> Hz
    usRegHoldingBuf[idx++] = (int16_t)m90_results.Temp;         // Nhiệt độ trực tiếp
    usRegHoldingBuf[idx++] = (int16_t)m90_results.PF_Total;     // Chia 1000
    usRegHoldingBuf[idx++] = (int16_t)m90_results.PF_A;
    usRegHoldingBuf[idx++] = (int16_t)m90_results.PF_B;
    usRegHoldingBuf[idx++] = (int16_t)m90_results.PF_C;
    usRegHoldingBuf[idx++] = (uint16_t)m90_results.UAngleA;     // Chia 10 -> Degree
    usRegHoldingBuf[idx++] = (uint16_t)m90_results.UAngleB;
    usRegHoldingBuf[idx++] = (uint16_t)m90_results.UAngleC;
    usRegHoldingBuf[idx++] = (uint16_t)m90_results.PAngleA;
    usRegHoldingBuf[idx++] = (uint16_t)m90_results.PAngleB;
    usRegHoldingBuf[idx++] = (uint16_t)m90_results.PAngleC;

    // 4. STATS Điện áp L-N (Scale x10) (22 - 33)
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.V1_Max / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.V1_Min / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.V1_Avg / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.V1_Dmd / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.V2_Max / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.V2_Min / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.V2_Avg / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.V2_Dmd / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.V3_Max / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.V3_Min / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.V3_Avg / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.V3_Dmd / 10);

    // 5. STATS Điện áp L-L (Scale x10) (34 - 45)
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.Vab_Max / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.Vab_Min / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.Vab_Avg / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.Vab_Dmd / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.Vbc_Max / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.Vbc_Min / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.Vbc_Avg / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.Vbc_Dmd / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.Vca_Max / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.Vca_Min / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.Vca_Avg / 10);
    usRegHoldingBuf[idx++] = (uint16_t)(meter_stats.Vca_Dmd / 10);

    // 6. STATS Dòng điện (Scale x100) (46 - 51)
    usRegHoldingBuf[idx++] = (uint16_t)((meter_stats.I1_Max * RAW_CURRENT_SCALE_MA) / 10);
    usRegHoldingBuf[idx++] = (uint16_t)((meter_stats.I1_Avg * RAW_CURRENT_SCALE_MA) / 10);
    usRegHoldingBuf[idx++] = (uint16_t)((meter_stats.I2_Max * RAW_CURRENT_SCALE_MA) / 10);
    usRegHoldingBuf[idx++] = (uint16_t)((meter_stats.I2_Avg * RAW_CURRENT_SCALE_MA) / 10);
    usRegHoldingBuf[idx++] = (uint16_t)((meter_stats.I3_Max * RAW_CURRENT_SCALE_MA) / 10);
    usRegHoldingBuf[idx++] = (uint16_t)((meter_stats.I3_Avg * RAW_CURRENT_SCALE_MA) / 10);


    /* =========================================================
     * PHẦN 2: DỮ LIỆU 32-BIT FLOAT (TỪ IDX 52 ĐẾN 135)
     * Dành cho Power và Energy. Không cần Scale ở SCADA.
     * ========================================================= */

    // 7. Active Power Forward (Import) (52 - 58)
    FloatToModbusRegisters(MODBUS_SCALE_P(m90_results.PmeanT_Forward), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(m90_results.PmeanA_Forward), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(m90_results.PmeanB_Forward), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(m90_results.PmeanC_Forward), usRegHoldingBuf, idx); idx += 2;

    // 8. Active Power Reverse (Export) (60 - 66)
    FloatToModbusRegisters(MODBUS_SCALE_P(m90_results.PmeanT_Reverse), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(m90_results.PmeanA_Reverse), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(m90_results.PmeanB_Reverse), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(m90_results.PmeanC_Reverse), usRegHoldingBuf, idx); idx += 2;

    // 9. Reactive Power (Q) (68 - 74)
    FloatToModbusRegisters(MODBUS_SCALE_P(m90_results.QmeanT), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(m90_results.QmeanA), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(m90_results.QmeanB), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(m90_results.QmeanC), usRegHoldingBuf, idx); idx += 2;

    // 10. Apparent Power (S) (76 - 82)
    FloatToModbusRegisters(MODBUS_SCALE_P(m90_results.SAmeanT), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(m90_results.SmeanA),  usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(m90_results.SmeanB),  usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(m90_results.SmeanC),  usRegHoldingBuf, idx); idx += 2;

    // 11. Energy Import (84 - 90)
    FloatToModbusRegisters(MODBUS_SCALE_ENERGY(m90_results.ActiveEnergy_Forward_T), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_ENERGY(m90_results.ActiveEnergy_Forward_A), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_ENERGY(m90_results.ActiveEnergy_Forward_B), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_ENERGY(m90_results.ActiveEnergy_Forward_C), usRegHoldingBuf, idx); idx += 2;

    // 12. Energy Export (92 - 98)
    FloatToModbusRegisters(MODBUS_SCALE_ENERGY(m90_results.ActiveEnergy_Reverse_T), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_ENERGY(m90_results.ActiveEnergy_Reverse_A), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_ENERGY(m90_results.ActiveEnergy_Reverse_B), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_ENERGY(m90_results.ActiveEnergy_Reverse_C), usRegHoldingBuf, idx); idx += 2;

    // 13. STATS Active Power P (100 - 116)
    FloatToModbusRegisters(MODBUS_SCALE_P(meter_stats.P1_Max), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(meter_stats.P1_Avg), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(meter_stats.P1_Dmd), usRegHoldingBuf, idx); idx += 2;

    FloatToModbusRegisters(MODBUS_SCALE_P(meter_stats.P2_Max), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(meter_stats.P2_Avg), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(meter_stats.P2_Dmd), usRegHoldingBuf, idx); idx += 2;

    FloatToModbusRegisters(MODBUS_SCALE_P(meter_stats.P3_Max), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(meter_stats.P3_Avg), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(meter_stats.P3_Dmd), usRegHoldingBuf, idx); idx += 2;

    // 14. STATS Apparent Power S (118 - 134)
    FloatToModbusRegisters(MODBUS_SCALE_P(meter_stats.S1_Max), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(meter_stats.S1_Avg), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(meter_stats.S1_Dmd), usRegHoldingBuf, idx); idx += 2;

    FloatToModbusRegisters(MODBUS_SCALE_P(meter_stats.S2_Max), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(meter_stats.S2_Avg), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(meter_stats.S2_Dmd), usRegHoldingBuf, idx); idx += 2;

    FloatToModbusRegisters(MODBUS_SCALE_P(meter_stats.S3_Max), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(meter_stats.S3_Avg), usRegHoldingBuf, idx); idx += 2;
    FloatToModbusRegisters(MODBUS_SCALE_P(meter_stats.S3_Dmd), usRegHoldingBuf, idx); idx += 2;
}

// -----------------------------------------------------------------------------
// FreeModbus Holding Register Callback
// -----------------------------------------------------------------------------
eMBErrorCode eMBRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
    eMBErrorCode eStatus = MB_ENOERR;
    int iRegIndex;

    int register_offset = (int)(usAddress - 1);

    if ((register_offset >= REG_HOLDING_START) && (register_offset + usNRegs <= REG_HOLDING_NREGS))
    {
        iRegIndex = register_offset;

        switch (eMode)
        {
            case MB_REG_READ:
                while (usNRegs > 0)
                {
                    *pucRegBuffer++ = (UCHAR)(usRegHoldingBuf[iRegIndex] >> 8);
                    *pucRegBuffer++ = (UCHAR)(usRegHoldingBuf[iRegIndex] & 0xFF);
                    iRegIndex++;
                    usNRegs--;
                }
                break;

            case MB_REG_WRITE:
                eStatus = MB_ENOREG;
                break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

// Các hàm Callback không sử dụng
eMBErrorCode eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs) { return MB_ENOREG; }
eMBErrorCode eMBRegCoilsCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode) { return MB_ENOREG; }
eMBErrorCode eMBRegDiscreteCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete) { return MB_ENOREG; }
