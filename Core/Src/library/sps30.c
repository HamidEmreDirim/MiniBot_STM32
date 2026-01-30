#include "sps30.h"
#include <string.h> // For memcpy

// Sensirion CRC-8 Polynomial
#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF

// Helper: Calculate CRC
static uint8_t Sensirion_CalcCRC(uint8_t *data, uint8_t len) {
    uint8_t crc = CRC8_INIT;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

// Helper: Convert 4 bytes (Big Endian) to Float (IEEE754)
static float BytesToFloat(uint8_t *raw) {
    uint32_t val = 0;
    val |= (uint32_t)raw[0] << 24;
    val |= (uint32_t)raw[1] << 16;
    val |= (uint32_t)raw[2] << 8;
    val |= (uint32_t)raw[3];
    
    float f;
    memcpy(&f, &val, 4);
    return f;
}

HAL_StatusTypeDef SPS30_Init(I2C_HandleTypeDef *hi2c) {
    // Perform a reset to ensure clean state
    uint8_t cmd[2] = {SPS30_CMD_RESET >> 8, SPS30_CMD_RESET & 0xFF};
    if (HAL_I2C_Master_Transmit(hi2c, SPS30_I2C_ADDRESS, cmd, 2, 100) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(100); // Wait for reset
    return HAL_OK;
}

HAL_StatusTypeDef SPS30_StartMeasurement(I2C_HandleTypeDef *hi2c) {
    uint8_t data[5];
    
    // Command ID
    data[0] = SPS30_CMD_START_MEASUREMENT >> 8;
    data[1] = SPS30_CMD_START_MEASUREMENT & 0xFF;
    
    // Argument: Output Format (Float)
    data[2] = SPS30_FORMAT_FLOAT >> 8;
    data[3] = SPS30_FORMAT_FLOAT & 0xFF;
    
    // Checksum for the argument
    data[4] = Sensirion_CalcCRC(&data[2], 2);
    
    return HAL_I2C_Master_Transmit(hi2c, SPS30_I2C_ADDRESS, data, 5, 100);
}

HAL_StatusTypeDef SPS30_StopMeasurement(I2C_HandleTypeDef *hi2c) {
    uint8_t cmd[2] = {SPS30_CMD_STOP_MEASUREMENT >> 8, SPS30_CMD_STOP_MEASUREMENT & 0xFF};
    return HAL_I2C_Master_Transmit(hi2c, SPS30_I2C_ADDRESS, cmd, 2, 100);
}

uint8_t SPS30_ReadDataReady(I2C_HandleTypeDef *hi2c) {
    uint8_t cmd[2] = {SPS30_CMD_READ_DATA_READY >> 8, SPS30_CMD_READ_DATA_READY & 0xFF};
    uint8_t rx_buf[3]; // 2 bytes data + 1 byte CRC
    
    if (HAL_I2C_Master_Transmit(hi2c, SPS30_I2C_ADDRESS, cmd, 2, 100) != HAL_OK) return 0;
    if (HAL_I2C_Master_Receive(hi2c, SPS30_I2C_ADDRESS, rx_buf, 3, 100) != HAL_OK) return 0;
    
    // Ideally check CRC here, but for boolean flag, simpler check:
    if (rx_buf[1] == 1) return 1;
    return 0;
}

HAL_StatusTypeDef SPS30_ReadMeasurement(I2C_HandleTypeDef *hi2c, SPS30_Data_t *data) {
    uint8_t cmd[2] = {SPS30_CMD_READ_VALUES >> 8, SPS30_CMD_READ_VALUES & 0xFF};
    
    // Buffer size: 10 values * (4 bytes float + 2 bytes CRC) = 60 bytes
    // Sensirion sends data as: [Byte1][Byte2][CRC] [Byte3][Byte4][CRC] ...
    uint8_t rx_buf[60]; 
    
    if (HAL_I2C_Master_Transmit(hi2c, SPS30_I2C_ADDRESS, cmd, 2, 100) != HAL_OK) return HAL_ERROR;
    
    if (HAL_I2C_Master_Receive(hi2c, SPS30_I2C_ADDRESS, rx_buf, 60, 500) != HAL_OK) return HAL_ERROR;
    
    // We need to extract the raw bytes and reassemble them skipping CRCs
    // A float consists of 2 words (2 bytes each), each word has a CRC.
    // Structure in RxBuf for ONE float: [MMSB, MLSB, CRC, LMSB, LLSB, CRC]
    
    float *pData = (float*)data; // Treat struct as array of floats
    
    for (int i = 0; i < 10; i++) {
        int base = i * 6; // 6 bytes per value in raw buffer
        
        // Check CRC 1 (High Word)
        if (Sensirion_CalcCRC(&rx_buf[base], 2) != rx_buf[base+2]) return HAL_ERROR;
        
        // Check CRC 2 (Low Word)
        if (Sensirion_CalcCRC(&rx_buf[base+3], 2) != rx_buf[base+5]) return HAL_ERROR;
        
        // Reassemble
        uint8_t raw_float[4];
        raw_float[0] = rx_buf[base];
        raw_float[1] = rx_buf[base+1];
        raw_float[2] = rx_buf[base+3];
        raw_float[3] = rx_buf[base+4];
        
        pData[i] = BytesToFloat(raw_float);
    }
    
    return HAL_OK;
}

HAL_StatusTypeDef SPS30_WakeUp(I2C_HandleTypeDef *hi2c) {
    // Wakeup requires writing 0x1103. 
    // Usually requires sending a "Low Pulse" on SDA first (Start+Stop) but often just sending the command twice works.
    uint8_t cmd[2] = {SPS30_CMD_WAKEUP >> 8, SPS30_CMD_WAKEUP & 0xFF};
    HAL_I2C_Master_Transmit(hi2c, SPS30_I2C_ADDRESS, cmd, 2, 100); // First attempt to wake interface
    HAL_Delay(5);
    return HAL_I2C_Master_Transmit(hi2c, SPS30_I2C_ADDRESS, cmd, 2, 100); // Actual command
}

HAL_StatusTypeDef SPS30_Sleep(I2C_HandleTypeDef *hi2c) {
    uint8_t cmd[2] = {SPS30_CMD_SLEEP >> 8, SPS30_CMD_SLEEP & 0xFF};
    return HAL_I2C_Master_Transmit(hi2c, SPS30_I2C_ADDRESS, cmd, 2, 100);
}