#ifndef SPS30_H
#define SPS30_H

#include "main.h" // Ensures generic HAL types are available
#include <stdint.h>

// I2C Address (0x69 shifted left by 1 for HAL)
#define SPS30_I2C_ADDRESS (0x69 << 1)

// Commands
#define SPS30_CMD_START_MEASUREMENT   0x0010
#define SPS30_CMD_STOP_MEASUREMENT    0x0104
#define SPS30_CMD_READ_DATA_READY     0x0202
#define SPS30_CMD_READ_VALUES         0x0300
#define SPS30_CMD_RESET               0xD304
#define SPS30_CMD_SLEEP               0x1001
#define SPS30_CMD_WAKEUP              0x1103

// Data format for Start Measurement (Float mode is standard)
#define SPS30_FORMAT_FLOAT            0x0300
#define SPS30_FORMAT_UINT16           0x0500

// Struct to hold sensor data
typedef struct {
    float mass_1p0;   // PM1.0 [µg/m³]
    float mass_2p5;   // PM2.5 [µg/m³]
    float mass_4p0;   // PM4.0 [µg/m³]
    float mass_10p0;  // PM10.0 [µg/m³]
    float num_0p5;    // Number Conc PM0.5 [#/cm³]
    float num_1p0;    // Number Conc PM1.0 [#/cm³]
    float num_2p5;    // Number Conc PM2.5 [#/cm³]
    float num_4p0;    // Number Conc PM4.0 [#/cm³]
    float num_10p0;   // Number Conc PM10.0 [#/cm³]
    float typical_size; // Typical Particle Size [µm]
} SPS30_Data_t;

// Function Prototypes
HAL_StatusTypeDef SPS30_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef SPS30_StartMeasurement(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef SPS30_StopMeasurement(I2C_HandleTypeDef *hi2c);
uint8_t SPS30_ReadDataReady(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef SPS30_ReadMeasurement(I2C_HandleTypeDef *hi2c, SPS30_Data_t *data);
HAL_StatusTypeDef SPS30_WakeUp(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef SPS30_Sleep(I2C_HandleTypeDef *hi2c);

#endif