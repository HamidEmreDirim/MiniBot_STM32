/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "string.h"
#include "stdarg.h"
#include "library/BNO055_STM32.h"
#include "library/gps.h"
#include "usbd_cdc_if.h"
#include "library/motorDriver.h"
#include "library/INA219.h"
#include "library/EEPROM.h"
#include "library/sps30.h"
#include "library/bme68x/bme68x.h"





/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Sensor Variables
// BME688 Variables
// struct bme68x_dev bme; // Handled by library
struct bme68x_data bme_data;
SPS30_Data_t sps_data;

// --- Sensor Frequencies (Hz) ---
uint32_t freq_gps_hz = 1;
uint32_t freq_air_hz = 1;
uint32_t freq_imu_hz = 10;
uint32_t freq_sps_hz = 1;

// --- Sensor Timers ---
uint32_t last_tick_gps = 0;
uint32_t last_tick_air = 0;
uint32_t last_tick_imu = 0;
uint32_t last_tick_sps = 0;

// --- BNO055 Data ---
BNO055_Sensors_t bno_data;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static uint8_t rx_buffer_main[1024];
static uint8_t tx_buffer_main[1024];

TIM_HandleTypeDef htim6; // Soft PWM Timer handle
volatile uint8_t cam_led_duty = 0; // 0 to 100
volatile uint8_t soft_pwm_cnt = 0; // 0 to 100

// --- Global Static Buffers for RX ---
static uint8_t rx_accum[2048];
static uint16_t rx_accum_head = 0;

void Process_Serial_Input(void); // Prototype

typedef enum {
    MODE_IDLE,
    MODE_ARMED,
    MODE_ARMED_PLUS,
    MODE_ECO
} RobotMode;

RobotMode current_mode = MODE_IDLE;
float speed_limit = 0.0f;
uint32_t last_telemetry_time = 0;

// Status Flags
int8_t status_imu = -1;
int8_t status_bme = -1;
int8_t status_sps = -1;
uint8_t status_ina = 0;
uint32_t last_status_time = 0;
uint8_t bno_calibration_flag = 0; // Set to 1 to run calibration mode

INA219_t ina219;

typedef struct {
    float voltage;
    float percent;
} BatteryCurvePoint;

const BatteryCurvePoint bat_curve[] = {
    {14.6f, 100.0f},
    {13.6f, 100.0f},
    {13.5f, 99.0f},
    {13.4f, 95.0f},
    {13.3f, 90.0f},
    {13.25f, 80.0f},
    {13.2f,  70.0f},
    {13.15f, 60.0f},
    {13.1f,  50.0f},
    {13.05f, 40.0f},
    {13.0f,  30.0f},
    {12.9f,  20.0f},
    {12.8f,  15.0f},
    {12.5f,  10.0f},
    {12.0f,  5.0f},
    {11.0f,  0.0f}
};

// Battery History for Peak Hold
#define BAT_HIST_SIZE 100
float bat_history[BAT_HIST_SIZE] = {0};
uint8_t bat_hist_idx = 0;
uint32_t last_bat_hist_time = 0;

float get_max_history() {
    float max_v = 0.0f;
    for(int i=0; i<BAT_HIST_SIZE; i++) {
        if(bat_history[i] > max_v) max_v = bat_history[i];
    }
    return max_v;
}

uint8_t get_battery_percentage(float voltage)
{
    // Check bounds
    if (voltage >= bat_curve[0].voltage) return (uint8_t)bat_curve[0].percent;
    int num_points = sizeof(bat_curve) / sizeof(bat_curve[0]);
    if (voltage <= bat_curve[num_points - 1].voltage) return (uint8_t)bat_curve[num_points - 1].percent;

    // Linear Interpolation
    for (int i = 0; i < num_points - 1; i++) {
        if (voltage <= bat_curve[i].voltage && voltage >= bat_curve[i+1].voltage) {
            float v_high = bat_curve[i].voltage;
            float v_low = bat_curve[i+1].voltage;
            float p_high = bat_curve[i].percent;
            float p_low = bat_curve[i+1].percent;
            
            float percent = p_low + (voltage - v_low) * (p_high - p_low) / (v_high - v_low);
            return (uint8_t)percent;
        }
    }
    return 0; // Should not reach here
}



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MX_TIM6_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/**
  * @brief Retargets the C library printf function to the USART.
  * @note  Output is sent to UART2 (Standard ST-Link VCP).
  */
#ifdef __GNUC__
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 100);
    return len;
}
#endif









/* USER CODE BEGIN PTD */
// Update the Struct to include Enable Pin info

/* USER CODE END PTD */

/* USER CODE BEGIN 0 */


/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

void Process_Serial_Input(void)
{
    // 1. Process Incoming Data with Buffering
    int reads = 0;
    while (VCP_available() > 0 && reads < 5)
    {
        reads++;
        // Read new data into temp buffer
        uint16_t len = VCP_read_buffer(rx_buffer_main, 1024);
        
        if (rx_accum_head + len < 2048) {
            memcpy(&rx_accum[rx_accum_head], rx_buffer_main, len);
            rx_accum_head += len;
            rx_accum[rx_accum_head] = '\0'; // Safety null
        } else {
            // Buffer overflow protection: Clear and restart
            rx_accum_head = 0;
            printf("RX Overflow! Clearing buffer.\r\n");
        }

        // Process Complete Lines
        char* newline_ptr;
        while ((newline_ptr = strchr((char*)rx_accum, '\n')) != NULL) 
        {
            // Temporarily terminate the line
            *newline_ptr = '\0';
            char* line = (char*)rx_accum;
            // printf("RX Line: %s\n", line); // Debug Print
            
            // --- PROCESS LINE START ---
            // Mode Switching
            if (strstr(line, "ARMED+")) {
                current_mode = MODE_ARMED_PLUS;
                speed_limit = 1.0f;
            }
            else if (strstr(line, "ARMED")) {
                current_mode = MODE_ARMED;
                speed_limit = 0.7f;
            }
            else if (strstr(line, "ECO")) {
                current_mode = MODE_ECO;
                speed_limit = 0.4f;
            }
            else if (strstr(line, "IDLE")) {
                current_mode = MODE_IDLE;
                speed_limit = 0.0f;
                // Force Stop
                Motor_Move(1, 1, 0, 1); Motor_Move(2, 1, 0, 1);
                Motor_Move(3, 1, 0, 1); Motor_Move(4, 1, 0, 1);
            }
            
            // Command Parsing (Only if NOT IDLE)
            if (current_mode != MODE_IDLE)
            {
                float v = 0.0f;
                float w = 0.0f;

                if (sscanf(line, "{v=%f, w=%f", &v, &w) == 2)
                {
                    // Apply Speed Limit
                    float scalar = 100.0f * speed_limit; 

                    float v_scaled = v * scalar;
                    float w_scaled = w * scalar;

                    float left_speed = v_scaled - w_scaled;
                    float right_speed = v_scaled + w_scaled;

                    int dir_left = (left_speed >= 0) ? 1 : -1;
                    int dir_right = (right_speed >= 0) ? 1 : -1;

                    uint32_t pwm_left = (uint32_t)fabs(left_speed);
                    uint32_t pwm_right = (uint32_t)fabs(right_speed);

                    if (pwm_left > 100) pwm_left = 100;
                    if (pwm_right > 100) pwm_right = 100;

                    Motor_Move(1, -dir_left, pwm_left, 1);     // Left Rear
                    Motor_Move(4, -dir_left, pwm_left, 1);     // Front Left
                    
                    Motor_Move(3, -dir_right, pwm_right, 1);   // Front Right
                    Motor_Move(2, dir_right, pwm_right, 1);    // Right Rear
                }
            }
            
            // Camera LED Control
            int cam_led_val = 0;
            if (sscanf(line, "cam_led %d", &cam_led_val) == 1 || sscanf(line, "cam_led:%d", &cam_led_val) == 1)
            {
                if (cam_led_val > 100) cam_led_val = 100;
                if (cam_led_val < 0) cam_led_val = 0;
                
                cam_led_duty = (uint8_t)cam_led_val;
            }
            // --- PROCESS LINE END ---

            // Remove processed line from buffer
            int line_len = (newline_ptr - (char*)rx_accum) + 1; 
            int remaining = rx_accum_head - line_len;
            
            if (remaining > 0) {
                memmove(rx_accum, newline_ptr + 1, remaining);
            }
            rx_accum_head = remaining;
            rx_accum[rx_accum_head] = '\0';
        }
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */





  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // Soft disconnect for USB
  // Simulate cable unplug by pulling D+ (PA12) low
  // Using open-drain with pull-down is safer than push-pull
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(1000); // Wait 1 second for host to detect disconnect
  
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12); // Release pin for USB IP
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM12_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */

  // Optional: Start PWM timers just to have signal on pins,
  // even if we don't actively change them during test loop.
  // This generates a 0% or low signal on PWM pins depending on config.



  Motor_Init();

  HAL_TIM_Base_Start_IT(&htim6); // Start Soft PWM Timer



    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, gps_dma_rx, sizeof(gps_dma_rx));


    printf("System Initializing...\r\n");

    // BNO055 (I2C3)
    printf("Initializing BNO055...\r\n");
    if (HAL_I2C_IsDeviceReady(&hi2c3, (0x28 << 1), 5, 100) == HAL_OK) {
        BNO055_Init_t bno_init = {
            .Unit_Sel     = UNIT_ORI_WINDOWS | UNIT_TEMP_CELCIUS | UNIT_EUL_DEG | UNIT_GYRO_DPS | UNIT_ACC_MS2,
            .Axis         = DEFAULT_AXIS_REMAP,
            .Axis_sign    = DEFAULT_AXIS_SIGN,
            .Mode         = BNO055_NORMAL_MODE,
            .OP_Modes     = NDOF,
            .Clock_Source = CLOCK_INTERNAL,
            .ACC_Range    = Range_4G,
        };
        BNO055_Init(bno_init);
        
        BNO055_Init(bno_init);
        
        uint8_t bno_offsets[22];

        // Calibration Routine Check
        if (bno_calibration_flag == 1) {
            printf("BNO055 Calibration Mode Enabled. Starting calibration sequence...\r\n");
            Calibrate_BNO055();
            printf("Calibration sequence completed.\r\n");
            
            // Get the new offsets and write to EEPROM
            getSensorOffsets(bno_offsets);
            EEPROM_Write(0, 0, bno_offsets, 22);
            printf("Calibration offsets written to EEPROM. Set bno_calibration_flag to 0 and reboot!\r\n");
            
            // Halt the system here so the user can reflash with flag 0
            while(1) {
                HAL_Delay(1000);
            }
        } else {
            // Read offsets from EEPROM and apply them
            EEPROM_Read(0, 0, bno_offsets, 22);
            
            // Basic sanity check: if the first few bytes are 0xFF (uninitialized EEPROM), warn the user
            if(bno_offsets[0] == 0xFF && bno_offsets[1] == 0xFF) {
                printf("WARNING: EEPROM offsets appear uninitialized. Please run Calibration Mode.\r\n");
            } else {
                setSensorOffsets(bno_offsets);
                printf("Loaded Calibration offsets from EEPROM.\r\n");
            }
        }

        status_imu = 0; // 0 is OK
        printf("BNO055 Init... OK\r\n");
    } else {
        printf("BNO055 Init... FAIL (Not Detected)\r\n");
        status_imu = -1;
    }

    printf("GPS: UART RX DMA Started\r\n");

    if(INA219_Init(&ina219, &hi2c3, INA219_ADDRESS))
    {
        printf("INA219: Connected\r\n");
        INA219_setCalibration_32V_2A(&ina219);
        status_ina = 1;
    }
    else
    {
        printf("INA219: Not Connected\r\n");
        status_ina = 0;
        
        // Debug: Check other likely addresses
        printf("Debugging INA219: Scanning common addresses...\r\n");
        for(uint16_t addr = 0x40; addr <= 0x4F; addr++) {
             if(HAL_I2C_IsDeviceReady(&hi2c3, addr << 1, 3, 100) == HAL_OK) {
                  printf("Potential Sensor found at 0x%02X! (Try changing INA219_ADDRESS)\r\n", addr);
             }
        }
    }

    // I2C Scanner removed intentionally to prevent BNO055 and BME688 zero-byte write crashes.

    /* --- BME688 & SPS30 Initialization --- */
    /* --- BME688 & SPS30 Initialization --- */
    printf("Initializing BME688 (New Lib)...\r\n");
    int8_t rslt = bme68x_start(&bme_data, &hi2c3);
    
    if(rslt == 0) // 0 is Success in BME68X API
    {
        printf("BME688 Initialized.\r\n");
        status_bme = 1;
    }
    else
    {
        printf("BME688 Init Failed: %d\r\n", rslt);
        status_bme = 0;
    }

    printf("Initializing SPS30...\r\n");
    SPS30_WakeUp(&hi2c3); // Wake up the module first because it often starts asleep and NACKs everything
    HAL_Delay(100);
    if(SPS30_Init(&hi2c3) == HAL_OK)
    {
        printf("SPS30 Initialized.\r\n");
        SPS30_StartMeasurement(&hi2c3);
        status_sps = 1;
        HAL_Delay(100); 
    }
    else
    {
        printf("SPS30 Init Failed.\r\n");
        status_sps = 0;
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  // --- Background Sensor Polling Variables ---
  uint32_t last_sps_poll_time = 0;
  uint32_t last_bme_poll_time = 0;
  uint8_t bme_waiting_data = 0;
  uint32_t bme_wait_ms = 0;
  
  // Battery Filter Variables
  float filt_bat_v = 0.0f;
  const float alpha_bat = 0.02f; // Very slow filter for battery (50 taps equiv)
  uint8_t first_bat_read = 1;

  uint32_t last_gps_ver = 0;

  while (1)
  {
    Process_Serial_Input();

    // --- GPS Data Publishing Loop (Event-Driven) ---
    if (gps_version != last_gps_ver) {
        last_gps_ver = gps_version;
        uint16_t size = 0;
        uint8_t* data = get_gps_data(&gps_ring, &size);
        if (size > 0) {
            if (size < 512) {
                data[size] = '\0';
            } else {
                data[511] = '\0';
            }
             
            if (VCP_is_tx_busy() == 0) {
                CDC_Transmit_FS(data, size);
            }
        }
    }

    // --- BNO055 ---
    if (freq_imu_hz > 0 && (HAL_GetTick() - last_tick_imu >= (1000 / freq_imu_hz)))
    {
        if (status_imu == 0) {
            ReadData(&bno_data,
                     SENSOR_ACCEL | SENSOR_GYRO | SENSOR_MAG |
                     SENSOR_EULER | SENSOR_LINACC | SENSOR_GRAVITY | SENSOR_QUATERNION);
        }

        char buf[512];
        int len = snprintf(buf, sizeof(buf),
            "{\"imu\":{"
            "\"accel\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
            "\"gyro\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
            "\"mag\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
            "\"euler\":{\"heading\":%.2f,\"roll\":%.2f,\"pitch\":%.2f},"
            "\"linacc\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
            "\"gravity\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
            "\"quat\":{\"w\":%.4f,\"x\":%.4f,\"y\":%.4f,\"z\":%.4f}"
            "}}\n",
            bno_data.Accel.X,     bno_data.Accel.Y,     bno_data.Accel.Z,
            bno_data.Gyro.X,      bno_data.Gyro.Y,      bno_data.Gyro.Z,
            bno_data.Magneto.X,   bno_data.Magneto.Y,   bno_data.Magneto.Z,
            bno_data.Euler.X,     bno_data.Euler.Y,     bno_data.Euler.Z,
            bno_data.LineerAcc.X, bno_data.LineerAcc.Y, bno_data.LineerAcc.Z,
            bno_data.Gravity.X,   bno_data.Gravity.Y,   bno_data.Gravity.Z,
            bno_data.Quaternion.W, bno_data.Quaternion.X,
            bno_data.Quaternion.Y, bno_data.Quaternion.Z);

        if (len > 0 && VCP_is_tx_busy() == 0) {
            CDC_Transmit_FS((uint8_t*)buf, len);
            last_tick_imu = HAL_GetTick();
        }
    }

    // --- Background Polling Loop for SPS30 (10Hz) ---
    if (HAL_GetTick() - last_sps_poll_time >= 100)
    {
        last_sps_poll_time = HAL_GetTick();
        if (status_sps == 1) {
            if (SPS30_ReadDataReady(&hi2c3)) {
                SPS30_ReadMeasurement(&hi2c3, &sps_data);
            }
        }
    }

    // --- Background Polling Loop for BME688 (1/3 Hz) ---
    if (status_bme == 1) {
        if (!bme_waiting_data && (HAL_GetTick() - last_bme_poll_time >= 3000))
        {
            last_bme_poll_time = HAL_GetTick();
            bme68x_trigger_measure();
            bme_wait_ms = bme68x_get_delay_ms();
            bme_waiting_data = 1;
        }
        else if (bme_waiting_data && (HAL_GetTick() - last_bme_poll_time >= bme_wait_ms))
        {
            bme68x_read_measure(&bme_data);
            bme_waiting_data = 0;
        }
    }

    // --- SPS30 Publishing ---
    if (freq_sps_hz > 0 && (HAL_GetTick() - last_tick_sps >= (1000 / freq_sps_hz)))
    {
        float pm1 = 0, pm25 = 0, pm4 = 0, pm10 = 0;

        if (status_sps == 1) {
            pm1 = sps_data.mass_1p0;
            pm25 = sps_data.mass_2p5;
            pm4 = sps_data.mass_4p0;
            pm10 = sps_data.mass_10p0;
        }

        char buf[256];
        int len = snprintf(buf, sizeof(buf), 
            "{\"sps\":{\"pm1.0\":%.2f,\"pm2.5\":%.2f,\"pm4.0\":%.2f,\"pm10\":%.2f}}\n", 
            pm1, pm25, pm4, pm10);
        if (len > 0 && VCP_is_tx_busy() == 0) {
            CDC_Transmit_FS((uint8_t*)buf, len);
            last_tick_sps = HAL_GetTick();
        }
    }

    // --- BME688 Publishing ---
    if (freq_air_hz > 0 && (HAL_GetTick() - last_tick_air >= (1000 / freq_air_hz)))
    {
        float iaq = 0, temp = 0, hum = 0;
        
        if (status_bme == 1) {
            iaq = bme68x_iaq();
            temp = bme_data.temperature;
            hum = bme_data.humidity;
        }

        char buf[128];
        int len = snprintf(buf, sizeof(buf), "{\"air\":{\"iaq\":%.2f,\"temp\":%.2f,\"hum\":%.2f}}\n", 
                           iaq, temp, hum);
        if (len > 0 && VCP_is_tx_busy() == 0) {
             CDC_Transmit_FS((uint8_t*)buf, len);
             last_tick_air = HAL_GetTick();
        }
    }

    // --- Battery & Status Telemetry (Sent in ALL modes) ---
    if (HAL_GetTick() - last_telemetry_time >= 2000) // 0.5Hz
    {
        if (VCP_is_tx_busy() == 0) {
            char telemetry[512];
            
            // Read INA if connected
            float bus_voltage_V = 0.0f;
            int16_t current_mA = 0;
            if (status_ina == 1) {
                bus_voltage_V = INA219_ReadBusVoltage(&ina219) / 1000.0f;
                current_mA = INA219_ReadCurrent(&ina219);
            }
            
            if (current_mA < 0) current_mA = -current_mA;

            // --- Battery Filtering ---
            if (first_bat_read) {
                filt_bat_v = bus_voltage_V;
                first_bat_read = 0;
            } else {
                if (bus_voltage_V < 1.0f) {
                    filt_bat_v = bus_voltage_V;
                } 
                else if (current_mA < 300) {
                    filt_bat_v = (alpha_bat * bus_voltage_V) + ((1.0f - alpha_bat) * filt_bat_v);
                }
            }

            // --- 100s Peak Hold Logic ---
            if (HAL_GetTick() - last_bat_hist_time >= 1000) {
                bat_history[bat_hist_idx] = filt_bat_v;
                bat_hist_idx = (bat_hist_idx + 1) % BAT_HIST_SIZE;
                last_bat_hist_time = HAL_GetTick();
            }
            
            if (bat_history[0] == 0.0f && filt_bat_v > 5.0f) {
                    for(int i=0; i<BAT_HIST_SIZE; i++) bat_history[i] = filt_bat_v;
            }

            float peak_bat_v = get_max_history();
            uint8_t bat_pct = get_battery_percentage(peak_bat_v);
            
            // Determine Mode String
            const char* mode_str = "IDLE";
            if (current_mode == MODE_ARMED) mode_str = "ARMED";
            else if (current_mode == MODE_ARMED_PLUS) mode_str = "ARMED+";
            else if (current_mode == MODE_ECO) mode_str = "ECO";

            sprintf(telemetry, "{\"status\":{\"mode\":\"%s\", \"bat_v\":%.2f, \"bat_pct\":%d, \"imu\":%d, \"bme\":%d, \"sps\":%d, \"ina\":%d}}\n", 
                    mode_str, bus_voltage_V, bat_pct, (status_imu==0?1:0), status_bme, status_sps, status_ina);

            CDC_Transmit_FS((uint8_t*)telemetry, strlen(telemetry));
            last_telemetry_time = HAL_GetTick();
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Simple software debounce to prevent "bouncing" contacts from spamming the UART
    // It ignores interrupts that happen within 50ms of the last one.

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
      soft_pwm_cnt++;
      if (soft_pwm_cnt >= 100) soft_pwm_cnt = 0;

      if (soft_pwm_cnt < cam_led_duty)
      {
          // ON
          HAL_GPIO_WritePin(CAM_LED_ON_OFF_GPIO_Port, CAM_LED_ON_OFF_Pin, GPIO_PIN_SET);
      }
      else
      {
          // OFF
          HAL_GPIO_WritePin(CAM_LED_ON_OFF_GPIO_Port, CAM_LED_ON_OFF_Pin, GPIO_PIN_RESET);
      }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
