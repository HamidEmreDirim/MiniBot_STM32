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
#include "library/adxl_345_imu.h"
#include "library/gps.h"
#include "usbd_cdc_if.h"
#include "library/motorDriver.h"
#include "library/INA219.h"
#include "library/EEPROM.h"





/* USER CODE END Includes */
#include "library/sps30.h"
#include "library/bme68x/bme68x_necessary_functions.h"

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
// struct bme68x_conf bme_conf; // Handled by library
// struct bme68x_heatr_conf bme_heatr_conf; // Handled by library
SPS30_Data_t sps_data;
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


const char* CMD_HELLO = "CMD_HELLO";
const char* ACK_ROCKPI = "ACK_READY";

typedef enum {
    MODE_IDLE,
    MODE_ARMED,
    MODE_ARMED_PLUS,
    MODE_ECO
} RobotMode;

RobotMode current_mode = MODE_IDLE;
float speed_limit = 0.0f;
uint32_t last_telemetry_time = 0;

uint8_t handshake_complete = 0;
uint32_t last_hello_time = 0;

// Status Flags
uint8_t status_imu = 0;
uint8_t status_bme = 0;
uint8_t status_sps = 0;
uint8_t status_ina = 0;
uint32_t last_status_time = 0;

INA219_t ina219;

typedef struct {
    float voltage;
    float percent;
} BatteryCurvePoint;

const BatteryCurvePoint bat_curve[] = {
    {14.6f, 100.0f},
    {13.6f, 100.0f},
    {13.4f, 99.0f},
    {13.3f, 90.0f},
    {13.2f, 70.0f},
    {13.1f, 40.0f},
    {13.0f, 30.0f},
    {12.9f, 20.0f},
    {12.8f, 17.0f},
    {12.5f, 14.0f},
    {12.0f, 9.0f},
    {10.0f, 0.0f}
};

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

/* USER CODE END 0 */

void Process_Serial_Input(void)
{
    // 1. Process Incoming Data with Buffering
    // Loop until we drain the buffer or hit a limit (flow control)
    // We limit to 5 reads per call to allow main loop to continue even under flood
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
            printf("RX Line: %s\n", line); // Debug Print
            
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

                // Parse message: {v=0.1, w=0.1}
                // Note: sscanf is robust enough to find the pattern even if there is noise around it
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

                    Motor_Move(1, dir_left, pwm_left, 1);
                    Motor_Move(2, dir_left, pwm_left, 1);
                    Motor_Move(3, dir_right, pwm_right, 1);
                    Motor_Move(4, dir_right, pwm_right, 1);
                }
            }
            
            // Camera LED Control
            int cam_led_val = 0;
            // Support both "cam_led:100" and "cam_led 100"
            if (sscanf(line, "cam_led %d", &cam_led_val) == 1 || sscanf(line, "cam_led:%d", &cam_led_val) == 1)
            {
                if (cam_led_val > 100) cam_led_val = 100;
                if (cam_led_val < 0) cam_led_val = 0;
                
                cam_led_duty = (uint8_t)cam_led_val;
                printf("CAM_LED set to %d%%\r\n", cam_led_duty);
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
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM12_Init();
  MX_TIM6_Init(); // Init Soft PWM Timer
  /* USER CODE BEGIN 2 */
  // Soft disconnect for USB
  // Force USB D+ (PA12) low for 500ms to ensure host sees a disconnect
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(500); // Wait for host to detect disconnect
  
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12); // Release pin for USB IP
  
  MX_USB_DEVICE_Init();

  // Optional: Start PWM timers just to have signal on pins,
  // even if we don't actively change them during test loop.
  // This generates a 0% or low signal on PWM pins depending on config.



  Motor_Init();

  HAL_TIM_Base_Start_IT(&htim6); // Start Soft PWM Timer



    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, gps_dma_rx, sizeof(gps_dma_rx));


    printf("System Initializing...\r\n");

    adxl_init();
    if (chipid == 0xE5) {
        printf("IMU: Connected (ID: 0xE5)\r\n");
        status_imu = 1;
    } else {
        printf("IMU: Not Connected (ID: 0x%02X)\r\n", chipid);
        status_imu = 0;
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

    /* --- I2C Scanner --- */
    printf("Scanning I2C3 bus...\r\n");
    for(uint16_t i = 1; i < 128; i++) {
        if(HAL_I2C_IsDeviceReady(&hi2c3, i << 1, 1, 10) == HAL_OK) {
            printf("I2C3 Device found at 0x%02X\r\n", i);
        }
    }
    printf("Scan complete.\r\n");

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
  uint32_t last_sps_time = 0;
  uint32_t last_bme_time = 0;
  
  // IMU Filter Variables
  float filt_x = 0, filt_y = 0, filt_z = 0;
  const float alpha = 0.1f; // Smoothing factor (0.0 - 1.0)
  
  uint32_t last_gps_ver = 0;

  while (1)
  {
    // --- Sensor Update Loop (1Hz) ---
    // --- Fast Sensor Loop (10Hz) for SPS30 ---
    if (HAL_GetTick() - last_sps_time >= 100)
    {
        last_sps_time = HAL_GetTick();
        
        // Read SPS30 Only if initialized
        if (status_sps) {
            if(SPS30_ReadDataReady(&hi2c3))
            {
                SPS30_ReadMeasurement(&hi2c3, &sps_data);
            }
        }
    }

    // --- Slow Sensor Loop (Every 3 Seconds) for BME688 ---
    if (HAL_GetTick() - last_bme_time >= 3000)
    {
        last_bme_time = HAL_GetTick();

        if (status_bme) {
            // Read BME688 (New Lib)
            int8_t result = bme68x_single_measure(&bme_data);
            
            float iaq_score = bme68x_iaq(); 

            if (result != 0)
            {
                 printf("BME688 Measure Failed: %d\r\n", result);
                 // If it fails repeatedly, maybe set status_bme = 0? 
                 // For now, just report error.
            }
            else
            {
                 // Debug Print for Logic Analysis
                 printf("BME688 [Debug] T: %.2f C | H: %.2f %% | P: %.2f hPa | G: %.2f Ohms | IAQ: %.2f\r\n", 
                        bme_data.temperature, bme_data.humidity, bme_data.pressure, bme_data.gas_resistance, iaq_score);
            }
        }
    }

    // --- GPS Print Loop ---
    if (gps_version != last_gps_ver) {
        last_gps_ver = gps_version;
        
        uint16_t osize = 0;
        uint8_t* gbuf = get_gps_data(&gps_ring, &osize);
        
        if(osize > 0) {
             // Safe null termination to prevent printing garbage or buffer overflow
             if (osize < 512) {
                 gbuf[osize] = '\0';
             } else {
                 gbuf[511] = '\0';
             }
             
             // printf("GPS: %s", (char*)gbuf); 
        }
    }


    // Handshake Logic
    if (!handshake_complete)
    {
      if (HAL_GetTick() - last_hello_time >= 100)
      {
        if (VCP_is_tx_busy() == 0)
        {
          CDC_Transmit_FS((uint8_t*)CMD_HELLO, strlen(CMD_HELLO));
          last_hello_time = HAL_GetTick();
        }
      }

      // Check for ACK
      if (VCP_available() > 0)
      {
         // Simple read for handshake is fine as it's a short, single message interaction
         uint16_t len = VCP_read_buffer(rx_buffer_main, 1024);
         rx_buffer_main[len] = '\0'; // Null terminate
         
         printf("HS RX: %s\n", rx_buffer_main); // Debug Handshake RX

         if (strstr((char*)rx_buffer_main, ACK_ROCKPI) != NULL)
         {
             handshake_complete = 1;
             printf("Handshake Complete!\n");
         }
      }
    }
    // Normal Operation Logic
    else
    {
        Process_Serial_Input();

         // 2. Global Telemetry (Sent in ALL modes)
         if (HAL_GetTick() - last_telemetry_time >= 100) // 10Hz
         {
             if (VCP_is_tx_busy() == 0) {
                char telemetry[512]; // Increased buffer size
                
                // Read Raw IMU if connected
                int16_t raw_x = 0, raw_y = 0, raw_z = 0;
                if (status_imu) {
                    raw_x = adxl_readx();
                    raw_y = adxl_ready();
                    raw_z = adxl_readz();
                }

                // Apply Low Pass Filter
                filt_x = (alpha * raw_x) + ((1.0f - alpha) * filt_x);
                filt_y = (alpha * raw_y) + ((1.0f - alpha) * filt_y);
                filt_z = (alpha * raw_z) + ((1.0f - alpha) * filt_z);

                // Read INA if connected
                float bus_voltage_V = 0.0f;
                if (status_ina) {
                    bus_voltage_V = INA219_ReadBusVoltage(&ina219) / 1000.0f;
                }
                uint8_t bat_pct = get_battery_percentage(bus_voltage_V);
                
                // Determine Mode String
                const char* mode_str = "IDLE";
                if (current_mode == MODE_ARMED) mode_str = "ARMED";
                else if (current_mode == MODE_ARMED_PLUS) mode_str = "ARMED+";
                else if (current_mode == MODE_ECO) mode_str = "ECO";

                // Send FILTERED IMU values (casted to int for display)
                // Get BME Values safely
                float temp = 0.0f, hum = 0.0f, press = 0.0f, gas = 0.0f, iaq = 0.0f;
                if (status_bme) {
                    temp = bme_data.temperature;
                    hum = bme_data.humidity;
                    press = bme_data.pressure;
                    gas = bme_data.gas_resistance;
                    iaq = bme68x_iaq();
                }

                sprintf(telemetry, "{mode:%s, x:%d, y:%d, z:%d, bat_v:%.2f, bat_pct:%d, temp:%.2f, hum:%.2f, press:%.2f, iaq:%.2f, gas_res:%.2f, pm1:%.2f, pm2p5:%.2f, pm4:%.2f, pm10:%.2f}\n", 
                        mode_str, (int)filt_x, (int)filt_y, (int)filt_z, bus_voltage_V, bat_pct,
                        temp, hum, press, iaq, gas,
                        sps_data.mass_1p0, sps_data.mass_2p5, sps_data.mass_4p0, sps_data.mass_10p0);

                CDC_Transmit_FS((uint8_t*)telemetry, strlen(telemetry));
                last_telemetry_time = HAL_GetTick();
             }
         }

          // 3. Status Telemetry (1Hz)
          if (HAL_GetTick() - last_status_time >= 1000) 
          {
              if (VCP_is_tx_busy() == 0) 
              {
                  char status_msg[128];
                  
                  // Re-determine Mode String (or reuse if scope allows, here we just check again)
                  const char* mode_str_sts = "IDLE";
                  if (current_mode == MODE_ARMED) mode_str_sts = "ARMED";
                  else if (current_mode == MODE_ARMED_PLUS) mode_str_sts = "ARMED+";
                  else if (current_mode == MODE_ECO) mode_str_sts = "ECO";

                  sprintf(status_msg, "{\"type\":\"STATUS\", \"mode\":\"%s\", \"imu\":%d, \"bme\":%d, \"sps\":%d, \"ina\":%d}\n",
                          mode_str_sts, status_imu, status_bme, status_sps, status_ina);

                  CDC_Transmit_FS((uint8_t*)status_msg, strlen(status_msg));
                  last_status_time = HAL_GetTick();
              }
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84 - 1; // 84MHz / 84 = 1MHz count freq
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100 - 1;   // 1MHz / 100 = 10kHz interrupt freq
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

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
