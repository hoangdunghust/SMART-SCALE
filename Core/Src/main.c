/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Smart Scale System with RFID, RTC, and 2x 5161BS LED Displays
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
/* USER CODE END Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DS3231_ADDRESS  (0xD0)
#define HX711_SCALE     -99.85f  // Hệ số hiệu chuẩn Load Cell (Thay đổi sau khi calib thực tế)
#define MAX_LEN         16      // Định nghĩa kích thước đệm cho RFID

// Lệnh điều khiển RFID RC522 thông qua thanh ghi cơ bản
#define PCD_IDLE              0x00
#define PCD_AUTHENT           0x0E
#define PCD_TRANSCEIVE        0x0C
#define PICC_REQIDL           0x26
#define PICC_ANTICOLL         0x93
#define CommandReg            0x01
#define FIFODataReg           0x09
#define FIFOLengthReg         0x0A
#define BitFramingReg         0x0D
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Biến lưu trữ trạng thái cân nặng
int32_t sample_offset = 0;
float current_weight = 0.0f;
float final_stable_weight = 0.0f;
float last_weight = 0.0f;
uint8_t stable_count = 0;

// Biến lưu thông tin thẻ RFID
uint8_t card_uid[5];
uint8_t is_card_detected = 0;

// Biến lưu trữ thời gian từ RTC
uint8_t rtc_seconds, rtc_minutes, rtc_hours;
uint8_t rtc_day, rtc_date, rtc_month, rtc_year;

// Buffer truyền UART
char uart_tx_buffer[256];

// Mảng mã LED 7 đoạn cho Anode chung (A->PD0, B->PD1, ..., G->PD6)
uint8_t LED_CA_CODE[] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90};
// Mảng lưu dữ liệu hiển thị hiện tại cho 2 LED (mặc định tắt - 0xFF)
uint8_t display_digits[2] = {0xFF, 0xFF};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */


void Send_Card_ID_To_PC(uint8_t *uid);
// Các hàm Driver HX711
int32_t HX711_Read(void);
void HX711_Tare(void);
float HX711_GetWeight(void);

// Các hàm xử lý thời gian RTC DS3231
uint8_t BCD_To_Decimal(uint8_t val);
void DS3231_ReadTime(void);

// Các hàm cơ bản giao tiếp RFID RC522 qua HAL SPI
void RC522_WriteReg(uint8_t reg, uint8_t val);
uint8_t RC522_ReadReg(uint8_t reg);
uint8_t RC522_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint32_t *backLen);
uint8_t RC522_CheckCard(uint8_t *id);
void RC522_Init(void);

// Hàm gửi dữ liệu đóng gói dạng JSON lên Máy tính
void Send_Data_To_PC(void);

// Các hàm cập nhật và Quét hiển thị LED 7 đoạn
void Update_LED_Buffer(float weight);
void LED_Scan_Routine(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_Init();
  SSD1306_Clear();
  SSD1306_GotoXY(0,0);
  SSD1306_Puts("SYSTEM STARTING...", &Font_7x10, 1);
  SSD1306_UpdateScreen();
  HAL_Delay(1000);

  // Khởi tạo các module ngoại vi
  RC522_Init();
  if (HAL_I2C_IsDeviceReady(&hi2c1, DS3231_ADDRESS, 3, 100) != HAL_OK) {
      // Nếu I2C không phản hồi, báo lỗi qua UART
      HAL_UART_Transmit(&huart1, (uint8_t*)"RTC ERROR!\r\n", 12, 100);
  }




    // --- TEST RTC ĐỘC LẬP ---
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart1, (uint8_t*)"Scanning I2C Bus...\r\n", 21, 100);
  for (uint8_t i = 0; i < 255; i++) {
      if (HAL_I2C_IsDeviceReady(&hi2c1, i, 1, 10) == HAL_OK) {
          char msg[30];
          sprintf(msg, "Found device at: 0x%02X\r\n", i);
          HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
      }
  }
  HAL_UART_Transmit(&huart1, (uint8_t*)"Scan complete.\r\n", 16, 100);
  /* USER CODE END 2 */





  uint8_t ver = RC522_ReadReg(0x37);
  sprintf(uart_tx_buffer, "VER: 0x%02X\r\n", ver);
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 1000);
  HAL_Delay(1000);
  HX711_Tare();   // Lấy điểm không cho cân khi vừa bật nguồn

  // Gửi tín hiệu hệ thống sẵn sàng qua UART
  sprintf(uart_tx_buffer, "=== SMART SCALE SYSTEM READY ===\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Cài đặt thời gian: 16:55:00 ngày 01 tháng 07 năm 2026

    uint8_t set_time[] = {0x00, 0x55, 0x16, 0x04, 0x01, 0x07, 0x26}; // S, M, H, Day, Date, Month, Year
    HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDRESS, 0x00, 1, set_time, 7, 100);


  /* Infinite loop */
    while (1)
    {
      // 1. LUÔN LUÔN QUÉT LED ĐỂ KHÔNG BỊ TẮT MÀN HÌNH
      LED_Scan_Routine();

      // 2. ƯU TIÊN KIỂM TRA THẺ RFID (Nếu chưa detect thẻ nào)
      if (!is_card_detected) {
    	  current_weight = HX711_GetWeight();
    	      Display_Data_On_OLED(current_weight, card_uid);
    	      Update_LED_Buffer(current_weight);
          if (RC522_CheckCard(card_uid) == 0) {
              // Nhận thẻ thành công -> Bật LED Xanh
              HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET);

              Send_Card_ID_To_PC(card_uid);
              is_card_detected = 1;
              stable_count = 0;
              last_weight = 0.0f;
              DS3231_ReadTime(); // Đọc thời gian
          } else {
              // Không có thẻ -> Cân và hiển thị cân nặng
              current_weight = HX711_GetWeight();
              Update_LED_Buffer(current_weight);
          }
      }
      // 3. XỬ LÝ KHI ĐÃ NHẬN THẺ
      else {
          current_weight = HX711_GetWeight();
          Update_LED_Buffer(current_weight);

          if (fabs(current_weight - last_weight) < 0.15f) {
              stable_count++;
              if (stable_count >= 15) { // Tăng lên 15 để chắc chắn cân ổn định
                  final_stable_weight = current_weight;

                  // LED Đỏ báo hiệu chốt số
                  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);

                  Send_Data_To_PC(); // Gửi JSON

                  HAL_Delay(1000); // Giữ kết quả 1 giây để người dùng nhìn thấy

                  // Reset trạng thái
                  is_card_detected = 0;
                  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET);
              }
          } else {
              stable_count = 0;
              last_weight = current_weight;
          }
      }

      // Giảm delay để quét LED mượt hơn
      HAL_Delay(1);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 10000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* Đảm bảo PA4 là GPIO Output thuần túy */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // Mặc định CS ở mức CAO (không chọn chip)
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, HX711_SCK_Pin|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RFID__CS_GPIO_Port, RFID__CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RFID_RST_GPIO_Port, RFID_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : HX711_DOUT_Pin */
  GPIO_InitStruct.Pin = HX711_DOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HX711_DOUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HX711_SCK_Pin PE6 */
  GPIO_InitStruct.Pin = HX711_SCK_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : RFID__CS_Pin */
  GPIO_InitStruct.Pin = RFID__CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RFID__CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3
                           PD4 PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : RFID_RST_Pin */
  GPIO_InitStruct.Pin = RFID_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RFID_RST_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ==========================================
// DRIVER NGOẠI VI 1: CẢM BIẾN TRỌNG LƯỢNG HX711
// ==========================================

int32_t HX711_Read(void) {
    int32_t data = 0;
    uint32_t timeout = 60000;

    while(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_SET) {
        if (--timeout == 0) return 0; // Tránh treo chip khi đứt dây cảm biến
    }

    for (int i = 0; i < 24; i++) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
        __NOP(); __NOP();
        data = data << 1;
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_SET) {
            data++;
        }
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
        __NOP(); __NOP();
    }

    // Xung thứ 25 đặt Gain 128
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    __NOP(); __NOP();
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    __NOP(); __NOP();

    if (data & 0x800000) {
        data |= 0xFF000000;
    }
    return data;
}

void HX711_Tare(void) {
    int64_t sum = 0;
    for (int i = 0; i < 15; i++) {
        sum += HX711_Read();
        HAL_Delay(10);
    }
    sample_offset = (int32_t)(sum / 15);
}

float HX711_GetWeight(void) {
    int64_t sum = 0;
    int count = 10; // Tăng mẫu lấy trung bình lên để chống nhiễu
    for (int i = 0; i < count; i++) {
        sum += HX711_Read();
        HAL_Delay(1);
    }
    float raw_val = (float)(sum / count);
    float weight = (raw_val - (float)sample_offset) / HX711_SCALE;
    return (weight < 0.1f) ? 0.0f : weight; // Nếu nhỏ hơn 0.1kg thì coi như 0
}


// ==========================================
// DRIVER NGOẠI VI 2: THỜI GIAN THỰC DS3231 (I2C)
// ==========================================

uint8_t BCD_To_Decimal(uint8_t val) {
    return ((val / 16) * 10) + (val % 16);
}

void DS3231_ReadTime(void) {
    uint8_t rtc_buffer[7];
    // Ghi địa chỉ thanh ghi bắt đầu 0x00 trước khi đọc
    if (HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, 0x00, 1, rtc_buffer, 7, 100) == HAL_OK) {
        rtc_seconds = BCD_To_Decimal(rtc_buffer[0] & 0x7F);
        rtc_minutes = BCD_To_Decimal(rtc_buffer[1]);
        rtc_hours   = BCD_To_Decimal(rtc_buffer[2] & 0x3F);
        rtc_day     = BCD_To_Decimal(rtc_buffer[3]);
        rtc_date    = BCD_To_Decimal(rtc_buffer[4]);
        rtc_month   = BCD_To_Decimal(rtc_buffer[5] & 0x1F);
        rtc_year    = BCD_To_Decimal(rtc_buffer[6]);
    } else {
        // Nếu lỗi, reset thời gian giả để không gây lỗi logic
        rtc_year = 26; rtc_month = 6; rtc_date = 30;
    }
}


// ==========================================
// DRIVER NGOẠI VI 3: ĐỌC THẺ RFID RC522 (SPI)
// ==========================================

void RC522_WriteReg(uint8_t reg, uint8_t val) {
    uint8_t buffer[2];
    buffer[0] = (reg << 1) & 0x7E;
    buffer[1] = val;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    __NOP();

    HAL_SPI_Transmit(&hspi1, buffer, 2, 100);

    __NOP();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // Kéo CS lên cao
}

uint8_t RC522_ReadReg(uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = ((reg << 1) & 0x7E) | 0x80;
    tx[1] = 0x00;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    return rx[1];
}

void RC522_Init(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(200);

    RC522_WriteReg(CommandReg, 0x0F); // soft reset
    HAL_Delay(10);

    // CONFIG TIMER
    RC522_WriteReg(0x2A, 0x8D);
    RC522_WriteReg(0x2B, 0x3E);
    RC522_WriteReg(0x2D, 30);
    RC522_WriteReg(0x2C, 0);

    // 🔥 RF CONFIG (QUAN TRỌNG)
    RC522_WriteReg(0x14, 0x8D);   // TxControl
    RC522_WriteReg(0x11, 0x3D);   // Mode

    // 🔥 BẬT ANTEN
    uint8_t val = RC522_ReadReg(0x14);
    RC522_WriteReg(0x14, val | 0x03);

    HAL_Delay(10);

    RC522_WriteReg(CommandReg, 0x0C); // Transmit RF
}

uint8_t RC522_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint32_t *backLen) {
    uint8_t status = 1;
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;

    if (command == PCD_TRANSCEIVE) {
        irqEn = 0x77;
        waitIRq = 0x30;
    }
    RC522_WriteReg(0x02, irqEn | 0x80);
    uint8_t clear = RC522_ReadReg(0x04);
    RC522_WriteReg(0x04, clear & (~0x80));
    RC522_WriteReg(CommandReg, PCD_IDLE);
    RC522_WriteReg(0x0A, 0x80);

    for (int i = 0; i < sendLen; i++) {
        RC522_WriteReg(FIFODataReg, sendData[i]);
    }
    RC522_WriteReg(CommandReg, command);
    if (command == PCD_TRANSCEIVE) {
        uint8_t set = RC522_ReadReg(BitFramingReg);
        RC522_WriteReg(BitFramingReg, set | 0x80);
    }

    uint32_t index = 2000;
    uint8_t n;
    do {
        n = RC522_ReadReg(0x04);
        index--;
    } while ((index != 0) && !(n & 0x01) && !(n & waitIRq));

    uint8_t set2 = RC522_ReadReg(BitFramingReg);
    RC522_WriteReg(BitFramingReg, set2 & (~0x80));

    if (index != 0) {
        if (!(RC522_ReadReg(0x06) & 0x1B)) {
            status = 0;
            if (n & irqEn & 0x01) status = 1;
            if (command == PCD_TRANSCEIVE) {
                n = RC522_ReadReg(FIFOLengthReg);
                uint8_t lastBits = RC522_ReadReg(BitFramingReg) & 0x07;
                if (lastBits) *backLen = (n - 1) * 8 + lastBits;
                else *backLen = n * 8;
                if (n == 0) n = 1;
                if (n > 16) n = 16;
                for (int i = 0; i < n; i++) {
                    backData[i] = RC522_ReadReg(FIFODataReg);
                }
            }
        }
    }
    return status;
}

uint8_t RC522_CheckCard(uint8_t *id)
{
    uint8_t status;
    uint8_t buf[16];
    uint32_t len;

    // 1. REQIDL: Yêu cầu thẻ trả lời
    buf[0] = 0x26;
    status = RC522_ToCard(PCD_TRANSCEIVE, buf, 1, buf, &len);
    sprintf(uart_tx_buffer,
            "RAW: %02X %02X %02X %02X %02X\r\n",
            buf[0], buf[1], buf[2], buf[3], buf[4]);

    HAL_UART_Transmit(&huart1,
                      (uint8_t*)uart_tx_buffer,
                      strlen(uart_tx_buffer),
                      100);
    if (status != 0) return 1;

    // 2. ANTICOLLISION: Lấy ID từ thẻ (xử lý va chạm)
    buf[0] = 0x93;
    buf[1] = 0x20;
    status = RC522_ToCard(PCD_TRANSCEIVE, buf, 2, buf, &len);
    if (status != 0) return 1;

    // Lưu tạm ID
    for (int i = 0; i < 4; i++) id[i] = buf[i];

    // 3. SELECT: Chốt thẻ (BƯỚC QUAN TRỌNG ĐỂ CỐ ĐỊNH ID)
    buf[0] = 0x93;
    buf[1] = 0x70;
    buf[2] = id[0]; buf[3] = id[1]; buf[4] = id[2]; buf[5] = id[3];
    // Tính CRC cho lệnh Select (RC522 cần bước này để hoàn tất chọn thẻ)
    status = RC522_ToCard(PCD_TRANSCEIVE, buf, 6, buf, &len);

    // Nếu status == 0, thẻ đã được chọn thành công
    return (status == 0) ? 0 : 1;
}

// ==========================================
// TRUYỀN DỮ LIỆU ĐÓNG GÓI CHUẨN JSON VỀ PC
// ==========================================

void Send_Data_To_PC(void) {
    // Ép kiểu chuỗi số nguyên để chống hiện tượng lỗi bộ nhớ và HardFault khi dùng %f
    int32_t weight_int = (int32_t)final_stable_weight;
    int32_t weight_frac = (int32_t)((final_stable_weight - weight_int) * 100);
    if (weight_frac < 0) weight_frac = -weight_frac;

    sprintf(uart_tx_buffer,
            "{\r\n  \"uid\": \"%02X%02X%02X%02X\",\r\n  \"weight\": %ld.%02ld,\r\n  \"timestamp\": \"20%02d-%02d-%02d %02d:%02d:%02d\"\r\n}\r\n",
            card_uid[0], card_uid[1], card_uid[2], card_uid[3],
            weight_int, weight_frac,
            rtc_year, rtc_month, rtc_date, rtc_hours, rtc_minutes, rtc_seconds);

    HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 1000);
}
void Send_Card_ID_To_PC(uint8_t *uid) {
    sprintf(uart_tx_buffer, "\r\n[INFO] Card Detected! ID: %02X%02X%02X%02X\r\n",
            uid[0], uid[1], uid[2], uid[3]);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 1000);
}


void Display_Data_On_OLED(float weight, uint8_t *uid) {
    char buffer[20];
    SSD1306_Clear();

    // 1. Hiển thị tiêu đề
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("SMART SCALE", &Font_7x10, 1);

    // 2. Hiển thị ID Thẻ
    SSD1306_GotoXY(0, 15);
    sprintf(buffer, "ID: %02X%02X%02X%02X", uid[0], uid[1], uid[2], uid[3]);
    SSD1306_Puts(buffer, &Font_6x8, 1);

    // 3. Hiển thị Cân nặng
    SSD1306_GotoXY(0, 30);
    SSD1306_Puts("WEIGHT:", &Font_7x10, 1);

    SSD1306_GotoXY(50, 30);
    sprintf(buffer, "%.2f g", weight);
    SSD1306_Puts(buffer, &Font_11x18, 1);

    SSD1306_UpdateScreen();
}
// ==========================================
// DRIVER NGOẠI VI 4: QUÉT HÀM HIỂN THỊ 2 LED 5161BS
// ==========================================

// 1. Hàm phân tách dữ liệu cân nặng (Ví dụ: 65.4f -> làm tròn 65 -> LED_chục = 6, LED_đơn_vị = 5)
void Update_LED_Buffer(float weight) {
    int16_t int_weight = (int16_t)(weight + 0.5f); // Làm tròn số thực về số nguyên gần nhất

    if (int_weight > 99) int_weight = 99; // Giới hạn dải đo hiển thị từ 0 - 99 kg
    if (int_weight < 0)  int_weight = 0;

    uint8_t hang_chuc = int_weight / 10;
    uint8_t hang_don_vi = int_weight % 10;

    // Nạp mã Hex của Anode chung tương ứng vào buffer
    display_digits[0] = LED_CA_CODE[hang_chuc];
    display_digits[1] = LED_CA_CODE[hang_don_vi];
}

// 2. Hàm thực hiện quét LED luân phiên
void LED_Scan_Routine(void) {
    static uint8_t current_led = 0;

    // 1. Tắt nguồn cấp cho cả 2 con LED (đưa về mức THẤP - 0V)
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);

    // 2. Xuất dữ liệu thanh (Port D)
    // Mã cần đảo bit (~) vì giờ bạn đang cắm trực tiếp (0V = Sáng)
    GPIOD->ODR = (GPIOD->ODR & 0xFF00) | (~display_digits[current_led]);

    // 3. Kích nguồn (3.3V) cho con LED cần hiển thị
    if (current_led == 0) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET); // Bật hàng chục
    } else {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET); // Bật hàng đơn vị
    }

    // 4. Luân chuyển giữa 2 LED
    current_led = (current_led == 0) ? 1 : 0;
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
#ifdef USE_FULL_ASSERT
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
