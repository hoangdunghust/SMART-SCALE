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
#include "SSD1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DS3231_ADDRESS (0xD0)
// HỆ SỐ CÂN ĐÃ ĐƯỢC CĂN CHỈNH: Trọng lượng thật 50g mà hiện 0.18kg => Hệ số mới = 210000 * (0.18 / 0.05) = 756000.0f
#define HX711_SCALE 75600.0f
#define MAX_LEN 16 // Định nghĩa kích thước đệm cho RFID

// Lệnh điều khiển RFID RC522 thông qua thanh ghi cơ bản
#define PCD_IDLE 0x00
#define PCD_AUTHENT 0x0E
#define PCD_TRANSCEIVE 0x0C
#define PICC_REQIDL 0x26
#define PICC_ANTICOLL 0x93
#define CommandReg 0x01
#define FIFODataReg 0x09
#define FIFOLengthReg 0x0A
#define BitFramingReg 0x0D
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Biến lưu trữ trạng thái cân nặng
// Biến toàn cục cho Cân điện tử
float current_weight = 0.0f;
float final_stable_weight = 0.0f;
float last_weight = 0.0f;
int32_t sample_offset = 0;
int32_t current_raw_val = 0;
uint8_t stable_count = 0;

// Biến lưu thông tin thẻ RFID
uint8_t card_uid[5];
uint8_t is_card_detected = 0;

// Biến lưu trữ thời gian từ RTC
uint8_t rtc_seconds, rtc_minutes, rtc_hours;
uint8_t rtc_day, rtc_date, rtc_month, rtc_year;

// Buffer truyền UART
char uart_tx_buffer[256];

// Mảng mã LED 7 đoạn cho Anode chung
uint8_t LED_CA_CODE[] = {0xC0, 0xCF, 0xA4, 0x86, 0x8B, 0x92, 0x90, 0xC7, 0x80, 0x82};
// Mảng lưu dữ liệu hiển thị hiện tại cho 2 LED (mặc định tắt - 0xFF)
uint8_t display_digits[2] = {0xFF, 0xFF};

typedef struct {
    uint8_t day, month, year;
    uint8_t hour, minute, second;
} TimeStamp_t;

typedef struct {
    uint8_t uid[4];
    float weight_history[3];
    TimeStamp_t time_history[3]; // Lưu thời gian tương ứng cho 3 lần cân
    uint8_t count;
} CardHistory_t;

CardHistory_t current_card_log = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);

/* USER CODE BEGIN PFP */

void Update_And_Log_History(uint8_t *uid, float new_weight);

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
void Send_Data_To_PC(float weight);

// Các hàm cập nhật và Quét hiển thị LED 7 đoạn
void Update_LED_Buffer(float weight);
void LED_Scan_Routine(void);
void Display_Data_On_OLED(float weight, uint8_t *uid);
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
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
    // Khởi tạo Màn hình OLED
    SSD1306_Init();

    // Khởi tạo thẻ RFID RC522
    RC522_Init();

    //HAL_UART_Transmit(&huart1, (uint8_t *)"Scanning I2C Bus...\r\n", 21, 100);
    for (uint8_t i = 0; i < 255; i++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, i, 1, 10) == HAL_OK)
        {
            char msg[30];
//            sprintf(msg, "Found device at: 0x%02X\r\n", i);
//            HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
        }
    }
    //HAL_UART_Transmit(&huart1, (uint8_t *)"Scan complete.\r\n", 16, 100);

    // Đợi mạch ổn định rồi mới lấy mẫu Tare (Zero)
    HAL_Delay(500);
    HX711_Tare();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    uint32_t last_update = 0;

    while (1)
    {
    	if (HAL_GetTick() - last_update >= 5000)
    	        {
    	            last_update = HAL_GetTick();

    	            // 1. Đọc dữ liệu cân nặng (chỉ thực hiện đúng 5s một lần)
    	            current_weight = HX711_GetWeight();

    	            // 2. Cập nhật dữ liệu cho buffer LED 7 thanh
    	            Update_LED_Buffer(current_weight);

    	            // 3. Cập nhật màn hình OLED (chỉ vẽ lại sau mỗi 5s, giúp OLED bền và không bị nháy)
    	            Display_Data_On_OLED(current_weight, card_uid);

    	            // 4. Gửi dữ liệu đóng gói JSON qua UART lên Hercules
    	            Send_Data_To_PC(current_weight);
    	        }

    	        // --- PHẦN ĐỌC THẺ RFID (Vẫn phải quét liên tục để nhạy thẻ) ---
    	        // Chúng ta đưa phần đọc RFID ra ngoài điều kiện 5s để người dùng
    	        // quẹt thẻ là nhận ngay lập tức, không bị trễ 5 giây.
    	// --- PHẦN ĐỌC THẺ RFID ---
    	static uint32_t last_card_detect_time = 0;

//    	if (RC522_CheckCard(card_uid) == 0) // Có thẻ
//    	{
//    	    is_card_detected = 1;
//    	    last_card_detect_time = HAL_GetTick(); // Cập nhật mốc thời gian mới nhất
//
//    	    Send_Card_ID_To_PC(card_uid);
//    	    Display_Data_On_OLED(current_weight, card_uid);
//    	    HAL_Delay(300);
//    	}
    	// Sửa trong vòng lặp while(1)
    	if (RC522_CheckCard(card_uid) == 0)
    	{
    	    is_card_detected = 1;
    	    last_card_detect_time = HAL_GetTick();

    	    // Thay vì chỉ Send_Card_ID, hãy gọi hàm log mới
    	    Update_And_Log_History(card_uid, current_weight);

    	    Display_Data_On_OLED(current_weight, card_uid);
    	    HAL_Delay(500); // Tăng delay để tránh log tràn liên tục
    	}
    	else // Không có thẻ
    	{
    	    // Kiểm tra nếu đã quá 2 giây kể từ lần cuối thấy thẻ
    	    if (HAL_GetTick() - last_card_detect_time > 2000)
    	    {
    	        if (is_card_detected == 1)
    	        {
    	            is_card_detected = 0;
    	            // XÓA DỮ LIỆU NGAY TẠI ĐÂY
    	            memset(card_uid, 0, 5);
    	            Display_Data_On_OLED(current_weight, card_uid);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  /* USER CODE END Check_RTC_BKUP */

  sTime.Hours = 0x10;
  sTime.Minutes = 0x32;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
  sDate.Month = RTC_MONTH_JULY;
  sDate.Date = 0x16;
  sDate.Year = 0x26;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // Lưu cờ đánh dấu đã cài đặt ngày giờ
  /* USER CODE END RTC_Init 2 */

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
//

void Update_And_Log_History(uint8_t *uid, float new_weight) {
    if (new_weight < 0.01f) return;

    RTC_TimeTypeDef sTime = {0};
        RTC_DateTypeDef sDate = {0};
        HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    // 1. Nếu là thẻ mới hoàn toàn (thẻ khác), mới reset lịch sử
    if (memcmp(current_card_log.uid, uid, 4) != 0) {
        memcpy(current_card_log.uid, uid, 4);
        current_card_log.count = 0;
        for(int i=0; i<3; i++) current_card_log.weight_history[i] = 0.0f;
    }

    // 2. Chống lặp: Chỉ lưu nếu giá trị khác với bản ghi cuối cùng đã lưu
    if (current_card_log.count > 0 && current_card_log.weight_history[current_card_log.count - 1] == new_weight)
        return;

    if (current_card_log.count == 3) {
            for (int i = 0; i < 2; i++) {
                current_card_log.weight_history[i] = current_card_log.weight_history[i + 1];
                current_card_log.time_history[i] = current_card_log.time_history[i + 1];
            }
            // Ghi bản ghi mới vào vị trí cuối (index 2)
            current_card_log.weight_history[2] = new_weight;
            current_card_log.time_history[2] = (TimeStamp_t){sDate.Date, sDate.Month, sDate.Year, sTime.Hours, sTime.Minutes, sTime.Seconds};
        } else {
            // Ghi vào vị trí tiếp theo nếu count < 3
            int idx = current_card_log.count;
            current_card_log.weight_history[idx] = new_weight;
            current_card_log.time_history[idx] = (TimeStamp_t){sDate.Date, sDate.Month, sDate.Year, sTime.Hours, sTime.Minutes, sTime.Seconds};
            current_card_log.count++;
        }

    // 4. In ra 3 bản ghi hiện có
    sprintf(uart_tx_buffer, "\r\n--- History (UID: %02X%02X%02X%02X) ---\r\n",
            uid[0], uid[1], uid[2], uid[3]);
    HAL_UART_Transmit(&huart1, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    for (int i = 0; i < current_card_log.count; i++) {
        int32_t h_int = (int32_t)current_card_log.weight_history[i];
        int32_t h_frac = (int32_t)((current_card_log.weight_history[i] - (float)h_int) * 100.0f);
        if (h_frac < 0) h_frac = -h_frac;

        sprintf(uart_tx_buffer, "%d: %ld.%02ld kg | %02d:%02d:%02d - %02d/%02d/20%02d\r\n",
                i + 1, h_int, h_frac,
                current_card_log.time_history[i].hour,
                current_card_log.time_history[i].minute,
                current_card_log.time_history[i].second,
                current_card_log.time_history[i].day,
                current_card_log.time_history[i].month,
                current_card_log.time_history[i].year);
        HAL_UART_Transmit(&huart1, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    }
}

// ==========================================
// DRIVER NGOẠI VI 1: CẢM BIẾN TRỌNG LƯỢNG HX711
// ==========================================

int32_t HX711_Read(void)
{
    int32_t data = 0;

    // Đợi chân DOUT xuống LOW (Cảm biến đã sẵn sàng)
    // Cảm biến chạy ở 10SPS cần tối đa 100ms để có dữ liệu mới. Đợi tối đa 150ms.
    uint32_t start_tick = HAL_GetTick();
    while (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_SET)
    {
        if (HAL_GetTick() - start_tick > 150)
            return 0; // Tránh treo chip khi đứt dây cảm biến
    }

    // TẮT NGẮT (SysTick) ĐỂ BẢO VỆ XUNG SCK CỦA HX711
    // Nếu bị ngắt quá 60us, HX711 sẽ bị Reset/Tắt nguồn gây ra dữ liệu rác!
    __disable_irq();

    for (int i = 0; i < 24; i++)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
        for (volatile int d = 0; d < 100; d++)
        {
        } // Tốc độ STM32F429 180MHz quá nhanh! Phải tạo trễ ~1us (> 200ns)

        data = data << 1;
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_SET)
        {
            data++;
        }

        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
        for (volatile int d = 0; d < 100; d++)
        {
        } // Trễ 1us cho xung cạnh xuống
    }

    // Xung thứ 25 đặt Gain 128
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    for (volatile int d = 0; d < 100; d++)
    {
    }
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 100; d++)
    {
    }

    // BẬT LẠI NGẮT
    __enable_irq();

    // Sign extension cho số âm 24-bit
    if (data & 0x00800000)
    {
        data |= 0xFF000000;
    }
    return data;
}

void HX711_Tare(void)
{
    // 1. Đọc bỏ (vứt đi) 5 mẫu đầu tiên vì khi HX711 vừa bật lên,
    // các tín hiệu đầu tiên thường chưa ổn định hoặc bị lưu rác (stale data).
    for (int i = 0; i < 5; i++)
    {
        HX711_Read();
        HAL_Delay(10);
    }

    // 2. Lấy trung bình 15 mẫu tiếp theo làm mốc số 0 (Zero/Tare)
    int64_t sum = 0;
    for (int i = 0; i < 15; i++)
    {
        sum += HX711_Read();
        HAL_Delay(10);
    }
    sample_offset = (int32_t)(sum / 15);
}

float HX711_GetWeight(void)
{
    int64_t sum = 0;
    int count = 3; // Giảm từ 10 xuống 3 để cân phản hồi cực nhanh, hết bị trễ
    for (int i = 0; i < count; i++)
    {
        sum += HX711_Read();
        // Không dùng HAL_Delay ở đây để đọc nhanh nhất có thể
    }
    float raw_val = (float)(sum / count);
    current_raw_val = (int32_t)raw_val; // Lưu lại để in ra debug
    float new_weight = (raw_val - (float)sample_offset) / HX711_SCALE;

    // Ép về 0 nếu trọng lượng bé hơn 10 gram (0.01kg).
    // Giúp mặt cân luôn hiển thị 0.00 tĩnh lặng khi không có vật, lọc bỏ nhiễu gió/rung.
    if (new_weight < 0.01f && new_weight > -0.01f)
    {
        return 0.0f;
    }
    if (new_weight < 0.01f)
        {
            return 0.0f;
        }

    return new_weight;
}

// ==========================================
// DRIVER NGOẠI VI 3: ĐỌC THẺ RFID RC522 (SPI)
// ==========================================

void RC522_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t buffer[2];
    buffer[0] = (reg << 1) & 0x7E;
    buffer[1] = val;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    __NOP();

    HAL_SPI_Transmit(&hspi1, buffer, 2, 100);

    __NOP();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // Kéo CS lên cao
}

uint8_t RC522_ReadReg(uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = ((reg << 1) & 0x7E) | 0x80;
    tx[1] = 0x00;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100); // Đổi HAL_MAX_DELAY thành 100ms tránh treo vi điều khiển
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
    RC522_WriteReg(0x14, 0x8D); // TxControl
    RC522_WriteReg(0x11, 0x3D); // Mode
    RC522_WriteReg(0x26, 0x70); // RFCfgReg: Set RxGain to 48dB (Bắt buộc cho mạch fake 0x82)

    // 🔥 BẬT ANTEN
    uint8_t val = RC522_ReadReg(0x14);
    RC522_WriteReg(0x14, val | 0x03);

    HAL_Delay(10);

    RC522_WriteReg(CommandReg, 0x0C); // Transmit RF
}

uint8_t RC522_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint32_t *backLen)
{
    uint8_t status = 1;
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;

    if (command == PCD_TRANSCEIVE)
    {
        irqEn = 0x77;
        waitIRq = 0x30;
    }
    RC522_WriteReg(0x02, irqEn | 0x80);
    RC522_WriteReg(0x04, 0x7F); // Clear all interrupt request bits (ghi 1 để clear)
    RC522_WriteReg(CommandReg, PCD_IDLE);
    RC522_WriteReg(0x0A, 0x80);

    for (int i = 0; i < sendLen; i++)
    {
        RC522_WriteReg(FIFODataReg, sendData[i]);
    }
    RC522_WriteReg(CommandReg, command);
    if (command == PCD_TRANSCEIVE)
    {
        uint8_t set = RC522_ReadReg(BitFramingReg);
        RC522_WriteReg(BitFramingReg, set | 0x80);
    }

    uint32_t index = 2000;
    uint8_t n;
    do
    {
        n = RC522_ReadReg(0x04);
        index--;
    } while ((index != 0) && !(n & 0x01) && !(n & waitIRq));

    uint8_t set2 = RC522_ReadReg(BitFramingReg);
    RC522_WriteReg(BitFramingReg, set2 & (~0x80));

    if (index != 0)
    {
        if (!(RC522_ReadReg(0x06) & 0x1B))
        {
            status = 0;
            if (n & irqEn & 0x01)
                status = 1;
            if (command == PCD_TRANSCEIVE)
            {
                n = RC522_ReadReg(FIFOLengthReg);
                uint8_t lastBits = RC522_ReadReg(0x0C) & 0x07; // 0x0C là ControlReg chứa RxLastBits
                if (lastBits)
                    *backLen = (n - 1) * 8 + lastBits;
                else
                    *backLen = n * 8;
                if (n == 0)
                    n = 1;
                if (n > 16)
                    n = 16;
                for (int i = 0; i < n; i++)
                {
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

    // 1. WUPA
    RC522_WriteReg(0x15, 0x40);
    RC522_WriteReg(BitFramingReg, 0x07);
    buf[0] = 0x52;
    status = RC522_ToCard(PCD_TRANSCEIVE, buf, 1, buf, &len);
    if (status != 0)
    {
        //HAL_UART_Transmit(&huart1, (uint8_t *)"[DEBUG] WUPA Failed\r\n", 21, 100);
        return 1;
    }

    // 2. ANTICOLLISION
    RC522_WriteReg(BitFramingReg, 0x00);
    for (int i = 0; i < 16; i++)
        buf[i] = 0x00; // Xóa sạch buffer cũ
    buf[0] = 0x93;
    buf[1] = 0x20;
    status = RC522_ToCard(PCD_TRANSCEIVE, buf, 2, buf, &len);
    if (status != 0)
    {
        //HAL_UART_Transmit(&huart1, (uint8_t *)"[DEBUG] ANTI Failed\r\n", 21, 100);
        return 1;
    }

    // Kiểm tra BCC
    uint8_t bcc = 0;
    for (int i = 0; i < 4; i++)
    {
        id[i] = buf[i];
        bcc ^= buf[i];
    }
    if (bcc != buf[4])
    {
//        sprintf(uart_tx_buffer, "[DEBUG] BCC Fail: %02X %02X %02X %02X -> BCC=%02X (calc=%02X)\r\n",
//                buf[0], buf[1], buf[2], buf[3], buf[4], bcc);
//        HAL_UART_Transmit(&huart1, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer), 100);
        return 1;
    }

    return 0; // Đọc thành công
}

// ==========================================
// TRUYỀN DỮ LIỆU ĐÓNG GÓI CHUẨN JSON VỀ PC
// ==========================================

void Send_Data_To_PC(float weight)
{
    int32_t w_int = (int32_t)weight;
    int32_t w_frac = (int32_t)((weight - (float)w_int) * 100.0f);
    if (w_frac < 0)
        w_frac = -w_frac;

    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    sprintf(uart_tx_buffer,
            "{\r\n  \"uid\": \"%02X%02X%02X%02X\",\r\n  \"weight\": %ld.%02ld,\r\n }\r\n",
            card_uid[0], card_uid[1], card_uid[2], card_uid[3],
            w_int, w_frac);

    if (is_card_detected == 0) {
            sprintf(uart_tx_buffer, "{\r\n  \"uid\": \"00000000\",\r\n  \"weight\": %ld.%02ld,\r\n }\r\n",
                    w_int, w_frac);
        } else {
            sprintf(uart_tx_buffer, "{\r\n  \"uid\": \"%02X%02X%02X%02X\",\r\n  \"weight\": %ld.%02ld,\r\n }\r\n",
                    card_uid[0], card_uid[1], card_uid[2], card_uid[3], w_int, w_frac);
        }

        HAL_UART_Transmit(&huart1, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer), 1000);

}
void Send_Card_ID_To_PC(uint8_t *uid)
{
//    sprintf(uart_tx_buffer, "\r\n[INFO] Card Detected! ID: %02X%02X%02X%02X\r\n",
//            uid[0], uid[1], uid[2], uid[3]);
//    HAL_UART_Transmit(&huart1, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer), 1000);
}

void Display_Data_On_OLED(float weight, uint8_t *uid)
{
    char buffer[20];

    SSD1306_Clear();

    // Lời chào
    SSD1306_GotoXY(0, 0);
    //SSD1306_Puts("Xin chao !", &Font_7x10, SSD1306_COLOR_WHITE);

    // Hiển thị Cân Nặng (Tách số thực thành 2 phần nguyên để in, tránh lỗi %f)
    int32_t w_int = (int32_t)weight;
    int32_t w_frac = (int32_t)((weight - (float)w_int) * 100.0f);
    if (w_frac < 0)
        w_frac = -w_frac;

    SSD1306_GotoXY(0, 20);
    sprintf(buffer, "Weight: %ld.%02ld kg", w_int, w_frac);
    SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);

    // Hiển thị ID Thẻ RFID và Version (Dùng để chẩn đoán lỗi SPI)
    SSD1306_GotoXY(0, 40);
    if (is_card_detected)
    {
        sprintf(buffer, "ID:%02X%02X%02X%02X", uid[0], uid[1], uid[2], uid[3]);
    }
    else
    {
        sprintf(buffer, "No Card");
    }
    SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);

    SSD1306_UpdateScreen();
}
// ==========================================
// DRIVER NGOẠI VI 4: QUÉT HÀM HIỂN THỊ 2 LED 5161BS
// ==========================================

// 1. Hàm phân tách dữ liệu cân nặng (Ví dụ: 65.4f -> làm tròn 65 -> LED_chục = 6, LED_đơn_vị = 5)
void Update_LED_Buffer(float weight)
{
    int16_t int_weight = (int16_t)(weight + 0.5f); // Làm tròn số thực về số nguyên gần nhất

    if (int_weight > 99)
        int_weight = 99; // Giới hạn dải đo hiển thị từ 0 - 99 kg
    if (int_weight < 0)
        int_weight = 0;

    uint8_t hang_chuc = int_weight / 10;
    uint8_t hang_don_vi = int_weight % 10;

    // Nạp mã Hex của Anode chung tương ứng vào buffer
    display_digits[0] = LED_CA_CODE[hang_chuc];
    display_digits[1] = LED_CA_CODE[hang_don_vi];
}

// 2. Hàm thực hiện quét LED luân phiên
void LED_Scan_Routine(void)
{
    static uint8_t current_led = 0;

    // 1. Tắt nguồn cấp cho cả 2 con LED (đưa về mức THẤP - 0V)
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);

    // 2. Xuất dữ liệu thanh (Port D)
    // Mã đã chuẩn cho Anode chung, không cần đảo bit (~)
    GPIOD->ODR = (GPIOD->ODR & 0xFF00) | (display_digits[current_led]);

    // 3. Kích nguồn (3.3V) cho con LED cần hiển thị
    if (current_led == 0)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET); // Bật hàng chục
    }
    else
    {
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
