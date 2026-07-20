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
#include "ds1307.h"
#include "eeprom.h"
#include "7seg.h"
#include "user.h"
#include "history.h"
// #include "7seg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DS3231_ADDRESS (0xD0)
#define HX711_SCALE 109652.0f
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
#define SEND_HISTORY 3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

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
volatile uint8_t is_card_detected = 0;
volatile uint32_t last_card_detect_time = 0;

// Biến lưu trữ thời gian từ RTC
uint8_t rtc_seconds, rtc_minutes, rtc_hours;
uint8_t rtc_day, rtc_date, rtc_month, rtc_year;

// Buffer truyền UART
char uart_tx_buffer[256];

// Mảng mã LED 7 đoạn cho Anode chung
uint8_t LED_CA_CODE[] = {
    0xC0, // 0
    0xF9, // 1
    0xA4, // 2
    0xB0, // 3
    0x99, // 4
    0x92, // 5
    0x82, // 6
    0xF8, // 7
    0x80, // 8
    0x90  // 9
};
// Mảng lưu dữ liệu hiển thị hiện tại cho 2 LED (mặc định tắt - 0xFF)
uint8_t display_digits[2] = {0xFF, 0xFF};


UserProfile_t users[MAX_USER];

volatile int8_t history_index = -1; // -1: Weighing Mode, 0-4: Index of history item
uint32_t history_view_start_time = 0; // Track auto-timeout
#define HISTORY_VIEW_TIMEOUT_MS 8000 // 8 seconds timeout
volatile uint8_t g_button_pressed_flag = 0; // Button press flag set by EXTI
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
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
void Send_Data_To_PC(float weight);

// Các hàm cập nhật và Quét hiển thị LED 7 đoạn
void Update_LED_Buffer(float weight);
void LED_Scan_Routine(void);
void Display_Data_On_OLED(float weight, uint8_t *uid);
void Send_History_To_PC(uint8_t *uid, float latest_weight);
void Read_History(LogEntry_t *logs, uint8_t number);
int Find_User(uint8_t *uid);

void Add_History(int user_id,float weight);

void Send_History_To_PC(uint8_t *uid,float weight);
void Display_Single_History_On_OLED(int user_id, int8_t hist_index, uint8_t total_count);
void Add_Global_History(float weight, TimeStamp_t ts);
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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim6);
    // Chèn vào đầu hàm main sau khi khởi tạo I2C
    for (uint8_t i = 0; i < 127; i++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 1, 10) == HAL_OK)
        {
            char msg[20];
            sprintf(msg, "Found: 0x%02X\r\n", i);
            HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
        }
    }
    // Cập nhật đúng thời gian thực hiện tại: 20/07/2026, 19:02:00
    // LƯU Ý: Sau khi nạp code lần đầu để cài giờ thành công, hãy comment dòng DS1307_SetTime bên dưới
    // và nạp lại để tránh RTC bị reset về thời điểm này mỗi khi mất nguồn/khởi động lại.
    DS1307_Time now = {0, 2, 19, 2, 20, 7, 26};
    //DS1307_SetTime(&now);
    // Khởi tạo Màn hình OLED
    SSD1306_Init();

    // Khởi tạo thẻ RFID RC522
    RC522_Init();

    // HAL_UART_Transmit(&huart1, (uint8_t *)"Scanning I2C Bus...\r\n", 21, 100);
    for (uint8_t i = 0; i < 255; i++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, i, 1, 10) == HAL_OK)
        {
            char msg[30];
            //            sprintf(msg, "Found device at: 0x%02X\r\n", i);
            //            HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
        }
    }

    // Khởi tạo và format EEPROM nếu đây là lần khởi động đầu tiên
    EEPROM_Init();

    // Đợi mạch ổn định rồi mới lấy mẫu Tare (Zero)
    HAL_Delay(2000);
    HX711_Tare();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    uint32_t last_update = 0;

    while (1)
    {
        // 1. KIỂM TRA CỜ HIỆU NÚT NHẤN (Được bật lên từ ngắt EXTI0)
        if (g_button_pressed_flag == 1)
        {
            g_button_pressed_flag = 0; // Reset cờ ngay lập tức

            if (is_card_detected == 0)
            {
                // Nếu chưa quét thẻ, nhấn nút sẽ thực hiện Tare (Trừ bì / Reset về 0)
                SSD1306_Clear();
                SSD1306_GotoXY(0, 0);
                SSD1306_Puts("--- TARE ZERO ---", &Font_7x10, SSD1306_COLOR_WHITE);
                SSD1306_GotoXY(0, 24);
                SSD1306_Puts("Dang Reset 0...", &Font_7x10, SSD1306_COLOR_WHITE);
                SSD1306_UpdateScreen();
                
                HX711_Tare();
                
                SSD1306_Clear();
                SSD1306_GotoXY(0, 0);
                SSD1306_Puts("--- TARE ZERO ---", &Font_7x10, SSD1306_COLOR_WHITE);
                SSD1306_GotoXY(0, 24);
                SSD1306_Puts("Da ve 0.00 kg!", &Font_7x10, SSD1306_COLOR_WHITE);
                SSD1306_UpdateScreen();
                HAL_Delay(1000);
                
                // Quay về màn hình cân
                history_index = -1;
                Display_Data_On_OLED(current_weight, card_uid);
            }
            else
            {
                // In debug ra UART để kiểm tra tại sao Find_User thất bại
                char dbg[128];
                sprintf(dbg, "[DEBUG] Card UID: %02X%02X%02X%02X\r\n", card_uid[0], card_uid[1], card_uid[2], card_uid[3]);
                HAL_UART_Transmit(&huart1, (uint8_t *)dbg, strlen(dbg), 100);

                for (int i = 0; i < MAX_USER; i++)
                {
                    UserProfile_t u;
                    EEPROM_Read(USER_ADDR(i), (uint8_t*)&u, sizeof(UserProfile_t));
                    sprintf(dbg, "[DEBUG] Slot %d UID: %02X%02X%02X%02X, count: %d\r\n", i, u.uid[0], u.uid[1], u.uid[2], u.uid[3], u.count);
                    HAL_UART_Transmit(&huart1, (uint8_t *)dbg, strlen(dbg), 100);
                }

                int user_id = Find_User(card_uid);
                if (user_id >= 0)
                {
                    UserProfile_t user;
                    EEPROM_Read(USER_ADDR(user_id), (uint8_t*)&user, sizeof(UserProfile_t));

                    uint8_t count = user.count;
                    if (count == 0)
                    {
                        // Hiển thị thông báo chưa có dữ liệu cho user này
                        SSD1306_Clear();
                        SSD1306_GotoXY(0, 0);
                        SSD1306_Puts("--- LICH SU ---", &Font_7x10, SSD1306_COLOR_WHITE);
                        SSD1306_GotoXY(0, 24);
                        SSD1306_Puts("Chua co du lieu!", &Font_7x10, SSD1306_COLOR_WHITE);
                        SSD1306_UpdateScreen();
                        HAL_Delay(1500);
                        
                        // Quay về màn hình cân
                        history_index = -1;
                        Display_Data_On_OLED(current_weight, card_uid);
                    }
                    else
                    {
                        // Chuyển tuần tự qua các bản ghi lịch sử của user
                        history_index++;
                        if (history_index >= count)
                        {
                            history_index = -1; // Quay lại màn hình cân
                        }

                        if (history_index == -1)
                        {
                            Display_Data_On_OLED(current_weight, card_uid);
                        }
                        else
                        {
                            history_view_start_time = HAL_GetTick(); // Ghi nhận mốc thời gian xem
                            Display_Single_History_On_OLED(user_id, history_index, count);
                        }
                    }
                }
                else
                {
                    // Thẻ không hợp lệ / không tìm thấy trong bộ nhớ
                    SSD1306_Clear();
                    SSD1306_GotoXY(0, 0);
                    SSD1306_Puts("--- LICH SU ---", &Font_7x10, SSD1306_COLOR_WHITE);
                    SSD1306_GotoXY(0, 24);
                    SSD1306_Puts("The ko hop le!", &Font_7x10, SSD1306_COLOR_WHITE);
                    SSD1306_UpdateScreen();
                    HAL_Delay(1500);
                    
                    history_index = -1;
                    Display_Data_On_OLED(current_weight, card_uid);
                }
            }
        }

        // 2. TỰ ĐỘNG QUAY VỀ MÀN HÌNH CÂN SAU KHI HẾT TIMEOUT XEM LỊCH SỬ (8 giây)
        if (history_index != -1 && (HAL_GetTick() - history_view_start_time >= HISTORY_VIEW_TIMEOUT_MS))
        {
            history_index = -1;
            Display_Data_On_OLED(current_weight, card_uid);
        }

        // 3. CẬP NHẬT CÂN, LED, OLED ĐỊNH KỲ 1 GIÂY (Chỉ cập nhật OLED nếu đang ở màn hình cân)
        if(HAL_GetTick()-last_update>=1000)
        {
            last_update=HAL_GetTick();

            float raw_weight = HX711_GetWeight();
            
            // Bộ lọc số mũ EMA công nghiệp để chống rung cơ khí và nhảy số biên
            static float filtered_weight = 0.0f;
            float diff = raw_weight - filtered_weight;
            if (diff < 0) diff = -diff;
            
            if (diff > 0.05f) // Có thay đổi lớn (trên 50g) -> Cập nhật tức thời
            {
                filtered_weight = raw_weight;
            }
            else // Dao động nhỏ (nhiễu/rung) -> Lọc số mũ mượt mà
            {
                filtered_weight = 0.1f * raw_weight + 0.9f * filtered_weight;
            }
            
            current_weight = filtered_weight;

            // In ra debug cổng serial để kiểm tra cảm biến lực HX711 hoạt động thế nào
            char dbg_hx[64];
            int w_int = (int)current_weight;
            int w_frac = (int)((current_weight - w_int) * 100);
            if (w_frac < 0) w_frac = -w_frac;
            sprintf(dbg_hx, "[DEBUG] Weight: %d.%02d kg, Raw ADC: %ld\r\n", w_int, w_frac, current_raw_val);
            HAL_UART_Transmit(&huart1, (uint8_t *)dbg_hx, strlen(dbg_hx), 100);

            Update_LED_Buffer(current_weight);

            if (history_index == -1)
            {
                Display_Data_On_OLED(current_weight,card_uid);
            }
        }

        // 4. --- PHẦN ĐỌC THẺ RFID ---

        uint8_t temp_uid[5];
        if(RC522_CheckCard(temp_uid)==0)
        {
            memcpy(card_uid, temp_uid, 4);
            is_card_detected = 1;
            history_index = -1; // Quay về màn hình cân khi phát hiện thẻ mới

            int user_id;


            user_id = Find_User(card_uid);


            if(user_id <0)
            {
                user_id = Create_User(card_uid);
            }


            // Đọc cân nặng ngay lập tức khi quét thẻ
            current_weight = HX711_GetWeight();


            Add_History(
                user_id,
                current_weight
            );


            Send_History_To_PC(
                    card_uid,
                    current_weight
            );

            // Cập nhật hiển thị lên OLED ngay lập tức
            Display_Data_On_OLED(current_weight, card_uid);
            
            // Ghi nhận mốc thời gian bắt đầu đếm ngược 5 giây SAU KHI đã cân xong và hiển thị
            last_card_detect_time = HAL_GetTick();
            
            char dbg_scan[64];
            sprintf(dbg_scan, "[DEBUG] Card set time: %lu\r\n", last_card_detect_time);
            HAL_UART_Transmit(&huart1, (uint8_t *)dbg_scan, strlen(dbg_scan), 100);
        }

        // Tự động xóa trạng thái nhận thẻ sau 10 giây để OLED quay lại hiện "No Card"
        if (history_index == -1 && is_card_detected && (HAL_GetTick() - last_card_detect_time >= 10000))
        {
            char dbg_timeout[128];
            sprintf(dbg_timeout, "[DEBUG] Card cleared. GetTick: %lu, diff: %lu\r\n", 
                    HAL_GetTick(), HAL_GetTick() - last_card_detect_time);
            HAL_UART_Transmit(&huart1, (uint8_t *)dbg_timeout, strlen(dbg_timeout), 100);

            is_card_detected = 0;
            Display_Data_On_OLED(current_weight, card_uid);
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 179;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
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
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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

  /*Configure GPIO pin : User_Button_Pin */
  GPIO_InitStruct.Pin = User_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(User_Button_GPIO_Port, &GPIO_InitStruct);

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
  /* Ghi đè cấu hình chân PA0 thành ngắt ngoài EXTI (Cạnh lên) để tránh bị cấu hình mặc định của CubeMX xóa mất */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Cài đặt độ ưu tiên và bật ngắt EXTI Line 0 */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


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
    int count = 3; // Đọc 3 mẫu để cập nhật nhanh (300ms)
    for (int i = 0; i < count; i++)
    {
        sum += HX711_Read();
    }
    float raw_val = (float)(sum / count);
    current_raw_val = (int32_t)raw_val; // Lưu lại để in ra debug
    float new_weight = (raw_val - (float)sample_offset) / HX711_SCALE;

    // Tự động bám điểm 0 (Zero Tracking) khi cân trống trong phạm vi 30g
    if (new_weight > -0.03f && new_weight < 0.03f)
    {
        // Nhích nhẹ sample_offset theo bộ lọc thông thấp để triệt tiêu trôi điểm 0
        sample_offset = (int32_t)(0.9f * (float)sample_offset + 0.1f * raw_val);
        new_weight = 0.0f;
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

    // RF CONFIG
    RC522_WriteReg(0x14, 0x8D); // TxControl
    RC522_WriteReg(0x11, 0x3D); // Mode
    RC522_WriteReg(0x26, 0x70); // RFCfgReg: Set RxGain to 48dB

    // BẬT ANTEN
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
        // HAL_UART_Transmit(&huart1, (uint8_t *)"[DEBUG] WUPA Failed\r\n", 21, 100);
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
        // HAL_UART_Transmit(&huart1, (uint8_t *)"[DEBUG] ANTI Failed\r\n", 21, 100);
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
        return 1;
    }

    return 0; // Đọc thành công
}

// ==========================================
// TRUYỀN DỮ LIỆU ĐÓNG GÓI CHUẨN JSON VỀ PC
// ==========================================
void Send_History_To_PC(uint8_t *uid, float latest)
{
    char buffer[512];
    LogEntry_t logs[MAX_HISTORY];

    Read_History(logs, SEND_HISTORY);

    // ===== Tách latest weight =====
    int latest_int = (int)latest;
    int latest_dec = (int)((latest - latest_int) * 100);

    if(latest_dec < 0)
        latest_dec = -latest_dec;

    sprintf(buffer,
            "{\r\n"
            "\"uid\":\"%02X%02X%02X%02X\",\r\n"
            "\"latest_weight\":%d.%02d,\r\n"
            "\"history\":[",
            uid[0],
            uid[1],
            uid[2],
            uid[3],
            latest_int,
            latest_dec);

    HAL_UART_Transmit(
        &huart1,
        (uint8_t *)buffer,
        strlen(buffer),
        1000);

    uint8_t count;
    EEPROM_Read(
        COUNT_ADDR,
        &count,
        1);

    if (count == 0xFF)
        count = 0;

    if(count > MAX_HISTORY)
        count = MAX_HISTORY;

    for(int i = 0; i < count; i++)
    {
        // ===== Tách weight lịch sử =====
        int weight_int = (int)logs[i].weight;
        int weight_dec = (int)((logs[i].weight - weight_int) * 100);

        if(weight_dec < 0)
            weight_dec = -weight_dec;

        sprintf(buffer,
                "{\"weight\":%d.%02d,"
                "\"date\":\"%02d/%02d/20%02d\","
                "\"time\":\"%02d:%02d:%02d\"}",
                weight_int,
                weight_dec,
                logs[i].ts.day,
                logs[i].ts.month,
                logs[i].ts.year,
                logs[i].ts.hour,
                logs[i].ts.minute,
                logs[i].ts.second);

        HAL_UART_Transmit(
            &huart1,
            (uint8_t *)buffer,
            strlen(buffer),
            1000);

        if(i < count-1)
        {
            HAL_UART_Transmit(
                &huart1,
                (uint8_t *)",",
                1,
                100);
        }
    }

    sprintf(buffer,
            "]\r\n"
            "}\r\n");

    HAL_UART_Transmit(
        &huart1,
        (uint8_t *)buffer,
        strlen(buffer),
        1000);
}
void Send_Data_To_PC(float weight)
{
    int32_t w_int = (int32_t)weight;
    int32_t w_frac = (int32_t)((weight - (float)w_int) * 100.0f);
    if (w_frac < 0)
        w_frac = -w_frac;
    DS1307_Time rtc;

    DS1307_GetTime(&rtc);

    sprintf(uart_tx_buffer,
            "{\r\n  \"uid\": \"%02X%02X%02X%02X\",\r\n  \"weight\": %ld.%02ld,\r\n }\r\n",
            card_uid[0], card_uid[1], card_uid[2], card_uid[3],
            w_int, w_frac);

    if (is_card_detected == 0)
    {
        sprintf(uart_tx_buffer, "{\r\n  \"uid\": \"00000000\",\r\n  \"weight\": %ld.%02ld,\r\n }\r\n",
                w_int, w_frac);
    }
    else
    {
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
void Read_History(LogEntry_t *logs, uint8_t number)
{
    uint8_t count;
    EEPROM_Read(COUNT_ADDR,&count,1);

    if (count == 0xFF) count = 0;

    uint8_t actual_count = count;

    if(count > MAX_HISTORY)
        count = MAX_HISTORY;

    if(number > count)
        number = count;

    for(int i=0;i<number;i++)
    {
        uint8_t index = (actual_count - 1 - i) % MAX_HISTORY;

        EEPROM_Read(
            START_ADDR + index*sizeof(LogEntry_t),
            (uint8_t*)&logs[i],
            sizeof(LogEntry_t)
        );
    }
}
void Add_History(
        int id,
        float weight)
{
    UserProfile_t user;

    EEPROM_Read(
        USER_ADDR(id),
        (uint8_t*)&user,
        sizeof(user)
    );

    for(int i=MAX_HISTORY-1;i>0;i--)
    {
        user.history[i]=user.history[i-1];
    }

    user.history[0].weight=weight;

    // Đọc thời gian thực từ chip RTC DS1307
    DS1307_Time rtc;
    DS1307_GetTime(&rtc);

    user.history[0].ts.day=rtc.date;
    user.history[0].ts.month=rtc.month;
    user.history[0].ts.year=rtc.year;
    user.history[0].ts.hour=rtc.hour;
    user.history[0].ts.minute=rtc.minute;
    user.history[0].ts.second=rtc.second;

    user.count++;

    if(user.count>MAX_HISTORY)
        user.count=MAX_HISTORY;

    EEPROM_Write(
        USER_ADDR(id),
        (uint8_t*)&user,
        sizeof(user)
    );

    // Ghi đồng thời vào Lịch sử hệ thống (Global History)
    Add_Global_History(weight, user.history[0].ts);
}
void Add_Global_History(float weight, TimeStamp_t ts)
{
    uint8_t count = 0;
    EEPROM_Read(COUNT_ADDR, &count, 1);
    
    if (count == 0xFF)
    {
        count = 0;
    }
    
    uint8_t index = count % MAX_HISTORY;
    
    LogEntry_t log;
    log.weight = weight;
    log.ts = ts;
    
    EEPROM_Write(
        START_ADDR + index * sizeof(LogEntry_t),
        (uint8_t*)&log,
        sizeof(LogEntry_t)
    );
    
    count++;
    EEPROM_Write(COUNT_ADDR, &count, 1);
}
void Display_Data_On_OLED(float weight, uint8_t *uid)
{
    char buffer[20];

    SSD1306_Clear();

    // Lời chào
    SSD1306_GotoXY(0, 0);
    // SSD1306_Puts("Xin chao !", &Font_7x10, SSD1306_COLOR_WHITE);

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
void Display_Single_History_On_OLED(int user_id, int8_t hist_index, uint8_t total_count)
{
    if (total_count == 0)
    {
        SSD1306_Clear();
        SSD1306_GotoXY(0, 0);
        SSD1306_Puts("--- LICH SU ---", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 24);
        SSD1306_Puts("Chua co du lieu!", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_UpdateScreen();
        return;
    }

    UserProfile_t user;
    EEPROM_Read(
        USER_ADDR(user_id),
        (uint8_t*)&user,
        sizeof(UserProfile_t)
    );

    // Lấy trực tiếp từ mảng history đã được dịch chuyển tuần tự trong cấu trúc User
    History_t log = user.history[hist_index];

    SSD1306_Clear();

    // Title
    SSD1306_GotoXY(0, 0);
    char title_buf[25];
    uint8_t display_total = total_count > MAX_HISTORY ? MAX_HISTORY : total_count;
    sprintf(title_buf, "--- LICH SU %d/%d ---", hist_index + 1, display_total);
    SSD1306_Puts(title_buf, &Font_7x10, SSD1306_COLOR_WHITE);

    // Weight line
    int w_int = (int)log.weight;
    int w_frac = (int)((log.weight - w_int) * 100);
    if (w_frac < 0) w_frac = -w_frac;
    
    char weight_buf[25];
    SSD1306_GotoXY(0, 16);
    sprintf(weight_buf, "W:%d.%02d kg", w_int, w_frac);
    SSD1306_Puts(weight_buf, &Font_11x18, SSD1306_COLOR_WHITE);

    // Timestamp line
    char time_buf[30];
    SSD1306_GotoXY(0, 38);
    sprintf(time_buf, "Time: %02d:%02d:%02d", log.ts.hour, log.ts.minute, log.ts.second);
    SSD1306_Puts(time_buf, &Font_7x10, SSD1306_COLOR_WHITE);

    SSD1306_GotoXY(0, 50);
    sprintf(time_buf, "Date: %02d/%02d/20%02d", log.ts.day, log.ts.month, log.ts.year);
    SSD1306_Puts(time_buf, &Font_7x10, SSD1306_COLOR_WHITE);

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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        LED_Scan_Routine();
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        static uint32_t last_press_time = 0;
        uint32_t current_time = HAL_GetTick();

        // Chống rung phím 250ms
        if (current_time - last_press_time > 250)
        {
            g_button_pressed_flag = 1;
            last_press_time = current_time;
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
