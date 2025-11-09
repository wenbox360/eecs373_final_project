/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32 + ILI9341 LCD with fast 5-0 countdown
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdint.h>

/* USER CODE BEGIN PD */
#define SPI_CS_GPIO    GPIOE
#define SPI_CS_PIN     GPIO_PIN_12
#define SPI_DC_GPIO    GPIOE
#define SPI_DC_PIN     GPIO_PIN_11
#define SPI_RESET_GPIO GPIOE
#define SPI_RESET_PIN  GPIO_PIN_10

#define CS_LOW()       HAL_GPIO_WritePin(SPI_CS_GPIO, SPI_CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH()      HAL_GPIO_WritePin(SPI_CS_GPIO, SPI_CS_PIN, GPIO_PIN_SET)
#define DC_LOW()       HAL_GPIO_WritePin(SPI_DC_GPIO, SPI_DC_PIN, GPIO_PIN_RESET)
#define DC_HIGH()      HAL_GPIO_WritePin(SPI_DC_GPIO, SPI_DC_PIN, GPIO_PIN_SET)
#define RESET_LOW()    HAL_GPIO_WritePin(SPI_RESET_GPIO, SPI_RESET_PIN, GPIO_PIN_RESET)
#define RESET_HIGH()   HAL_GPIO_WritePin(SPI_RESET_GPIO, SPI_RESET_PIN, GPIO_PIN_SET)

#define WIDTH  240
#define HEIGHT 320

SPI_HandleTypeDef hspi1;
uint8_t RGB_MODE = 0x55; // 16-bit color (RGB565)
uint16_t frame[WIDTH*HEIGHT]; // Full screen buffer

/* USER CODE END PD */

/* USER CODE BEGIN 0 */
void WriteCommand(uint8_t cmd){
    DC_LOW();
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    CS_HIGH();
}

void WriteData(uint8_t *data, uint16_t size){
    DC_HIGH();
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, data, size, HAL_MAX_DELAY);
    CS_HIGH();
}

void ILI9341_Reset(void){
    RESET_LOW();
    HAL_Delay(1000);
    RESET_HIGH();
    HAL_Delay(1000);
}

// Fill the frame buffer
void FillFrame(uint16_t color){
    for(uint32_t i=0;i<WIDTH*HEIGHT;i++) frame[i] = color;
}

// Send the frame buffer to the display
void ILI9341_DisplayFrame(void){
    // Column address: 0 → WIDTH-1
    WriteCommand(0x2A);
    uint8_t col[4] = {0x00, 0x00, (WIDTH-1)>>8, (WIDTH-1)&0xFF};
    WriteData(col, 4);

    // Page address: 0 → HEIGHT-1
    WriteCommand(0x2B);
    uint8_t page[4] = {0x00, 0x00, (HEIGHT-1)>>8, (HEIGHT-1)&0xFF};
    WriteData(page, 4);

    // Memory write
    WriteCommand(0x2C);
    DC_HIGH();
    CS_LOW();

    // Send row by row to ensure correct orientation
    for(uint16_t y=0; y<HEIGHT; y++){
        HAL_SPI_Transmit(&hspi1, (uint8_t*)&frame[y*WIDTH], WIDTH*2, HAL_MAX_DELAY);
    }

    CS_HIGH();
}

// 8x8 font for digits 0-9
const uint8_t font_digits[10][8] = {
    {0x3C,0x66,0x6E,0x7E,0x76,0x66,0x3C,0x00}, //0
    {0x18,0x38,0x18,0x18,0x18,0x18,0x3C,0x00}, //1
    {0x3C,0x66,0x06,0x0C,0x18,0x30,0x7E,0x00}, //2
    {0x3C,0x66,0x06,0x1C,0x06,0x66,0x3C,0x00}, //3
    {0x0C,0x1C,0x3C,0x6C,0x7E,0x0C,0x0C,0x00}, //4
    {0x7E,0x60,0x7C,0x06,0x06,0x66,0x3C,0x00}, //5
    {0x1C,0x30,0x60,0x7C,0x66,0x66,0x3C,0x00}, //6
    {0x7E,0x66,0x0C,0x18,0x18,0x18,0x18,0x00}, //7
    {0x3C,0x66,0x66,0x3C,0x66,0x66,0x3C,0x00}, //8
    {0x3C,0x66,0x66,0x3E,0x06,0x0C,0x38,0x00}  //9
};

// Draw digit into frame buffer (fast)
void DrawDigitToFrame(int x,int y,uint8_t digit,uint16_t color,uint8_t scale){
    if(digit>9) return;
    for(uint8_t row=0; row<8; row++){
    	for(uint8_t col=0; col<8; col++){
    	    if(font_digits[digit][row] & (1 << col)){  // <--- use LSB first
    	        for(uint8_t i=0;i<scale;i++){
    	            for(uint8_t j=0;j<scale;j++){
    	                int px = x + col*scale + i;
    	                int py = y + row*scale + j;
    	                if(px<WIDTH && py<HEIGHT) frame[py*WIDTH + px] = color;
    	            }
    	        }
    	    }
    	}
    }
}
/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

int main(void){
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    HAL_Delay(1000); // Sleep out
    ILI9341_Reset();
    HAL_Delay(1000); // Sleep out
    WriteCommand(0x11);
    HAL_Delay(120); // Sleep out
    WriteCommand(0x3A);
    WriteData(&RGB_MODE,1); // Pixel format
    WriteCommand(0x29);
    HAL_Delay(20); // Display on

    uint16_t bg_colors[6]={0x0000,0x001F,0x07E0,0xFFE0,0x07FF,0xF81F}; // black, blue, green, yellow, cyan, magenta

    while(1){
        for(int i=5;i>=0;i--){
            FillFrame(bg_colors[i]);                  // Fill background
            DrawDigitToFrame((WIDTH-8*20)/2, (HEIGHT-8*20)/2, i, 0xF800, 20);
            ILI9341_DisplayFrame();                   // Display entire frame
            HAL_Delay(500);                           // 0.5s per step
        }
    }
}

/* System Clock Configuration */
void SystemClock_Config(void){
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) Error_Handler();
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

/* SPI1 init */
static void MX_SPI1_Init(void){
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    if(HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

/* GPIO init */
static void MX_GPIO_Init(void){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitStruct.Pin = SPI_CS_PIN|SPI_DC_PIN|SPI_RESET_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

/* Error Handler */
void Error_Handler(void){
    __disable_irq();
    while(1){}
}
