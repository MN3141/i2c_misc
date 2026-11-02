/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* HAL takes 8-bit (shifted) addresses, both for targets and for OwnAddress1. */
#define DGT_CLOCK_ADDR (0x08 << 1)
#define DGT_OWN_ADDR (0x10 << 1) /* Must match hi2c2.Init.OwnAddress1 */

#define DGT_MSG_ACK 0x01
#define DGT_CMD_CHANGE_STATE 0x0B
#define DGT_CMD_SET_CC 0x0F
#define DGT_ACK_POSITIVE 0x08

#define DGT_TX_TIMEOUT_MS 10
#define DGT_ACK_TIMEOUT_MS 50
#define DGT_CONFIGURE_ATTEMPTS 6

/* Same meaning as the dgtpi return codes */
#define DGT_OK 0
#define DGT_ERR_NEGATIVE_ACK -1 /* Clock answered, but refused the command */
#define DGT_ERR_I2C -2          /* Bus stuck or timed out */
#define DGT_ERR_RETRY -3        /* Collision, or no ack received */
#define DGT_ERR_CLOCK_OFF -7    /* Address not acked, clock probably off */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
/* A frame's length is a single byte, so this can never overflow. */
static uint8_t dgt_rx_frame[256];
static volatile uint8_t dgt_ack_cmd;
static volatile uint8_t dgt_ack_status;

const uint8_t crc_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
/* The CRC also covers the destination address byte, which is not part of the
   frame buffers: the HAL puts it on the wire. */
static uint8_t DGT_Crc(uint8_t addr_byte, const uint8_t *frame, uint16_t length)
{
  uint8_t crc = crc_table[addr_byte];

  for (uint16_t i = 0; i < length; i++)
    crc = crc_table[crc ^ frame[i]];

  return crc;
}

void Wake_DGT3000(void)
{

  uint8_t ping_frame[] = {32, 5, 13, 70};
  uint8_t timeout = 100;
  uint8_t clock_addr = (uint8_t)40 << 1;
  volatile HAL_StatusTypeDef tx_status = HAL_OK;

  for (int i = 0; i < 9; i++)
  {
    tx_status = HAL_I2C_Master_Transmit(&hi2c1, clock_addr, ping_frame, sizeof(ping_frame), timeout);
    HAL_Delay(1000);
  }
}

/* Send a frame to the clock and wait for the ack of its command, which the
   clock sends to our slave address. */
static int DGT_SendCommand(uint8_t *frame, uint16_t length, uint8_t *ack_status)
{
  uint8_t cmd = frame[2];

  /* Cleared before sending: the ack can arrive right after the stop condition */
  dgt_ack_cmd = 0;

  if (HAL_I2C_Master_Transmit(&hi2c1, DGT_CLOCK_ADDR, frame, length, DGT_TX_TIMEOUT_MS) != HAL_OK)
  {
    /* HAL only checks ARLO in IT mode, here it shows up as a timeout */
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_ARLO))
    {
      __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_ARLO);
      return DGT_ERR_RETRY;
    }
    if (HAL_I2C_GetError(&hi2c1) & HAL_I2C_ERROR_AF)
      return DGT_ERR_CLOCK_OFF;
    return DGT_ERR_I2C;
  }

  uint32_t start = HAL_GetTick();
  while (dgt_ack_cmd != cmd)
  {
    if (HAL_GetTick() - start >= DGT_ACK_TIMEOUT_MS)
      return DGT_ERR_RETRY;
  }

  *ack_status = dgt_ack_status;
  return DGT_OK;
}

/* Returns DGT_ERR_NEGATIVE_ACK if the clock is not in central control. */
int SetMode25_DGT3000(void)
{
  uint8_t mode25_frame[] = {0x20, 0x06, DGT_CMD_CHANGE_STATE, 0x39, 0xB9};
  uint8_t status = 0;

  int e = DGT_SendCommand(mode25_frame, sizeof(mode25_frame), &status);
  if (e != DGT_OK)
    return e;

  return (status == DGT_ACK_POSITIVE) ? DGT_OK : DGT_ERR_NEGATIVE_ACK;
}

/* Returns DGT_ERR_NEGATIVE_ACK if the clock refuses, e.g. because it is running. */
int SetCCMode_DGT3000(void)
{
  uint8_t central_control_frame[] = {0x20, 0x05, DGT_CMD_SET_CC, 0x48};
  uint8_t status = 0;

  int e = DGT_SendCommand(central_control_frame, sizeof(central_control_frame), &status);
  if (e != DGT_OK)
    return e;

  return (status & DGT_ACK_POSITIVE) ? DGT_OK : DGT_ERR_NEGATIVE_ACK;
}

/* Get the clock into mode 25, as dgt3000Configure() does: mode 25 is tried
   first, central control is only needed when the clock refuses it. */
int Configure_DGT3000(void)
{
  int e = DGT_ERR_I2C;

  for (int i = 0; i < DGT_CONFIGURE_ATTEMPTS; i++)
  {
    e = SetMode25_DGT3000();
    if (e == DGT_OK)
      return DGT_OK;

    if (e == DGT_ERR_NEGATIVE_ACK)
      SetCCMode_DGT3000();
    else if (e == DGT_ERR_CLOCK_OFF)
      Wake_DGT3000();
  }

  return e;
}
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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  /* Listen before sending anything, the clock answers as soon as a frame ends */
  if (HAL_I2C_EnableListen_IT(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  Wake_DGT3000();
  volatile int status = Configure_DGT3000();
  (void)status;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 32;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  UNUSED(AddrMatchCode);

  if (hi2c == &hi2c2 && TransferDirection == I2C_DIRECTION_TRANSMIT)
  {
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, dgt_rx_frame, sizeof(dgt_rx_frame), I2C_LAST_FRAME);
  }
}

/* The frame length is not known up front, so the receive buffer is never filled
   and the HAL ends each frame at the stop condition, reporting it as an
   incomplete transfer. That also ends the listen mode, so restart it here. */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c != &hi2c2)
    return;

  /* Ack frame: source, length (counts the address byte), type, command, status,
     ..., crc. Broadcasts have another CRC seed, so they are dropped here. */
  uint16_t received = hi2c->XferSize - hi2c->XferCount;
  if (received >= 6 && dgt_rx_frame[1] == received + 1 && dgt_rx_frame[2] == DGT_MSG_ACK &&
      DGT_Crc(DGT_OWN_ADDR, dgt_rx_frame, received - 1) == dgt_rx_frame[received - 1])
  {
    dgt_ack_status = dgt_rx_frame[4];
    dgt_ack_cmd = dgt_rx_frame[3];
  }

  HAL_I2C_EnableListen_IT(hi2c);
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
