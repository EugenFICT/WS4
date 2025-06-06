/* STM32 PROGRAM HEADER
 *
 * ===============================================================
 * Project: LED Control Application
 * Description: Basic LED control through UART interface
 * MCU: STM32F4 series
 * ===============================================================
 *
 * Modified: May 2025
 * License: Proprietary - All rights reserved
 *
 * ===============================================================
 */

#include "main.h"

/* Configuration Definitions */
/* ----------------------------- */
#define LED_GREEN_ID    GPIO_PIN_12
#define LED_AMBER_ID    GPIO_PIN_13
#define LED_CRIMSON_ID  GPIO_PIN_14
#define LED_COBALT_ID   GPIO_PIN_15

/* Private Variables */
/* ----------------------------- */
UART_HandleTypeDef huart3; /* Must use this exact name for compatibility */

/* Function Prototypes */
/* ----------------------------- */
void ConfigureSystemClock(void);
static void SetupGPIOPins(void);
static void InitializeUART(void);
static void ProcessUARTCommand(uint8_t cmd);
static void SendStatusMessage(const char* message, uint16_t length);

/**
 * @brief Application Entry Point
 */
int main(void)
{
  /* Hardware initialization sequence */
  HAL_Init();
  ConfigureSystemClock();
  SetupGPIOPins();
  InitializeUART();

  /* Main application loop */
  while (1)
  {
    uint8_t command_byte;
    HAL_StatusTypeDef rx_status;

    /* Check for incoming UART data with short timeout */
    rx_status = HAL_UART_Receive(&huart3, &command_byte, 1, 10);

    /* Process valid received commands */
    if (rx_status == HAL_OK) {
      ProcessUARTCommand(command_byte);
    }
  }
}

/**
 * @brief Process received UART command and control LEDs accordingly
 * @param cmd The command byte received from UART
 */
static void ProcessUARTCommand(uint8_t cmd)
{
  /* Remove unused variables */
  switch (cmd)
  {
    case '1': /* Toggle Green LED */
      HAL_GPIO_TogglePin(GPIOD, LED_GREEN_ID);
      if (HAL_GPIO_ReadPin(GPIOD, LED_GREEN_ID)) {
        SendStatusMessage(" > Green LED: ON\r\n", 17);
      } else {
        SendStatusMessage(" > Green LED: OFF\r\n", 18);
      }
      break;

    case '2': /* Toggle Amber LED */
      HAL_GPIO_TogglePin(GPIOD, LED_AMBER_ID);
      if (HAL_GPIO_ReadPin(GPIOD, LED_AMBER_ID)) {
        SendStatusMessage(" > Amber LED: ON\r\n", 17);
      } else {
        SendStatusMessage(" > Amber LED: OFF\r\n", 18);
      }
      break;

    case '3': /* Toggle Crimson LED */
      HAL_GPIO_TogglePin(GPIOD, LED_CRIMSON_ID);
      if (HAL_GPIO_ReadPin(GPIOD, LED_CRIMSON_ID)) {
        SendStatusMessage(" > Crimson LED: ON\r\n", 19);
      } else {
        SendStatusMessage(" > Crimson LED: OFF\r\n", 20);
      }
      break;

    case '4': /* Toggle Cobalt LED */
      HAL_GPIO_TogglePin(GPIOD, LED_COBALT_ID);
      if (HAL_GPIO_ReadPin(GPIOD, LED_COBALT_ID)) {
        SendStatusMessage(" > Cobalt LED: ON\r\n", 18);
      } else {
        SendStatusMessage(" > Cobalt LED: OFF\r\n", 19);
      }
      break;

    case '5': /* Turn ON all LEDs */
      HAL_GPIO_WritePin(GPIOD,
                       LED_GREEN_ID | LED_AMBER_ID | LED_CRIMSON_ID | LED_COBALT_ID,
                       GPIO_PIN_SET);
      SendStatusMessage(" > All LEDs enabled\r\n", 20);
      break;

    case '6': /* Turn OFF all LEDs */
      HAL_GPIO_WritePin(GPIOD,
                       LED_GREEN_ID | LED_AMBER_ID | LED_CRIMSON_ID | LED_COBALT_ID,
                       GPIO_PIN_RESET);
      SendStatusMessage(" > All LEDs disabled\r\n", 21);
      break;

    default: /* Invalid command */
      SendStatusMessage(" > Invalid command\r\n", 19);
      break;
  }
}

/**
 * @brief Send status message over UART
 * @param message Pointer to message string
 * @param length Length of message
 */
static void SendStatusMessage(const char* message, uint16_t length)
{
  HAL_UART_Transmit(&huart3, (uint8_t*)message, length, 100);
}

/**
 * @brief Configure system clock
 */
void ConfigureSystemClock(void)
{
  RCC_OscInitTypeDef oscConfig = {0};
  RCC_ClkInitTypeDef clkConfig = {0};

  /* Enable power controller clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Configure HSI as clock source */
  oscConfig.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  oscConfig.HSIState = RCC_HSI_ON;
  oscConfig.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscConfig.PLL.PLLState = RCC_PLL_NONE;

  if (HAL_RCC_OscConfig(&oscConfig) != HAL_OK) {
    Error_Handler();
  }

  /* Configure clock buses */
  clkConfig.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clkConfig.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  clkConfig.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkConfig.APB1CLKDivider = RCC_HCLK_DIV1;
  clkConfig.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&clkConfig, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Initialize UART peripheral
 */
static void InitializeUART(void)
{
  /* Configure UART */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Initialize GPIO pins for LEDs
 */
static void SetupGPIOPins(void)
{
  GPIO_InitTypeDef gpio_config = {0};

  /* Enable clock for GPIOD */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Set all LEDs to initially OFF */
  HAL_GPIO_WritePin(GPIOD,
                   LED_GREEN_ID | LED_AMBER_ID | LED_CRIMSON_ID | LED_COBALT_ID,
                   GPIO_PIN_RESET);

  /* Configure LED pins as outputs */
  gpio_config.Pin = LED_GREEN_ID | LED_AMBER_ID | LED_CRIMSON_ID | LED_COBALT_ID;
  gpio_config.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_config.Pull = GPIO_NOPULL;
  gpio_config.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOD, &gpio_config);
}

/**
 * @brief Error handler function
 */
void Error_Handler(void)
{
  /* Disable interrupts */
  __disable_irq();

  /* Infinite loop on error */
  while (1) {
    /* System halted */
  }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief Assert failed callback
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* For debug purposes - not used in production */
  /* Implementation could log the assert info */
}
#endif /* USE_FULL_ASSERT */
