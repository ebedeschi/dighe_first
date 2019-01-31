/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "opamp.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ism330dlc/ism330dlc_reg.h"
#include "SHT2x/SHT2x.h"
#include "HTS221/HTS221Sensor.h"
#include "HTS221/HTS221_Driver.h"
#include "voltage/voltage.h"
#include <stdio.h>
#include <stdarg.h>
#define PRINTF(...)     vcom_Send(__VA_ARGS__)
#define BUFSIZE 128
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis1bit16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static uint8_t whoamI, rst;
uint8_t flag;
int16_t _tout, _tout0, _tout1;

char data[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t iw;
static char buff[BUFSIZE+16];
void vcom_Send( char *format, ... )
{
  va_list args;
  va_start(args, format);

  /*convert into string at buff[0] of length iw*/
  iw = vsprintf(&buff[0], format, args);

  HAL_UART_Transmit(&huart3,(uint8_t *)&buff[0], iw, 300);

  va_end(args);
}
static int32_t platform_write(void *handle, uint8_t Reg, uint8_t *Bufp,
                              uint16_t len)
{
  if (handle == &hi2c2)
  {
    HAL_I2C_Mem_Write(handle, ISM330DLC_I2C_ADD_L, Reg,
                      I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
  return 0;
}
static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
                             uint16_t len)
{
  if (handle == &hi2c2)
  {
      HAL_I2C_Mem_Read(handle, ISM330DLC_I2C_ADD_L, Reg,
                       I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
  return 0;
}
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
  MX_CAN1_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_OPAMP1_Init();
  MX_ADC1_Init();
  MX_OPAMP2_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  HAL_OPAMP_Start(&hopamp1);

  /*##-3- Calibrate ADC then Start the conversion process ####################*/
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) !=  HAL_OK)
  {
    /* ADC Calibration Error */
    Error_Handler();
  }

  PRINTF("START\n");

  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_SET);
////  HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_RESET);
////  HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_SET);
////  HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_RESET);

//  HAL_GPIO_WritePin(EN_PWR_OUT_GPIO_Port, EN_PWR_OUT_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(EN_PWR_OUT_GPIO_Port, EN_PWR_OUT_Pin, GPIO_PIN_RESET);
////  HAL_GPIO_WritePin(EN_PWR_OUT_GPIO_Port, EN_PWR_OUT_Pin, GPIO_PIN_SET);
////  HAL_GPIO_WritePin(EN_PWR_OUT_GPIO_Port, EN_PWR_OUT_Pin, GPIO_PIN_RESET);
//
//  HAL_GPIO_WritePin(EN_RELE1_GPIO_Port, EN_RELE1_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(EN_RELE1_GPIO_Port, EN_RELE1_Pin, GPIO_PIN_SET);
//
//  HAL_GPIO_WritePin(EN_RELE2_GPIO_Port, EN_RELE2_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(EN_RELE2_GPIO_Port, EN_RELE2_Pin, GPIO_PIN_SET);
//
//  HAL_GPIO_WritePin(EN_PWR_OUT_GPIO_Port, EN_PWR_OUT_Pin, GPIO_PIN_SET);
//
////  HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_RESET);
//
//  HAL_GPIO_WritePin(EN_RELE2_GPIO_Port, EN_RELE1_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(EN_RELE2_GPIO_Port, EN_RELE1_Pin, GPIO_PIN_SET);


  HTS221Sensor(&hi2c2, HTS221_I2C_ADDRESS);
  HTS221SensorEnable();
//  HTS221SensorSetODR(1.0f);
//  HTS221SensorReset();
//  uint8_t err = 0;
//  flag = 0;
//  uint8_t id = 0;
//  err = HTS221SensorReadID(&id);
//  sprintf((char*)data, "%d %d id: %d\r\n", flag, err, id );
//  PRINTF("%s\n", data);


  ism330dlc_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c2;
  /*
   *  Check device ID
   */
  whoamI = 0;
  ism330dlc_device_id_get(&dev_ctx, &whoamI);
  /*
   *  Restore default configuration
   */
  ism330dlc_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    ism330dlc_reset_get(&dev_ctx, &rst);
  } while (rst);
  /*
   *  Enable Block Data Update
   */
  ism330dlc_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /*
   * Set Output Data Rate
   */
  ism330dlc_xl_data_rate_set(&dev_ctx, ISM330DLC_XL_ODR_12Hz5);
  ism330dlc_gy_data_rate_set(&dev_ctx, ISM330DLC_GY_ODR_12Hz5);
  /*
   * Set full scale
   */
  ism330dlc_xl_full_scale_set(&dev_ctx, ISM330DLC_2g);
  ism330dlc_gy_full_scale_set(&dev_ctx, ISM330DLC_2000dps);

  /*
   * Configure filtering chain(No aux interface)
   */
  /* Accelerometer - analog filter */
  ism330dlc_xl_filter_analog_set(&dev_ctx, ISM330DLC_XL_ANA_BW_400Hz);

  /* Accelerometer - LPF1 path ( LPF2 not used )*/
  //ism330dlc_xl_lp1_bandwidth_set(&dev_ctx, ISM330DLC_XL_LP1_ODR_DIV_4);

  /* Accelerometer - LPF1 + LPF2 path */
  ism330dlc_xl_lp2_bandwidth_set(&dev_ctx, ISM330DLC_XL_LOW_NOISE_LP_ODR_DIV_100);

  /* Accelerometer - High Pass / Slope path */
  //ism330dlc_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
  //ism330dlc_xl_hp_bandwidth_set(&dev_ctx, ISM330DLC_XL_HP_ODR_DIV_100);

  /* Gyroscope - filtering chain */
  ism330dlc_gy_band_pass_set(&dev_ctx, ISM330DLC_HP_260mHz_LP1_STRONG);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	 uint16_t s = 0;
	 uint8_t res = 0;
	 HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_SET);
	 HAL_Delay(500);
	 HAL_GPIO_WritePin(EN_PWR_OUT_GPIO_Port, EN_PWR_OUT_Pin, GPIO_PIN_RESET);
	 HAL_Delay(100);
	 res = get420(&s);
	 HAL_Delay(100);
	 if(res == 0)
	 {
		sprintf(data,"S1 ADC: %d", s);
		PRINTF("%s\n", data);
		sprintf(data,"S1 I: %f", (float)((double)s)*0.000008824359940);
		PRINTF("%s\n", data);
	 }
	 HAL_GPIO_WritePin(EN_RELE2_GPIO_Port, EN_RELE2_Pin, GPIO_PIN_RESET);
	 HAL_Delay(1000);
	 res = get420(&s);
	 HAL_Delay(100);
	 if(res == 0)
	 {
		sprintf(data,"S2 ADC: %d", s);
		PRINTF("%s\n", data);
		sprintf(data,"S2 I: %f", (float)((double)s)*0.000008824359940);
		PRINTF("%s\n", data);
	 }
	 HAL_GPIO_WritePin(EN_RELE2_GPIO_Port, EN_RELE2_Pin, GPIO_PIN_SET);
	 HAL_Delay(100);
	 HAL_GPIO_WritePin(EN_PWR_OUT_GPIO_Port, EN_PWR_OUT_Pin, GPIO_PIN_SET);
	 HAL_Delay(100);
	 HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_RESET);

//	uint16_t sT, sH;
//	float temperatureC, humidityH;           //variable for temperature[°C] as float
//	uint8_t  error = 0;              //variable for error code. For codes see system.h
//	error |= SHT2x_MeasureHM(TEMP, &sT);
//	temperatureC = SHT2x_CalcTemperatureC(sT);
//	sprintf((char*)data, "T: %6.2f\r\n", temperatureC );
//	PRINTF("%s\n", data);
//	error |= SHT2x_MeasureHM(HUMIDITY, &sH);
//	humidityH = SHT2x_CalcRH(sH);
//	sprintf((char*)data, "H: %6.2f\r\n", humidityH );
//	PRINTF("%s\n", data);

//	  uint8_t err = 0;
//	  flag = 0;
//	  uint8_t id = 0;
//	  err = HTS221SensorReadID(&id);
//	  sprintf((char*)data, "%d %d id: %d\r\n", flag, err, id );
//	  PRINTF("%s\n", data);
//	  err = 0;
//	  flag = 0;
//	  float temp = 5;
//	  err = HTS221SensorGetTemperature(&temp);
//	  sprintf((char*)data, "%d %d Temp: %6.2f\r\n", flag, err, temp );
//	  PRINTF("%s\n", data);
//	  sprintf((char*)data, "tout: %d\r\n", _tout );
//	  PRINTF("%s\n", data);
//	  sprintf((char*)data, "tout0: %d\r\n", _tout0 );
//	  PRINTF("%s\n", data);
//	  sprintf((char*)data, "tout1: %d\r\n", _tout1 );
//	  PRINTF("%s\n", data);
//	  err = 0;
//	  flag = 0;
//	  float hum = 0;
//	  err = HTS221SensorGetHumidity(&hum);
//	  sprintf((char*)data, "%d %d Hum: %6.2f\r\n", flag, err, hum );
//	  PRINTF("%s\n", data);
//	  err = 0;
//	  flag = 0;
//	  float odr = 0;
//	  err = HTS221SensorGetODR(&odr);
//	  sprintf((char*)data, "%d %d odr: %6.2f\r\n", flag, err, odr );
//	  PRINTF("%s\n", data);

	/*
	 * Read output only if new value is available
	 */
	ism330dlc_reg_t reg;
	ism330dlc_status_reg_get(&dev_ctx, &reg.status_reg);

	if (reg.status_reg.xlda)
	{
	  /* Read magnetic field data */
	  memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
	  ism330dlc_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
	  acceleration_mg[0] = ISM330DLC_FROM_FS_2g_TO_mg( data_raw_acceleration.i16bit[0]);
	  acceleration_mg[1] = ISM330DLC_FROM_FS_2g_TO_mg( data_raw_acceleration.i16bit[1]);
	  acceleration_mg[2] = ISM330DLC_FROM_FS_2g_TO_mg( data_raw_acceleration.i16bit[2]);

	  sprintf((char*)data, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
			  acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
	  PRINTF("%s\n", data);
	}
	if (reg.status_reg.gda)
	{
	  /* Read magnetic field data */
	  memset(data_raw_angular_rate.u8bit, 0x00, 3*sizeof(int16_t));
	  ism330dlc_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
	  angular_rate_mdps[0] = ISM330DLC_FROM_FS_2000dps_TO_mdps(data_raw_angular_rate.i16bit[0]);
	  angular_rate_mdps[1] = ISM330DLC_FROM_FS_2000dps_TO_mdps(data_raw_angular_rate.i16bit[1]);
	  angular_rate_mdps[2] = ISM330DLC_FROM_FS_2000dps_TO_mdps(data_raw_angular_rate.i16bit[2]);

	  sprintf((char*)data, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
			  angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
	  PRINTF("%s\n", data);
	}
	if (reg.status_reg.tda)
	{
	  /* Read temperature data */
	  memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
	  ism330dlc_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
	  temperature_degC = ISM330DLC_FROM_LSB_TO_degC( data_raw_temperature.i16bit );

	  sprintf((char*)data, "Temperature [degC]:%6.2f\r\n", temperature_degC );
	  PRINTF("%s\n", data);
	}

	  HAL_Delay(30000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 10;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
