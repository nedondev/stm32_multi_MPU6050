/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#ifdef _GNUC_
#define PUTCHAR_PROTOTYPE int __to_putchar(int ch)
	#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
	#endif
#include "string.h"
#include "math.h"
#include "usbd_cdc_if.h"
//#include "ESP-01_STM32.h"
//#include "MQTTPacket.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union 
{
  struct
  {
    unsigned char x_accel_h;
    unsigned char x_accel_l;
    unsigned char y_accel_h;
    unsigned char y_accel_l;
    unsigned char z_accel_h;
    unsigned char z_accel_l;
    unsigned char t_h;
    unsigned char t_l;
    unsigned char x_gyro_h;
    unsigned char x_gyro_l;
    unsigned char y_gyro_h;
    unsigned char y_gyro_l;
    unsigned char z_gyro_h;
    unsigned char z_gyro_l;
  } reg;
  struct 
  {
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t temperature;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
  } value;
}accel_t_gyro_union;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//for Wifi
//char Rx[20];
//char ate[] = "ATE0\r\n";
//char at[] = "AT\r\n";
//uint32_t count = 1;

//MQTTString topicString;

int sensor_list[SENSOR_NUM] = {1,2,3,5}; 
uint32_t adc_val[WEIGHT_NUM], buffer[WEIGHT_NUM];

// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
uint32_t last_read_time[8];
float         last_x_angle[8];  // These are the filtered angles
float         last_y_angle[8];
float         last_z_angle[8];  
float         last_gyro_x_angle[8];  // Store the gyro angles to compare drift
float         last_gyro_y_angle[8];
float         last_gyro_z_angle[8];

//  Use the following global variables and access functions
//  to calibrate the acceleration sensor
float    base_x_accel[8];
float    base_y_accel[8];
float    base_z_accel[8];

float    base_x_gyro[8];
float    base_y_gyro[8];
float    base_z_gyro[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//for Wifi
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	//printf("DMA: ");
	//printf("%s\n",Rx);
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
	
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
	for (int i =0; i<WEIGHT_NUM; i++)
	{
	  adc_val[i] = buffer[i];
	}
}	

int MPU6050_read(unsigned char start, unsigned char *pData, int size)
{
  int i, n, error;

	unsigned char *MPU6050write = &start;
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_I2C_ADDRESS<<1, MPU6050write, 1, 1000);
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1, MPU6050_I2C_ADDRESS<<1, pData, size, 1000);
  return (0);  // return : no error
}

int MPU6050_write(unsigned char start, unsigned char *pData, int size)
{
  unsigned char temp[5];
	//out of code limit and mpu6050 seem to have only to control with 1 byte data
	if(size > 5) return -2;
	temp[0] = start;
	int i;
	for(i = 0; i < size;i++){
		temp[i+1] = *(pData+i);
	}
	size++;
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_I2C_ADDRESS<<1, temp, size, 1000);
  return (0);         // return : no error
}

int MPU6050_write_reg(int reg, unsigned char data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}

void set_last_read_angle_data(uint32_t time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro, uint8_t index) {
  last_read_time[index] = time;
  last_x_angle[index] = x;
  last_y_angle[index] = y;
  last_z_angle[index] = z;
  last_gyro_x_angle[index] = x_gyro;
  last_gyro_y_angle[index] = y_gyro;
  last_gyro_z_angle[index] = z_gyro;
}

inline uint32_t get_last_time(uint8_t index) {return last_read_time[index];}
inline float get_last_x_angle(uint8_t index) {return last_x_angle[index];}
inline float get_last_y_angle(uint8_t index) {return last_y_angle[index];}
inline float get_last_z_angle(uint8_t index) {return last_z_angle[index];}
inline float get_last_gyro_x_angle(uint8_t index) {return last_gyro_x_angle[index];}
inline float get_last_gyro_y_angle(uint8_t index) {return last_gyro_y_angle[index];}
inline float get_last_gyro_z_angle(uint8_t index) {return last_gyro_z_angle[index];}

int read_gyro_accel_vals(unsigned char* accel_t_gyro_ptr) {  
  accel_t_gyro_union* accel_t_gyro = (accel_t_gyro_union *)accel_t_gyro_ptr;
   
  int error = MPU6050_read(MPU6050_ACCEL_XOUT_H, (unsigned char *)accel_t_gyro, sizeof(*accel_t_gyro));

  unsigned char swap;
  #define SWAP(x,y) swap = x; x = y; y = swap

  SWAP ((*accel_t_gyro).reg.x_accel_h, (*accel_t_gyro).reg.x_accel_l);
  SWAP ((*accel_t_gyro).reg.y_accel_h, (*accel_t_gyro).reg.y_accel_l);
  SWAP ((*accel_t_gyro).reg.z_accel_h, (*accel_t_gyro).reg.z_accel_l);
  SWAP ((*accel_t_gyro).reg.t_h, (*accel_t_gyro).reg.t_l);
  SWAP ((*accel_t_gyro).reg.x_gyro_h, (*accel_t_gyro).reg.x_gyro_l);
  SWAP ((*accel_t_gyro).reg.y_gyro_h, (*accel_t_gyro).reg.y_gyro_l);
  SWAP ((*accel_t_gyro).reg.z_gyro_h, (*accel_t_gyro).reg.z_gyro_l);

  return error;
}

// The sensor should be motionless on a horizontal surface 
//  while calibration is happening
void selectI2CChannels(uint8_t i) {
	if (i > 7) return;
	unsigned char temp[1];
	temp[0] = 1 << i;
	//0x70 is address
	HAL_I2C_Master_Transmit(&hi2c1, 0x70<<1, temp, 1, 100);
}
/*
//for Wifi
void SendMQTT(char* payload){
	// Variable for Packect.
	unsigned char buf[200];
	// Length of buf.
	int buflen = sizeof(buf);
	// Set Name TOPIC
	topicString.cstring ="Test";
	int payloadlen = strlen(payload);
	// Create Packet to PUBLISH!
	int lenB = MQTTSerialize_publish(buf, buflen, 0, 0, 0, 0, topicString, (unsigned char*)payload, payloadlen);
	// Send PUBLISH to...Server.
	uint8_t check1 = sendData(buf, lenB);
	// CLEAR RX
	Rx[0] = 0;
}*/

void calibrate_sensors(uint8_t index) {
	
	selectI2CChannels(index);
	int                   num_readings = 10;
	float                 x_accel = 0;
	float                 y_accel = 0;
	float                 z_accel = 0;
	float                 x_gyro = 0;
	float                 y_gyro = 0;
	float                 z_gyro = 0;
	accel_t_gyro_union    accel_t_gyro;
	unsigned char c;
	int error;
	int16_t reading;
	
	//for USB
	printf("Starting Calibration :%d\n",index);	
	
	//for Wifi
	//char payload[150];
	//sprintf(payload,"Starting Calibration :%d\n",index);
	//SendMQTT(payload);
	
	read_gyro_accel_vals((unsigned char *) &accel_t_gyro);
		
	
	for (int i = 0; i < num_readings; i++) {
		read_gyro_accel_vals((unsigned char *) &accel_t_gyro);
		x_accel += accel_t_gyro.value.x_accel;
		y_accel += accel_t_gyro.value.y_accel;
		z_accel += accel_t_gyro.value.z_accel;
		x_gyro += accel_t_gyro.value.x_gyro;
		y_gyro += accel_t_gyro.value.y_gyro;
		z_gyro += accel_t_gyro.value.z_gyro;
		HAL_Delay(100);
	}
	x_accel /= num_readings;
	y_accel /= num_readings;
	z_accel /= num_readings;
	x_gyro /= num_readings;
	y_gyro /= num_readings;
	z_gyro /= num_readings;
	
	// Store the raw calibration values globally
	base_x_accel[index] = x_accel;
	base_y_accel[index] = y_accel;
	base_z_accel[index] = z_accel;
	base_x_gyro[index] = x_gyro;
	base_y_gyro[index] = y_gyro;
	base_z_gyro[index] = z_gyro;
		
	//for USB
	printf("Finishing Calibration :%d\n", index);
	//for Wifi
	//sprintf(payload,"Finishing Calibration :%d\n", index);
	//SendMQTT(payload);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	//HAL_ADC_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, buffer, WEIGHT_NUM);
	/*
	//for Wifi
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
	// INIT ESP-01
	while(!(initESP01(&huart2))) {
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
		HAL_Delay(100);
	}
	// CLEAR RX
	Rx[0] = 0;
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
	HAL_Delay(500);
	// CONNECT TO ... WIFI!
	while(!(connectWIFI("nm","rtx2080ti"))) {
	//while(!(connectWIFI("huawei2","88888888"))) {
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
		Rx[0] = 0;
	}
	// CLEAR RX
	Rx[0] = 0;
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
		HAL_Delay(1500);
	
	
	// OPEN TCP CONNECTION!
	while(!(openTCPConnect("tailor.cloudmqtt.com", 16347))){
	//while(!(openTCPConnect("192.168.137.1", 1883))){
		
		if(!waitForString("ALREAY CONNECT",10,5000))
			break;
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
		Rx[0] = 0;
	}
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
		HAL_Delay(1000);
	// CLEAR RX
	Rx[0] = 0;
	
	//MAKE CONNECT!
	
	// Init Struct to Create Packect. 
	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	// Variable for Packect.
	unsigned char buf[200];
	// Length of buf.
	int buflen = sizeof(buf);
	// Don't know!
	int msgid = 1;
	int req_qos = 0;
	// Struct for Name of Toppic!
	MQTTString temp_topicString = MQTTString_initializer;
	topicString = temp_topicString;
	// Text For Publish
	char* payload = "HELLO N8NNY!!!!";
	int payloadlen = strlen(payload);
	// Text For Publish
	
	// Byte of Packet. 
	uint16_t lenB = 0;
	// SET INSTRUCTION CONNECT PACKET
	data.clientID.cstring = "me";
	data.keepAliveInterval = 20;
	data.cleansession = 1;
	data.username.cstring = "tizxinbp";
	data.password.cstring = "u813eLQKHMrn";
	// Create Packet to Connect...Server.
	lenB = MQTTSerialize_connect(buf, buflen, &data);
	// Send CONNECT Packet to...Server.
	uint8_t check = sendData(buf, lenB);
	Rx[0] = 0;

	// Set Name TOPIC
	topicString.cstring ="Test";
	payloadlen = strlen(payload);
	// Create Packet to PUBLISH!
	lenB = MQTTSerialize_publish(buf, buflen, 0, 0, 0, 0, topicString, (unsigned char*)payload, payloadlen);
	// Send PUBLISH to...Server.
	uint8_t check1 = sendData(buf, lenB);
	// CLEAR RX
	Rx[0] = 0;
	*/
	unsigned char MPU6050write[1];
	unsigned char MPU6050read[1];
	int16_t reading;
	//UART_string("Start communicate", &huart2);
	//printf("Start communicate\n");
	
	//setup
	for(int j =0; j< SENSOR_NUM;j++){
		selectI2CChannels(sensor_list[j]);
		unsigned char c;
		int error;
		//int error = MPU6050_read(MPU6050_WHO_AM_I, &c, 1);
		//HAL_Delay(10);
		//reading = c;
		
		// According to the datasheet, the 'sleep' bit
		// should read a '1'. But I read a '0'.
		// That bit has to be cleared, since the sensor
		// is in sleep mode at power-up. Even if the
		// bit reads '0'.
		error = MPU6050_read(MPU6050_PWR_MGMT_2, &c, 1);
		HAL_Delay(10);
		reading = c;
		//printf("PWR_MGMT_2: %x\n", reading);

		// Clear the 'sleep' bit to start the sensor.
		MPU6050_write_reg(MPU6050_PWR_MGMT_1, 0);
		HAL_Delay(10);
		
		//Initialize the angles
		calibrate_sensors(sensor_list[j]);  
		set_last_read_angle_data(HAL_GetTick(), 0, 0, 0, 0, 0, 0, sensor_list[j]);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		for(uint8_t index = 0; index < SENSOR_NUM; index++){
			selectI2CChannels(sensor_list[index]);
			int error;
			//for(uint8_t weight_index = 0; weight_index < WEIGHT_NUM;weight_index++){
			//	while(HAL_ADC_PollForConversion(&hadc1,100) != HAL_OK){}
			//	adc_val[weight_index]= HAL_ADC_GetValue(&hadc1);
			//}
			accel_t_gyro_union accel_t_gyro;
			
			// Read the raw values.
			error = read_gyro_accel_vals((unsigned char*) &accel_t_gyro);
			
			// Get the time of reading for rotation computations
			uint32_t t_now = HAL_GetTick();
		 
			// Convert gyro values to degrees/sec
			float FS_SEL = 131;
			
			float gyro_x = (accel_t_gyro.value.x_gyro - base_x_gyro[index])/FS_SEL;
			float gyro_y = (accel_t_gyro.value.y_gyro - base_y_gyro[index])/FS_SEL;
			float gyro_z = (accel_t_gyro.value.z_gyro - base_z_gyro[index])/FS_SEL;
		
		
			// Get raw acceleration values
			//float G_CONVERT = 16384;
			float accel_x = accel_t_gyro.value.x_accel;
			float accel_y = accel_t_gyro.value.y_accel;
			float accel_z = accel_t_gyro.value.z_accel;

			// Get angle values from accelerometer
			float RADIANS_TO_DEGREES = 180/3.14159;
			//  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
			float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
			float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
			float accel_angle_z = 0;
		
			// Compute the (filtered) gyro angles
			float dt =(t_now - get_last_time(index))/1000.0;
			float gyro_angle_x = gyro_x*dt + get_last_x_angle(index);
			float gyro_angle_y = gyro_y*dt + get_last_y_angle(index);
			float gyro_angle_z = gyro_z*dt + get_last_z_angle(index);
		
			// Compute the drifting gyro angles
			float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle(index);
			float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle(index);
			float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle(index);
		
			// Apply the complementary filter to figure out the change in angle - choice of alpha is
			// estimated now.  Alpha depends on the sampling rate...
			float alpha = 0.96;
			float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
			float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
			float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle
		
			// Update the saved data with the latest values
			set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z, index);

			
			//printf("Index:%d",index);
			//printf("#DEL:%.2f",dt);
			//printf("#FIL:%.2f",angle_x);
			//printf(",%.2f",angle_y);
			//printf(",%.2f\n",angle_z);
			printf("Index:%d#DEL:%.2f#FIL:%.2f,%.2f,%.2f\n", index, dt, angle_x, angle_y, angle_z);
			//for USB
			printf("ADC:%d\n", adc_val[0]);
			//for Wifi
			//char payload[150];
			//sprintf(payload,"Index:%d#DEL:%.2f#FIL:%.2f,%.2f,%.2f", index, dt, angle_x, angle_y, angle_z);
			//sprintf(payload,"Index:%d#DEL:%.2f#FIL:%.2f,%.2f,%.2f", index, dt, accel_angle_x, accel_angle_y, accel_angle_z);
			//SendMQTT(payload);
			/*
			for(int adc_index = 0; adc_index < WEIGHT_NUM; adc_index++){
				sprintf(payload,"Index:%d#ADC:%d",index, adc_val[adc_index]);
			}
			SendMQTT(payload);
			*/
		}
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
PUTCHAR_PROTOTYPE
{
	while(!(CDC_Transmit_FS((uint8_t *)&ch, 1)) == USBD_OK);
	return ch;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
