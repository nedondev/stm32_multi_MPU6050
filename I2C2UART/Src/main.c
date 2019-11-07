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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "math.h"
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
#define MPU6050_AUX_VDDIO          0x01   // R/W
#define MPU6050_SMPLRT_DIV         0x19   // R/W
#define MPU6050_CONFIG             0x1A   // R/W
#define MPU6050_GYRO_CONFIG        0x1B   // R/W
#define MPU6050_ACCEL_CONFIG       0x1C   // R/W
#define MPU6050_FF_THR             0x1D   // R/W
#define MPU6050_FF_DUR             0x1E   // R/W
#define MPU6050_MOT_THR            0x1F   // R/W
#define MPU6050_MOT_DUR            0x20   // R/W
#define MPU6050_ZRMOT_THR          0x21   // R/W
#define MPU6050_ZRMOT_DUR          0x22   // R/W
#define MPU6050_FIFO_EN            0x23   // R/W
#define MPU6050_I2C_MST_CTRL       0x24   // R/W
#define MPU6050_I2C_SLV0_ADDR      0x25   // R/W
#define MPU6050_I2C_SLV0_REG       0x26   // R/W
#define MPU6050_I2C_SLV0_CTRL      0x27   // R/W
#define MPU6050_I2C_SLV1_ADDR      0x28   // R/W
#define MPU6050_I2C_SLV1_REG       0x29   // R/W
#define MPU6050_I2C_SLV1_CTRL      0x2A   // R/W
#define MPU6050_I2C_SLV2_ADDR      0x2B   // R/W
#define MPU6050_I2C_SLV2_REG       0x2C   // R/W
#define MPU6050_I2C_SLV2_CTRL      0x2D   // R/W
#define MPU6050_I2C_SLV3_ADDR      0x2E   // R/W
#define MPU6050_I2C_SLV3_REG       0x2F   // R/W
#define MPU6050_I2C_SLV3_CTRL      0x30   // R/W
#define MPU6050_I2C_SLV4_ADDR      0x31   // R/W
#define MPU6050_I2C_SLV4_REG       0x32   // R/W
#define MPU6050_I2C_SLV4_DO        0x33   // R/W
#define MPU6050_I2C_SLV4_CTRL      0x34   // R/W
#define MPU6050_I2C_SLV4_DI        0x35   // R  
#define MPU6050_I2C_MST_STATUS     0x36   // R
#define MPU6050_INT_PIN_CFG        0x37   // R/W
#define MPU6050_INT_ENABLE         0x38   // R/W
#define MPU6050_INT_STATUS         0x3A   // R  
#define MPU6050_ACCEL_XOUT_H       0x3B   // R  
#define MPU6050_ACCEL_XOUT_L       0x3C   // R  
#define MPU6050_ACCEL_YOUT_H       0x3D   // R  
#define MPU6050_ACCEL_YOUT_L       0x3E   // R  
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R  
#define MPU6050_ACCEL_ZOUT_L       0x40   // R  
#define MPU6050_TEMP_OUT_H         0x41   // R  
#define MPU6050_TEMP_OUT_L         0x42   // R  
#define MPU6050_GYRO_XOUT_H        0x43   // R  
#define MPU6050_GYRO_XOUT_L        0x44   // R  
#define MPU6050_GYRO_YOUT_H        0x45   // R  
#define MPU6050_GYRO_YOUT_L        0x46   // R  
#define MPU6050_GYRO_ZOUT_H        0x47   // R  
#define MPU6050_GYRO_ZOUT_L        0x48   // R  
#define MPU6050_EXT_SENS_DATA_00   0x49   // R  
#define MPU6050_EXT_SENS_DATA_01   0x4A   // R  
#define MPU6050_EXT_SENS_DATA_02   0x4B   // R  
#define MPU6050_EXT_SENS_DATA_03   0x4C   // R  
#define MPU6050_EXT_SENS_DATA_04   0x4D   // R  
#define MPU6050_EXT_SENS_DATA_05   0x4E   // R  
#define MPU6050_EXT_SENS_DATA_06   0x4F   // R  
#define MPU6050_EXT_SENS_DATA_07   0x50   // R  
#define MPU6050_EXT_SENS_DATA_08   0x51   // R  
#define MPU6050_EXT_SENS_DATA_09   0x52   // R  
#define MPU6050_EXT_SENS_DATA_10   0x53   // R  
#define MPU6050_EXT_SENS_DATA_11   0x54   // R  
#define MPU6050_EXT_SENS_DATA_12   0x55   // R  
#define MPU6050_EXT_SENS_DATA_13   0x56   // R  
#define MPU6050_EXT_SENS_DATA_14   0x57   // R  
#define MPU6050_EXT_SENS_DATA_15   0x58   // R  
#define MPU6050_EXT_SENS_DATA_16   0x59   // R  
#define MPU6050_EXT_SENS_DATA_17   0x5A   // R  
#define MPU6050_EXT_SENS_DATA_18   0x5B   // R  
#define MPU6050_EXT_SENS_DATA_19   0x5C   // R  
#define MPU6050_EXT_SENS_DATA_20   0x5D   // R  
#define MPU6050_EXT_SENS_DATA_21   0x5E   // R  
#define MPU6050_EXT_SENS_DATA_22   0x5F   // R  
#define MPU6050_EXT_SENS_DATA_23   0x60   // R  
#define MPU6050_MOT_DETECT_STATUS  0x61   // R  
#define MPU6050_I2C_SLV0_DO        0x63   // R/W
#define MPU6050_I2C_SLV1_DO        0x64   // R/W
#define MPU6050_I2C_SLV2_DO        0x65   // R/W
#define MPU6050_I2C_SLV3_DO        0x66   // R/W
#define MPU6050_I2C_MST_DELAY_CTRL 0x67   // R/W
#define MPU6050_SIGNAL_PATH_RESET  0x68   // R/W
#define MPU6050_MOT_DETECT_CTRL    0x69   // R/W
#define MPU6050_USER_CTRL          0x6A   // R/W
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_PWR_MGMT_2         0x6C   // R/W
#define MPU6050_FIFO_COUNTH        0x72   // R/W
#define MPU6050_FIFO_COUNTL        0x73   // R/W
#define MPU6050_FIFO_R_W           0x74   // R/W
#define MPU6050_WHO_AM_I           0x75   // R

#define MPU6050_I2C_ADDRESS 0x68
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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

//Show to UART
void UART_raw(unsigned char *value, int size, UART_HandleTypeDef *huart){//, char *end){
	unsigned char *plt_value = value;
	int i = 0;
	while(i < size){
		char buffer[10];
		sprintf(buffer, "%2.2x, ", *(plt_value+i));
		while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC)==RESET){}
		HAL_UART_Transmit(huart, (uint8_t*) buffer, strlen(buffer),1000);
		i++;
	}
}

void UART_hex(int16_t hex, UART_HandleTypeDef *huart){//, char *end){
	char buffer[50];
	sprintf(buffer, "%4.4x, ", hex);
	//if(strlen(end) != 0){
	//	int i;
	//	for(i = strlen(buffer); i < strlen(buffer) + strlen(end); i++)
	//		buffer[i] = end[i - (strlen(buffer))];
	//	buffer[i+1] = '\0';
	//}
	//sprintf(buffer, "%4.4x\n", strlen(buffer));
	while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC)==RESET){}
	HAL_UART_Transmit(huart, (uint8_t*) buffer, strlen(buffer),1000);
}

void UART_template(const char *template_uart, int16_t value, UART_HandleTypeDef *huart){//, char *end){
	char buffer[50];
	sprintf(buffer, template_uart, value);
	while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC)==RESET){}
	HAL_UART_Transmit(huart, (uint8_t*) buffer, strlen(buffer),1000);
}

void UART_float(const char *template_uart, float value, UART_HandleTypeDef *huart){//, char *end){
	char buffer[50];
	sprintf(buffer, template_uart, value);
	while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC)==RESET){}
	HAL_UART_Transmit(huart, (uint8_t*) buffer, strlen(buffer),1000);
}

void UART_string(char *string, UART_HandleTypeDef *huart){
	while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC)==RESET){}
	HAL_UART_Transmit(huart, (uint8_t*) string, strlen(string),1000);
}

// --------------------------------------------------------
// MPU6050_read
//
// This is a common function to read multiple bytes 
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus. 
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read. 
// There is no function for a single byte.
//
int MPU6050_read(unsigned char start, unsigned char *pData, int size)
{
  int i, n, error;

  //Begin a transmission to the I2C slave device with the given address. 
  // Subsequently, queue bytes for transmission with the write() function and transmit them by calling endTransmission().
	unsigned char *MPU6050write = &start;
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_I2C_ADDRESS<<1, MPU6050write, 1, 100);
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1, MPU6050_I2C_ADDRESS<<1, pData, size, 100);
	//Test read raw data
	//UART_string("rawData:", &huart2);
	//UART_raw(pData, size, &huart2);
	//UART_string("\r\n", &huart2);
	//UART_string("\n", &huart2);
  return (0);  // return : no error
}


// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);

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
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_I2C_ADDRESS<<1, temp, size, 100);
  return (0);         // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//

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
  // Read the raw values.
  // Read 14 bytes at once, 
  // containing acceleration, temperature and gyro.
  // With the default settings of the MPU-6050,
  // there is no filter enabled, and the values
  // are not very stable.  Returns the error value
  
  accel_t_gyro_union* accel_t_gyro = (accel_t_gyro_union *)accel_t_gyro_ptr;
   
  int error = MPU6050_read(MPU6050_ACCEL_XOUT_H, (unsigned char *)accel_t_gyro, sizeof(*accel_t_gyro));

  // Swap all high and low bytes.
  // After this, the registers values are swapped, 
  // so the structure name like x_accel_l does no 
  // longer contain the lower byte.
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
		
	//Serial.println("Starting Calibration");
	UART_string("Starting Calibration : ", &huart2);
	UART_template("%d", index, &huart2);
	//UART_string("\r\n", &huart2);
	UART_string("\n", &huart2);
		
	// Discard the first set of values read from the IMU
	read_gyro_accel_vals((unsigned char *) &accel_t_gyro);
		
	// Read and average the raw values from the IMU
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
		
	//Serial.println("Finishing Calibration");
	UART_string("Finishing Calibration :", &huart2);
	UART_template("%d", index, &huart2);
	//UART_string("\r\n", &huart2);
	UART_string("\n", &huart2);
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	unsigned char MPU6050write[1];
	unsigned char MPU6050read[1];
	int16_t reading;
	UART_string("Start communicate", &huart2);
	
	//setup
	for(int j =0; j< 6;j++){
		selectI2CChannels(j);
		unsigned char c;
		int error = MPU6050_read(MPU6050_WHO_AM_I, &c, 1);
		HAL_Delay(10);
		reading = c;
		UART_string("WHO_AM_I: ", &huart2);
		UART_hex(reading, &huart2);
		//UART_string("\r\n", &huart2);
		UART_string("\n", &huart2);
		
		// According to the datasheet, the 'sleep' bit
		// should read a '1'. But I read a '0'.
		// That bit has to be cleared, since the sensor
		// is in sleep mode at power-up. Even if the
		// bit reads '0'.
		error = MPU6050_read(MPU6050_PWR_MGMT_2, &c, 1);
		HAL_Delay(10);
		reading = c;
		UART_string("PWR_MGMT_2: ", &huart2);
		UART_hex(reading = c, &huart2);
		//UART_string("8", &huart2);
		UART_string("\n", &huart2);

		// Clear the 'sleep' bit to start the sensor.
		MPU6050_write_reg(MPU6050_PWR_MGMT_1, 0);
		HAL_Delay(10);
		
		//Initialize the angles
		calibrate_sensors(j);  
		set_last_read_angle_data(HAL_GetTick(), 0, 0, 0, 0, 0, 0, j);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		for(uint8_t index = 0; index < 6; index++){
			selectI2CChannels(index);
			int error;
			accel_t_gyro_union accel_t_gyro;
			
			// Read the raw values.
			error = read_gyro_accel_vals((unsigned char*) &accel_t_gyro);
		
			// Get the time of reading for rotation computations
			uint32_t t_now = HAL_GetTick();
		 
			/*
			Serial.print(F("Read accel, temp and gyro, error = "));
			Serial.println(error,DEC);
			*/
			//UART_string("Read accel, temp and gyro, error = ", &huart2);
			//UART_template("%d\r\n", error, &huart2);
			/*
			// Print the raw acceleration values
			Serial.print(F("accel x,y,z: "));
			Serial.print(accel_t_gyro.value.x_accel, DEC);
			Serial.print(F(", "));
			Serial.print(accel_t_gyro.value.y_accel, DEC);
			Serial.print(F(", "));
			Serial.print(accel_t_gyro.value.z_accel, DEC);
			Serial.println(F(""));
			*/
			
			/*
			UART_string("accel x,y,z: ", &huart2);
			UART_template("%d", accel_t_gyro.value.x_accel, &huart2);
			UART_string(", ", &huart2);
			UART_template("%d", accel_t_gyro.value.y_accel, &huart2);
			UART_string(", ", &huart2);
			UART_template("%d\r\n", accel_t_gyro.value.z_accel, &huart2);
			*/
			
			// The temperature sensor is -40 to +85 degrees Celsius.
			// It is a signed integer.
			// According to the datasheet: 
			//   340 per degrees Celsius, -512 at 35 degrees.
			// At 0 degrees: -512 - (340 * 35) = -12412
			/*  
			Serial.print(F("temperature: "));
			dT = ( (double) accel_t_gyro.value.temperature + 12412.0) / 340.0;
			Serial.print(dT, 3);
			Serial.print(F(" degrees Celsius"));
			Serial.println(F(""));
		

			// Print the raw gyro values.
			Serial.print(F("raw gyro x,y,z : "));
			Serial.print(accel_t_gyro.value.x_gyro, DEC);
			Serial.print(F(", "));
			Serial.print(accel_t_gyro.value.y_gyro, DEC);
			Serial.print(F(", "));
			Serial.print(accel_t_gyro.value.z_gyro, DEC);
			Serial.print(F(", "));
			Serial.println(F(""));
			*/

			// Convert gyro values to degrees/sec
			float FS_SEL = 131;
			/*
			float gyro_x = (accel_t_gyro.value.x_gyro - base_x_gyro)/FS_SEL;
			float gyro_y = (accel_t_gyro.value.y_gyro - base_y_gyro)/FS_SEL;
			float gyro_z = (accel_t_gyro.value.z_gyro - base_z_gyro)/FS_SEL;
			*/
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

			// Send the data to the serial port
			UART_string("Index:", &huart2);
			UART_template("%d", index, &huart2);
			//Serial.print(F("DEL:"));              //Delta T
			UART_string("#DEL:", &huart2);
			//Serial.print(dt, DEC);
			UART_float("%.2f", dt, &huart2);
			//Serial.print(F("#ACC:"));              //Accelerometer angle
			UART_string("#ACC:", &huart2);
			//Serial.print(accel_angle_x, 2);
			UART_float("%.2f", accel_angle_x, &huart2);
			//Serial.print(F(","));
			UART_string(",", &huart2);
			//Serial.print(accel_angle_y, 2);
			UART_float("%.2f", accel_angle_y, &huart2);
			//Serial.print(F(","));
			UART_string(",", &huart2);
			//Serial.print(accel_angle_z, 2);
			UART_float("%.2f", accel_angle_z, &huart2);
			//Serial.print(F("#GYR:"));
			UART_string("#GYR:", &huart2);
			//Serial.print(unfiltered_gyro_angle_x, 2);        //Gyroscope angle
			UART_float("%.2f", unfiltered_gyro_angle_x, &huart2);
			//Serial.print(F(","));
			UART_string(",", &huart2);
			//Serial.print(unfiltered_gyro_angle_y, 2);
			UART_float("%.2f", unfiltered_gyro_angle_y, &huart2);
			//Serial.print(F(","));
			UART_string(",", &huart2);
			//Serial.print(unfiltered_gyro_angle_z, 2);
			UART_float("%.2f", unfiltered_gyro_angle_z, &huart2);
			//Serial.print(F("#FIL:"));             //Filtered angle
			UART_string("#FIL:", &huart2);
			//Serial.print(angle_x, 2);
			UART_float("%.2f", angle_x, &huart2);
			//Serial.print(F(","));
			UART_string(",", &huart2);
			//Serial.print(angle_y, 2);
			UART_float("%.2f", angle_y, &huart2);
			//Serial.print(F(","));
			UART_string(",", &huart2);
			//Serial.print(angle_z, 2);
			UART_float("%.2f", angle_z, &huart2);
			//Serial.println(F(""));
			//UART_string("\r\n", &huart2);
			UART_string("\n", &huart2);
			// Delay so we don't swamp the serial port
			//HAL_Delay(5);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
