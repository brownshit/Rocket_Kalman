/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx.h"
#include "BMP180.h"
#include "stdio.h"
#include "MPU6050.h"
#include "math.h"
#include "stdint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD_TO_DEG 57.29578
#define DEG_TO_RAD 0.017453
#define ALPHA 0.9996                          // Complementary Filter alpha value
//#define dt 0.00078                            // check while loop time

//#define iter_temp		10
#define iter			1000
#define row_1			1
#define col_1			1
#define row_2			2
#define col_2			2
//#define row_3			3
//#define col_3			3
#define row_10			10
#define add				1
#define substract		0
#define math_pi			3.1415926535
#define gravity_const	9.81
#define dt				0.1
//user define value... should be modified... by measurememt.
//do not Dynamic allocation in KF algorithms... user heap is extra small

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float temp, press, altitude;                 	// Define User Variables Here!
int second = 0;                               	// time out variance
int Parachute = 0;                       		// state of parachute 0 is not deployed
int start = 0;                               	// state of time out func

//===========================================================================================================================
//define our matrices as global array.

float noise[] = {0.124};		//noise value could be replaced into new value.
static float values_1[] = { 1.0, 0.0 };
static float values_2[] = { dt, 1.0 };

static float* F_mat[row_2] = {
	values_1,
	values_2
};
static float* H_mat[row_1] = {
	values_1
};

static float* R_mat[row_1] = {
	noise
};

//for state, cov for Gyro
//prev esti
float prev_gyro_x_state[row_2][col_1] = {
	{0},{0}
};
float prev_gyro_y_state[row_2][col_1] = {
	{0},{0}
};
float prev_gyro_z_state[row_2][col_1] = {
	{0},{0}
};

float prev_gyro_x_cov[row_2][col_2] = {
	{0,0},{0,0}
};
float prev_gyro_y_cov[row_2][col_2] = {
	{0,0},{0,0}
};
float prev_gyro_z_cov[row_2][col_2] = {
	{0,0},{0,0}
};

float prev_gyro_x_measu[row_1][col_1] = {
	{0}
};  //prev, pres
float prev_gyro_y_measu[row_1][col_1] = {
	{0}
};  //prev, pres
float prev_gyro_z_measu[row_1][col_1] = {
	{0}
};  //prev, pres

//pres esti
float gyro_x_state[row_2][col_1] = {
	{0},{0}
};
float gyro_y_state[row_2][col_1] = {
	{0},{0}
};
float gyro_z_state[row_2][col_1] = {
	{0},{0}
};

float gyro_x_cov[row_2][col_2] = {
	{0,0},{0,0}
};
float gyro_y_cov[row_2][col_2] = {
	{0,0},{0,0}
};
float gyro_z_cov[row_2][col_2] = {
	{0,0},{0,0}
};

float gyro_x_measu[row_1][col_1] = {
	{0}
};  //prev, pres
float gyro_y_measu[row_1][col_1] = {
	{0}
};  //prev, pres
float gyro_z_measu[row_1][col_1] = {
	{0}
};  //prev, pres

//acc and vel matrices
float prev_x_state[row_2][col_1] = {
	{0},{0}
};
float prev_y_state[row_2][col_1] = {
	{0},{0}
};
float prev_z_state[row_2][col_1] = {
	{0},{0}
};

float prev_x_cov[row_2][col_2] = {
	{0,0},{0,0}
};
float prev_y_cov[row_2][col_2] = {
	{0,0},{0,0}
};
float prev_z_cov[row_2][col_2] = {
	{0,0},{0,0}
};

float prev_acc_x_measu[row_1][col_1] = {
	{0}
};  //prev, pres
float prev_acc_y_measu[row_1][col_1] = {
	{0}
};  //prev, pres
float prev_acc_z_measu[row_1][col_1] = {
	{0}
};  //prev, pres

float x_state[row_2][col_1] = {
	{0},{0}
};
float y_state[row_2][col_1] = {
	{0},{0}
};
float z_state[row_2][col_1] = {
	{0},{0}
};

float x_cov[row_2][col_2] = {
	{0,0},{0,0}
};
float y_cov[row_2][col_2] = {
	{0,0},{0,0}
};
float z_cov[row_2][col_2] = {
	{0,0},{0,0}
};

float acc_x_measu[row_1][col_1] = {
	{0}
};  //prev, pres
float acc_y_measu[row_1][col_1] = {
	{0}
};  //prev, pres
float acc_z_measu[row_1][col_1] = {
	{0}
};  //prev, pres

float alt_z_measu[2] = { 0,0 };

//by acc and vel, get pos by linear approximation
float pos_x = 0; float pos_y = 0; float pos_z = 0;
//===========================================================================================================================
float Z_velocity = 0.0f, Z_velgap = 0.0f, Z_stack = 0.0f, Z_velmean = 0.0f;
float gyroAngleX = 0.0f, gyroAngleY = 0.0f; 	// pitch(X) and roll(Y) angle (Euler Angle) by gyroscope
float accelAngleX = 0.0f, accelAngleY = 0.0f; 	// pitch(X) and roll(Y) angle (Euler Angle) by AccelerateScope
float compAngleX = 0.0f, compAngleY = 0.0f; 	// Complementary Filter result angle        URL https://blog.naver.com/intheglass14/222777512235 https://yjhtpi.tistory.com/352
float Rocket_vector[3] = { 0.0f };              // rocket vector(attitude) init
float Z_unitvector[3] = { 0.0f, 0.0f, 1.0f };   // Z-axis vector
float Rocket_Angle = 0.0f;    					// result of dot product Rocket Vector with Z-axis
float a, b, c = 0.0f;                         	// just acos variables

uint32_t startTick, endTick, elapsedTicks, costTime_us; //just check while loop time
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void SysTick_Init(void);               // systick  = like tic toc func in matlab
void SysTick_Delay_us(uint32_t us);
//===========================================================================================================================
//fucntion define.
//void disp_mat_2dim(float* mat[], int row, int col);
//void disp_mat(float mat[][row_2][col_2], int iter_inner, int row, int col);

float average_filter(float values[], int ind);

float** Transpose(float* matrix[], int row, int col);

float** Matrix_adder(float* matrix_1[], float* matrix_2[], int row, int col, int option);

float** Matrix_multiplicator(float* matrix_1[], float* matrix_2[], int r1, int c1, int r2, int c2);

void KF_alg(float* esti_state_prev[], float* esti_cov_prev[], float* measu_state[], float sigma, float* esti_state_out[], float* esti_cov_out[]);

void scalar_KF(float* prev_alt, float* prev_alt_cov, float measu_alt, float* esti_alt, float* esti_alt_cov);

//===========================================================================================================================
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// to use printf
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}
// use printf

// sdcard part
FATFS fs; // file system
FIL fil;  // File
FILINFO fno;
FRESULT fresult; // result
UINT br, bw;     // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;
char str[16]; // Enough to hold all numbers up to 32-bit int
// sdcard part

// mpu6050
RawData_Def myAccelRaw, myGyroRaw;
ScaledData_Def myAccelScaled, myGyroScaled;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	MPU_ConfigTypeDef myMpuConfig;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  SysTick_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	initialBMP180();
	MPU6050_Init(&hi2c1);
	myMpuConfig.Accel_Full_Scale = AFS_SEL_16g; 		// full scale setting
	myMpuConfig.ClockSource = Internal_8MHz;
	myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
	myMpuConfig.Gyro_Full_Scale = FS_SEL_500;
	myMpuConfig.Sleep_Mode_Bit = 0;
	MPU6050_Config(&myMpuConfig);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim3);

	fresult = f_mount(&fs, "/", 1);
	/* Check free space */
	f_getfree("", &fre_clust, &pfs);
	total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);
	/* Open file to write/ create a file if it doesn't exist */
	fresult = f_open(&fil, "file1.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	/* Writing text */


	//eliminate gravity comp.
	float gravity_x = 0.0f;
	float gravity_y = 0.0f;
	float gravity_z = 0.0f;
	//define lambda
	float lambda = 0.0f;
	float standard_alt = 0.0f;
	int iter_temp = 1;  //begin num 1, because, we should use previous values

	float tmp_values[10] = 0;
	for (int i = 0; i < 10; i++) {
		altitude = readTrueAltitude(0);		//by 1ms
		tmp_values[i] = altitude;
	}
	standard_alt = average_filter(tmp_values,10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		startTick = SysTick->VAL;             	// tic (check loop time)
		// bmp180 part
		temp = readTrueTemp();
		press = readTruePress(0);

		//get altitude
		alt_z_measu[0] = alt_z_measu[1];
		for (int i = 0; i < 10; i++) {
			altitude = readTrueAltitude(0);		//by 1ms
			tmp_values[i] = altitude;
		}
		alt_z_measu[1] = average_filter(tmp_values, 10) - standard_alt;

		MPU6050_Get_Accel_Scale(&myAccelScaled);
		MPU6050_Get_Gyro_Scale(&myGyroScaled);

		//Gyro KF
		//gyro x
		gyro_x_measu[1][1] = myGyroScaled.x;
		sigma = gyro_x_measu[1][1] - prev_gyro_x_measu[1][1];
		KF_alg(prev_gyro_x_state, prev_gyro_x_cov, gyro_x_measu, sigma, gyro_x_state, gyro_x_cov);
		prev_gyro_x_measu[1][1] = gyro_x_measu[1][1];
		//gyro y
		gyro_y_measu[1][1] = myGyroScaled.y;
		sigma = gyro_y_measu[1][1] - prev_gyro_y_measu[1][1];
		KF_alg(prev_gyro_y_state, prev_gyro_y_cov, gyro_y_measu, sigma, gyro_y_state, gyro_y_cov);
		prev_gyro_y_measu[1][1] = gyro_y_measu[1][1];
		//gyro z
		gyro_z_measu[1][1] = myGyroScaled.z;
		sigma = gyro_z_measu[1][1] - prev_gyro_z_measu[1][1];
		KF_alg(prev_gyro_z_state, prev_gyro_z_cov, gyro_z_measu, sigma, gyro_z_state, gyro_z_cov);
		prev_gyro_z_measu[1][1] = gyro_z_measu[1][1];


		// printf("%.2f\r\n", altitude);      	// test printf
		// bmp180 part

		// mpu6050 part
		//    printf("Accel: X=%.2f, Y=%.2f, Z=%.2f\n ", myAccelScaled.x, myAccelScaled.y, myAccelScaled.z);
		//    HAL_Delay(50);
		//    printf("Accelraw: X=%.2f, Y=%.2f, Z=%.2f\n ", myAccelRaw.x, myAccelRaw.y, myAccelRaw.z);
		//    HAL_Delay(50);
		//    printf("Gyro: X=%.2f, Y=%.2f, Z=%.2f\r\n", myGyroScaled.x, myGyroScaled.y, myGyroScaled.z);
		//    HAL_Delay(50);
		// mpu6050 part

		/*                        this code will be run after remove gravity element from accelerate scope and add time out start point here
		 if (start == 0 || myAccelScaled.z >= 15.0)
		 {
		 Parachute = 1;
		 start = 1;
		 printf("Start deploying parachute system\r\n");
		 } */

		accelAngleX = atan2f(myAccelScaled.y,
				sqrtf(myAccelScaled.x * myAccelScaled.x
								+ myAccelScaled.z * myAccelScaled.z)); 	// RAD pitch and roll by accerlate scope
		accelAngleY = atan2f(-myAccelScaled.x,
				sqrtf(myAccelScaled.y * myAccelScaled.y
								+ myAccelScaled.z * myAccelScaled.z));

		gyroAngleX += myGyroScaled.x * dt; 								// RAD by gyro scope
		gyroAngleY += myGyroScaled.y * dt;
		//printf("accAngleX: %.2f accAngleY: %.2f\n",accelAngleX*RAD_TO_DEG, accelAngleY*RAD_TO_DEG);
		//HAL_Delay(50);
		//printf("gyroAngleX: %.2f gyroAngleY: %.2f\n",gyroAngleX*RAD_TO_DEG, gyroAngleY*RAD_TO_DEG);
		// HAL_Delay(50);

		compAngleX = ALPHA * gyroAngleX + (1.0 - ALPHA) * accelAngleX;
		compAngleY = ALPHA * gyroAngleY + (1.0 - ALPHA) * accelAngleY;

		Rocket_vector[0] = -sin(compAngleX);           					// Euler angle to vector (incorrect)
		Rocket_vector[1] = sin(compAngleY) * cos(compAngleX); 			// https://stackoverflow.com/questions/1568568/how-to-convert-euler-angles-to-directional-vector
		Rocket_vector[2] = cos(compAngleY) * cos(compAngleX);
		// printf("vec: %.2f  %.2f  %.2f \n",Rocket_vector[0],Rocket_vector[1],Rocket_vector[2]);
		a = Rocket_vector[0] * Z_unitvector[0]
				+ Rocket_vector[1] * Z_unitvector[1]
				+ Rocket_vector[2] * Z_unitvector[2];
		b = sqrt(Rocket_vector[0] * Rocket_vector[0]
						+ Rocket_vector[1] * Rocket_vector[1]
						+ Rocket_vector[2] * Rocket_vector[2]);
		c = sqrt(Z_unitvector[0] * Z_unitvector[0]
						+ Z_unitvector[1] * Z_unitvector[1]
						+ Z_unitvector[2] * Z_unitvector[2]);
		
		Rocket_Angle = acos(a / (b * c)) * RAD_TO_DEG;                            // inner product(dot product) Rocket vector with Z unit vector
		printf("Rocket Angle: %.2f\r\n", Rocket_Angle); 					                // test code

		/*accZ_raw = myAccelScaled.z * cos(compAngleX)      // remove gravity to accel data
				- myAccelScaled.y * sin(compAngleX); 						// z y
		accY_raw = myAccelScaled.y * cos(compAngleX)
				+ myAccelScaled.z * sin(compAngleX); 						// y z

		accZ_rot = -accZ_raw * sin(compAngleY) + accY_raw * cos(compAngleY);
		//printf("pure Z acc : %.2f\r\n", accZ_rot); 						// test code not pass

		Z_stack -= Z_velocity;             								// delete pre-prev vel
		Z_velocity += accZ_rot * dt;       								// calc Zvelocity
		Z_stack += Z_velocity;             								// add updated vel
		Z_velmean = Z_stack / 2;           								// mean of prev and
		Z_velgap = Z_velocity - Z_velmean;*/

		// deploy parachute part
		if (Parachute == 0)                           					// == 1 ( 0 is just test )
		{
			/*if (altitude >= 380) {
				Parachute = 0;
				printf("deploy parachute: altitude\r\n");
			}
			if (Rocket_Angle >= desiredAngle) {
				Parachute = 0;
				printf("deploy parachute: Desired Angle\r\n");
			}
			// if (Z_velgap <= 0.1f)
			// {
			//   Parachute = 0;
			//   printf("deploy parachute : Z velocity");
			// }
			if (elapsed_time_ms >= 5000) // 10 s
					{
				Parachute = 0;
				printf("deploy parachute : time out\r\n");
			}*/
		}

		// sdcard part
		char buffer[100]; // Buffer to hold string

		sprintf(buffer,
				"altitude: %.2f Gyro: X=%.2f Y=%.2f Z=%.2f Accel: X=%.2f Y=%.2f Z=%.2f\r\n",
				altitude, myGyroScaled.x, myGyroScaled.y, myGyroScaled.z,
				myAccelScaled.x, myAccelScaled.y, myAccelScaled.z);
		if (fresult == FR_OK) {
			fresult = f_write(&fil, buffer, strlen(buffer), &bw); // Write the string to file
			f_sync(&fil);                    // Ensure data is written and saved
		}

		// sdcard part
	}
	// sdcard part
	if (fresult == FR_OK) {
		fresult = f_close(&fil);
	}
	// sdcard part

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1280 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 6400 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // time out private function
{
	if (htim->Instance == TIM3) {
		second++;
		if (second == 5) {
			//      printf("timeout ok\r\n");
			second = 0;
		}
	}
}

void SysTick_Init(void)                      // tic toc function - once of while
{
	SysTick->LOAD = 0xFFFFFF;                   // set max value
	SysTick->VAL = 0;                           // init present count to 0
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
	SysTick_CTRL_ENABLE_Msk;    // timer start using CPU Clock
}

void SysTick_Delay_us(uint32_t us)            // tic toc function
{
	SysTick->LOAD = us * (64000000 / 8 / 1000000) - 1;
	SysTick->VAL = 0;
	while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
		;
}
//======================================================================================================
//======================================================================================================

float average_filter(float values[], int ind) {
	// Doesn't have any Exception handling
	float sum = 0;
	// Calculate the initial sum
	for (int i = 0; i < 10; i++) {
		sum += values[i];
	}
	return sum / 10;
}

float** Transpose(float* matrix[], int row, int col)
{
	if (matrix == NULL || row <= 0 || col <= 0) {
		printf("[error occured matrix_size]\n");
		return NULL;
	}

	float** result = (float**)calloc(col, sizeof(float*));
	for (int i = 0; i < col; i++) {
		result[i] = (float*)calloc(row, sizeof(float));
	}

	for (int i = 0; i < row; i++) {
		for (int j = 0; j < col; j++) {
			result[j][i] = matrix[i][j];
			//printf("%.2f", result[j][i]);
		}
		//printf("\n");
	}
	//printf("\n\n");
	//need to free result memeroy in main function call
	return result;
}

//matrix_1 : output / matrix_1, matrix_2 : paramter for operation.
float** Matrix_adder(float* matrix_1[], float* matrix_2[], int row, int col, int option)
{
	float** result = (float**)calloc(row, sizeof(float*));
	for (int i = 0; i < row; i++) {
		result[i] = (float*)calloc(col, sizeof(float));
	}
	if (option == 1) {//add
		for (int i = 0; i < row; i++) {
			for (int j = 0; j < col; j++) {
				result[i][j] = matrix_1[i][j] + matrix_2[i][j];
			}
		}
	}
	else if (option == 0) {//substract
		for (int i = 0; i < row; i++) {
			for (int j = 0; j < col; j++) {
				result[i][j] = matrix_1[i][j] - matrix_2[i][j];
			}
		}
	}
	else {
		//warning
		printf("[adder function error]\n");
	}
	return result;
}

float** Matrix_multiplicator(float* matrix_1[], float* matrix_2[], int r1, int c1, int r2, int c2) {

	if (c1 != r2) {// exception dealer
		printf("[Matrix Mult function has Error...]\n");
		return NULL;
	}
	// if heap is too small to make result mat, use array to make return matrix.
	float** result = (float**)calloc(r1, sizeof(float*));
	for (int i = 0; i < r1; i++) {
		result[i] = (float*)calloc(c2, sizeof(float));
	}

	for (int i = 0; i < r1; i++) {
		for (int j = 0; j < c2; j++) {
			float tempSum = 0;

			for (int k = 0; k < c1; k++) {
				tempSum += matrix_1[i][k] * matrix_2[k][j];
			}
			result[i][j] = tempSum;
		}
	}

	return result;
}

void KF_alg(float* esti_state_prev[], float* esti_cov_prev[], float* measu_state[], float sigma, float* esti_state_out[], float* esti_cov_out[]) {
	/*
	* pred_state, pred_cov, Kalman Gain,
	* -> using dynamic allocation.
	*/
	float** pred_state = (float**)calloc(row_2, sizeof(float*));
	for (int i = 0; i < row_2; i++) {
		pred_state[i] = (float*)calloc(col_1, sizeof(float));
	}
	float** pred_cov = (float**)calloc(row_2, sizeof(float*));
	for (int i = 0; i < row_2; i++) {
		pred_cov[i] = (float*)calloc(col_2, sizeof(float));
	}
	float** Kalman_Gain = (float**)calloc(row_2, sizeof(float*));
	for (int i = 0; i < row_2; i++) {
		Kalman_Gain[i] = (float*)calloc(col_1, sizeof(float));
	}
	float** trans_F = Transpose(F_mat, row_2, col_2);
	float** trans_H = Transpose(H_mat, row_1, col_2);

	//define Q matrix
	float noise_w1[] = { dt*sigma };
	float noise_w2[] = { sigma };
	float* process_noise[row_2] = {
		noise_w1,
		noise_w2
	};
	float** trans_process_noise = Transpose(process_noise, row_2, col_1);
	float** Q_mat = Matrix_multiplicator(process_noise, trans_process_noise, row_2, col_1, row_1, col_2);

	//pred_state
	pred_state = Matrix_multiplicator(F_mat, esti_state_prev, row_2, col_2, row_2, col_1);

	//pred_cov
	pred_cov = Matrix_multiplicator(F_mat, esti_cov_prev, row_2, col_2, row_2, col_2);
	pred_cov = Matrix_multiplicator(pred_cov, trans_F, row_2, col_2, row_2, col_2);
	pred_cov = Matrix_adder(pred_cov, Q_mat, row_2, col_2, add);

	//Kalman_Gain
	float** inv_term_sub = Matrix_multiplicator(H_mat, pred_cov, row_1, col_2, row_2, col_2);
	float** inv_term = Matrix_multiplicator(inv_term_sub, trans_H, row_1, col_2, row_2, col_1);
	inv_term = Matrix_adder(inv_term,R_mat, row_1, col_1, add);
	inv_term[0][0] = 1 / (inv_term[0][0]);
	//2x1 matrix.
	Kalman_Gain = Matrix_multiplicator(pred_cov, trans_H, row_2, col_2, row_2, col_1);
	Kalman_Gain = Matrix_multiplicator(Kalman_Gain, inv_term, row_2, col_1, row_1, col_1);

	//esti_state
	//reuse inv_term to calc innovation term.
	float** HX = Matrix_multiplicator(H_mat, pred_state, row_1, col_2, row_2, col_1);
	inv_term = Matrix_adder(measu_state, HX, row_1, col_1, substract);
	float** innov = Matrix_multiplicator(Kalman_Gain, inv_term, row_2, col_1, row_1, col_1);
	esti_state_out = Matrix_adder(pred_state, innov, row_2, col_1, add);

	//esti_cov
	float** KH = Matrix_multiplicator(Kalman_Gain,H_mat, row_2, col_1, row_1, col_2);
	KH = Matrix_multiplicator(KH, pred_cov, row_2, col_2, row_2, col_2);
	esti_cov_out = Matrix_adder(pred_cov, KH, row_2, col_2, substract);

	//free memory using Transpose..!
	for (int i = 0; i < col_1; i++) {
		free(trans_process_noise[i]);
	}
	free(trans_process_noise);
	for (int i = 0; i < col_2; i++) {
		free(trans_H[i]);
	}
	free(trans_H);
	for (int i = 0; i < col_2; i++) {
		free(trans_F[i]);
	}
	free(trans_F);

	//free pred_state, pred_cov, Kalman Gain
	for (int i = 0; i < col_2; i++) {
		free(Kalman_Gain[i]);
	}
	free(Kalman_Gain);
	for (int i = 0; i < col_2; i++) {
		free(pred_cov[i]);
	}
	free(pred_cov);
	for (int i = 0; i < col_1; i++) {
		free(pred_state[i]);
	}
	free(pred_state);
}

void scalar_KF(float* prev_alt, float* prev_alt_cov, float measu_alt, float* esti_alt, float* esti_alt_cov) {
	float R_scalar = 0.025;  //by Average filter, var of R decrease into 1/10    //  in low power mode, its error var is 50cm
	float H_scalar = 1, F_scalar = 1;



}
//======================================================================================================
//======================================================================================================

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
	while (1) {
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
