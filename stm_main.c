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
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD_TO_DEG 57.29578
#define DEG_TO_RAD 0.017453

//define parameters
#define row_1 1
#define col_1 1
#define row_2 2
#define col_2 2
#define row_3 3
#define col_3 3
#define row_10 10
#define iter 1000
#define add 1
#define substract 0
#define math_pi 3.1415926535
#define gravity_const 9.81
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
float temp, press, altitude;
float count;
int second = 0;

//for return
typedef struct
{
    float** estimate_cov;
    float** estimate_state;
    float** Kalman_Gain;
}KF_return;

typedef struct
{
    float estimate_cov;
    float estimate_state;
    float Kalman_Gain;
}scalar_KF_return;

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


float*** Matrix_gen(int row_n, int col_n, int iter_n);      //?��?��?�� [iter][row][col] ?��
void Matrix_free(float*** matrix, int row_n, int iter_n);

float** Matrix_gen_2dim(int row_n, int col_n);
void Matrix_free_2dim(float** matrix, int row_n);
float average_filter(float* values);
float** inv_2x2(float** matrix);
float** Transpose(float** matrix, int row, int col);
float** Matrix_adder(float** matrix_1, float** matrix_2, int row, int col, int option);
float** Matrix_multiplicator(float** matrix_1, float** matrix_2, int r1, int c1, int r2, int c2);
KF_return KF_alg(float**F, float** H, float** R, float** esti_state_prev, float** esti_cov_prev, float** measu_state, float sigma, float del_t);
scalar_KF_return scalar_KF_alg(float F_scalar, float H_scalar, float R_scalar, float esti_state_prev, float esti_cov_prev, float measu_state, float sigma_w, float del_t);

void SysTick_Init(void)
{
    SysTick->LOAD = 0xFFFFFF; // 최�?값으�? ?��?��
    SysTick->VAL = 0;         // ?��?�� 카운?�� 값을 0?���? 초기?��
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_ENABLE_Msk; // CPU ?��?��?�� 그�?�? ?��?��?���? ???���? ?��?��
}

void SysTick_Delay_us(uint32_t us)
{
  SysTick->LOAD = us * (64000000 / 8 / 1000000) - 1;
  SysTick->VAL = 0;
  while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
    ;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// use printf
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
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
float desiredAngle = 45.0;
float gyroAngle = 0.0;
float dt = 0.00078;
// mpu6050
float count = 0;

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
  myMpuConfig.Accel_Full_Scale = AFS_SEL_16g;
  myMpuConfig.ClockSource = Internal_8MHz;
  myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
  myMpuConfig.Gyro_Full_Scale = FS_SEL_500;
  myMpuConfig.Sleep_Mode_Bit = 0;
  MPU6050_Config(&myMpuConfig);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim3);

  int Parachute = 0; // parachute deployment state
  int start = 0;     // state of time out count
  float accZ_raw, accY_raw, accZ_rot;
  float Z_velocity = 0.0f, Z_velgap = 0.0f, Z_stack = 0.0f, Z_velmean = 0.0f;
  // ?���??? ?��?�� ?��?�� (LPF?? HPF ?��?��)
  float alpha = 0.004f, beta = 0.996f;            // HPF ?��?��  LPF ?��?�� https://blog.naver.com/intheglass14/222777512235
  float gyroAngleX = 0.0f, gyroAngleY = 0.0f;   // ?��?���??? ?��?���??? 추정?�� 각도
  float accelAngleX = 0.0f, accelAngleY = 0.0f; // �????��?�� ?��?���??? 추정?�� 각도
  float compAngleX = 0.0f, compAngleY = 0.0f;   // ?���??? ?��?���??? ?��?�� 추정?�� 각도
  float AngleX = 0.0f, AngleY = 0.0f;           // Radian angle
  float Rocket_vector[3] = {0.0f};              // rocket vector
  float Rocket_Angle = 0.0f;                    // with Z axis
  float Z_unitvector[3] = {0.0f, 0.0f, 1.0f};
  float a, b, c = 0.0f; // for acos variables
  float KF_z_pos = 0.0f;
  float KF_z_acc_vel = 0.0f;
  uint32_t startTick, endTick, elapsedTicks, costTime_us;

  //eliminate gravity comp.
  float gravity_x = 0.0f;
  float gravity_y = 0.0f;
  float gravity_z = 0.0f;
  //define lambda
  float lambda = 0.0f;

  //del_t should be changed into dt later
  //by simulation, we should get optimal del_t
  float del_t = 0.1;  //it means 100ms in operation, but in HW, we use ms scale
                      //additionally, it should be revised into accurate del_t

  //iter counter
  int iter_temp = 1;  //begin num 1, because, we should use previous values

  //Gyro KF to eliminate gravity component in our MPU acc datas.
  float** R_mat_gyro = Matrix_gen_2dim(row_1, col_1);
  float** H_vec_gyro = Matrix_gen_2dim(row_1, col_2);
  float** F_vec_gyro = Matrix_gen_2dim(row_2, col_2);
  R_mat_gyro[0][0] = 0.124;                                       //?��?���? ?��?��
  H_vec_gyro[0][0] = 1; H_vec_gyro[0][1] = 0;
  F_vec_gyro[0][0] = 1; F_vec_gyro[0][1] = 0; F_vec_gyro[1][0] = del_t; F_vec_gyro[1][1] = 1;

  float*** measu_Ang_acc_x = Matrix_gen(row_1, col_1, iter);
  float*** est_state_rho_x = Matrix_gen(row_2, col_1, iter);
  float*** est_cov_rho_x = Matrix_gen(row_2, col_2, iter);

  float*** measu_Ang_acc_y = Matrix_gen(row_1, col_1, iter);
  float*** est_state_pi_y = Matrix_gen(row_2, col_1, iter);
  float*** est_cov_pi_y = Matrix_gen(row_2, col_2, iter);

  float*** measu_Ang_acc_z = Matrix_gen(row_1, col_1, iter);
  float*** est_state_thea_z = Matrix_gen(row_2, col_1, iter);
  float*** est_cov_thea_z = Matrix_gen(row_2, col_2, iter);

  //initializing
  est_state_rho_x[0][0][0] = myGyroScaled.x;      //changed Gyro data
  est_cov_rho_x[0][0][0] = R_mat_gyro[0][0];
  est_state_pi_y[0][0][0] = myGyroScaled.y;
  est_cov_pi_y[0][0][0] = R_mat_gyro[0][0];
  est_state_thea_z[0][0][0] = myGyroScaled.z;
  est_cov_thea_z[0][0][0] = R_mat_gyro[0][0];


  //=========need Kalman Gain and COV of estimation       R, F, H / Q : made in KF algorithm
  //  [ acc; vel; pos ]
  float** R_mat = Matrix_gen_2dim(row_1, col_1);
  float** H_vec = Matrix_gen_2dim(row_1, col_2);
  float** F_vec = Matrix_gen_2dim(row_2, col_2);
  R_mat[0][0] = 0.0144;
  H_vec[0][0] = 1;H_vec[0][1] = 0;
  F_vec[0][0] = 1; F_vec[0][1] = 0; F_vec[1][0] = del_t; F_vec[1][1] = 1;

  //for altitude, scalar KF
  float R_scalar = 0.025;  //by Average filter, var of R decrease into 1/10    //  in low power mode, its error var is 50cm
  float H_scalar = 1, F_scalar = 1;

  //sigma_w is defined by differentation between 2-steps


  //for X,Y
  //KF to estimate vel, acc. this cause linear approximation of postion of x,y
  float*** measu_acc_x = Matrix_gen(row_1, col_1, iter);
  float*** est_state_x = Matrix_gen(row_3, col_1, iter);  //in est_x;
  float*** est_cov_x = Matrix_gen(row_2, col_2, iter);

  float*** measu_acc_y = Matrix_gen(row_1, col_1, iter);
  float*** est_state_y = Matrix_gen(row_3, col_1, iter);
  float*** est_cov_y = Matrix_gen(row_2, col_2, iter);

  //for Z
  //KF to estimate vel, acc. to store
  //scalar KF to estimate location based on BMP180 measurement data
  float*** measu_acc_z = Matrix_gen(row_1, col_1, iter);

  float standard_alt;
  float*** average_pos_z = Matrix_gen(row_1, col_1, iter);     //for barometer, we should apply Average filter

  float*** est_state_z = Matrix_gen(row_3, col_1, iter);
  float*** est_cov_z = Matrix_gen(row_2, col_2, iter);     //for detecting location(scalar KF)
  float* est_cov_z_for_pos = (float*)malloc(iter*sizeof(float));      //cov for scalar KF

  //init call sensing values

  // mpu6050 part
  MPU6050_Get_Accel_Scale(&myAccelScaled);
  MPU6050_Get_Gyro_Scale(&myGyroScaled);

  float tmp_values[10] = {0};
  for (int i = 0; i < 10; i++) {

	  //measure position 10 times
	  //measu_pos_z[i][1][iter_temp] = ?���? 측정 값코?��;
	  // bmp180 part
	  temp = readTrueTemp();
	  press = readTruePress(0);
	  altitude = readTrueAltitude(0);

	  tmp_values[i] = altitude;
  }
  standard_alt = average_filter(tmp_values);
  average_pos_z[0][0][0] = 0;     //relative altitude from lift off


  //initializing
  est_state_x[0][0][0] = myAccelScaled.x;
  est_cov_x[0][0][0] = R_mat[0][0];
  est_state_y[0][0][0] = myAccelScaled.y;
  est_cov_y[0][0][0] = R_mat[0][0];
  est_state_z[0][0][0] = myAccelScaled.z - gravity_const;
  est_cov_z[0][0][0] = R_mat[0][0];
  //position initializing     for linear KF
  est_state_z[0][2][0] = average_pos_z[0][0][0];
  est_cov_z_for_pos[0] = R_scalar;


  fresult = f_mount(&fs, "/", 1);

  /* Check free space */
  f_getfree("", &fre_clust, &pfs);

  total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);

  free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);

  /* Open file to write/ create a file if it doesn't exist */
  fresult = f_open(&fil, "file1.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

  /* Writing text */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    startTick = SysTick->VAL; // ?��?�� ?���? ???��

    // mpu6050 part
	MPU6050_Get_Accel_Scale(&myAccelScaled);
	MPU6050_Get_Gyro_Scale(&myGyroScaled);


    //store angular acc to matrix
    //
    // should elimnate off_set value of Gyro. vel.
    //
    measu_Ang_acc_x[iter_temp][0][0] = myGyroScaled.x;
    measu_Ang_acc_y[iter_temp][0][0] = myGyroScaled.y;
    measu_Ang_acc_z[iter_temp][0][0] = myGyroScaled.z;

    //Average Filter to optimize noise      to get positon of z
    for (int i = 0; i < 10;i++) {

        // bmp180 part
    	temp = readTrueTemp();
    	press = readTruePress(0);
    	altitude = readTrueAltitude(0);
        //measure position 10 times
        //measu_pos_z[i][1][iter_temp] = 위치 측정 값코드;

        tmp_values[i] = altitude;
    }
    average_pos_z[iter_temp][0][0] = average_filter(tmp_values)-standard_alt;


    //other code
    //about...
    //KF, scalar KF
    //KF_return KF_alg(float** F, float** H, float** R, float** esti_state_prev, float** esti_cov_prev, float** measu_state, float sigma_w, float del_t)

    //at first step, in our MPU sensor includes gravity component in our acceleration data.
    //so we should elminate them, following steps.
    //
    //Gyro KF
    //-> mismatch of coordinate system's angle
    //-> gravity do not affect not only axis z but also x,y
    //-> compensation with tranformation of coordinate system
    //-> eliminate gravity comp

    //      2023 09 27 latest patch...
	//
	//for x pitch ; rho
	KF_return KF_rho_x_vel_ang;
	float sigma_w_rho_x = measu_Ang_acc_x[iter_temp][0][0] - measu_Ang_acc_x[iter_temp - 1][0][0];
	KF_rho_x_vel_ang = KF_alg(F_vec_gyro, H_vec_gyro, R_mat_gyro, est_state_rho_x[iter_temp - 1], est_cov_rho_x[iter_temp - 1], measu_Ang_acc_x[iter_temp], sigma_w_rho_x, del_t);
	est_state_rho_x[iter_temp][0][0] = KF_rho_x_vel_ang.estimate_state[0][0];
	est_state_rho_x[iter_temp][1][0] = KF_rho_x_vel_ang.estimate_state[1][0];

	//for y yaw ; pi
	KF_return KF_pi_y_vel_ang;
	float sigma_w_pi_y = measu_Ang_acc_y[iter_temp][0][0] - measu_Ang_acc_y[iter_temp - 1][0][0];
	KF_pi_y_vel_ang = KF_alg(F_vec_gyro, H_vec_gyro, R_mat_gyro, est_state_pi_y[iter_temp - 1], est_cov_pi_y[iter_temp - 1], measu_Ang_acc_y[iter_temp], sigma_w_pi_y, del_t);
	est_state_pi_y[iter_temp][0][0] = KF_pi_y_vel_ang.estimate_state[0][0];
	est_state_pi_y[iter_temp][1][0] = KF_pi_y_vel_ang.estimate_state[1][0];

	//for z roll ; thea (theata)
	KF_return KF_thea_z_vel_ang;
	float sigma_w_thea_z = measu_Ang_acc_z[iter_temp][0][0] - measu_Ang_acc_z[iter_temp - 1][0][0];
	KF_thea_z_vel_ang = KF_alg(F_vec_gyro, H_vec_gyro, R_mat_gyro, est_state_thea_z[iter_temp - 1], est_cov_thea_z[iter_temp - 1], measu_Ang_acc_z[iter_temp], sigma_w_thea_z, del_t);
	est_state_thea_z[iter_temp][0][0] = KF_thea_z_vel_ang.estimate_state[0][0];
	est_state_thea_z[iter_temp][1][0] = KF_thea_z_vel_ang.estimate_state[1][0];

	// angle of lambda is radian angle...       mod of angle with axis z, in fact it could not be over 30(deg.)

	//by upper codes, we can calculate angles of eachb axis.
	//but by Gyro sensing, High frequency comps have high accuracy. in contrast, it could acuumulates errors by spin of each axises goes on

	//we should generate solution of this problem.
	//230930
	// if we do not minimize error rates, we could not estimate our rocket's state

	//calculate acc comp by gravity
	if (est_state_thea_z[iter_temp][1][0]>(math_pi / 4)) {
		//calc with sine    /use rho and thea
		lambda = (float)atan(tan((double)est_state_rho_x[iter_temp][1][0])/sin((double)est_state_thea_z[iter_temp][1][0]));
	}
	else {
		//calc with cosine  /use pi and thea
		lambda = (float)atan(tan((double)est_state_pi_y[iter_temp][1][0]) / cos((double)est_state_thea_z[iter_temp][1][0]));
	}

	//with lambda, we should calculate gravity comp of each axis.
	//gravity = 9.81m/s^2
	gravity_x = gravity_const * sin((double)lambda) * cos((double)est_state_thea_z[iter_temp][1][0]);
	gravity_y = gravity_const * sin((double)lambda) * sin((double)est_state_thea_z[iter_temp][1][0]);
	gravity_z = gravity_const * cos((double)lambda);


	//store acc to matrix
	measu_acc_x[iter_temp][0][0] = myAccelScaled.x-gravity_x;
	measu_acc_y[iter_temp][0][0] = myAccelScaled.y-gravity_y;
	measu_acc_z[iter_temp][0][0] = myAccelScaled.z-gravity_z;


	//for x
	KF_return KF_x_acc_vel;
	float sigma_w_x = measu_acc_x[iter_temp][0][0] - measu_acc_x[iter_temp - 1][0][0];
	KF_x_acc_vel = KF_alg(F_vec, H_vec, R_mat, est_state_x[iter_temp-1],est_cov_x[iter_temp-1],measu_acc_x[iter_temp], sigma_w_x, del_t);

	est_cov_x[iter_temp] = KF_x_acc_vel.estimate_cov;
	//we can get KalmanGain but, I think that could be needed more computational efforts
	//x linear approximation with storage
	est_state_x[iter_temp][0][0] = KF_x_acc_vel.estimate_state[0][0];        //acc
	est_state_x[iter_temp][1][0] = KF_x_acc_vel.estimate_state[1][0];        //vel
	est_state_x[iter_temp][2][0] = est_state_x[iter_temp - 1][2][0] + est_state_x[iter_temp - 1][1][0] * del_t + est_state_x[iter_temp - 1][0][0] * del_t * del_t / 2;


	//for y
	KF_return KF_y_acc_vel;
	float sigma_w_y = measu_acc_y[iter_temp][0][0] - measu_acc_y[iter_temp - 1][0][0];
	KF_y_acc_vel = KF_alg(F_vec, H_vec, R_mat, est_state_y[iter_temp - 1], est_cov_y[iter_temp - 1], measu_acc_y[iter_temp], sigma_w_y, del_t);

	est_cov_y[iter_temp] = KF_y_acc_vel.estimate_cov;
	//we can get KalmanGain but, I think that could be needed more computational efforts
	//y linear approximation with storage
	est_state_y[iter_temp][0][0] = KF_y_acc_vel.estimate_state[0][0];        //acc
	est_state_y[iter_temp][1][0] = KF_y_acc_vel.estimate_state[1][0];        //vel
	est_state_y[iter_temp][2][0] = est_state_y[iter_temp - 1][2][0] + est_state_y[iter_temp - 1][1][0] * del_t + est_state_y[iter_temp - 1][0][0] * del_t * del_t / 2;


	//for z
	KF_return KF_z_acc_vel;
	scalar_KF_return KF_z_pos;

	//여기서 동적할당이 필요하려나??!
	float sigma_w_z_acc = measu_acc_z[iter_temp][0][0] - measu_acc_z[iter_temp - 1][0][0];
	KF_z_acc_vel = KF_alg(F_vec, H_vec, R_mat, est_state_z[iter_temp - 1], est_cov_z[iter_temp - 1], measu_acc_z[iter_temp], sigma_w_z_acc, del_t);

	est_cov_z[iter_temp] = KF_z_acc_vel.estimate_cov;
	//we can get KalmanGain but, I think that could be needed more computational efforts
	//y linear approximation with storage
	est_state_z[iter_temp][0][0] = KF_z_acc_vel.estimate_state[0][0];        //acc
	est_state_z[iter_temp][1][0] = KF_z_acc_vel.estimate_state[1][0];
	//vel
	//est_state_y[iter_temp][2][0] = est_state_y[iter_temp - 1][2][0] + est_state_y[iter_temp - 1][1][0] * del_t + est_state_y[iter_temp - 1][0][0] * del_t * del_t / 2;

	//scalar KF for position
	//scalar_KF_return scalar_KF_alg(float F_scalar, float H_scalar, float R_scalar, float esti_state_prev, float esti_cov_prev, float measu_state, float sigma_w, float del_t);
	float sigam_w_pos = average_pos_z[iter_temp][0][0] - average_pos_z[iter_temp-1][0][0];
	KF_z_pos = scalar_KF_alg(F_scalar, H_scalar, R_scalar, est_state_z[iter_temp-1][2][0], est_cov_z_for_pos[iter_temp-1], average_pos_z[iter_temp][0][0], sigam_w_pos, del_t);
	est_state_z[iter_temp][2][0] = KF_z_pos.estimate_state;
	est_cov_z_for_pos[iter_temp] = KF_z_pos.estimate_cov;



	//this code display values of position
	printf("[ ACC_X : %.2f ]\n[ ACC_Y : %.2f ]\n[ ACC_Z : %.2f ]\n\n",est_state_x[iter_temp][2][0],est_state_y[iter_temp][2][0],est_state_z[iter_temp][2][0]);





	//add deployment of parachute code in here
			//minjun code

    // HAL_Delay(10);
    // printf("%.2f\r\n", altitude);


//    printf("Accel: X=%.2f, Y=%.2f, Z=%.2f\n ", myAccelScaled.x, myAccelScaled.y, myAccelScaled.z);
//    HAL_Delay(50);
//    printf("Accelraw: X=%.2f, Y=%.2f, Z=%.2f\n ", myAccelRaw.x, myAccelRaw.y, myAccelRaw.z);
//    HAL_Delay(50);
//    printf("Gyro: X=%.2f, Y=%.2f, Z=%.2f\r\n", myGyroScaled.x, myGyroScaled.y, myGyroScaled.z);
//    HAL_Delay(50);
    // mpu6050 part

    // Variables = Rocekt_Angle, altitude, Z_velocity(Arr_velocity),elapsedSeconds (?��?���????��?�� �????��)
    /*if (start == 0 || Z_velocity >= 5)
    {
      Parachute = 1;
      start = 1;

      printf("Start deploying parachute system\r\n");
    }
*/
    if (Parachute == 0)  // set 1
    {
      // deploy Parachute part^^7 https://yjhtpi.tistory.com/352 //https://blog.naver.com/intheglass14/222777512235 MPU6050 ?��보필?�� �????�� 블로�??? �???
      accelAngleX = atan2f(myAccelScaled.y, sqrtf(myAccelScaled.x * myAccelScaled.x + myAccelScaled.z * myAccelScaled.z)); //RAD
      accelAngleY = atan2f(-myAccelScaled.x, sqrtf(myAccelScaled.y * myAccelScaled.y + myAccelScaled.z * myAccelScaled.z));

      gyroAngleX += myGyroScaled.x * dt; //RAD
      gyroAngleY += myGyroScaled.y * dt;
      //printf("accAngleX: %.2f accAngleY: %.2f\n",accelAngleX*RAD_TO_DEG, accelAngleY*RAD_TO_DEG);
//      HAL_Delay(50);
      printf("gyroAngleX: %.2f gyroAngleY: %.2f\n",gyroAngleX*RAD_TO_DEG, gyroAngleY*RAD_TO_DEG);
      //HAL_Delay(50);

      //HAL_Delay(50);
      compAngleX = beta * (compAngleX + gyroAngleX) + alpha * accelAngleX;
      compAngleY = beta * (compAngleY + gyroAngleY) + alpha * accelAngleY;
      AngleX = compAngleX * RAD_TO_DEG;
      AngleY = compAngleY * RAD_TO_DEG;
//      printf("AngleX: %.2f AngleY: %.2f\n",AngleX,AngleY);
      // Euler angle to vector https://stackoverflow.com/questions/1568568/how-to-convert-euler-angles-to-directional-vector
      Rocket_vector[0] = cos(AngleX) * cos(AngleY);
      Rocket_vector[1] = sin(AngleX) * cos(AngleY);
      Rocket_vector[2] = sin(AngleY);
      a = Rocket_vector[0] * Z_unitvector[0] + Rocket_vector[1] * Z_unitvector[1] + Rocket_vector[2] * Z_unitvector[2];
      b = sqrt(Rocket_vector[0] * Rocket_vector[0] + Rocket_vector[1] * Rocket_vector[1] + Rocket_vector[2] * Rocket_vector[2]);
      c = sqrt(Z_unitvector[0] * Z_unitvector[0] + Z_unitvector[1] * Z_unitvector[1] + Z_unitvector[2] * Z_unitvector[2]);
      // final Rocket Angle; Z 축에?�� ?��마나 벗어?��?���??? 계산
      Rocket_Angle = acos(a / (b * c)) * RAD_TO_DEG;
      /*printf("Rocket Angle: %.2f\r\n", Rocket_Angle); // test code
      HAL_Delay(500);*/
      // �????��?���??? ?��?��?���??? 방향?�� ?��?�� ?��?��?��켜서 중력 �????��?�� ?���???
      accZ_raw = myAccelScaled.y * cos(AngleX) - myAccelScaled.z * sin(AngleX); // z y
      accY_raw = myAccelScaled.z * cos(AngleX) + myAccelScaled.y * sin(AngleX); // y z
      // �????��?���??? ?��?��?���??? 방향?�� ?��?�� ?��?��?��켜서 중력 �????��?�� ?���???
      accZ_rot = -accZ_raw * sin(AngleY) + accY_raw * cos(AngleY);
      /*printf("pure Z acc : %.2f\r\n", accZ_rot); // test code
      HAL_Delay(500);*/
      // velocity part
      Z_stack -= Z_velocity;             // delete pre-prev vel
      Z_velocity += accZ_rot * dt;       // calc Zvelocity
      Z_stack += Z_velocity;             // add updated vel
      Z_velmean = Z_stack / 2;           // mean of prev and
      Z_velgap = Z_velocity - Z_velmean; //
      // velocity part
      //printf("elapsed time: %lu\r\n", elapsed_time_ms);
      /*if (altitude >= 380)
      {
    	  Parachute = 0;
    	  printf("deploy parachute: altitude\r\n");
    	  HAL_Delay(10);
      }
      if (Rocket_Angle >= desiredAngle)
      {
    	  Parachute = 0;
    	  printf("deploy parachute: Desired Angle\r\n");
    	  HAL_Delay(10);
      }
      // if (Z_velgap <= 0.1f)
      // {
      //   Parachute = 0;
      //   printf("deploy parachute : Z velocity");
      // }
      /*if (elapsed_time_ms >= 5000) // 10 s
            {
              Parachute = 0;
              printf("deploy parachute : time out\r\n");
              HAL_Delay(10);
            }*/


      //free dy. memory...
      //memory free in here
	  Matrix_free_2dim(KF_z_acc_vel.Kalman_Gain, row_2);
	  Matrix_free_2dim(KF_z_acc_vel.estimate_state, row_2);
	  Matrix_free_2dim(KF_z_acc_vel.estimate_cov, row_2);

	  Matrix_free_2dim(KF_y_acc_vel.Kalman_Gain, row_2);
	  Matrix_free_2dim(KF_y_acc_vel.estimate_state, row_2);
	  Matrix_free_2dim(KF_y_acc_vel.estimate_cov, row_2);

	  Matrix_free_2dim(KF_x_acc_vel.Kalman_Gain, row_2);
	  Matrix_free_2dim(KF_x_acc_vel.estimate_state, row_2);
	  Matrix_free_2dim(KF_x_acc_vel.estimate_cov, row_2);


	  //to escape endless while.

	  if (iter_temp == iter) {
	  	break;
	  }
	  //upper iter 500 and less than 3 m
	  if ((average_pos_z[iter_temp][0][0] < 3) && (iter_temp > 500)) {
	  	break;
	  }
	  iter_temp++;
    }
    // deploy Parachute part^^7
    // sdcard part
    char buffer[100]; // Buffer to hold string
    // sdcard part
    sprintf(buffer,
            "altitude: %.2f Gyro: X=%.2f Y=%.2f Z=%.2f Accel: X=%.2f Y=%.2f Z=%.2f\r\n",
            altitude,
            myGyroScaled.x,
            myGyroScaled.y,
            myGyroScaled.z,
            myAccelScaled.x,
            myAccelScaled.y,
            myAccelScaled.z);
    if (fresult == FR_OK)
    {
      fresult = f_write(&fil, buffer, strlen(buffer), &bw); // Write the string to file
      f_sync(&fil);                                         // Ensure data is written and saved
    }



    // sdcard part
  }
  // sdcard part
  if (fresult == FR_OK)
  {
    fresult = f_close(&fil);
  }
  // sdcard part


  // free the memory
  free(est_cov_z_for_pos);

  Matrix_free(est_cov_z, row_2, iter);
  Matrix_free(est_state_z, row_3, iter);  // Free the allocated memory
  Matrix_free(average_pos_z, row_1, iter);
  Matrix_free(measu_acc_z, row_1, iter);

  Matrix_free(est_cov_y, row_2, iter);
  Matrix_free(est_state_y, row_3, iter);  // Free the allocated memory
  Matrix_free(measu_acc_y, row_1, iter);

  Matrix_free(est_cov_x, row_2, iter);
  Matrix_free(est_state_x, row_3, iter);  // Free the allocated memory
  Matrix_free(measu_acc_x, row_1, iter);

  Matrix_free_2dim(F_vec, row_2);
  Matrix_free_2dim(H_vec, row_1);
  Matrix_free_2dim(R_mat, row_1);

  Matrix_free(est_cov_thea_z, row_2, col_2);
  Matrix_free(est_state_thea_z, row_2, col_1);
  Matrix_free(measu_Ang_acc_z, row_1, iter);

  Matrix_free(est_cov_pi_y, row_2, col_2);
  Matrix_free(est_state_pi_y, row_2, col_1);
  Matrix_free(measu_Ang_acc_y, row_1, iter);

  Matrix_free(est_cov_rho_x, row_2, col_2);
  Matrix_free(est_state_rho_x, row_2, col_1);
  Matrix_free(measu_Ang_acc_x, row_1, iter);

  Matrix_free_2dim(F_vec_gyro, row_2);
  Matrix_free_2dim(H_vec_gyro, row_1);
  Matrix_free_2dim(R_mat_gyro, row_1);

  return 0;
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    second++;
    if (second == 5)
    {
//      printf("timeout ok\r\n");
      second = 0;
    }
  }
}

//function in Kalman Filter

float*** Matrix_gen(int row_n, int col_n, int iter_n) {
    float*** Ret_Matrix;
    Ret_Matrix = (float***)calloc(iter_n, sizeof(float**));

    if (!Ret_Matrix) {
        return NULL;
    }

    for (int i = 0; i < iter_n; i++) {
        Ret_Matrix[i] = (float**)calloc(row_n, sizeof(float*));
        if (!Ret_Matrix[i]) {
            for (int k = i - 1; k >= 0; k--) {
                free(Ret_Matrix[k]);
            }
            free(Ret_Matrix);
            return NULL;
        }

        for (int j = 0; j < row_n; j++) {
            Ret_Matrix[i][j] = (float*)calloc(col_n, sizeof(float));
            if (!Ret_Matrix[i][j]) {
                for (int k = j - 1; k >= 0; k--) {
                    free(Ret_Matrix[i][k]);
                }
                free(Ret_Matrix[i]);
                for (int k = i - 1; k >= 0; k--) {
                    free(Ret_Matrix[k]);
                }
                free(Ret_Matrix);
                return NULL;
            }
        }
    }

    return Ret_Matrix;
}

void Matrix_free(float*** matrix, int row_n, int iter_n) {
    for (int i = 0; i < iter_n; i++) {
        for (int j = 0; j < row_n; j++) {
            free(matrix[i][j]);
        }
        free(matrix[i]);
    }
    free(matrix);
}

//for 2dim
float** Matrix_gen_2dim(int row_n, int col_n) {
    float** Ret_Matrix = (float**)calloc(row_n, sizeof(float*));
    if (!Ret_Matrix) {
        return NULL;
    }

    for (int i = 0; i < row_n; i++) {
        Ret_Matrix[i] = (float*)calloc(col_n, sizeof(float));
        if (!Ret_Matrix[i]) {
            for (int k = i - 1; k >= 0; k--) {
                free(Ret_Matrix[k]);
            }
            free(Ret_Matrix);
            return NULL;
        }
    }

    return Ret_Matrix;
}

void Matrix_free_2dim(float** matrix, int row_n) {
    for (int i = 0; i < row_n; i++) {
        free(matrix[i]);
    }
    free(matrix);
}

// this code should be replace to just Average Filter _Daemin Kang deadline 0928
//
float average_filter(float* values) {
    // Doesn't have any Exception handling
    float sum = 0;
    // Calculate the initial sum
    for (int i = 0; i < 10; i++) {
        sum += values[i];
    }
    return sum / 10;
}
/*
*
// recursive MAF.
float recursive_average_filter(float* values, int index) {

    //should find is this code is right...
    float previous_average;
    if (index == 0)
        return values[0];

    previous_average = recursive_average_filter(values, index - 1);
    return (previous_average * index + values[index]) / (index + 1);
}
*/

//function for inv in 2 x 2
float** inv_2x2(float** matrix)
{
    float det = matrix[0][0] * matrix[1][1]
        - matrix[1][0] * matrix[0][1];

    if (det == 0)
    {
        return 0;
    }
    else
    {
        float temp = -matrix[1][0];
        matrix[1][0] = -1*matrix[0][1]/det;
        matrix[0][1] = temp/det;

        temp = matrix[1][1];
        matrix[1][1] = matrix[0][0]/det;
        matrix[0][0] = temp/det;
        return matrix;
    }

}

//function for transpose
float** Transpose(float** matrix, int row, int col)
{
    if (matrix == NULL || row <= 0 || col <= 0)
        return NULL;

    float** result = (float**)calloc(row, sizeof(float*));
    for (int i = 0; i < row; i++) {
        result[i] = (float*)calloc(col, sizeof(float));
    }

    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            result[j][i] = matrix[i][j];
        }
    }

    //need to free result memeroy in main function call
    return result;
}


//function for matrix adder with substracter
float** Matrix_adder(float** matrix_1, float** matrix_2, int row, int col, int option)  //option == 1 : adder / == 0 : substracter
{
    if (option == 1)
    {
        for (int i = 0; i < row; i++) {
            for (int j = 0; j < col; j++) {
                matrix_1[i][j] += matrix_2[i][j];
            }
        }
    }
    else if (option == 0)
    {
        for (int i = 0; i < row; i++) {
            for (int j = 0; j < col; j++) {
                matrix_1[i][j] -= matrix_2[i][j];
            }
        }
    }
    else
    //?��?��처리 추�??�� ?�� ?���?.

    return matrix_1;
}



//function for matrix multiplication
//c1==r2         need revision
float** Matrix_multiplicator(float** matrix_1, float** matrix_2,int r1, int c1, int r2, int c2){
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

//function for KF axis acceleration.
KF_return KF_alg(float** F, float** H, float** R, float** esti_state_prev, float** esti_cov_prev, float** measu_state, float sigma_w, float del_t) {
    KF_return KF;
    KF.estimate_cov = Matrix_gen_2dim(row_2, col_2);
    KF.estimate_state = Matrix_gen_2dim(row_2, col_1);
    KF.Kalman_Gain = Matrix_gen_2dim(row_2, col_1);

    //같�? ?��?��즈�?? �?�? 메모리�?? ?��?��?��?�� ?��?�� ?��?���? 최적?���? 진행?��?��?�� �? 같다_0915
    float** predict_cov = Matrix_gen_2dim(row_2, col_2);
    float** predict_state = Matrix_gen_2dim(row_2, col_1);
    float** transp_F = Matrix_gen_2dim(row_2, col_2);
    float** Q = Matrix_gen_2dim(row_2, col_2);
    float** transp_H = Matrix_gen_2dim(row_2, col_2);

    float** inv_term_1 = Matrix_gen_2dim(row_1,col_2);
    float** inv_term_2 = Matrix_gen_2dim(row_1, col_1);
    float** inv_term_tot = Matrix_gen_2dim(row_1, col_1);

    float** innov_1 = Matrix_gen_2dim(row_2, col_1);
    float** innov_tot = Matrix_gen_2dim(row_1, col_1);
    float** innov_Kal = Matrix_gen_2dim(row_2, col_1);

    float** renewal_cov_1 = Matrix_gen_2dim(row_2, col_2);


    //predict state
    predict_state = Matrix_multiplicator(F, esti_state_prev, row_2, col_2, row_2, col_1);

    //predict cov
    float sigma_sq = sigma_w * sigma_w;
    //set Q
    Q[0][0] = del_t*del_t* sigma_sq; Q[0][1] = del_t* sigma_sq; Q[1][0] = del_t * sigma_sq; Q[1][1] = sigma_sq;

    predict_cov = Matrix_multiplicator(F,esti_cov_prev, row_2, col_2, row_2, col_2);
    transp_F = Transpose(F,row_2,col_2);
    predict_cov = Matrix_multiplicator(predict_cov, transp_F, row_2, col_2, row_2, col_2);
    predict_cov = Matrix_adder(predict_cov,Q,row_2, col_2, add);

    //Kalman Gain
    inv_term_1 = Matrix_multiplicator(H, predict_cov, row_1, col_2, row_2, col_2);
    inv_term_2 = Matrix_multiplicator(inv_term_1, transp_H, row_1, col_2, row_2, col_1);
    KF.Kalman_Gain = Matrix_multiplicator(predict_cov, transp_H, row_2, col_2, row_2, col_1);
    **inv_term_tot = 1 / (**inv_term_2);

    KF.Kalman_Gain = Matrix_multiplicator(KF.Kalman_Gain, inv_term_tot, row_2, col_1, row_1, col_1);

    //Estimation of state
    innov_1 = Matrix_multiplicator(H, predict_state, row_1, col_2, row_2, col_1);
    innov_tot = Matrix_adder(measu_state, innov_1, row_1, col_1,substract);
    KF.estimate_state = Matrix_multiplicator(KF.Kalman_Gain, innov_tot, row_2, col_1, row_1, col_1);
    KF.estimate_state = Matrix_adder(predict_state ,KF.estimate_state, row_2, col_1, add);

    //Estimation of cov.
    renewal_cov_1 = Matrix_multiplicator(KF.Kalman_Gain,H,row_2, col_1, row_1, col_2);
    renewal_cov_1 = Matrix_multiplicator(renewal_cov_1, predict_cov, row_2, col_2, row_2, col_2);
    KF.estimate_cov = Matrix_adder(predict_cov, renewal_cov_1, row_2, col_2, substract);

    //free memory KF in main function.
    Matrix_free_2dim(renewal_cov_1,row_2);
    Matrix_free_2dim(innov_Kal, row_2);
    Matrix_free_2dim(innov_tot, row_1);
    Matrix_free_2dim(innov_1, row_2);

    Matrix_free_2dim(inv_term_tot, row_1);
    Matrix_free_2dim(inv_term_2, row_1);
    Matrix_free_2dim(inv_term_1, row_1);

    Matrix_free_2dim(transp_H, row_2);
    Matrix_free_2dim(Q, row_2);
    Matrix_free_2dim(transp_F, row_2);
    Matrix_free_2dim(predict_state, row_2);
    Matrix_free_2dim(predict_cov, row_2);

    return KF;//KF_return KF;   ?�� 구조체에 ???�� ?��?��?��?�� 문제�? ?�� 찾아보기.
}


//function for scalar KF
scalar_KF_return scalar_KF_alg(float F_scalar, float H_scalar, float R_scalar, float esti_state_prev, float esti_cov_prev, float measu_state, float sigma_w, float del_t) {
    scalar_KF_return scalar_KF;

    //estimation
    float sigma_sq = sigma_w * sigma_w;
    float Q = sigma_sq;
    float pred_state;
    float pred_cov;
    float inv_term;
    float innovation;
    float Kalman_term;


    //pred of state
    pred_state = F_scalar*esti_state_prev;

    //pred of cov
    pred_cov = F_scalar * esti_cov_prev;
    pred_cov = pred_cov * F_scalar;
    pred_cov = pred_cov + Q;

    //Kalman Gain
    scalar_KF.Kalman_Gain = pred_cov * H_scalar;
    inv_term = H_scalar*pred_cov;
    inv_term = inv_term * H_scalar;
    inv_term = inv_term + R_scalar;
    inv_term = 1 / inv_term;    //caution; integer type casting problem
    scalar_KF.Kalman_Gain = scalar_KF.Kalman_Gain * inv_term;

    //Estimation of state
    innovation = measu_state - esti_state_prev;
    Kalman_term = scalar_KF.Kalman_Gain * innovation;
    scalar_KF.estimate_state = pred_state + Kalman_term;

    //Estimation of cov.
    scalar_KF.estimate_cov = (1 - scalar_KF.estimate_state) * pred_cov;

    return scalar_KF;
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
