#include<stdio.h>
#include<stdlib.h>
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

float Q_mat[row_2][col_2] = { 0 };
float inv_term_sub[row_1][col_2] = { 0 };
float inv_term[row_1][col_1] = { 0 };
float HX[row_1][col_1] = { 0 };
float innov[row_2][col_1] = { 0 };
float KH[row_2][col_2] = { 0 };


//fucntion define.
void disp_mat_2dim(float* mat[], int row, int col);
void disp_mat(float mat[][row_2][col_2], int iter_inner, int row, int col);

float average_filter(float values[], int ind);

float** Transpose(float* matrix[], int row, int col);

float** Matrix_adder(float* matrix_1[], float* matrix_2[], int row, int col, int option);

float** Matrix_multiplicator(float* matrix_1[], float* matrix_2[], int r1, int c1, int r2, int c2);

void KF_alg(float* esti_state_prev[], float* esti_cov_prev[], float* measu_state[], float sigma, float* esti_state_out[], float* esti_cov_out[]);

void scalar_KF(float* prev_alt, float* prev_alt_cov, float measu_alt, float* esti_alt, float* esti_alt_cov);

int main()
{
	/*
		//this proj is for testing KF
	for (int i = 0; i < iter_temp; i++) {
		for(int j = 0; j < row_2; j++) {
			for(int k = 0; k < col_2; k++) {
				test_mat[i][j][k] = (float)(i + j + k);
			}
		}
	}
	
	float aver = average_filter(for_average_Arr, row_10);
	//printf("%.2f\n", aver);
	
	//=========================================================================================
	//caution : likewise lower codes, we should make some lines for 3-dim -> 2-dim's downsizing
	//=========================================================================================
	float* transpose_row[row_2];
	// Assign the pointers to each element of transpose_row.
	for (int i = 0; i < row_2; i++) {
		transpose_row[i] = test_mat[0][i];
	}
	//=========================================================================================

	printf("[Transpose]\n");
	float** transpose_mat = Transpose(transpose_row, row_2, col_2);
	//printf("%.2f\n", transpose_mat[0][0]);
	disp_mat_2dim(transpose_mat, row_2, col_2);
	printf("[Adder]\n");
	Matrix_adder(transpose_mat, transpose_row, row_2, col_2, add);
	disp_mat_2dim(transpose_mat, row_2, col_2);
	Matrix_adder(transpose_mat, transpose_row, row_2, col_2, substract);
	disp_mat_2dim(transpose_mat, row_2, col_2);
	disp_mat_2dim(transpose_row, row_2, col_2);
	printf("[Mult]\n");
	float** Mult_mat = Matrix_multiplicator(transpose_mat, transpose_row, row_2, col_2, row_2, col_2);
	disp_mat_2dim(Mult_mat, row_2, col_2);

	//linear algebra function testing end...
	//to simplify our calc, by each steps, we should make our problem in only matrices.



	//disp_mat(test_mat, iter_temp, row_2, col_2);
	*/
	//we should define almost of our matrices as global var
	printf("[F_mat]\n");
	disp_mat_2dim(F_mat, row_2, col_2);
	printf("[H_mat]\n");
	disp_mat_2dim(H_mat, row_1, col_2);
	printf("[R_mat]\n");
	disp_mat_2dim(R_mat, row_1, col_1);

	//testing transpose
	printf("[Transpose_H_mat]\n");
	float** trans_H = Transpose(H_mat, row_1, col_2);
	disp_mat_2dim(trans_H, row_2, col_1);
	printf("[Transpose_F_mat]\n");
	float** trans_F = Transpose(F_mat, row_2, col_2);
	disp_mat_2dim(trans_F, row_2, col_2);
	

	//testing multiplication
	printf("[Mult_H_with_F]\n");
	float** mult_HF = Matrix_multiplicator(H_mat, F_mat, row_1, col_2, row_2, col_2);
	disp_mat_2dim(mult_HF, row_1, col_2);

	//testing adder
	printf("[Add]\n");
	float** adder = Matrix_adder(H_mat, H_mat, row_1, col_2, add);
	disp_mat_2dim(adder,row_1, col_2);
	printf("[Substract]\n");
	adder = Matrix_adder(H_mat, H_mat, row_1, col_2, substract);
	disp_mat_2dim(adder, row_1, col_2);


	//eliminate gravity comp.
	float gravity_x = 0.0f;
	float gravity_y = 0.0f;
	float gravity_z = 0.0f;
	//define lambda
	float lambda = 0.0f;
	float standard_alt;
	int iter_temp = 1;  //begin num 1, because, we should use previous values
	float sigma = 0.0f;

	float tmp_values[10] = 0;
	for (int i = 0; i < 10; i++) {
		altitude = readTrueAltitude(0);		//by 1ms
		tmp_values[i] = altitude;
	}
	standard_alt = average_filter(tmp_values,10);


	//Kalman Filter Algorithm testing..?
	while (1) {
		/*
		// bmp180 part
		temp = readTrueTemp();
		press = readTruePress(0);
		altitude = readTrueAltitude(0);
		// HAL_Delay(10);
		// printf("%.2f\r\n", altitude);
		// bmp180 part

		// mpu6050 part
		MPU6050_Get_Accel_Scale(&myAccelScaled);
		MPU6050_Get_Gyro_Scale(&myGyroScaled);
		myGyroScaled.x;
		myGyroScaled.y;
		myGyroScaled.z;

		*/
		alt_z_measu[0] = alt_z_measu[1];
		for (int i = 0; i < 10; i++) {
			altitude = readTrueAltitude(0);		//by 1ms
			tmp_values[i] = altitude;
		}
		alt_z_measu[1] = average_filter(tmp_values, 10) - standard_alt;

		//Gyro KF
		MPU6050_Get_Gyro_Scale(&myGyroScaled);
		//gyro x 
		gyro_x_measu[0][0] = myGyroScaled.x;
		sigma = gyro_x_measu[0][0] - prev_gyro_x_measu[0][0];
		KF_alg(prev_gyro_x_state, prev_gyro_x_cov, gyro_x_measu, sigma, gyro_x_state, gyro_x_cov);
		prev_gyro_x_measu[0][0] = gyro_x_measu[0][0];
		//gyro y 
		gyro_y_measu[0][0] = myGyroScaled.y;
		sigma = gyro_y_measu[0][0] - prev_gyro_y_measu[0][0];
		KF_alg(prev_gyro_y_state, prev_gyro_y_cov, gyro_y_measu, sigma, gyro_y_state, gyro_y_cov);
		prev_gyro_y_measu[0][0] = gyro_y_measu[0][0];
		//gyro z 
		gyro_z_measu[0][0] = myGyroScaled.z;
		sigma = gyro_z_measu[0][0] - prev_gyro_z_measu[0][0];
		KF_alg(prev_gyro_z_state, prev_gyro_z_cov, gyro_z_measu, sigma, gyro_z_state, gyro_z_cov);
		prev_gyro_z_measu[0][0] = gyro_z_measu[0][0];

		printf("[ GYRO x ] : %.2f\n[ GYRO y ] : %.2f\n[ GYRO z ] : %.2f\n\n", gyro_x_state[1][0], gyro_y_state[1][0], gyro_z_state[1][0]);
		//calc Ang

		//vel KF with pos estimate
		
	}
	return 0;
}

void disp_mat_2dim(float* mat[], int row, int col) {
	//printf("///error_testing///\n");
	for (int j = 0; j < row; j++) {
		for (int k = 0; k < col; k++) {
			printf("%.2f ", mat[j][k]);//여기서 문제; fixed : 선언할때의 문제.
		}
		printf("\n");
	}
}

void disp_mat(float mat[][row_2][col_2], int iter_inner, int row, int col){
	for (int i = 0; i < iter_inner; i++) {
		for (int j = 0; j < row; j++) {
			for (int k = 0; k < col; k++) {
				printf("%.2f ", mat[i][j][k]);
			}
			printf("\n");
		}
		printf("\n\n");
	}
}

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
		//float** Q_mat = Matrix_multiplicator(process_noise, trans_process_noise, row_2, col_1, row_1, col_2);
	
	Q_mat[0][0] = process_noise[0][0] * trans_process_noise[0][0];
	Q_mat[0][1] = process_noise[0][0] * trans_process_noise[0][1];
	Q_mat[1][0] = process_noise[1][0] * trans_process_noise[0][0];
	Q_mat[1][1] = process_noise[1][0] * trans_process_noise[0][1];

	//pred_state
		//pred_state = Matrix_multiplicator(F_mat, esti_state_prev, row_2, col_2, row_2, col_1);
	pred_state[0][0] = F_mat[0][0] * esti_state_prev[0][0] + F_mat[0][1] * esti_state_prev[1][0];
	pred_state[1][0] = F_mat[1][0] * esti_state_prev[0][0] + F_mat[1][1] * esti_state_prev[1][0];

	//pred_cov
		//pred_cov = Matrix_multiplicator(F_mat, esti_cov_prev, row_2, col_2, row_2, col_2);
	pred_cov[0][0] = F_mat[0][0] * esti_cov_prev[0][0] + F_mat[0][1] * esti_cov_prev[1][0];
	pred_cov[0][1] = F_mat[0][0] * esti_cov_prev[0][1] + F_mat[0][1] * esti_cov_prev[1][1];
	pred_cov[1][0] = F_mat[1][0] * esti_cov_prev[0][0] + F_mat[1][1] * esti_cov_prev[1][0];
	pred_cov[1][1] = F_mat[1][0] * esti_cov_prev[0][1] + F_mat[1][1] * esti_cov_prev[1][1];
		//pred_cov = Matrix_multiplicator(pred_cov, trans_F, row_2, col_2, row_2, col_2);
	pred_cov[0][0] = pred_cov[0][0] * trans_F[0][0] + pred_cov[0][1] * trans_F[1][0];
	pred_cov[0][1] = pred_cov[0][0] * trans_F[0][1] + pred_cov[0][1] * trans_F[1][1];
	pred_cov[1][0] = pred_cov[1][0] * trans_F[0][0] + pred_cov[1][1] * trans_F[1][0];
	pred_cov[1][1] = pred_cov[1][0] * trans_F[0][1] + pred_cov[1][1] * trans_F[1][1];
		//pred_cov = Matrix_adder(pred_cov, Q_mat, row_2, col_2, add);
	pred_cov[0][0] = pred_cov[0][0] + Q_mat[0][0];
	pred_cov[0][1] = pred_cov[0][1] + Q_mat[0][1];
	pred_cov[1][0] = pred_cov[1][0] + Q_mat[1][0];
	pred_cov[1][1] = pred_cov[1][1] + Q_mat[1][1];

	//Kalman_Gain
	
		//= Matrix_multiplicator(H_mat, pred_cov, row_1, col_2, row_2, col_2);
	inv_term_sub[0][0] = H_mat[0][0] * pred_cov[0][0] + H_mat[0][1] * pred_cov[1][0];
	inv_term_sub[0][1] = H_mat[0][0] * pred_cov[0][1] + H_mat[0][1] * pred_cov[1][1];

	
		//= Matrix_multiplicator(inv_term_sub, trans_H, row_1, col_2, row_2, col_1);
	inv_term[0][0] = inv_term_sub[0][0] * trans_H[0][0] + inv_term_sub[0][1] * trans_H[1][0];
		//inv_term = Matrix_adder(inv_term,R_mat, row_1, col_1, add);
		//inv_term[0][0] = 1 / (inv_term[0][0]);
	inv_term[0][0] = 1 / (inv_term[0][0] + R_mat[0][0]);
	
	//2x1 matrix.
		//Kalman_Gain = Matrix_multiplicator(pred_cov, trans_H, row_2, col_2, row_2, col_1);
	Kalman_Gain[0][0] = pred_cov[0][0] * trans_H[0][0] + pred_cov[0][1] * trans_H[1][0];
	Kalman_Gain[1][0] = pred_cov[1][0] * trans_H[0][0] + pred_cov[1][1] * trans_H[1][0];
		//Kalman_Gain = Matrix_multiplicator(Kalman_Gain, inv_term, row_2, col_1, row_1, col_1);
	Kalman_Gain[0][0] = Kalman_Gain[0][0] * inv_term[0][0];
	Kalman_Gain[1][0] = Kalman_Gain[1][0] * inv_term[0][0];





	//esti_state
	//reuse inv_term to calc innovation term.
	
		//Matrix_multiplicator(H_mat, pred_state, row_1, col_2, row_2, col_1);
	HX[0][0] = H_mat[0][0] * pred_state[0][0] + H_mat[0][1] * pred_state[1][0];
		//inv_term = Matrix_adder(measu_state, HX, row_1, col_1, substract);
	inv_term[0][0] = measu_state[0][0] - HX[0][0];
	
		//= Matrix_multiplicator(Kalman_Gain, inv_term, row_2, col_1, row_1, col_1);
	innov[0][0] = Kalman_Gain[0][0] * inv_term[0][0];
	innov[1][0] = Kalman_Gain[1][0] * inv_term[0][0];
		//esti_state_out = Matrix_adder(pred_state, innov, row_2, col_1, add);
	esti_state_out[0][0] = pred_state[0][0] + innov[0][0];
	esti_state_out[1][0] = pred_state[1][0] + innov[1][0];

	//esti_cov
	
		//= Matrix_multiplicator(Kalman_Gain, H_mat, row_2, col_1, row_1, col_2);
	KH[0][0] = Kalman_Gain[0][0] * H_mat[0][0];
	KH[0][1] = Kalman_Gain[0][0] * H_mat[0][1];
	KH[1][0] = Kalman_Gain[1][0] * H_mat[0][0];
	KH[1][1] = Kalman_Gain[1][0] * H_mat[0][1];
		//KH = Matrix_multiplicator(KH, pred_cov, row_2, col_2, row_2, col_2);
	KH[0][0] = KH[0][0] * pred_cov[0][0] + KH[0][1] * pred_cov[1][0];
	KH[0][1] = KH[0][0] * pred_cov[0][1] + KH[0][1] * pred_cov[1][1];
	KH[1][0] = KH[1][0] * pred_cov[0][0] + KH[1][1] * pred_cov[1][0];
	KH[1][1] = KH[1][0] * pred_cov[0][1] + KH[1][1] * pred_cov[1][1];
		//esti_cov_out = Matrix_adder(pred_cov, KH, row_2, col_2, substract);
	esti_cov_out[0][0] = pred_cov[0][0] - KH[0][0];
	esti_cov_out[0][1] = pred_cov[0][1] - KH[0][1];
	esti_cov_out[1][0] = pred_cov[1][0] - KH[1][0];
	esti_cov_out[1][1] = pred_cov[1][1] - KH[1][1];

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
