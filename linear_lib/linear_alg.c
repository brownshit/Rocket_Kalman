#include<stdio.h>
#include<stdlib.h>
#define iter_temp	10
#define iter		1000
#define row_1		1
#define col_1		1
#define row_2		2
#define col_2		2
#define row_3 3
#define col_3 3
#define row_10 10
#define iter 1000
#define add 1
#define substract 0
#define math_pi 3.1415926535
#define gravity_const 9.81
//do not Dynamic allocation in KF algorithms... user heap is extra small

//define our matrices as global array.
float test_mat[iter_temp][row_2][col_2] = { 0 };
float for_average_Arr[row_10] = {1,2,1,2,1,2,1,2,1,2};

//fucntion define.
void disp_mat_2dim(float* mat[], int row, int col);
void disp_mat(float mat[][row_2][col_2], int iter_inner, int row, int col);

float average_filter(float values[], int ind);

float** Transpose(float* matrix[], int row, int col);

void Matrix_adder(float* matrix_1[], float* matrix_2[], int row, int col, int option);

float** Matrix_multiplicator(float* matrix_1[], float* matrix_2[], int r1, int c1, int r2, int c2);

int main()
{
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

	return 0;
}

void disp_mat_2dim(float* mat[], int row, int col) {
	
	for (int j = 0; j < row; j++) {
		for (int k = 0; k < col; k++) {
			printf("%.2f ", mat[j][k]);
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

	float** result = (float**)calloc(row, sizeof(float*));
	for (int i = 0; i < row; i++) {
		result[i] = (float*)calloc(col, sizeof(float));
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
void Matrix_adder(float* matrix_1[], float* matrix_2[], int row, int col, int option)
{
	if (option == 1) {//add
		for (int i = 0; i < row; i++) {
			for (int j = 0; j < col; j++) {
				matrix_1[i][j] += matrix_2[i][j];
			}
		}
	}
	else if (option == 0) {//substract
		for (int i = 0; i < row; i++) {
			for (int j = 0; j < col; j++) {
				matrix_1[i][j] -= matrix_2[i][j];
			}
		}
	}
	else {
		//warning
		printf("[adder function error]\n");
	}
}

float** Matrix_multiplicator(float* matrix_1[], float* matrix_2[], int r1, int c1, int r2, int c2) {

	if (c1 != r2) {// exception dealer
		printf("[Matrix Mult function has Error...]\n");
		return NULL;
	}
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
