#include <stdio.h>
#include <stdlib.h>

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

//should be eliminate
typedef struct
{
    int x, y, z;
}myAccelScaled;
typedef struct
{
    int x, y, z;
}myGyroScaled;


//=======================

float*** Matrix_gen(int row_n, int col_n, int iter_n);      //인자는 [iter][row][col] 순
void Matrix_free(int*** matrix, int row_n, int iter_n);     

float** Matrix_gen_2dim(int row_n, int col_n);
void Matrix_free_2dim(float** matrix, int row_n);

float recursive_average_filter(float* values, int num_values, int index);
float** inv_2x2(float** matrix);
float** Transpose(float** matrix, int row, int col);
float** Matrix_adder(float** matrix_1, float** matrix_2, int row, int col, int option);
float** Matrix_multiplicator(float** matrix_1, float** matrix_2, int r1, int c1, int r2, int c2);
KF_return KF_alg(float**F, float** H, float** R, float** esti_state_prev, float** esti_cov_prev, float** measu_state, float sigma, float del_t);
scalar_KF_return scalar_KF_alg(float F_scalar, float H_scalar, float R_scalar, float esti_state_prev, float esti_cov_prev, float measu_state, float sigma_w, float del_t);

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

int main(void) {
    //measuremenmt pseudo
    float altitude=1, press=1, temp=1;
    myAccelScaled myAccelScaled;
    myGyroScaled myGyroScaled;

    //by simulation, we should get optimal del_t
    float del_t = 0.1;  //it means 100ms in operation, but in HW, we use ms scale   
                        //additionally, it should be revised into accurate del_t
    
    //iter counter
    int iter_temp = 1;  //begin num 1, because, we should use previous values

    //=========need Kalman Gain and COV of estimation       R, F, H / Q : made in KF algorithm
    float** R_mat = Matrix_gen_2dim(row_1, col_1);
    float** H_vec = Matrix_gen_2dim(row_1, col_2);
    float** F_vec = Matrix_gen_2dim(row_2, col_2);
    R_mat[0][0] = 0.124;
    H_vec[0][0] = 0;H_vec[0][1] = 1;
    F_vec[0][0] = 1; F_vec[0][1] = del_t; F_vec[1][0] = 0; F_vec[1][1] = 1;

    //for altitude, scalar KF
    float R_scalar = 0.05;  //by Average filter, var of R decrease into 1/10    //  in low power mode, its error var is 50cm 
    float H_scalar = 1, F_scalar = 1;

    //sigma_w is defined by differentation between 2-steps


    //for X,Y
    //KF to estimate vel, acc. this cause linear approximation of postion of x,y
    float*** measu_acc_x = Matrix_gen(row_1, col_1, iter);
    float*** measu_Ang_acc_x = Matrix_gen(row_1, col_1, iter);
    float*** est_state_x = Matrix_gen(row_3, col_1, iter);  //in est_x; 
    float*** est_cov_x = Matrix_gen(row_2, col_2, iter);

    float*** measu_acc_y = Matrix_gen(row_1, col_1, iter);
    float*** measu_Ang_acc_y = Matrix_gen(row_1, col_1, iter);
    float*** est_state_y = Matrix_gen(row_3, col_1, iter);
    float*** est_cov_y = Matrix_gen(row_2, col_2, iter);

    //for Z
    //KF to estimate vel, acc. to store
    //scalar KF to estimate location based on BMP180 measurement data
    float*** measu_acc_z = Matrix_gen(row_1, col_1, iter);
    float*** measu_Ang_acc_z = Matrix_gen(row_1, col_1, iter);      //just for storage

    float*** measu_pos_z = Matrix_gen(row_10, col_1, iter); 
    float*** average_pos_z = Matrix_gen(row_1, col_1, iter);     //for barometer, we should apply Average filter

    float*** est_state_z = Matrix_gen(row_3, col_1, iter);
    float*** est_cov_z = Matrix_gen(row_2, col_2, iter);     //for detecting location(scalar KF)
    float* est_cov_z_for_pos = (float*)malloc(iter*sizeof(float));      //cov for scalar KF

    float tmp_values[10] = 0;
    for (int i = 0; i < 10; i++) {


        //measure position 10 times
        //measu_pos_z[i][1][iter_temp] = 위치 측정 값코드;


        tmp_values[i] = measu_pos_z[0][i][0];
    }
    average_pos_z[0][0][0] = recursive_average_filter(tmp_values, row_10);


    //var for first meas of altitude
    int first_alt = 1;      //altitude standard
    
    //initializing
    est_state_x[0][0][0] = myAccelScaled.x;
    est_cov_x[0][0][0] = R_mat[0][0];
    est_state_y[0][0][0] = myAccelScaled.y;
    est_cov_y[0][0][0] = R_mat[0][0];
    est_state_z[0][0][0] = myAccelScaled.z;
    est_cov_z[0][0][0] = R_mat[0][0];
    //position initializing
    est_state_z[0][2][0] = average_pos_z[0][0][0];
    est_cov_z_for_pos[0] = R_scalar;

    //pseudo while
    while (1) {
        //init alititude        to calculate difference of altitude
        if (first_alt == 1) {
                    //iter, row, col
            measu_pos_z[1][0][0] = altitude;
            first_alt = 0;
        }

        //store acc to matrix
        measu_acc_x[iter_temp][0][0] = myAccelScaled.x;
        measu_acc_y[iter_temp][0][0] = myAccelScaled.y;
        measu_acc_z[iter_temp][0][0] = myAccelScaled.z;
        //store angular acc to matrix
        measu_Ang_acc_x[iter_temp][0][0] = myGyroScaled.x;
        measu_Ang_acc_y[iter_temp][0][0] = myGyroScaled.y;
        measu_Ang_acc_z[iter_temp][0][0] = myGyroScaled.z;

        //Average Filter to optimize noise      to get positon of z
        float tmp_values[10] = 0;
        for (int i = 0; i < 10;i++) {


            //measure position 10 times
            //measu_pos_z[i][1][iter_temp] = 위치 측정 값코드;


            tmp_values[i] = measu_pos_z[iter_temp][i][0];
        }
        average_pos_z[iter_temp][0][0] = recursive_average_filter(tmp_values,row_10);

        //other code
        //about...
        //KF, scalar KF
        //KF_return KF_alg(float** F, float** H, float** R, float** esti_state_prev, float** esti_cov_prev, float** measu_state, float sigma_w, float del_t)

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

        //여기서 동적할당이 필요하려나??!
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
        est_state_z[iter_temp][1][0] = KF_z_acc_vel.estimate_state[1][0];        //vel
        //est_state_y[iter_temp][2][0] = est_state_y[iter_temp - 1][2][0] + est_state_y[iter_temp - 1][1][0] * del_t + est_state_y[iter_temp - 1][0][0] * del_t * del_t / 2;

        //scalar KF for position
        //scalar_KF_return scalar_KF_alg(float F_scalar, float H_scalar, float R_scalar, float esti_state_prev, float esti_cov_prev, float measu_state, float sigma_w, float del_t);
        float sigam_w_pos = average_pos_z[iter_temp][0][0] - average_pos_z[iter_temp-1][0][0];
        KF_z_pos = scalar_KF_alg(F_scalar, H_scalar, R_scalar, est_state_z[iter_temp-1][2][0], est_cov_z_for_pos[iter_temp-1], average_pos_z[iter_temp][0][0], sigam_w_pos, del_t);
        est_state_z[iter_temp][2][0] = KF_z_pos.estimate_state;
        est_cov_z_for_pos[iter_temp] = KF_z_pos.estimate_cov;
        

        //to escape endless while.
        if (iter_temp == iter) {
            break;
        }
        //upper iter 500 and less than 3 m
        if ((average_pos_z[iter_temp][0][0] < 3) && (iter_temp > 500)) {
            break;
        }

        //add deployment of parachute code in here
        //...




        //memory free in here
        iter_temp++;
    }


    /*
    for (int i = 0; i < iter; i++) {
        for (int j = 0; j < row; j++) {  // Corrected order here
            for (int k = 0; k < col; k++) {  // Corrected order here
                printf("%d ", matrix[i][j][k]);  // Added space for better clarity
            }
            printf("\n");
        }
        printf("\n\n");
    }
    */

    // free the memory
    free(est_cov_z_for_pos);

    Matrix_free(est_cov_z, row_2, iter);
    Matrix_free(est_state_z, row_3, iter);  // Free the allocated memory
    Matrix_free(average_pos_z, row_1, iter);
    Matrix_free(measu_pos_z, row_10, iter);
    Matrix_free(measu_Ang_acc_z, row_1, iter);
    Matrix_free(measu_acc_z, row_1, iter);

    Matrix_free(est_cov_y, row_2, iter);
    Matrix_free(est_state_y, row_3, iter);  // Free the allocated memory
    Matrix_free(measu_Ang_acc_y, row_1, iter);
    Matrix_free(measu_acc_y, row_1, iter);

    Matrix_free(est_cov_x, row_2, iter);
    Matrix_free(est_state_x, row_3, iter);  // Free the allocated memory
    Matrix_free(measu_Ang_acc_x, row_1, iter);
    Matrix_free(measu_acc_x, row_1, iter);
    return 0;
}

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

// recursive MAF.
float recursive_average_filter(float* values, int index) {

    //should find is this code is right...
    float previous_average;
    if (index == 0)
        return values[0];

    previous_average = recursive_average_filter(values, index - 1);
    return (previous_average * index + values[index]) / (index + 1);
}

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

    for (int i = 0; i < row; i++) {
        free(matrix[i]);
    }
    free(matrix);


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

    for (int i = 0; i < row; i++) {
        free(matrix_2[i]);
    }
    free(matrix_2);


    return matrix_1;
}



//function for matrix multiplication
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

    return KF;
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
