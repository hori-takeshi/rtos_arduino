#ifndef _R2CA_APP_H_
#define _R2CA_APP_H_

#define R2CA_NUM_TASK 0

/*
 *  Use Serial3
 */
#define R2CA_USE_SERIAL3

#define  R2CA_TASK2_LOOP_PRI  4 //Web
#define  R2CA_TASK3_LOOP_PRI  3 //LED
#define  R2CA_TASK4_LOOP_PRI  7 //TFT
#define  R2CA_TASK5_LOOP_PRI  6 //Processing

#define R2CA_ENABLE_PROFILING
#define R2CA_PROFILING_CYC_MS  1

#endif /* _R2CA_APP_H_ */
