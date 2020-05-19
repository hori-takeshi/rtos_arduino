#include "r2ca_app.h"

/*
 *  Max number of task
 */
#define R2CA_NUM_TASK_MAX  5

/*
 *  Check number of task
 */
#if R2CA_NUM_TASK > R2CA_NUM_TASK_MAX
#error The number of Task is over!
#endif /* R2CA_NUM_TASK > R2CA_NUM_TASK_MAX */

/*
 *  Idle task loop count per 10ms at idle state
 */
#define IDLE_TASK_IDLE_LOOP_10MS 42178

/*
 * Default Macro  
 */

/*
 *  Priotiry
 */
#ifndef R2CA_MAINTASK_PRI
#define  R2CA_MAINTASK_PRI  5
#endif /*  R2CA_MAINTASK_SETUP_PRI */

#ifndef R2CA_TASK1_PRI
#define  R2CA_TASK1_PRI  5
#endif /*  R2CA_TASK1_SETUP_PRI */

#ifndef R2CA_TASK2_PRI
#define  R2CA_TASK2_PRI  5
#endif /*  R2CA_TASK2_SETUP_PRI */

#ifndef R2CA_TASK3_PRI
#define  R2CA_TASK3_PRI  5
#endif /*  R2CA_TASK3_SETUP_PRI */

#ifndef R2CA_TASK4_PRI
#define  R2CA_TASK4_PRI  5
#endif /*  R2CA_TASK4_SETUP_PRI */

#ifndef R2CA_TASK5_PRI
#define  R2CA_TASK5_PRI  5
#endif /*  R2CA_TASK3_SETUP_PRI */

/*
 *  Stack Size
 */
#ifndef R2CA_MAINTASK_STACK_SIZE
#define R2CA_MAINTASK_STACK_SIZE 2048
#endif  /* R2CA_MAINTASK_STACK_SIZE */

#ifndef R2CA_TASK1_STACK_SIZE
#define R2CA_TASK1_STACK_SIZE 1024
#endif  /* R2CA_TASK1_STACK_SIZE */

#ifndef R2CA_TASK2_STACK_SIZE
#define R2CA_TASK2_STACK_SIZE 1024
#endif  /* R2CA_TASK2_STACK_SIZE */

#ifndef R2CA_TASK3_STACK_SIZE
#define R2CA_TASK3_STACK_SIZE 1024
#endif  /* R2CA_TASK3_STACK_SIZE */

#ifndef R2CA_TASK4_STACK_SIZE
#define R2CA_TASK4_STACK_SIZE 1024
#endif  /* R2CA_TASK4_STACK_SIZE */

#ifndef R2CA_TASK5_STACK_SIZE
#define R2CA_TASK5_STACK_SIZE 1024
#endif  /* R2CA_TASK5_STACK_SIZE */


/*
 *  Round Robin Scheduling cycle
 */
#ifndef R2CA_RR_SCHEDULE_CYCLE
#define R2CA_RR_SCHEDULE_CYCLE 1
#endif /* R2CA_RR_SCHEDULE_CYCLE */

#ifndef R2CA_RR_SCHEDULE_PRI
#define R2CA_RR_SCHEDULE_PRI 0x0000
#endif  /* R2CA_RR_SCHEDULE_PRI */
  
#ifdef __cplusplus
extern "C" {
#endif

extern void r2ca_init(intptr_t exinf);
extern void r2ca_maintask(intptr_t exinf);
extern void r2ca_task1(intptr_t exinf);
extern void r2ca_task2(intptr_t exinf);
extern void r2ca_task3(intptr_t exinf);
extern void r2ca_task4(intptr_t exinf);
extern void r2ca_task5(intptr_t exinf);
extern void r2ca_CycHandler(intptr_t exinf);

extern void r2ca_USB_Handler(void);
extern void r2ca_SERCOM0_Handler(void);
extern void r2ca_SERCOM2_Handler(void);
extern void r2ca_SERCOM3_Handler(void);
extern void r2ca_SERCOM4_Handler(void);
extern void r2ca_SERCOM5_Handler(void);
extern void r2ca_EIC_Handler(void);
extern void r2ca_TC5_Handler(void);
extern void r2ca_RTC_Handler(void);

#ifdef R2CA_ENABLE_PROFILING
extern void r2ca_idle_task(intptr_t exinf);
extern volatile uint32_t r2ca_idle_result;
extern volatile uint32_t r2ca_isr_result;
extern volatile uint32_t r2ca_dispatch_result;
extern volatile uint32_t r2ca_timer_isr_result;
extern volatile uint32_t r2ca_usb_isr_result;
extern volatile uint32_t r2ca_sercom0_isr_result;
extern volatile uint32_t r2ca_sercom4_isr_result;
extern volatile uint32_t r2ca_sercom5_isr_result;
extern volatile uint32_t r2ca_eic_isr_result;
extern volatile uint32_t r2ca_tc5_isr_result;
extern volatile uint32_t r2ca_rtc_isr_result;

#ifdef R2CA_USE_SERIAL3
extern volatile uint32_t r2ca_sercom3_isr_result;
#endif /* R2CA_USE_SERIAL3 */
  
#endif /* R2CA_ENABLE_PROFILING */

#define WIRE_ENTER_CRITICAL wai_sem(WIRE_SEM);
#define WIRE_LEAVE_CRITICAL sig_sem(WIRE_SEM);

#ifdef __cplusplus
}
#endif
