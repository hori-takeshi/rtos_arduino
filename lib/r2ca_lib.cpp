#include "r2ca.h"
#include "r2ca_lib.h"

extern "C" {
extern void yield(void);
extern void r2ca_ena_int(uint_t intno);
extern void r2ca_dis_int(uint_t intno);
}

#ifdef R2CA_ENABLE_PROFILING
volatile uint32_t r2ca_idle_result;
volatile uint32_t r2ca_isr_result;
volatile uint32_t r2ca_dispatch_result;
volatile uint32_t r2ca_timer_isr_result;
volatile uint32_t r2ca_usb_isr_result;
volatile uint32_t r2ca_sercom0_isr_result;
volatile uint32_t r2ca_sercom2_isr_result;
volatile uint32_t r2ca_sercom3_isr_result;
volatile uint32_t r2ca_sercom4_isr_result;
volatile uint32_t r2ca_sercom5_isr_result;
volatile uint32_t r2ca_eic_isr_result;
volatile uint32_t r2ca_tc5_isr_result;
volatile uint32_t r2ca_rtc_isr_result;

static volatile uint32_t r2ca_idle_cnt;
static volatile uint32_t r2ca_isr_cnt;
static volatile uint32_t r2ca_dispatch_cnt;
static volatile uint32_t r2ca_timer_isr_cnt;
static volatile uint32_t r2ca_usb_isr_cnt;
static volatile uint32_t r2ca_sercom0_isr_cnt;
static volatile uint32_t r2ca_sercom2_isr_cnt;
static volatile uint32_t r2ca_sercom3_isr_cnt;
static volatile uint32_t r2ca_sercom4_isr_cnt;
static volatile uint32_t r2ca_sercom5_isr_cnt;
static volatile uint32_t r2ca_eic_isr_cnt;
static volatile uint32_t r2ca_tc5_isr_cnt;
static volatile uint32_t r2ca_rtc_isr_cnt;     
static volatile uint32_t r2ca_profiling_cyccnt;
#endif /* R2CA_ENABLE_PROFILING */

Inline void
r2ca_di(void){
	Asm("cpsid f":::"memory");
}

/*
 *  FAULTMASK‚ÌƒNƒŠƒA
 */
Inline void
r2ca_ei(void){
	Asm("cpsie f":::"memory");
}

void
r2ca_init(intptr_t exinf)
{
	init(); //wiring.c
}

void
r2ca_ena_int(uint_t intno) {
	if (ena_int(intno + 16) != E_OK) {
		while(1); //ToDo
	}
}

void
r2ca_dis_int(uint_t intno) {
	if (dis_int(intno + 16) != E_OK) {
		while(1); //ToDo
	}
}

extern const DeviceVectors exception_table;

static uint32_t time_slice = 0;

/*
 *  Cyclic Handler
 */
void
r2ca_CycHandler(intptr_t exinf)
{    
	int i;
	if (++time_slice == R2CA_RR_SCHEDULE_CYCLE) {
		time_slice = 0;
		for(i = 1; i < 16; i++){
			if (R2CA_RR_SCHEDULE_PRI & (1 << i)) {
				irot_rdq(i);
			}
		}
	}
#ifdef R2CA_ENABLE_PROFILING    
	r2ca_di();
	r2ca_isr_cnt++;
	r2ca_timer_isr_cnt++;
	r2ca_profiling_cyccnt++;
	if(r2ca_profiling_cyccnt == R2CA_PROFILING_CYC_MS){
		r2ca_idle_result=        r2ca_idle_cnt;
		r2ca_isr_result=         r2ca_isr_cnt;
		r2ca_dispatch_result=    r2ca_dispatch_cnt;
		r2ca_timer_isr_result=   r2ca_timer_isr_cnt;
		r2ca_usb_isr_result=     r2ca_usb_isr_cnt;
		r2ca_sercom0_isr_result= r2ca_sercom0_isr_cnt;
		r2ca_sercom2_isr_result= r2ca_sercom2_isr_cnt;        
		r2ca_sercom3_isr_result= r2ca_sercom3_isr_cnt;
		r2ca_sercom4_isr_result= r2ca_sercom4_isr_cnt;
		r2ca_sercom5_isr_result= r2ca_sercom5_isr_cnt;
		r2ca_eic_isr_result=     r2ca_eic_isr_cnt;
		r2ca_tc5_isr_result=     r2ca_tc5_isr_cnt;
		r2ca_rtc_isr_result=     r2ca_rtc_isr_cnt;             
		r2ca_idle_cnt = 0;
		r2ca_isr_cnt = 0;
		r2ca_dispatch_cnt = 0;
		r2ca_timer_isr_cnt = 0;
		r2ca_usb_isr_cnt = 0;
		r2ca_sercom0_isr_cnt = 0;
		r2ca_sercom2_isr_cnt = 0;        
		r2ca_sercom3_isr_cnt = 0;
		r2ca_sercom4_isr_cnt = 0;
		r2ca_sercom5_isr_cnt = 0;
		r2ca_eic_isr_cnt = 0;
		r2ca_tc5_isr_cnt = 0;
		r2ca_rtc_isr_cnt = 0;
		r2ca_profiling_cyccnt = 0;
	}
	r2ca_ei();
#endif /* R2CA_ENABLE_PROFILING */    
	((void(*)(void))(exception_table.pfnSysTick_Handler))();        
}

/*
 *  Interrupt Handler
 */
void
r2ca_USB_Handler(void)
{
#ifdef R2CA_ENABLE_PROFILING
	r2ca_di();
	r2ca_usb_isr_cnt++;
	r2ca_isr_cnt++;
	r2ca_ei();
#endif /* R2CA_ENABLE_PROFILING */    
	((void(*)(void))(exception_table.pfnUSB_Handler))();
}

void
r2ca_SERCOM0_Handler(void)
{
#ifdef R2CA_ENABLE_PROFILING
	r2ca_di();
	r2ca_sercom0_isr_cnt++;
	r2ca_isr_cnt++;
	r2ca_ei();
#endif /* R2CA_ENABLE_PROFILING */    
	((void(*)(void))(exception_table.pfnSERCOM0_Handler))();
}

#ifdef R2CA_USE_SERIAL3
#define PIN_SERIAL3_RX 49  /* D5 */
#define PIN_SERIAL3_TX 48  /* D4 */

Uart Serial3(&sercom2, PIN_SERIAL3_RX, PIN_SERIAL3_TX);

void
r2ca_SERCOM2_Handler(void)
{
#ifdef R2CA_ENABLE_PROFILING
	r2ca_di();
	r2ca_sercom2_isr_cnt++;
	r2ca_isr_cnt++;
	r2ca_ei();
#endif /* R2CA_ENABLE_PROFILING */
	Serial3.IrqHandler();
}
#endif /* R2CA_USE_SERIAL3 */

void
r2ca_SERCOM3_Handler(void)
{
#ifdef R2CA_ENABLE_PROFILING
	r2ca_di();
	r2ca_sercom3_isr_cnt++;
	r2ca_isr_cnt++;
	r2ca_ei();
#endif /* R2CA_ENABLE_PROFILING */    
	((void(*)(void))(exception_table.pfnSERCOM3_Handler))();
}

void
r2ca_SERCOM4_Handler(void)
{
#ifdef R2CA_ENABLE_PROFILING
	r2ca_di();
	r2ca_sercom4_isr_cnt++;
	r2ca_isr_cnt++;
	r2ca_ei();
#endif /* R2CA_ENABLE_PROFILING */    
	((void(*)(void))(exception_table.pfnSERCOM4_Handler))();
}

#ifdef TOPPERS_USE_ARDUINO_SERIAL
void
r2ca_SERCOM5_Handler(void)
{
#ifdef R2CA_ENABLE_PROFILING
	r2ca_di();
	r2ca_sercom5_isr_cnt++;
	r2ca_isr_cnt++;
	r2ca_ei();
#endif /* R2CA_ENABLE_PROFILING */    
	((void(*)(void))(exception_table.pfnSERCOM5_Handler))();
}
#endif /* TOPPERS_USE_ARDUINO_SERIAL */

void
r2ca_EIC_Handler(void)
{
#ifdef R2CA_ENABLE_PROFILING
	r2ca_di();
	r2ca_eic_isr_cnt++;
	r2ca_isr_cnt++;
	r2ca_ei();
#endif /* R2CA_ENABLE_PROFILING */    
	((void(*)(void))(exception_table.pfnEIC_Handler))();
}

void
r2ca_TC5_Handler(void)
{
#ifdef R2CA_ENABLE_PROFILING
	r2ca_di();
	r2ca_tc5_isr_cnt++;
	r2ca_isr_cnt++;
	r2ca_ei();
#endif /* R2CA_ENABLE_PROFILING */    
	((void(*)(void))(exception_table.pfnTC5_Handler))();
}

void
r2ca_RTC_Handler(void)
{
#ifdef R2CA_ENABLE_PROFILING
	r2ca_di();
	r2ca_rtc_isr_cnt++;
	r2ca_isr_cnt++;
	r2ca_ei();
#endif /* R2CA_ENABLE_PROFILING */
	((void(*)(void))(exception_table.pfnRTC_Handler))();
}

/*
 *  For delay
 */
void
yield(void){
	time_slice = 0;
#ifdef R2CA_ENABLE_PROFILING
	r2ca_di();
	r2ca_dispatch_cnt++;
	r2ca_ei();
#endif /* R2CA_ENABLE_PROFILING */    
	dly_tsk(0);
}

extern void setup(void);
extern void loop(void);


const ID task_id[] = {
#if R2CA_NUM_TASK > 0
	R2CA_TASK1,
#endif /* R2CA_NUM_TASK > 0 */
#if R2CA_NUM_TASK > 1
	R2CA_TASK2,
#endif /* R2CA_NUM_TASK > 1 */
#if R2CA_NUM_TASK > 2
	R2CA_TASK3,
#endif /* R2CA_NUM_TASK > 2 */
#if R2CA_NUM_TASK > 3
	R2CA_TASK4,
#endif /* R2CA_NUM_TASK > 3 */
#if R2CA_NUM_TASK > 4
	R2CA_TASK5,
#endif /* R2CA_NUM_TASK > 4 */
};

void
r2ca_maintask(intptr_t exinf)
{
	int i;

	syslog(LOG_NOTICE, "Arduino Main Task start!");

#ifdef USBCON
	USBDevice.init();
	USBDevice.attach();
#endif /* USBCON */
    
	analogReference(AR_DEFAULT);
    
	setup();
    
	for(i = 0; i < R2CA_NUM_TASK; i++) {
		act_tsk(task_id[i]);
	}
    
	while(1){
		loop();
		if (serialEventRun) serialEventRun();
	}    
}

#define R2CA_TASK_BODY(NUM) \
extern void loop##NUM(void); \
 \
void \
r2ca_task##NUM(intptr_t exinf) \
{ \
    while(1){ \
        loop##NUM(); \
    }     \
}

#if R2CA_NUM_TASK > 0
R2CA_TASK_BODY(1)
#endif /* R2CA_NUM_TASK > 0 */

#if R2CA_NUM_TASK > 1
R2CA_TASK_BODY(2)
#endif /* R2CA_NUM_TASK > 1 */

#if R2CA_NUM_TASK > 2
R2CA_TASK_BODY(3)
#endif /* R2CA_NUM_TASK > 2 */

#if R2CA_NUM_TASK > 3
R2CA_TASK_BODY(4)
#endif /* R2CA_NUM_TASK > 3 */

#if R2CA_NUM_TASK > 4
R2CA_TASK_BODY(5)
#endif /* R2CA_NUM_TASK > 4 */

#ifdef R2CA_ENABLE_PROFILING
void
r2ca_idle_task(intptr_t exinf) {
	while(1){
		r2ca_di();
		r2ca_idle_cnt++;
		r2ca_ei();
	}
}
#endif /* R2CA_ENABLE_PROFILING */
