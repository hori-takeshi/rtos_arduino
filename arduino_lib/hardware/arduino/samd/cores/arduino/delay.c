#include "delay.h"
#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Tick Counter united by ms */
static volatile uint32_t _ulTickCount=0 ;

uint32_t millis( void )
{
// todo: ensure no interrupts
  return _ulTickCount ;
}

// Interrupt-compatible version of micros
// Theory: repeatedly take readings of SysTick counter, millis counter and SysTick interrupt pending flag.
// When it appears that millis counter and pending is stable and SysTick hasn't rolled over, use these
// values to calculate micros. If there is a pending SysTick, add one to the millis counter in the calculation.
uint32_t micros( void )
{
  uint32_t ticks, ticks2;
  uint32_t pend, pend2;
  uint32_t count, count2;

  ticks2  = SysTick->VAL;
  pend2   = !!(SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)  ;
  count2  = _ulTickCount ;

  do
  {
    ticks=ticks2;
    pend=pend2;
    count=count2;
    ticks2  = SysTick->VAL;
    pend2   = !!(SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)  ;
    count2  = _ulTickCount ;
  } while ((pend != pend2) || (count != count2) || (ticks < ticks2));

  return ((count+pend) * 1000) + (((SysTick->LOAD  - ticks)*(1048576/(VARIANT_MCK/1000000)))>>20) ;
  // this is an optimization to turn a runtime division into two compile-time divisions and
  // a runtime multiplication and shift, saving a few cycles
}

void delay( uint32_t ms )
{
  if ( ms == 0 )
  {
    return ;
  }

  uint32_t start = _ulTickCount ;

  do
  {
    yield() ;
  } while ( _ulTickCount - start <= (ms) ) ;
}

void SysTick_Handler( void )
{
  // Increment tick count each ms
  _ulTickCount++ ;
}

void delayMicroseconds(uint32_t usec)
{
	unsigned int presc=0, ref=0, sot=1;
	double ref_val=0.0, div_val=0.0, usec_val=0.0;
        unsigned long int i=0,limit=0;
	
        if (usec <=0) return;
	usec_val=usec * 1.0;
	
	if(usec <= 20)
       {
            
			  
			if(usec == 1) limit = usec * 2;
			else if(usec == 2) limit = usec * 6;
			else if(usec == 3) limit = usec * 8;
			else if(usec == 4) limit = usec * 9;
			else if(usec == 5) limit = usec * 9;
			else limit = usec * 11;
           
			for(i=0; i <= limit; i++);
                {
                  asm("NOP");
                }  
                return;
         }  
    else if(usec <=1363)
	{
		presc=TC_CTRLA_PRESCALER_DIV1;
		div_val=1.0;
	        ref = (uint16_t)(usec_val * 48.0 / div_val);
                if((usec > 20) & (usec <=60)) ref= ref - 905;
			if(usec > 60) ref= ref - 921;
	}	

        else if((usec > 1363)  & (usec <= 5461))
	{
		presc=TC_CTRLA_PRESCALER_DIV4;
		div_val=4.0;
		ref = (uint16_t)(usec_val * 48.0 / div_val);
                
	}	
	else if((usec > 5461) & (usec <= 10922))
	{	
		presc=TC_CTRLA_PRESCALER_DIV8;
		div_val=8.0;
		ref = (uint16_t)(usec_val * 48.0 / div_val);
	}	
	else if((usec > 10922) & (usec <= 21845))
	{
		presc=TC_CTRLA_PRESCALER_DIV16;
		div_val=16.0;
		ref = (uint16_t)(usec_val * 48.0 / div_val);
	}	
	else if(usec > 21845)
	{
		presc=TC_CTRLA_PRESCALER_DIV64;
		div_val=64.0;
		ref = (uint16_t)(usec_val * 48.0 / div_val);
	}
    else;
    
    
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TC4_TC5 ));
	
	 TC4->COUNT16.CTRLA.reg &=~(TC_CTRLA_ENABLE);
      TC4->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16 | presc;
      TC4->COUNT16.READREQ.reg = 0x4002;
      TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
      
	  
	  while(TC4->COUNT16.COUNT.reg < ref);
	  	
return;
}

#ifdef __cplusplus
}
#endif
