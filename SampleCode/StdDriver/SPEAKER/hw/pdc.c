#include "M451Series.h" //CPU definitions
#include "gpc.h" //IO procedures

#include "pdc.h"

#define PDC_CNT 1000 //radio output testing loop iterations
#define PDC_FIX 3 //number of pulses in test for carrier present

#pragma push // Save existing optimization level
  #pragma O0   // Optimization level now O0 
  
volatile short pdc_cnt=0; //radio out testing  loop
volatile short pdc_fix=0; //radio out pulses counter

//entering power down mode (exiting over chip reset)
void pdc_init(void)
{
  //set gpio modes for power down 
	CMT2300A_WriteTST_1(); //on-board led on
	cmt_gpio_pd(); //set most gpio as inputs for low power
	 
	//change system clock	
  SYS_UnlockReg(); //unlock system registers
  CLK_DisableSysTick(); //disable systick for power saving
 
	//enable interupt from button
  GPIO_CLR_INT_FLAG(PC, BIT7); 
	GPIO_EnableInt(PC, 7, GPIO_INT_FALLING);
  NVIC_EnableIRQ(GPC_IRQn); //enable IRQ for wake
	
pde: //entering power down

	//enable HORC, switch core clock to HIRC
	CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk); //start 10 KHz internal LIRC
  CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk); //wait LIRC clock ready
  CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1)); //switch system clock to LIRC
  
//disable HXTEN and LIRC  	
	CLK_DisablePLL(); //disable PLL for power saving
  CLK_DisableXtalRC(CLK_PWRCTL_HXTEN_Msk); //disable external 12MHz crystal HXTE
  CLK_DisableXtalRC(CLK_PWRCTL_LIRCEN_Msk); //disable internal 22.4 KHz clock
  SystemCoreClockUpdate(); //update core clock
	
  //entering power down
 
	//wait gpio is stable zero and enable interupt on rising edge
  while(PE5) {;} //wait GPIO is 0	
  CMT2300A_WriteTST_0();
	GPIO_CLR_INT_FLAG(PE, BIT5);   //clear interupt flag 
  GPIO_EnableInt(PE, 5, GPIO_INT_RISING);
  NVIC_EnableIRQ(GPE_IRQn); //enable IRQ for wake 
  //CMT2300A_WriteTST_0();
  //entering cpu  in power down mode untill interupt on gpio		
  CLK_PowerDown(); //entering power down mode 
	
  //wake up	
  NVIC_DisableIRQ(GPE_IRQn); //disable IRQ
  //CMT2300A_WriteTST_1();		
		
	//start LIRC, swithc system clock to LIRC and disable HIRC	
  CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk); //start 10 KHz internal LIRC
  CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk); //wait LIRC clock ready
  CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_LIRC, CLK_CLKDIV0_HCLK(1)); //switch system clock to LIRC	
  CLK_DisableXtalRC(CLK_PWRCTL_HIRCEN_Msk); //disable internal 22.4 KHz clock
		
pdt:	
  //poll GPIO pin in loop		
  pdc_cnt=PDC_CNT; pdc_fix=0; //set time conter and clear test value
	CMT2300A_WriteTST_1();
	while(pdc_cnt--) if(PE5) pdc_fix++; //count cases gpio=1 during specified time
  	
	//decice is there carrier or not
  if(!pdc_fix) goto pde; //no more activity on gpio: back to power down
  else if(pdc_fix<PDC_FIX) goto pdt; //middle activity: try check once more
  SYS_ResetChip(); //great activity: restart chip wor entering work mode
}

#pragma pop  //Restore original optimization level


//GPIO interupt hundler
void GPE_IRQHandler(void)
{
	GPIO_DisableInt(PE, 5); //disable GPIO interups
	GPIO_CLR_INT_FLAG(PE, BIT5);   //clear interupt flag
}

//GPIO interupt hundler
void GPC_IRQHandler(void)
{
	SYS_ResetChip(); //great activity: restart chip wor entering work mode
}

