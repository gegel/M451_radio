#include "M451Series.h"

#include "typedefs.h"
#include "gpc.h"

unsigned char cmt_tout=0;

//=================Timings============================

//1us delay for software SPI by loop
void cmt_spi3_delay(void)
{
    volatile u16 n = SPI_DELAY_COUNT_MS;
    while(n--);
}

//1-65535 us delay by timer
void system_delay_us (unsigned short us)
{	
 unsigned char delay=7; //number of MCU cycles waits for Timer start
	
 TIMER2->CTL = 0;  //clear Timer
 TIMER2->EXTCTL = 0;
 TIMER2->CMP = us; //set delay in uS
 TIMER2->CTL = TIMER_CTL_CNTEN_Msk | TIMER_ONESHOT_MODE | 11; //start timer in oneshot mode with 1Mhz clock (12 Mhz/(11+1))
 for(; delay > 0; delay--) {__NOP();}  //wait a few for Timer start 
 while(TIMER2->CTL & TIMER_CTL_ACTSTS_Msk); //wait wor timer shot
}



void cmt_gpio_pd(void)
{
	//disable IRQ
	NVIC_DisableIRQ(PDMA_IRQn);
  NVIC_DisableIRQ(SPI0_IRQn); 
  NVIC_DisableIRQ(SPI1_IRQn);
	//set all used GPIO as input
	GPIO_SetMode(PC, BIT7, GPIO_MODE_INPUT); //BTN
  GPIO_SetMode(PC, BIT6, GPIO_MODE_INPUT);//LED
  GPIO_SetMode(PE, BIT9, GPIO_MODE_INPUT); //CS
  GPIO_SetMode(PE, BIT8, GPIO_MODE_INPUT); //CLK
  GPIO_SetMode(PE, BIT1, GPIO_MODE_INPUT);  //DIO
  GPIO_SetMode(PE, BIT5, GPIO_MODE_INPUT); //FB
  //GPIO_SetMode(PC, BIT9, GPIO_MODE_INPUT);//TST
  //GPIO_SetMode(PA, BIT15, GPIO_MODE_INPUT); //MIC
  //GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT); //SPC	
	PA15=0; PB2=0; //disable MIC and speaker
	GPIO_SetMode(PB, BIT13, GPIO_MODE_INPUT); //COUNT TEST
	GPIO_SetMode(PB, BIT14, GPIO_MODE_INPUT); //COUNT GND
}


//=====================GPIO===================================

//initialize GPIO
void cmt_gpio_init(void)
{
  GPIO_SetMode(PC, BIT7, GPIO_MODE_QUASI); PC7=1;//BTN
  GPIO_SetMode(PC, BIT6, GPIO_MODE_OUTPUT); PC6=1;//LED
  GPIO_SetMode(PE, BIT9, GPIO_MODE_OUTPUT); PE9=1; //CS
  GPIO_SetMode(PE, BIT8, GPIO_MODE_OUTPUT); PE8=1; //CLK
  GPIO_SetMode(PE, BIT1, GPIO_MODE_OUTPUT); PE1=1; //DIO
  GPIO_SetMode(PE, BIT4, GPIO_MODE_OUTPUT); PE4=1; //FB
  GPIO_SetMode(PE, BIT5, GPIO_MODE_INPUT); //GPIO
  GPIO_SetMode(PC, BIT9, GPIO_MODE_OUTPUT); PC9=1;//TST
  GPIO_SetMode(PA, BIT15, GPIO_MODE_OUTPUT); PA15=0;//MIC
  GPIO_SetMode(PB, BIT2, GPIO_MODE_OUTPUT); PB2=0;//SPC
  GPIO_SetMode(PB, BIT13, GPIO_MODE_QUASI); PB13=1;//COUNT TEST
	GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT); PB14=0; //COUNT GND	
}


unsigned char cmt_read_p3(void)
{
	return (unsigned char)CMT2300A_ReadGpio3();	
}








//void procedure for emtu macro
void cmtvoid(void)
{
	
}

/*
////void cmt_spi3_csb_out(void){}//      SET_GPIO_OUT(CMT_CSB_GPIO)
void cmt_spi3_fcsb_out(void){}//     SET_GPIO_OUT(CMT_FCSB_GPIO)
void cmt_spi3_sclk_out(void){}//     SET_GPIO_OUT(CMT_SCLK_GPIO)
void cmt_spi3_sdio_out(void){}//     SET_GPIO_OUT(CMT_SDIO_GPIO)
void cmt_spi3_sdio_in(void){}//      SET_GPIO_IN(CMT_SDIO_GPIO)

void cmt_spi3_csb_1(void){}//        SET_GPIO_H(CMT_CSB_GPIO)
void cmt_spi3_csb_0(void){}//        SET_GPIO_L(CMT_CSB_GPIO)

void cmt_spi3_fcsb_1(void){}//       SET_GPIO_H(CMT_FCSB_GPIO)
void cmt_spi3_fcsb_0(void){}//       SET_GPIO_L(CMT_FCSB_GPIO)
    
void cmt_spi3_sclk_1(void){}//       SET_GPIO_H(CMT_SCLK_GPIO)
void cmt_spi3_sclk_0(void){}//       SET_GPIO_L(CMT_SCLK_GPIO)

void cmt_spi3_sdio_1(void){}//       SET_GPIO_H(CMT_SDIO_GPIO)
void cmt_spi3_sdio_0(void){}//       SET_GPIO_L(CMT_SDIO_GPIO)
unsigned char cmt_spi3_sdio_read(void){return 0;}//    READ_GPIO_PIN(CMT_SDIO_GPIO)


void CMT2300A_SetGpio1In(void) //           SET_GPIO_IN(CMT_GPIO1_GPIO)
{
  //set GPIO to input mode
}

void CMT2300A_SetGpio2In(void) //           SET_GPIO_IN(CMT_GPIO2_GPIO)
{

 //set GPIO to input mode
}


void CMT2300A_SetGpio3In(void)//           SET_GPIO_IN(CMT_GPIO3_GPIO)
{
 //set GPIO to input mode
}

unsigned char CMT2300A_ReadGpio1(void)//            READ_GPIO_PIN(CMT_GPIO1_GPIO)
{
 return 0;
}

unsigned char CMT2300A_ReadGpio2(void)//            READ_GPIO_PIN(CMT_GPIO2_GPIO)
{
 return 0;
}

unsigned char CMT2300A_ReadGpio3(void)//            READ_GPIO_PIN(CMT_GPIO3_GPIO)
{
 return 0;
}

*/

