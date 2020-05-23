#ifndef __HW_LAYER_H
#define __HW_LAYER_H

#include "typedefs.h"

#ifdef __cplusplus 
extern "C" { 
#endif



// ************************************************************************
//  The following need to be modified by user
//  ************************************************************************ 




//===========================Timings================================

#define SPI_DELAY_COUNT_MS 100  //loop itteration for ius delay depends by HW speed

void cmt_spi3_delay(void); //delay 1mS by loop
void system_delay_us (unsigned short us); //delay in us by timer


//===============================GPIO===================================

void cmt_gpio_init(void);
void cmt_gpio_pd(void);
unsigned char cmt_read_p3(void);


void cmtvoid(void);

//---------------IO_RADIO------------------------
//VSS P63
//VDD P64
//BTN PC7 P52 in
//LED PC6 P51 out
//CS PE9 P67 out
//CLK PE8 P66 out
//DIO PE1 P65 in/out
//FB PE4 P53 out
//GPIO PE5 P54 in

//----------------IO BOARD------------------------
//TST PC9 P38 out  onboard led to vcc

//-----------------IO AUDIO-----------------------
//VDD P88
//AVDD P89
//VSS P87
//DAC PB0 P91
//ADC1 PB1 P92
//MIC PA14 P85 out
//SPC PA15 P86 out





//pin direction not changed because startup is used
#define cmt_spi3_csb_out()      cmtvoid()
#define cmt_spi3_csb_out()      cmtvoid()
#define cmt_spi3_fcsb_out()     cmtvoid()
#define cmt_spi3_sclk_out()     cmtvoid()

//pin direction not changed because bidirectional mode is used
//#define cmt_spi3_sdio_out()     PE->MODE = (PE->MODE | (0x1 << (BIT1 <<1)))
//#define cmt_spi3_sdio_in()      PE->MODE = (PE->MODE & ~(0x3 << (BIT1 << 1)))

#define cmt_spi3_sdio_out()     PE->MODE = (PE->MODE | 4)
#define cmt_spi3_sdio_in()      PE->MODE = (PE->MODE & ~0xC)

//set spi outputs
#define cmt_spi3_sdio_1()       PE1=1
#define cmt_spi3_sdio_0()       PE1=0;

#define cmt_spi3_sclk_1()       PE8=1;
#define cmt_spi3_sclk_0()       PE8=0;

#define cmt_spi3_csb_1()        PE9=1;
#define cmt_spi3_csb_0()        PE9=0;

#define cmt_spi3_fcsb_1()       PE4=1;
#define cmt_spi3_fcsb_0()       PE4=0;
   





//read SPI data
#define cmt_spi3_sdio_read()    (PE1)

//read gpion input controls
#define CMT2300A_ReadGpio1()    (PE5)
#define CMT2300A_ReadGpio2()    (PE5)
#define CMT2300A_ReadGpio3()    (PE5)

//pin directing not changed because startup is used
#define CMT2300A_SetGpio1In()   cmtvoid()
#define CMT2300A_SetGpio2In()   cmtvoid()
#define CMT2300A_SetGpio3In()   cmtvoid()

//user control 
#define CMT2300A_ReadBTN()     (PC7)
#define CMT2300A_ReadGPI()     (PE5)
#define CMT2300A_ReadTST()     (PB13)

#define CMT2300A_WriteTST_1()  PC9=0
#define CMT2300A_WriteTST_0()  PC9=1

#define CMT2300A_WriteLED_1()  PC6=0
#define CMT2300A_WriteLED_0()  PC6=1

#define CMT2300A_WriteMIC_1()  PA15=1
#define CMT2300A_WriteMIC_0()  PA15=0

#define CMT2300A_WriteSPC_1()  PB2=1
#define CMT2300A_WriteSPC_0()  PB2=0






/*
void cmt_spi3_csb_out(void);
void cmt_spi3_fcsb_out(void);
void cmt_spi3_sclk_out(void);
void cmt_spi3_sdio_out(void);
void cmt_spi3_sdio_in(void);

void cmt_spi3_csb_1(void);
void cmt_spi3_csb_0(void);

void cmt_spi3_fcsb_1(void);
void cmt_spi3_fcsb_0(void);
    
void cmt_spi3_sclk_1(void);
void cmt_spi3_sclk_0(void);

void cmt_spi3_sdio_1(void);
void cmt_spi3_sdio_0(void);
unsigned char cmt_spi3_sdio_read(void);
*/

//====================================================

/*
void CMT2300A_SetGpio1In(void); //           SET_GPIO_IN(CMT_GPIO1_GPIO)
void CMT2300A_SetGpio2In(void); //           SET_GPIO_IN(CMT_GPIO2_GPIO)
void CMT2300A_SetGpio3In(void); //           SET_GPIO_IN(CMT_GPIO3_GPIO)
unsigned char CMT2300A_ReadGpio1(void);//            READ_GPIO_PIN(CMT_GPIO1_GPIO)
unsigned char CMT2300A_ReadGpio2(void);//            READ_GPIO_PIN(CMT_GPIO2_GPIO)
unsigned char CMT2300A_ReadGpio3(void);//            READ_GPIO_PIN(CMT_GPIO3_GPIO)
*/


#ifdef __cplusplus
} 
#endif

#endif







