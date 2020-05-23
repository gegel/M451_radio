#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "M451Series.h"

#include "com.h"

//----------global variables--------------
unsigned char com_tbuf[COM_TBUF]; //transmitt queue
volatile short com_tin=0; //pointer of input data
volatile short com_tout=0; //pointer of output data

unsigned char com_rbuf[COM_RBUF]; //receiving buffer
unsigned char com_rout[COM_RBUF]; //output buffer for store received data block
volatile short com_rptr=0; //pointer to data in receiving buffer
volatile short com_rflg=0; //length of block in output buffer

//internal funtion's prototypes
void uart_set(void);
void uart_get(void);

//initialize UART0
void com_init(void)
{
  SYS_ResetModule(UART0_RST); //reset UART0 module
	UART_Open(UART0, COMBDR); //open UART0 with specified baudrate (interupts must be rary as possible
  UART0->FIFO |=UART_FIFO_RFITL_14BYTES; //set RX fifo level to maximal value
  UART_SetTimeoutCnt(UART0, COMTOUT);	//set commbytetimeut
	NVIC_EnableIRQ(UART0_IRQn);  //enable UART0 global interupt
  UART_EnableInt(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk)); //enable interupts: RX fifo level reached, RX timeout
}


short com_write(unsigned char* data, short len)
{
	short i;
	
	for(i=0;i<len;i++) //process all bytes
	{
	 com_tbuf[com_tin++]=data[i]; //put next byte to queue and move pointer
   if(com_tin >= COM_TBUF) com_tin=0; //ring pointer to buffer size
   if(com_tin==com_tout) //check buffer full
	 {
    com_tin--; //back pointer
    if(com_tin<0) com_tin=COM_TBUF-1; //ring buffer
		len=0; //for break from loop 
	 }		 	
	}
	UART_EnableInt(UART0, UART_INTEN_THREIEN_Msk);
	return i; //return number of bytes put to buffer
}

//poll having received block
short com_poll(void)
{
	return com_rflg; //return number of bytes in received block or 0
}

//get pointer to received data and clear flags
unsigned char* com_read(void)
{
	memcpy(com_rout, com_rbuf, com_rflg); //copy data from received buffer to output
	com_rptr=0; //clear pointer to data in received buffer 
	com_rflg=0; //clear number of bytes ready in block
	return com_rout; //returns pointer to received block
}

//-----------------------internal procedures-----------------------
//putc all avaliable in TX buffer bytes to TX fifo
void uart_set(void)
{
	while(!UART_IS_TX_FULL(UART0)) //put data to TX fifo
	{
		if(com_tin==com_tout) 
		{
			UART_DisableInt(UART0, UART_INTEN_THREIEN_Msk); //disable interupts
			break; //check there are no data in TX ring buffer: out pointer reached in pointer
		}
		UART_WRITE(UART0, com_tbuf[com_tout++]); //put byte to TX FIFO, move pointer
		if(com_tout >= COM_TBUF) com_tout=0; //ring output pointer 
	}	
}

//read data from RX FIFO
void uart_get(void)
{
  unsigned char c; 
	
	while(!UART_GET_RX_EMPTY(UART0))  //read all data while FIFO will be empty
	{
		c=UART_READ(UART0); //read byte from FIFO
		if(!com_rflg) //check we haven't received unprocessed block yet
		{
		 com_rbuf[com_rptr++]=c; //put byte to buffer, move pointer
     if(com_rptr==COM_RBUF) com_rflg=com_rptr; //check buffer is full, set flag in number of bytes in buffer
		}			   
	}
}


//--------------------------UART0 interupt handler----------------------------
void UART0_IRQHandler(void)
{
 uint32_t u32IntSts = UART0->INTSTS;
	
 //TX FIFO is empty	
 if(u32IntSts & UART_INTSTS_THREINT_Msk)
 {
	uart_set();	//putc all avaliable in TX buffer bytes to TX fifo
 }	
 
 //RX FIFO full
 if(u32IntSts & UART_INTSTS_RDAINT_Msk)
 {
	uart_get(); //read data from RX fifo to rx buffer untill RX fifo will be empty
 }
 
 //RX timeout
 if(u32IntSts & UART_INTSTS_RXTOINT_Msk)
 {
	uart_get(); //read all data in RX fifo to buffer
  com_rflg=com_rptr; //set flag is length of data in block	 
 }
 
	
}



