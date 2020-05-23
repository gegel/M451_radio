#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "M451Series.h"

#include "aud.h"

//------------------Audio grabbing with ADC / playing with DAC  using DMA------------------------------
//------------------UART receiving /transmitting using DMA---------------------------------------------


//Audio grabbing global variables
uint16_t aud_in[2*FRAMELENGTH];   //PCM recording double buffer
volatile uint8_t aud_in_flag=0;  //flag of frame was recorded (avaliable in buf 0 or 1)
volatile uint8_t aud_in_buf=0;   //number of buffer where pkt was recorded  (avaliable in buf 0 or 1)

//Audio playing variables
uint16_t aud_out[2*FRAMELENGTH];  //PCM playing double buffer
volatile uint8_t aud_out_flag=0; //flag of frame was played (in buf 1 or 2)
volatile uint8_t aud_out_buf=0;  //flag of empty playing buffer  (we can write new to buf 0 or 1
volatile signed char aud_rate=0; //playing rate correction (signed, 0 is base rate)

//UART transmitting
//uint8_t com_out[UARTLEN];       //UART0 packet transmitting single buffer
//volatile uint8_t com_out_buf=0; //number of buffer will be decoded
//volatile uint8_t com_out_flag=0; //flag of pkt is transmitted now

//UART receiving
//uint8_t com_in[2*UARTLEN]; //UART0 packet receiving double buffer
//volatile uint8_t com_in_flag=0; //flag of packet was received
//volatile uint8_t com_in_buf=0;  //number of buffer recorded now
//volatile signed char com_in_cnt=0; //counter of DMA received bytes
//volatile int com_in_tmr=0; //time point for commbytetimeout


//Internal procesures
void _ReloadPDMA_EADC(uint8_t dbuf); //restart audio grabbing DMA
void _ReloadPDMA_DAC(uint8_t dbuf); //restart audio playing DMA
//void _ReloadPDMA_RXD(uint8_t qbuf); //restart UART receiving DMA 
//void _SendPDMA_TXS(char* buf, short len); //start UART transmitting DMA

//Initilaize audio/uart module
void aud_init(void)
{
	short i;
	
	//clear global values
	aud_in_flag=0;
	aud_in_buf=0;
	aud_out_flag=0;
	aud_out_buf=0;
	aud_rate=0;
	//com_out_buf=0;
	//com_out_flag=0;
	//com_in_buf=0;
	//com_in_flag=0;
	//com_in_cnt=0;
	//com_in_tmr=0;
	
	//clear buffers
	memset(aud_in, 0, sizeof(aud_in));
	memset(aud_out, 0, sizeof(aud_out));
	//memset(com_out, 0, sizeof(com_out));
	//memset(com_in, 0, sizeof(com_in));
	
	for(i=0;i<2*FRAMELENGTH;i++) aud_out[i]=0x8000;
	
	
	//Open UART 
   /// UART_Open(UART0, 115200); //data port

	  //Init timer for recording rate 8K fixed
    TIMER_SET_CMP_VALUE(TIMER0, TMR8KR);
    TIMER0->CTL = TIMER_PERIODIC_MODE | TIMER_CTL_TRGEADC_Msk;

    //Init timer for UART commbytetimeout
    TIMER_SET_CMP_VALUE(TIMER1, 100);
    TIMER1->CTL = TIMER_PERIODIC_MODE | TIMER_CTL_TRGDAC_Msk;
		 		  
	  //Init timer for UART commbytetimeout
    //TIMER_SET_CMP_VALUE(TIMER3, 0);
    //TIMER3->CTL = TIMER_PERIODIC_MODE | TMR1MC;
	
	 //configure ADC: module=0, ch=1, flag on end,  trigget=TMR0, DMA=160 samples 
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END); //Set the ADC internal sampling time, input mode as single-end and enable the A/D converter
    EADC_SetInternalSampleTime(EADC, 1); //set ADC conversion time
		EADC_SET_DMOF(EADC, EADC_CTL_DMOF_TWOS_COMPLEMENT);  //ADC will be output signed -2048 to 2048
    EADC_ConfigSampleModule(EADC, ADC_MOD, EADC_TIMER0_TRIGGER, ADC_CH); // Configure the sample module 0 for analog input channel 1 and enable Timer0 trigger source 
    EADC_SetExtendSampleTime(EADC, ADC_MOD, 168); //extend sampling time for ADC module 
	  EADC_ENABLE_PDMA(EADC); //enable DMA for ADC
	
//-------------------------------------------------------------------------------	
	//configure DAC: trigger=TMR1, DMA=160 samples
    DAC_Open(DAC, 0, DAC_TIMER1_TRIGGER); //set DAC trigger is TIMER1
    DAC_SetDelayTime(DAC, 8); //set delay time aftre Triggen signal to DAC conversion
		DAC_ENABLE_LEFT_ALIGN(DAC); //DAC will be accept unsigned short from 0x0000 to 0xFFF0
    DAC_CLR_INT_FLAG(DAC, 0); //clear DAC int flags
		DAC_ENABLE_PDMA(DAC); //enable DMA for DAC
				
//==============================configure DMA====================================
	  SYS_ResetModule(PDMA_RST); //reset PDMA module
    //open DMA channels: ADC, DAC, UART0RX, UART0TX
    ////PDMA_Open((1<<ADC_PDMA)|(1<<DAC_PDMA)|(1<<TXD_PDMA)|(1<<RXD_PDMA));
		PDMA_Open((1<<ADC_PDMA)|(1<<DAC_PDMA));
	
//--------------PDMA for EADC----------------------
    PDMA_SetTransferCnt(ADC_PDMA, PDMA_WIDTH_16, FRAMELENGTH); //set transfer length is audio frame length
    PDMA_SetTransferAddr(ADC_PDMA, (uint32_t)&EADC->DAT[ADC_MOD], PDMA_SAR_FIX, (uint32_t)aud_in, PDMA_DAR_INC); //set destination is first part of double buffer
    PDMA_SetTransferMode(ADC_PDMA, PDMA_ADC_RX, FALSE, 0); //set transfer mode: normal
    PDMA_SetBurstType(ADC_PDMA, PDMA_REQ_SINGLE, PDMA_BURST_4); //set transfer mode: single transfer on request
		PDMA_EnableInt(ADC_PDMA, PDMA_INT_TRANS_DONE); //enable interupt
   
//--------------PDMA for DAC-------------------------  
    PDMA_SetTransferCnt(DAC_PDMA, PDMA_WIDTH_16, FRAMELENGTH);
    PDMA_SetTransferAddr(DAC_PDMA, (uint32_t)aud_out, PDMA_SAR_INC, (uint32_t)&DAC->DAT, PDMA_DAR_FIX);
    PDMA_SetTransferMode(DAC_PDMA, PDMA_DAC_TX, FALSE, 0);
    PDMA_SetBurstType(DAC_PDMA, PDMA_REQ_SINGLE, PDMA_BURST_128);
		PDMA_EnableInt(DAC_PDMA, PDMA_INT_TRANS_DONE);
	
//--------------PDMA for UART0 TX-------------------------  
		//PDMA_SetTransferCnt(TXD_PDMA, PDMA_WIDTH_8, UARTLEN); 
    //PDMA_SetTransferAddr(TXD_PDMA, (uint32_t)com_out, PDMA_SAR_INC, (uint32_t)&UART0->DAT, PDMA_DAR_FIX);   
    //PDMA_SetTransferMode(TXD_PDMA, PDMA_UART0_TX, FALSE, 0);
    //PDMA_SetBurstType(TXD_PDMA, PDMA_REQ_SINGLE, PDMA_BURST_128);
    //PDMA->DSCT[TXD_PDMA].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk; 
		//PDMA_EnableInt(TXD_PDMA, PDMA_INT_TRANS_DONE);	
		
//--------------PDMA for UART0 RX-------------------------  
		//PDMA_SetTransferCnt(RXD_PDMA, PDMA_WIDTH_8, UARTLEN); 
    //PDMA_SetTransferAddr(RXD_PDMA, (uint32_t)&UART0->DAT, PDMA_SAR_FIX, (uint32_t)com_in, PDMA_DAR_INC);    
    //PDMA_SetTransferMode(RXD_PDMA, PDMA_UART0_RX, FALSE, 0);
    //PDMA_SetBurstType(RXD_PDMA, PDMA_REQ_SINGLE, PDMA_BURST_128);
		//PDMA_EnableInt(RXD_PDMA, PDMA_INT_TRANS_DONE);         
    //UART0->INTEN |= UART_INTEN_RXPDMAEN_Msk; 
//--------------------------------------------------------------

		NVIC_EnableIRQ(PDMA_IRQn);  //enable DMA global interupt		
//==============================================================
		
		TIMER_Start(TIMER0); //start recording timer
		TIMER_Start(TIMER1); //start playing timer
		//TIMER_Start(TIMER3); //start UART commbytetimeout timer
}

//set playing rate correction value
void aud_set(short rate)
{
 aud_rate=rate;	
}

//chech we have grabbed frame: returns pointer to frame or 0
unsigned short* aud_grab(void)
{
	unsigned short* ptr=0;
	
	if(aud_in_flag) //check recording DMA is compleet
	{
	 aud_in_flag=0; //clear recording flag
	 if(aud_in_buf)  ptr=aud_in+FRAMELENGTH; else ptr=aud_in; //position of recently recorded frame		
	}
	
	return ptr; //returns pointer to recorded frame or 0 if frame not ready yet
}

//check we need next frame for playing: return pointer to buffer where frame must be palced or 0
unsigned short* aud_play(void)
{
  unsigned short* ptr=0;
	
	if(aud_out_flag) //check playing DMA is compleet
	{
	 aud_out_flag=0; //clear playing flag
	 if(aud_out_buf)  ptr=aud_out+FRAMELENGTH; else ptr=aud_out; //position of recently played frame		
	}
	
	return ptr;	//return pointer to buffer for new frame for playing or 0	
}


/*
//check we receive some data bytes over UART RX
signed char com_poll(void)
{
  signed char cnt;
	int tmr;
	
	//check DMA compleet flag (buffer full)
	if(com_in_flag) return UARTLEN; //check UART RX DMA complee and returns buufer length
	
	//check we have some received bytes in buffer
	cnt=(UARTLEN-1)-((PDMA->DSCT[RXD_PDMA].CTL)>>16); //get UART RX DMA counter and compute number of already received bytes 
	if(!cnt) return 0; //returns if no bytes were received yet
	
	//is received new bytes after last chack 
	tmr=TIMER_GetCounter(TIMER3); //get 10 uS time counter current value
 	if(cnt!=com_in_cnt) //compare old byte counter with current< if some bytes were received:
	{
	 com_in_tmr=tmr+RX_TOUT; //set new timeout point based on actual timer and specified commbytetimeout value
	 com_in_tmr&=0xFFFFFF; //restrict timer to 24 bits (hardware timer width)
	 com_in_cnt=cnt; //save actual number of received bytes for next
	 return 0;   //we have some received bytes but not output now: wait for other bytes		
	}
	
	//check timeout after last received byte
	tmr-=com_in_tmr;  //time diff
  if(!((tmr>>16)&1)) //fast check for timeout
	{
	  com_in_flag=1; //set flag "received data is ready"
		com_in_buf^=1; //set next receiving buffer part for receiving  	
		_ReloadPDMA_RXD(com_in_buf); //restart UART DMA receiver	
		return cnt; //returns number of received bytes for reading
	}

	return 0; //or wait for timeout
}

//read received bytes (calls immediately after com_poll() returns non-zero
//returns pointer to bytes were received
unsigned char* com_read(void)
{
  unsigned char* ptr=0;
	
	if(com_in_flag) //check receiving flag
	{
	 com_in_flag=0; //clear receiveing flag
	 if(com_in_buf)  ptr=com_in+UARTLEN; else ptr=com_in; //position of recently rreceived bytes		
	}
	
	return ptr;			
	
}

//write len bytes in data over UART TX using DMA
unsigned char com_write(unsigned char* data, unsigned char len)
{
 unsigned char ret=0;

 if(!com_out_flag) //check prevoius DMA TX is compleet
 {	 
  if(len>UARTLEN) len=UARTLEN; //restrict maximal bytes for sending
	memcpy(com_out, data, len); //copy bytes to send buffer
	 _SendPDMA_TXS((char*)data, (short)len); //start UART DMA TX
	com_out_flag=1; //set TX is busy (wiil be cleared in DMA interupt)
  ret=len; //return number of bytes accepted for sending	 
 }	
	return ret;
}

*/

//-----------------------------------------------------------------------------------
//Internal procedures
//-----------------------------------------------------------------------------------

//------------------------Audio interface---------------------
void _ReloadPDMA_EADC(uint8_t dbuf)  //start frame recording
{
	//actually not need to correct record rate
	//because master is SPI1 with fixed rate derived from HXT
	//and slave is TMR0 also with fixed rate derived from HXT
 
	  PDMA_SetTransferCnt(ADC_PDMA, PDMA_WIDTH_16, FRAMELENGTH); //set frame length
    PDMA_SetTransferAddr(ADC_PDMA, (uint32_t)&EADC->DAT[ADC_MOD], PDMA_SAR_FIX, (uint32_t)(aud_in+dbuf*FRAMELENGTH), PDMA_DAR_INC); //set destination
    PDMA_SetTransferMode(ADC_PDMA, PDMA_ADC_RX, FALSE, 0); //run ADC in DMA mode	 
}

void _ReloadPDMA_DAC(uint8_t dbuf)  //start frame playing
{          
  //need to correct playing rate
  //because master rate is SPI0 with rate corrected by demodulator
  //actually SPI0 rate is equal to SPI1 transmitter rate on remote side
	//slave is our TMR0 rate can be different from their TMR1 rate
	
	
	short r;
	
  r=TMR8KP - OUT_RATE_COEF*aud_rate; //check correction change and set new timer's rate    
	TIMER_SET_CMP_VALUE(TIMER1, r); //apply timer rate to divider
	PDMA_SetTransferCnt(0, PDMA_WIDTH_16, FRAMELENGTH); //set frame length
  PDMA_SetTransferAddr(DAC_PDMA, (uint32_t)(aud_out+dbuf*FRAMELENGTH), PDMA_SAR_INC, (uint32_t)&DAC->DAT, PDMA_DAR_FIX); //set source 
  PDMA_SetTransferMode(0, PDMA_DAC_TX, FALSE, 0); //run DAC in DMA	 mode
	
}


/*
//----------------Data UART0 interface--------------------------
void _ReloadPDMA_RXD(uint8_t qbuf)   //start receive data packet
{
		UART0->FIFO |= (1ul << 1); //clear UART0 FIFO  
		PDMA_SetTransferCnt(RXD_PDMA, PDMA_WIDTH_8, UARTLEN); //set fixed length of received packet
		PDMA_SetTransferAddr(RXD_PDMA, (uint32_t)&UART0->DAT, PDMA_SAR_FIX, (uint32_t)(com_in + qbuf*UARTLEN), PDMA_DAR_INC);  
    PDMA_SetTransferMode(RXD_PDMA, PDMA_UART0_RX, FALSE, 0); //run UART0 RX in DMA mode
}

void _SendPDMA_TXD(uint8_t* buf) //start transmitt data packet
{
		//buf[PKTLENGTH-1]=SYNC_BYTE;
	  PDMA_SetTransferCnt(TXD_PDMA, PDMA_WIDTH_8, UARTLEN); //set length of transmitted data
    PDMA_SetTransferAddr(TXD_PDMA, (uint32_t)buf, PDMA_SAR_INC, (uint32_t)&UART0->DAT, PDMA_DAR_FIX);  //set sorce  
    PDMA_SetTransferMode(TXD_PDMA, PDMA_UART0_TX, FALSE, 0); //run UART0 TX in DMA mode
	  UART0->INTEN |= UART_INTEN_TXPDMAEN_Msk;
}

void _SendPDMA_TXS(char* buf, short len) //start transmitt data packet
{
		//buf[PKTLENGTH-1]=SYNC_BYTE;
	  PDMA_SetTransferCnt(TXD_PDMA, PDMA_WIDTH_8, len); //set length of transmitted data
    PDMA_SetTransferAddr(TXD_PDMA, (uint32_t)buf, PDMA_SAR_INC, (uint32_t)&UART0->DAT, PDMA_DAR_FIX);  //set sorce  
    PDMA_SetTransferMode(TXD_PDMA, PDMA_UART0_TX, FALSE, 0); //run UART0 TX in DMA mode
	  UART0->INTEN |= UART_INTEN_TXPDMAEN_Msk;
}

*/


//---------------------------------------------------------------------------------------------------------
// PDMA interrupt handler                                                                                  
//---------------------------------------------------------------------------------------------------------
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(); //read DMA IRQ flags
	 
    if(status & PDMA_INTSTS_ABTIF_Msk)    // DMA abort event 
    {
        PDMA_CLR_ABORT_FLAG(PDMA_ABTSTS_ABTIFn_Msk); //clear DMA Abort flag
    } //end of DMA Abort event handler
    
		if(status & PDMA_INTSTS_TDIF_Msk) // DMA done event 
    {
       //ADC DMA done (frame was recorded) 
			 if(PDMA_GET_TD_STS() & (1<<ADC_PDMA))  
				{
					
					_ReloadPDMA_EADC(aud_in_buf);  //restart ADC DMA for recording next frame
					aud_in_buf^=1;  //swap buffer
					aud_in_flag = 1; //set "frame ready" flag
					PDMA_CLR_TD_FLAG( (1<<ADC_PDMA) ); //clear DMA interupt flag
				} //end of IRQ DMA ADC
				
				//DAC DMA done (frame was played)
				if(PDMA_GET_TD_STS() & (1<<DAC_PDMA)) 
				{  
					_ReloadPDMA_DAC(aud_out_buf); //restart DAC DMA for playing next frame
					aud_out_buf^=1; //swap buffer
					aud_out_flag = 1; //set "playbuffer is empty" flag
					PDMA_CLR_TD_FLAG( (1<<DAC_PDMA) ); //clear DMA interupt flag
				}	//end of IRQ DMA DAC
        
				//UART0 TX DMA done
        //if (PDMA_GET_TD_STS() & (1<<TXD_PDMA)) 
       // {
        //  UART0->INTEN &= (~UART_INTEN_TXPDMAEN_Msk); //disable UART transmitter
				//	com_out_flag=0;  //clear flag "data in transmitting"
				//	PDMA_CLR_TD_FLAG( (1<<TXD_PDMA) ); //clear DMA interupt flag				
        //} //end of IRQ DMA UART0 TX
				
				//UART0 RX DMA done       
       // if (PDMA_GET_TD_STS() & (1<<RXD_PDMA)) //uart0 RX DONE
       // {
  			//	 com_in_flag=1; //set flag "received data is ready"
				//	 com_in_buf^=1; //set next receiving buffer part for receiving
				//	 _ReloadPDMA_RXD(com_in_buf); //restart UART receiver
				//	 PDMA_CLR_TD_FLAG( (1<<RXD_PDMA) );		//clear DMA interupt flag
        //} //end of IRQ DMA UART0 RX
						
    } //end of DMA done event handler
    
} //end of DMA IRQn_Type handler



