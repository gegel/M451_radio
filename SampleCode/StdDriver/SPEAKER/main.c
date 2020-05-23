
/******************************************************************************
 * @file     main.c
 * @version  V0.1
 * $Revision: 1 $
 * $Date: 12/01/08 $
 * @brief    Demonstrate speech recodring - coding - transitting - receivind - decoding - playing
 * @note     Author: Sergey Gayevsky, 'Gamma UA', info@nuvoton.com.ua
 * Licence: LGPL 3.0 <http://www.gnu.org/licenses/lgpl.html>
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "M451Series.h" //CPU definitions
#include "gpc.h" //io hardware 
#include "rf/radio_if.h"  //radio control
#include "hw/spp.h" //spi receive/send
#include "hw/aud.h"  //audio record/play
#include "hw/com.h"  //audio record/play
#include "hw/pdc.h"  //entering power down mode
#include "prc.h"   //processing


/*
#include "arm_math.h"
#include "flt/interf_enc.h"
#include "flt/interf_dec.h"
#include "mdm/mdm.h"
#include "voice.h"

///////#define SP_TEST

unsigned short sp_cnt=0;

//#include "sp_enc.h" //AMR encoder
//#include "sp_dec.h" //AMR decoder
//-------------------------------------------

//ADC settings
#define ADC_MOD 0  //EADC module used for recording
#define ADC_CH 1   //PB1, pin45 EADC channel used for recording

//DMA settingth
#define ADC_PDMA 2 //PDMA channel used for recording
#define DAC_PDMA 0 //PDMA channel used for playing
#define TXD_PDMA 1 //PDMA channel used for data UART0 TX 
#define RXD_PDMA 4 //pdma channel used for data UART0 RX

//TMR settingth
#define TMR8KR 1500 //72 MHz timer period for 8000KHz sampling rate for recording -20 ->2/3  +20 -> 1/2    0 -> 1/2 
#define TMR8KP 1500 //72 MHz timer period for 8000KHz sampling rate for playing +20 ->2/3  -20 -> 1/2
#define TMR_SPI 149 //12MHz timer period for 80000KHz SPI clock rate

//other settings
#define FRAMELENGTH  160 //number of PCM sampling in AMR frame
#define PKTLENGTH 12     //number of bytes in UART data packet for AMR mode 0
#define RINGBUFLEN 32    //volume of ring buffer (maximum number of received packets)
#define GUARD 4          //critical ring buffer fill (for jitter compensation)
#define KRATE 40         //koefficient of playing rate variation (ratio of delta of 'timer compare value' to delta 'buffer fill' from optimal) 
#define KFILL 16         //koefficient of measured jitter adaptation

#define FIFOCNT 3       //maximal number of encoded speech frames in FIFO


#define BLOCKLENGTH  50 //32bit words in SPI block 

//------------------------------------------AMR---------------------------
int *amrenstate=0; //AMR encode state
int *amrdestate=0; //AMR decode state

//Speech_Decode_FrameState *speech_decoder_state = 0;
//Speech_Encode_FrameState *speech_encoder_state = 0;

char amrmode=0; //4750 bps mode
char amrcbr=0; //vbr active
const char amr_block_size[8]={12, 13, 15, 17, 19, 20, 26, 31 }; //AMR packet length for mode
//-------------------------------------------------------------------------

//general variables
volatile int a=0; //for restore UART
int i;
short j, k, n;

uint16_t *pcm_pos; //pointer to used pcm frame
uint8_t *pkt_pos;  //pointer to used data packet

//ring buffer
uint8_t ring_buf[RINGBUFLEN][PKTLENGTH]; //ring buffer
uint8_t ring_in_ptr=0; //pointer to frame will be received
uint8_t ring_out_ptr=0; //pointer to frame will be transmitted
short ring_max_fill=GUARD*KFILL; //statistic minimal number of frames in ring buffer during work
short ring_min_fill=GUARD*KFILL; //statistic maximal number of frames in ring buffer during work
short delta_rate=0; //playing rate adjusting (0 is nominal rate 8000 Hz)
uint8_t record_pause=0; //flag of recording speech pause
uint8_t play_pause=0; //flag of playing speech pause 

//doube buffers
uint16_t pcm_in[2*FRAMELENGTH];   //PCM recording double buffer
uint16_t pcm_out[2*FRAMELENGTH];  //PCM playing double buffer
uint8_t pkt_in[2*PKTLENGTH]; //UART0 packet receiving double buffer
uint8_t pkt_out[PKTLENGTH];       //UART0 packet transmitting single buffer

//recording & playing flags
#define IN_RATE_COEF 2
volatile uint8_t pcm_in_flag=0;  //flag of frame was recorded (avaliable in buf 0 or 1)
volatile uint8_t pcm_in_buf=0;   //number of buffer where pkt was recorded  (avaliable in buf 0 or 1)
volatile unsigned short pcm_in_rate=0; //rate correction value

#define OUT_RATE_COEF 1
volatile uint8_t pcm_out_flag=0; //flag of frame was played (in buf 1 or 2)
volatile uint8_t pcm_out_buf=0;  //flag of empty playing buffer  (we can write new to buf 0 or 1)
volatile unsigned short pcm_out_rate=0; //rate correction value

volatile uint8_t pkt_in_flag=0; //flag of packet was received
volatile uint8_t pkt_in_buf=0;  //number of buffer recorded now
volatile uint8_t pkt_out_buf=0; //number of buffer will be decoded
volatile uint8_t pkt_out_flag=0; //flag of pkt is transmitted now
volatile uint8_t pkt_sync=1;  //packet boundary aligning flag
volatile uint8_t mute=0; //mute flag

//------------SPI RCVD-------------------------
unsigned int spi_dbuf[2*BLOCKLENGTH]; //u32 word buffer for spi rcvd
unsigned char spi_ptr=0; //pointer to TX/RX word in buffer
volatile unsigned char spi_flg=0; //flag of frame ready
volatile unsigned char spi_r=0; //rate will be applied to SPI clock
volatile unsigned char spi_c=0; //counter of periods rate will be applied 
unsigned char spi_n=0;
short spi_d=0;


//------------SPI SEND-------------------------
unsigned int spi_out_dbuf[2*BLOCKLENGTH]; //u32 word buffer for spi send
volatile unsigned char spi_out_ptr=0; //pointer to TX word in buffer
volatile unsigned char spi_out_flg=0; //flag of frame sended
volatile unsigned char tspi_r=0;  //new rate for change
volatile unsigned char tspi_c=0;  //counter of apply rate changing


//----------------FIFO-----------------------------
unsigned char fifo_buf[FIFOCNT][PKTLENGTH]; //fifo of encoded frames
volatile signed char fifo_in_ptr=2; //pointer to frame will be input
volatile signed char fifo_out_ptr=0; //pointer to frame will be output
volatile signed char fifo_dif=2; //ptr diff in moment of spi receve block
volatile signed char fifo_level=0;  //fifo level


//----------------TFIFO-----------------------------
unsigned char tfifo_buf[FIFOCNT][PKTLENGTH]; //fifo of encoded frames
volatile signed char tfifo_in_ptr=2; //pointer to frame will be input
volatile signed char tfifo_out_ptr=0; //pointer to frame will be output
volatile signed char tfifo_dif=2; //pointer 
volatile signed char tfifo_level=0; //tfifo level

//----------------UART--------------------------------
char str[16];

//Function prototype declaration
void SYS_Init(void);   //first initialization
void amr_ini(int dtx); //amr codec initialization
void amr_fin(void);    //amr finalization (not used here)
void ReloadPDMA_EADC(uint8_t dbuf); //start recording next frame 
void ReloadPDMA_DAC(uint8_t dbuf);  //start playing next frame
void SendPDMA_TXD(uint8_t* buf);    //start transmitting next data packet
void SendPDMA_TXS(char* buf, short len);
void ReloadPDMA_RXD(uint8_t qbuf);  //start receiving next data packet

volatile unsigned int ttt=0;
volatile unsigned int ttt1=0;
volatile int rrr=0;
volatile int ccc=0;
volatile int uuu=0;
volatile int vvv=0;

volatile int allmem=0;
volatile unsigned char dd=0;

unsigned char c;
unsigned char voice_flg=0;



volatile unsigned char spi_tst[100]={0,};
unsigned char oldc=0;


unsigned short nn;

unsigned char rrcc=0;




#define SSS 4
short wave[8]={3827>>SSS,9239>>SSS,9239>>SSS,3827>>SSS,-3827>>SSS,-9239>>SSS,-9239>>SSS, -3827>>SSS};
void AMR_Encode_Test(int16_t* pcm, unsigned char* dat)
{
	if(dat[0]&1) memset(pcm, 0, 320);
	else	for(i=0;i<20;i++) memcpy(pcm+i*8, wave, 16);	
	
}

volatile unsigned int sp1cnt=0;
volatile unsigned int sp0cnt=0;



#define PS1US 11  //prescaler for TIMER2 (1 uS tick)

*/

//-------------------definitions-------------------
#define SLEEP_TOUT 3000  //x20mS timeout of inactivity for entering sleep mode 
#define PRE_TX 50 //number of silency frames transmitted on TX start
#define PRE_RX 50 //number of frames muted on RX start
#define LOG_ON //output modem statistic to UART
//------------------prototypes---------------
void SYS_Init(void);   //system initialization
void SYS_PD(void); //preparing to power down mode
//---------------global values---------------
unsigned short* sptr=0; //bits/pcm pointer

unsigned char ison=0; //flag of radio avaliable
unsigned char rch=0; //radio channel
unsigned char rmode=1;  //radio mode (0-TX, 1-RX)

unsigned short w; //modem state
unsigned char syn=0; //RX carrier lock flag
unsigned char vad=0; //TX voice detector flag

unsigned char cnt=0; //LED blink counter
unsigned short tmr=0; //sleep timer

char str[16];   //string for output statistic in test

volatile int ccc=0;


//----------------------main procedure------------------
int32_t main(void)
{  
	 SYS_Init(); //init clocks and pins
   radio_ch(rch); //set radio channel  
	 radio_init(1); //init radio to work mode
	 com_init(); //initialize uart
   amr_init(); //initialize codec	
	 spp_init();  //init spi 
	 aud_init();  //init audio and uart
   
	strt:	  //entering to main loop
	 prc_init(); //init processing engine
	 tmr=0; //clear sleep timer
	 rmode = CMT2300A_ReadBTN(); //set RX or TX mode mode by button press/release
	 
	 if(CMT2300A_ReadTST()) ison=radio_mode(rmode); //apply receiving/transmitting radio mode and channel
	 else //test mode
	 {
	  ison=radio_mode(0); //force TX
	  prc_vt(1); //force voice count instead speech from mike
	 }
	
	 if(ison) CMT2300A_WriteTST_1(); else CMT2300A_WriteTST_0();  //off on-board LED
	 CMT2300A_WriteSPC_0(); //disable speaker
	 if(!rmode) //transmitting
	 {
		 CMT2300A_WriteMIC_1(); //enable MIC on TX mode
   }
	 else //transmitting
	 {
		 CMT2300A_WriteMIC_0(); //receiving
	 }
	
	 while(rmode==CMT2300A_ReadBTN()) //loop while button state keep stable
	 //while(1)
	 {
    //--------------------Transmitting---------------------
		 
		 sptr=aud_grab(); //check audio grabbing pcm frame
     if(sptr) vad=prc_enc((short*)sptr); //encoding	grabbed pcm frame
		 
		 sptr=spp_send(); //check radio send bits
		 if(sptr) prc_mdm(sptr); //modulating new bits for next sending
		 
		//---------------------Receiving------------------------

    sptr=spp_rcvd();  //check radio receiving bits
    if(sptr) //demodulation 
		{
     unsigned char frame_num;
		 unsigned char bit_lag;			
		 unsigned char play_lag;	
			
		 w=prc_dmd(sptr); //demodulate received bits, get resulting flags	
     spp_rate(w); //correct receiving rate
		 syn=w>>15; //get sync flag
		 frame_num=(w>>4)&0xF; //get frame number
     bit_lag=w&0x0F; //get bit lag
     play_lag=prc_tst(); //get state of playing fifo
#ifdef LOG_ON
     //output statistic for test			
		 sprintf(str, "%X%c%X%c%d\r\n", frame_num, syn+'*', bit_lag, (char)vad+'*', play_lag);
     com_write((unsigned char*)str, 7);
#endif
     //LED indication
	   if(!rmode)  //TX
		 {                                //during TX:
			if(vad) CMT2300A_WriteLED_1(); else CMT2300A_WriteLED_0(); //set LED on speech/silency 
      tmr=0; //clear sleep timeout on sync			  
		 }                                          //during RX:
		 else if(syn) //sync OK
		 {
			if(cnt&2) CMT2300A_WriteLED_1(); else CMT2300A_WriteLED_0(); //blink fast
      CMT2300A_WriteSPC_1(); //speaker on
      tmr=0; //clear sleep timeout on sync			 
		 }
		 else //no sync
		 {
			if((cnt&0x3F)<4) CMT2300A_WriteLED_1(); else CMT2300A_WriteLED_0(); //blink slow
			CMT2300A_WriteSPC_0(); //speaker off
		 }
		 cnt++;	 
		 
			
	   //check for sleep
     if((++tmr)>SLEEP_TOUT) //check for sleep timeout
		 { 
			 SYS_PD(); //preparing MC for power down: disable all periferal modules and interupts 
			 radio_init(0); //initialize radio to power saving mode
			 pdc_init(); //set IO pins, change system clock to internal RC and entering power down mode 
		 }
		 
		}	//end of demodulation		
		
		sptr=aud_play(); //check audio play frame
		if(sptr) prc_decr((short*)sptr); //decode new frame for next playing	
		
//-------------------Reading UART command--------------------------			
	  w=com_poll(); //poll having data block receiving over uart
		if(w)
		{
			unsigned char* cmd;
			if(w>16) w=16; //restrict length
			cmd=com_read(); //get pointer to received data
			memcpy(str, cmd, w); //copy received data to string (for test)
      w=radio_cmd((unsigned char*)str);
      if(w) com_write((unsigned char*)str, w); 			
		}
	 
	 
	 
	 } //and of while (main loop): button state was changed
	
		
		
		
	 goto strt; //button stae was changed: reenter loop	
} //end of main procedure
	
	
	/*
//main procedure
int32_t main(void)
{   
	
    SYS_Init(); //init clocks and pins 
	
	cmt_gpio_init();  //initialize gpio
	
	//otput test
	
	CMT2300A_WriteTST_1(); //blink onboard LED
	CMT2300A_WriteTST_0();
	CMT2300A_WriteTST_1();
	
	CMT2300A_WriteLED_1(); //blink user LED
	CMT2300A_WriteLED_0();
	CMT2300A_WriteLED_1();
	
	CMT2300A_WriteMIC_1(); //blink MIC on
	CMT2300A_WriteMIC_0();
	CMT2300A_WriteMIC_1();
	
	CMT2300A_WriteSPC_1(); //blink SPC on
	CMT2300A_WriteSPC_0();
	CMT2300A_WriteSPC_1();
	
	
	cmt_spi3_sclk_1(); //blink CLK
	cmt_spi3_sclk_0();
	cmt_spi3_sclk_1();
	
	cmt_spi3_csb_1(); //blink CS
	cmt_spi3_csb_0();
	cmt_spi3_csb_1();
	
	cmt_spi3_fcsb_1(); //blink FB
	cmt_spi3_fcsb_0();
	cmt_spi3_fcsb_1();
	
	
	//input test
	
	w=CMT2300A_ReadBTN();  //get BTN value
	w=CMT2300A_ReadBTN();
	w=CMT2300A_ReadBTN();
	
	w=CMT2300A_ReadGpio3();  //get GPIO value
	w=CMT2300A_ReadGpio3();
	w=CMT2300A_ReadGpio3();
	
	//DIO test
	
	cmt_spi3_sdio_1();  //blink DIO out
	cmt_spi3_sdio_0();
	cmt_spi3_sdio_1();
	
	cmt_spi3_sdio_in(); //switch DIO to in
	
	cmt_spi3_sdio_1();  //blink DIO out
	cmt_spi3_sdio_0();
	cmt_spi3_sdio_1();
	
	
	w=cmt_spi3_sdio_read(); //read DIO
	w=cmt_spi3_sdio_read();
	w=cmt_spi3_sdio_read();
	
	cmt_spi3_sdio_out(); //back dio to OUT
	
	cmt_spi3_sdio_1();
	cmt_spi3_sdio_0();
	cmt_spi3_sdio_1();
	
	
	//TIMR test
	while(1)
	{
		for(w=0;w<10000;w++) system_delay_us(100);
		ison^=1;
		if(ison) CMT2300A_WriteTST_1();
    else CMT2300A_WriteTST_0();		
	}
	
	*/
	
	/*
	
	Mcu_Init();  //initialize radio
  //ison=radio_mode(1,0); //rx
  ison=radio_mode(0,0); //tx
	
	
	
	
	
    //clears PCM buffers
	   for(i=0;i<2*FRAMELENGTH;i++)
		 {
			 pcm_in[i]=0;
			 pcm_out[i]=0;			 
		 }
	 
		 memset(tfifo_buf, 0xFF, sizeof(tfifo_buf));
		 memset(fifo_buf, 0xFF, sizeof(fifo_buf));
		 
		//Open UARTs 
    UART_Open(UART0, 115200); //data port

	  //Init timer for recording rate 8K fixed
    TIMER_SET_CMP_VALUE(TIMER0, TMR8KR);
    TIMER0->CTL = TIMER_PERIODIC_MODE | TIMER_CTL_TRGEADC_Msk;

    //Init timer for playing rate 8K adjustable
    TIMER_SET_CMP_VALUE(TIMER1, 100);
    TIMER1->CTL = TIMER_PERIODIC_MODE | TIMER_CTL_TRGDAC_Msk;
		 		 
 	 
	 //configure ADC: module=0, ch=1, flag on end,  trigget=TMR0, DMA=160 samples
	// Set the ADC internal sampling time, input mode as single-end and enable the A/D converter 
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);
    EADC_SetInternalSampleTime(EADC, 1);
		EADC_SET_DMOF(EADC, EADC_CTL_DMOF_TWOS_COMPLEMENT);  //ADC will be output signed -2048 to 2048
    // Configure the sample module 0 for analog input channel 1 and enable Timer0 trigger source 
    EADC_ConfigSampleModule(EADC, ADC_MOD, EADC_TIMER0_TRIGGER, ADC_CH);
    EADC_SetExtendSampleTime(EADC, ADC_MOD, 168);
	  EADC_ENABLE_PDMA(EADC);
	
//-------------------------------------------------------------------------------	
	//configure DAC: trigger=TMR1, DMA=160 samples
    DAC_Open(DAC, 0, DAC_TIMER1_TRIGGER);
    DAC_SetDelayTime(DAC, 8);
		DAC_ENABLE_LEFT_ALIGN(DAC); //DAC will be accept unsigned short from 0x0000 to 0xFFF0
    DAC_CLR_INT_FLAG(DAC, 0);
		DAC_ENABLE_PDMA(DAC);
				
//==============================configure DMA====================================
	  SYS_ResetModule(PDMA_RST); //reset PDMA module
    //open DMA channels: ADC, DAC, UART0RX, UART0TX
    PDMA_Open((1<<ADC_PDMA)|(1<<DAC_PDMA)|(1<<TXD_PDMA)|(1<<RXD_PDMA));
	
//--------------PDMA for EADC----------------------
    PDMA_SetTransferCnt(ADC_PDMA, PDMA_WIDTH_16, FRAMELENGTH);
    PDMA_SetTransferAddr(ADC_PDMA, (uint32_t)&EADC->DAT[ADC_MOD], PDMA_SAR_FIX, (uint32_t)pcm_in, PDMA_DAR_INC);
    PDMA_SetTransferMode(ADC_PDMA, PDMA_ADC_RX, FALSE, 0);
    PDMA_SetBurstType(ADC_PDMA, PDMA_REQ_SINGLE, PDMA_BURST_4);
		PDMA_EnableInt(ADC_PDMA, PDMA_INT_TRANS_DONE);
   
//--------------PDMA for DAC-------------------------  
    PDMA_SetTransferCnt(DAC_PDMA, PDMA_WIDTH_16, FRAMELENGTH);
    PDMA_SetTransferAddr(DAC_PDMA, (uint32_t)pcm_out, PDMA_SAR_INC, (uint32_t)&DAC->DAT, PDMA_DAR_FIX);
    PDMA_SetTransferMode(DAC_PDMA, PDMA_DAC_TX, FALSE, 0);
    PDMA_SetBurstType(DAC_PDMA, PDMA_REQ_SINGLE, PDMA_BURST_128);
		PDMA_EnableInt(DAC_PDMA, PDMA_INT_TRANS_DONE);
	
//--------------PDMA for UART0 TX-------------------------  
		PDMA_SetTransferCnt(TXD_PDMA, PDMA_WIDTH_8, PKTLENGTH); //40 bytes
    PDMA_SetTransferAddr(TXD_PDMA, (uint32_t)pkt_out, PDMA_SAR_INC, (uint32_t)&UART0->DAT, PDMA_DAR_FIX);   
    PDMA_SetTransferMode(TXD_PDMA, PDMA_UART0_TX, FALSE, 0);
    PDMA_SetBurstType(TXD_PDMA, PDMA_REQ_SINGLE, PDMA_BURST_128);
    PDMA->DSCT[TXD_PDMA].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk; 
		PDMA_EnableInt(TXD_PDMA, PDMA_INT_TRANS_DONE);	
		
//--------------PDMA for UART0 RX-------------------------  
		PDMA_SetTransferCnt(RXD_PDMA, PDMA_WIDTH_8, PKTLENGTH); //40 bytes
    PDMA_SetTransferAddr(RXD_PDMA, (uint32_t)&UART0->DAT, PDMA_SAR_FIX, (uint32_t)pkt_in, PDMA_DAR_INC);    
    PDMA_SetTransferMode(RXD_PDMA, PDMA_UART0_RX, FALSE, 0);
    PDMA_SetBurstType(RXD_PDMA, PDMA_REQ_SINGLE, PDMA_BURST_128);
		PDMA_EnableInt(RXD_PDMA, PDMA_INT_TRANS_DONE);         
    UART0->INTEN |= UART_INTEN_RXPDMAEN_Msk; 
//--------------------------------------------------------------

		NVIC_EnableIRQ(PDMA_IRQn);  //enable DMA interupt		
//==============================================================
		
		//Initialize AMR codec engine
   amr_ini(1); //dtx0/1
		
		TIMER_Start(TIMER0); //start recording timer
		TIMER_Start(TIMER1); //start playing timer
		

   // TIMER_SET_CMP_VALUE(TIMER3, 0xFFFFFFFF);
   // TIMER3->CTL = TIMER_PERIODIC_MODE;
	//	TIMER_Start(TIMER3);
		
		
		
		
		
		
		//SPI0 in master mode clocked by Timer3 external connecting
		
		
		
		SPI_Open(SPI0, SPI_MASTER, SPI_MODE_3, 32, 80000); //CLK idle low, x change on rising, rx latch in falling
    SPI_SET_LSB_FIRST(SPI0); //set LSB to MSB mode
		SPI_SET_SUSPEND_CYCLE(SPI0, 0); //no suspend beetwen words
		
		SPI_ENABLE_3WIRE_MODE(SPI0); //not use SS line
    SPI_ClearRxFIFO(SPI0); //clear RX FIFO
		SPI_ClearTxFIFO(SPI0); //clear RX FIFO
    SPI_SetFIFO(SPI0, 6, 6); //set FIFO levels
		
		//ccc= SPI0->CLKDIV;
		
		NVIC_EnableIRQ(SPI0_IRQn); //enable IRQ
		SPI_EnableInt(SPI0, SPI_FIFO_RXTH_INT_MASK); //enable RX interupt
	  for(i=0;i<8;i++) SPI_WRITE_TX(SPI0, 0xFFFFFFFF); //start work 
    //SPI_WRITE_TX(SPI0, 0xFFFFFFFF); //start work 
		//SPI_WRITE_TX(SPI0, 0xFFFFFFFF); //start work 
		//SPI_WRITE_TX(SPI0, 0xFFFFFFFF); //start work 
		//SPI_WRITE_TX(SPI0, 0xFFFFFFFF); //start work 
		//SPI_WRITE_TX(SPI0, 0xFFFFFFFF); //start work 
		//SPI_WRITE_TX(SPI0, 0xFFFFFFFF); //start work 
    //SPI_WRITE_TX(SPI0, 0xFFFFFFFF); //start work
		
		//SPI1 in master mode clocked by HXT 
		SPI_Open(SPI1, SPI_MASTER, SPI_MODE_3, 32, 80000); //CLK idle low, x change on rising, rx latch in falling
		SPI_SET_LSB_FIRST(SPI1); //set LSB to MSB mode
		SPI_SET_SUSPEND_CYCLE(SPI1, 0); //no suspend beetwen words
		
		SPI_ENABLE_3WIRE_MODE(SPI1); //not use SS line
    SPI_ClearRxFIFO(SPI1); //clear RX FIFO
		SPI_ClearTxFIFO(SPI1); //clear RX FIFO
    SPI_SetFIFO(SPI0, 3, 3); //set FIFO levels
		
		//SPI_SetFIFO(SPI0, 1, 1); //set FIFO level
		NVIC_EnableIRQ(SPI1_IRQn); //enable IRQ
		SPI_EnableInt(SPI1, SPI_FIFO_TXTH_INT_MASK); //enable TX interupt
    
		SPI_WRITE_TX(SPI1, 0xFFFFFFFF);
	  SPI_WRITE_TX(SPI1, 0xFFFFFFFF);
	  SPI_WRITE_TX(SPI1, 0xFFFFFFFF);
	
		//Timer output: PD10 pin 26
		//TIMER_SET_CMP_VALUE(TIMER2, TMR_SPI); //40KHz 
    //TIMER2->CTL = TIMER_TOGGLE_MODE; //meandr
		//TIMER_SELECT_TOUT_PIN(TIMER2, TIMER_TOUT_PIN_FROM_TX); //output
		//TIMER_Start(TIMER2); //run spi
		

    //main infinite loop
    while(1) 
		{				

//------------------------------------------------------------------------------			
		 //check SPI TX processed
			if(spi_out_flg)
			{
				c=spi_out_flg-1; //number of buffer with data
				spi_out_flg=0; //clear flag	
				//modulate next block from tfifo to spi double buffer for next processing by spi tx
			  mdm_modulate(tfifo_buf[tfifo_out_ptr++], (unsigned short*)(spi_out_dbuf+c*BLOCKLENGTH));
			  if(tfifo_out_ptr==FIFOCNT) tfifo_out_ptr=0;
			}
			

			//check SPI RX data ready
			if(spi_flg)
			{	
				c=spi_flg-1; //number of buffer with data
				spi_flg=0; //clear flag
        //memcpy((void*)spi_tst, spi_dbuf+c*25, 100); //copy received data for out
        //memset(spi_dbuf+c*25, 0, 100); //clear buffer
				//demodulate spi received block to fifo
				nn=mdm_demodulate((unsigned short*)(spi_dbuf+c*BLOCKLENGTH), fifo_buf[fifo_in_ptr++]);
				if(fifo_in_ptr==FIFOCNT) fifo_in_ptr=0;
				
				if(nn&0x8000) //check sync ok 
				{
				 //set ALS here !!!
         //spi rx rate correction
				 j=(short)(nn&0x0F)-8; //convert bit boundary 0-15 to range -8 to 7 
         if(j<0) j++; //set to symmetric range -7 to 7 with two centering zeroes
				 spi_d+=j; //average value
				 spi_n++;  //count averaged values
					
					
				 //check we have 8 averaged rate values in accumulator	
				 if(spi_n&4) 
				 {	 		
					spi_d>>=2; //get averaged value 
				  c=0; //clear timer rate
				  if(spi_d <-1) c=TMR_SPI-1; //or set timer rate for increasing bit position
          if(spi_d > 1) c=TMR_SPI+1; //or set timer rate for decreasing bit position
				  if(spi_d) spi_c=1  +  ((abs(spi_d))>>2); // spi_c=2; //1+abs(j); //set count of spi words will be rx with corrected rate
				  spi_r=c; //set rate will be applied from next spi word (in interupt)
					spi_d=0; //clear accumulator
					spi_n=0; 
				 }
				 
				 
				}
				else //no sync
				{
					spi_d=0; //clear averaged values
					spi_n=0; //clear number ov values
				}
				
				//rrcc++;
				//if(!(rrcc&0x08))
				//{
				// tspi_c=2;
        // tspi_r=TMR_SPI-1;					
				//	
				//}
				
				
				//output statistic
				if(1)
				{
					//for output statistic			
				 unsigned char bit_lag=nn&0x0F;  //bit lag (0-7)
				 unsigned char frame_lag=(nn>>8)&0x7F; //frame lag (0-99)
				 unsigned char frame_num=(nn>>4)&0x0F; //frame number (0-15)
				 char frame_sync=(nn>>15)&1; //sync flag (0/1)
				
         sprintf(str, "%X%c%X%c%d%d\r\n", frame_num, frame_sync+'*', bit_lag, (char)voice_flg+'*', fifo_level, tfifo_level); //frame_lag);					
				 //strcpy(str, "test00\r\n");
					if(!pkt_out_flag)
				 {
					 SendPDMA_TXS(str, 8);		
				 }
				}
				
				
			}
	
		

  // ccc=SPI_GET_TX_FIFO_FULL_FLAG(SPI0);
	//		ccc=0x40000000&(SPI0->STATUS);
			
	//		if(ccc==0)
   // {
   //   
	//		ccc=SPI0->STATUS;
	//		SPI0->TX=0xFFFFFFFF;
	//		dd=1;
	//		dd++;
  //  }
	//	else 
	//	{
	//		ccc=SPI0->STATUS;
	//		dd=0;
	//		dd++;
			
	//	}
		
			
			
//recording and sending
			if(pcm_in_flag) //frame was recorded and avaliable in pcm_in_buf
			{	
				short* ppp;
				pcm_in_flag=0; //clear recording flag
				if(pcm_in_buf)  pcm_pos=pcm_in+FRAMELENGTH; else pcm_pos=pcm_in; //position of recently recorded frame
				//ADC outputs signed shorts and right aligned  (12-bit signed format  -2048 - 2048)) !!! 
				//encoder requires signed short and left aligned -7FF0  to 7FF0
				//multipled by 16 in sc_enc.c, Pre_Process(), line 10871
				
				//ppp=(short*)pcm_pos; 
				//for(i=0;i<160;i++) 
				//{
				// ppp[i]*=16;	
				//}
				//
				
        ///ttt=TIMER_GetCounter(TIMER3);
				
				
				
        //i=AMR475_encode(amrenstate, (int16_t*)pcm_pos, pkt_out);
				
				//encode speech frame from record buffer to tx fifo 
				//returns 0 for silency or 12 for voice)
        i=AMR475_encode(amrenstate, (int16_t*)pcm_pos, tfifo_buf[tfifo_in_ptr]);   
        
	#ifdef	SP_TEST
	      memcpy(tfifo_buf[tfifo_in_ptr], speech_tbl[sp_cnt], 12);
				sp_cnt++;
				if(sp_cnt>=AMR_SPEECH) sp_cnt=0;	
	#endif

        tfifo_in_ptr++;
        if(tfifo_in_ptr==FIFOCNT) tfifo_in_ptr=0;
        if(i) voice_flg=1; else voice_flg=0;
				
				///ttt1=TIMER_GetCounter(TIMER3);
				///rrr=ttt1-ttt;
				///ccc++;
				//if(!(pkt_out[0]&1)) //check for VAD flag 
				//{	 //cleared: frame is speech
				// if(record_pause) pkt_out[0]|=1; //check for previous speech pause, set flag
				// record_pause=0; //clear previous pause flag	
				// pkt_out_flag=1; //set output flag
			 // SendPDMA_TXD(pkt_out); //set packet over uart
				//}
				//else record_pause=1; //or set speech pause recording flag if current frame is not speech
			} //end of recording
//------------------------------------------------------------------------------

			//------------------------------------------------------------------------------			
//playing		
			if(pcm_out_flag) //frame was played and pcm_out_buf is empty now
			{
				signed char r;
				pcm_out_flag=0; //clear playing flag
				//set pointer to the frame will be played
        if(pcm_out_buf) pcm_pos=pcm_out+FRAMELENGTH; else pcm_pos=pcm_out; 
							
				
				
	

       // AMR_Encode_Test((int16_t*)pcm_pos, 0);
       // if(pkt_out[0]&1) 
				//{
				//	memset(pcm_pos, 0, 320);
				//else AMR_Encode_Test((int16_t*)pcm_pos, 0); 
				 // TIMER_Stop(TIMER2);
				//}	
				
				//else 
				//{
					///ttt=TIMER_GetCounter(TIMER3);
					
					//AMR475_decode(amrdestate, pkt_out, (int16_t*)pcm_pos);
					
					//decode block from rx fifo to play buffer
					//return 0 for silency or 160 for voice
					i=AMR475_decode(amrdestate, fifo_buf[fifo_out_ptr++], (int16_t*)pcm_pos);
					if(fifo_out_ptr==FIFOCNT) fifo_out_ptr=0;
					
					///ttt1=TIMER_GetCounter(TIMER3);				
          ///rrr=ttt1-ttt;
          ///ccc++;
					//TIMER_Start(TIMER2);
				//}

				for(i=0;i<FRAMELENGTH;i++) 
				{ //encoder outputs left aligned signed short pcm
					//DAC requre left aligned but unsigned short pcm			
					pcm_pos[i]^=0x8000; //convert signed short PCM samples to unsigned format for DAC
				}
			} //end of playing
			
		} //end of loop
		
		
} //end of main



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
					
					ReloadPDMA_EADC(pcm_in_buf);  //restart ADC DMA for recording next frame
					pcm_in_buf^=1;  //swap buffer
					pcm_in_flag = 1; //set "frame ready" flag
					PDMA_CLR_TD_FLAG( (1<<ADC_PDMA) ); //clear DMA interupt flag
				} //end of IRQ DMA ADC
				
				//DAC DMA done (frame was played)
				if(PDMA_GET_TD_STS() & (1<<DAC_PDMA)) 
				{  
					ReloadPDMA_DAC(pcm_out_buf); //restart DAC DMA for playing next frame
					pcm_out_buf^=1; //swap buffer
					pcm_out_flag = 1; //set "playbuffer is empty" flag
					PDMA_CLR_TD_FLAG( (1<<DAC_PDMA) ); //clear DMA interupt flag
				}	//end of IRQ DMA DAC
        
				//UART0 TX DMA done
        if (PDMA_GET_TD_STS() & (1<<TXD_PDMA)) 
        {
          UART0->INTEN &= (~UART_INTEN_TXPDMAEN_Msk); //disable UART transmitter
					pkt_out_flag=0;  //clear flag "data in transmitting"
					PDMA_CLR_TD_FLAG( (1<<TXD_PDMA) ); //clear DMA interupt flag				
        } //end of IRQ DMA UART0 TX
				
				//UART0 RX DMA done       
        if (PDMA_GET_TD_STS() & (1<<RXD_PDMA)) //uart0 RX DONE
        {
  				 pkt_in_flag=1; //set flag "received data is ready"
					 pkt_in_buf++; //set next receiving buffer part for receiving
					 pkt_in_buf&=1; //receiving buffer is a circullar buffer
					 ReloadPDMA_RXD(pkt_in_buf); //restart UART receiver
					 PDMA_CLR_TD_FLAG( (1<<RXD_PDMA) );		//clear DMA interupt flag
        } //end of IRQ DMA UART0 RX
						
    } //end of DMA done event handler
    
} //end of DMA IRQn_Type handler





//-----------------------------------------------------------------------------------
//Internal procedures
//-----------------------------------------------------------------------------------

//------------------------Audio interface---------------------
void ReloadPDMA_EADC(uint8_t dbuf)  //start frame recording
{
	//actually not need to correct record rate
	//because master is SPI1 with fixed rate derived from HXT
	//and slave is TMR0 also with fixed rate derived from HXT
	
	short r;
	  //r=tfifo_in_ptr-tfifo_out_ptr; //-2 to 2
	  r=tfifo_dif;
		if(r<=0) r+=3; //1 to 3
	  tfifo_level=r;	//number of unplayed samples in fifo 
	  //r-=2; //correction value (-1 to 1), must lock to 0
	  //r=TMR8KR + IN_RATE_COEF*r; //check correction change and set new timer's rate
    //TIMER_SET_CMP_VALUE(TIMER0, r);  
	  PDMA_SetTransferCnt(ADC_PDMA, PDMA_WIDTH_16, FRAMELENGTH); //set frame length
    PDMA_SetTransferAddr(ADC_PDMA, (uint32_t)&EADC->DAT[ADC_MOD], PDMA_SAR_FIX, (uint32_t)(pcm_in+dbuf*FRAMELENGTH), PDMA_DAR_INC); //set destination
    PDMA_SetTransferMode(ADC_PDMA, PDMA_ADC_RX, FALSE, 0); //run ADC in DMA mode	 
}

void ReloadPDMA_DAC(uint8_t dbuf)  //start frame playing
{          
  //need to correct playing rate
  //because master rate is SPI0 with rate corrected by demodulator
  //actually SPI0 rate is equal to SPI1 transmitter rate on remote side
	//slave is our TMR0 rate can be different from their TMR1 rate
	
	
	short r;
	//sp1cnt++;
	
	   //r=fifo_in_ptr-fifo_out_ptr; //-2 to 2
	   r=fifo_dif;
		 if(r<=0) r+=3; //1 to 3
	   fifo_level=r;	//number of unplayed samples in fifo 
	   r-=2; //correction value (-1 to 1), must lock to 0
		 r=TMR8KP - OUT_RATE_COEF*r; //check correction change and set new timer's rate    
	   TIMER_SET_CMP_VALUE(TIMER1, r);
	   PDMA_SetTransferCnt(0, PDMA_WIDTH_16, FRAMELENGTH); //set frame length
     PDMA_SetTransferAddr(DAC_PDMA, (uint32_t)(pcm_out+dbuf*FRAMELENGTH), PDMA_SAR_INC, (uint32_t)&DAC->DAT, PDMA_DAR_FIX); //set source 
		 PDMA_SetTransferMode(0, PDMA_DAC_TX, FALSE, 0); //run DAC in DMA	 mode
	
}

//----------------Data UART0 interface--------------------------
void ReloadPDMA_RXD(uint8_t qbuf)   //start receive data packet
{
		UART0->FIFO |= (1ul << 1); //clear UART0 FIFO  
		PDMA_SetTransferCnt(RXD_PDMA, PDMA_WIDTH_8, PKTLENGTH); //set fixed length of received packet
		PDMA_SetTransferAddr(RXD_PDMA, (uint32_t)&UART0->DAT, PDMA_SAR_FIX, (uint32_t)(pkt_in + qbuf*PKTLENGTH), PDMA_DAR_INC);  
    PDMA_SetTransferMode(RXD_PDMA, PDMA_UART0_RX, FALSE, 0); //run UART0 RX in DMA mode
}

void SendPDMA_TXD(uint8_t* buf) //start transmitt data packet
{
		//buf[PKTLENGTH-1]=SYNC_BYTE;
	  PDMA_SetTransferCnt(TXD_PDMA, PDMA_WIDTH_8, PKTLENGTH); //set length of transmitted data
    PDMA_SetTransferAddr(TXD_PDMA, (uint32_t)buf, PDMA_SAR_INC, (uint32_t)&UART0->DAT, PDMA_DAR_FIX);  //set sorce  
    PDMA_SetTransferMode(TXD_PDMA, PDMA_UART0_TX, FALSE, 0); //run UART0 TX in DMA mode
	  UART0->INTEN |= UART_INTEN_TXPDMAEN_Msk;
}

void SendPDMA_TXS(char* buf, short len) //start transmitt data packet
{
		//buf[PKTLENGTH-1]=SYNC_BYTE;
	  PDMA_SetTransferCnt(TXD_PDMA, PDMA_WIDTH_8, len); //set length of transmitted data
    PDMA_SetTransferAddr(TXD_PDMA, (uint32_t)buf, PDMA_SAR_INC, (uint32_t)&UART0->DAT, PDMA_DAR_FIX);  //set sorce  
    PDMA_SetTransferMode(TXD_PDMA, PDMA_UART0_TX, FALSE, 0); //run UART0 TX in DMA mode
	  UART0->INTEN |= UART_INTEN_TXPDMAEN_Msk;
	  pkt_out_flag=0;
}

//---------------------------------AMR------------------------------------
//AMR initialization
void amr_ini(int dtx)
{
 amrenstate = Encoder_Interface_init(dtx); //create AMR encoder
 amrdestate = Decoder_Interface_init();    //create AMR decoder
	
	////Speech_Encode_Frame_init(&speech_encoder_state, 1);  //dtx=1
  ////Speech_Decode_Frame_init(&speech_decoder_state);
}

//AMR finalization
void amr_fin(void)
{
 if(amrenstate) Encoder_Interface_exit(amrenstate); //free  AMR encoder
 if(amrdestate) Decoder_Interface_exit(amrdestate); //free AMR decoder
}



//SPI0 data received
void SPI0_IRQHandler(void)
  {	
	  fifo_dif=fifo_in_ptr-fifo_out_ptr;
		
		//apply SPI0 rate correction value
		if(spi_r) //check new rate correction value
	  {
		 SPI0->CLKDIV=spi_r;	//set new rate
	   spi_r=0;	//clearerr value (set once!)
	  }
		 
		//restore SPI0 base rate
	  if(spi_c) //check rate correction counter
	  {
	   spi_c--; //decrement counter
     if(!spi_c) SPI0->CLKDIV=TMR_SPI; //if correction interval elapser set basic rate						
	  }
		
		//get all received data from RX FIFO
		while(SPI_GET_TX_FIFO_FULL_FLAG(SPI0) == 0) 
		{
			
			SPI_WRITE_TX(SPI0, 0xFFFFFFFF);
	  }
		
		while(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0) == 0)
    {	
			spi_dbuf[spi_ptr] = SPI_READ_RX(SPI0); //read data from FIFO
			spi_ptr++; //move pointer
			if(spi_ptr==BLOCKLENGTH) spi_flg=1; //check first part is ready, set flag
			else if(spi_ptr==2*BLOCKLENGTH) //check second part is ready, set flag and ring double buffer pointer
			{
				spi_ptr=0;
				spi_flg=2;
			}	 
    }	
  }
	

	//SPI1 data send
	void SPI1_IRQHandler(void)
  {	
		
		//ccc= (SPI1->CLKDIV & (~SPI_CLKDIV_DIVIDER_Msk))>> SPI_CLKDIV_DIVIDER_Pos;
		//ccc= SPI1->CLKDIV; 
		//if(spi_r)
		//{
		// SPI1->CLKDIV=150;	
		// spi_r=0;
		//}
		//else  SPI1->CLKDIV=149;
		
		tfifo_dif=tfifo_in_ptr-tfifo_out_ptr;
		
		//apply SPI0 rate correction value
		if(tspi_r) //check new rate correction value
	  {
		 SPI1->CLKDIV=tspi_r;	//set new rate
	   tspi_r=0;	//clearerr value (set once!)
	  }
		 
		//restore SPI0 base rate
	  if(tspi_c) //check rate correction counter
	  {
	   tspi_c--; //decrement counter
     if(!tspi_c) SPI1->CLKDIV=TMR_SPI; //if correction interval elapser set basic rate						
	  }
		
		//put data to FIFO up to fullfill
		while(SPI_GET_TX_FIFO_FULL_FLAG(SPI1) == 0)
    {
			
			SPI_WRITE_TX(SPI1, spi_out_dbuf[spi_out_ptr]); //put data
			spi_out_ptr++; //move pointer
			if(spi_out_ptr==BLOCKLENGTH) spi_out_flg=1; //check first part is processed, set flag
			else if(spi_out_ptr==2*BLOCKLENGTH) //check second part is processed, set flag and ring double buffer pointer
			{
				spi_out_ptr=0;
				spi_out_flg=2;
			}	
    }
  }
	
*/	
	

//-----------------------------------------------------------------------------------
//Systen initialization
//-----------------------------------------------------------------------------------
void SYS_Init(void)
{		
		
   //-------------------------Init System Clock---------------------------------------
  
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable clock source */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock source ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_HXTSTB_Msk);

    /* Disable PLL first to avoid unstable when setting PLL */
    CLK_DisablePLL();

    /* Set PLL frequency */
    CLK->PLLCTL = (CLK->PLLCTL & ~(0x000FFFFFul)) | 0x0000C02Eul;

    /* Waiting for PLL ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* If the defines do not exist in your project, please refer to the related clk.h in the clk_h folder appended to the tool package. */
    /* Set HCLK clock */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));

    /* Set PCLK-related clock */
    CLK->CLKSEL0 = CLK->CLKSEL0 & ~CLK_CLKSEL0_PCLK0SEL_Msk;
    CLK->CLKSEL0 = CLK->CLKSEL0 & ~CLK_CLKSEL0_PCLK1SEL_Msk;

    /* Enable IP clock */
    //CLK_EnableModuleClock(CRC_MODULE);
    CLK_EnableModuleClock(DAC_MODULE);  //play
    CLK_EnableModuleClock(EADC_MODULE); //grab
    CLK_EnableModuleClock(ISP_MODULE);
    CLK_EnableModuleClock(PDMA_MODULE);
    CLK_EnableModuleClock(SPI0_MODULE); //rcvd 
    CLK_EnableModuleClock(SPI1_MODULE); //send
    CLK_EnableModuleClock(TMR0_MODULE); //grab
    CLK_EnableModuleClock(TMR1_MODULE); //play
    CLK_EnableModuleClock(TMR2_MODULE); //delay
    CLK_EnableModuleClock(TMR3_MODULE); //event
    CLK_EnableModuleClock(UART0_MODULE);
    //CLK_EnableModuleClock(UART1_MODULE);
    CLK_EnableSysTick(CLK_CLKSEL0_STCLKSEL_HCLK, 0);

    /* Set IP clock */
    CLK_SetModuleClock(EADC_MODULE, MODULE_NoMsk, CLK_CLKDIV0_EADC(8));
    //CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, MODULE_NoMsk);
		CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_HXT, MODULE_NoMsk);
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_HXT, MODULE_NoMsk);
		
		//CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, MODULE_NoMsk); //8KHz sampling rate trigger for ADC
    //CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, MODULE_NoMsk); //8 KHz sampling rate trigger for DAC  
		CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, MODULE_NoMsk); //8KHz sampling rate trigger for ADC
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HXT, MODULE_NoMsk); //8 KHz sampling rate trigger for DAC 
		
		CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_HXT, MODULE_NoMsk);   //40KHz CLK output for SPI
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_HXT, MODULE_NoMsk); //
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    //CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

   
    SystemCoreClockUpdate(); //update system clocks

//-------------------------Init Pins---------------------------------------

    //If the defines do not exist in your project, please refer to the related sys.h in the sys_h folder appended to the tool package.
    SYS->GPA_MFPH = 0x00000000;
		SYS->GPA_MFPL = SYS_GPA_MFPL_PA7MFP_SPI1_CLK | SYS_GPA_MFPL_PA6MFP_SPI1_MISO | SYS_GPA_MFPL_PA5MFP_SPI1_MOSI | SYS_GPA_MFPL_PA4MFP_SPI1_SS;
    //////SYS->GPA_MFPL = SYS_GPA_MFPL_PA3MFP_UART0_RXD | SYS_GPA_MFPL_PA2MFP_UART0_TXD | SYS_GPA_MFPL_PA1MFP_UART1_RXD | SYS_GPA_MFPL_PA0MFP_UART1_TXD;
    //SYS->GPA_MFPL = 0x00000000; //!!!
		SYS->GPB_MFPH = 0x00000000;
    //SYS->GPB_MFPL = SYS_GPB_MFPL_PB7MFP_ACMP0_P0 | SYS_GPB_MFPL_PB5MFP_EADC_CH13| SYS_GPB_MFPL_PB4MFP_ACMP0_N | SYS_GPB_MFPL_PB1MFP_EADC_CH1 | SYS_GPB_MFPL_PB0MFP_DAC;
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB1MFP_EADC_CH1 | SYS_GPB_MFPL_PB0MFP_DAC;
		
		SYS->GPC_MFPH = 0x00000000;
    SYS->GPC_MFPL = 0x00000000;
    /////SYS->GPD_MFPH = 0x00000000;
		SYS->GPD_MFPH = SYS_GPD_MFPH_PD10MFP_T2;
    ////SYS->GPD_MFPL = SYS_GPD_MFPL_PD7MFP_ACMP0_O;
    SYS->GPD_MFPL = SYS_GPD_MFPL_PD6MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD;
		
		
		SYS->GPE_MFPH = SYS_GPE_MFPH_PE13MFP_SPI0_CLK | SYS_GPE_MFPH_PE12MFP_SPI0_SS | SYS_GPE_MFPH_PE11MFP_SPI0_MOSI0 | SYS_GPE_MFPH_PE10MFP_SPI0_MISO0;
    SYS->GPE_MFPL = SYS_GPE_MFPL_PE0MFP_PWM0_CH0;
		
    SYS->GPF_MFPL = SYS_GPF_MFPL_PF6MFP_ICE_DAT | SYS_GPF_MFPL_PF5MFP_ICE_CLK | SYS_GPF_MFPL_PF4MFP_XT1_IN | SYS_GPF_MFPL_PF3MFP_XT1_OUT;

		GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 0));  //DAC
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 1));  //EADC1
		//GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 4));  //N 
    //GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 5));  //CH13
    //GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 7));  //P0		

    SYS_LockReg(); //lock system registers
    return;
}

	
void SYS_PD(void)
{
 //disable all interupts
			 NVIC_DisableIRQ(SPI0_IRQn); //enable IRQ
       SPI_DisableInt(SPI0, SPI_FIFO_RXTH_INT_MASK); //enable RX interupt
       SPI_Close(SPI0);
			 
			 NVIC_DisableIRQ(SPI1_IRQn); //enable IRQ
       SPI_DisableInt(SPI1, SPI_FIFO_TXTH_INT_MASK); //enable TX interupt
       SPI_Close(SPI1);
			 
			 PDMA_DisableInt(DAC_PDMA, PDMA_INT_TRANS_DONE);
       PDMA_DisableInt(ADC_PDMA, PDMA_INT_TRANS_DONE); //enable interupt
       NVIC_DisableIRQ(PDMA_IRQn);  //enable DMA global interupt		
       PDMA_Close();
			 EADC_Close(EADC);
			 DAC_Close(DAC, 0);
			 
       UART_DisableInt(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));
       NVIC_DisableIRQ(UART0_IRQn);  //enable UART0 global interupt
			 UART_Close(UART0);
}




