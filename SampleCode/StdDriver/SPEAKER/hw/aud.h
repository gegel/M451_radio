#define FRAMELENGTH  160 //number of PCM sampling in AMR frame
//#define UARTLEN 64 //maximal length of UART data packet
#define TMR8KR 1500 //72 MHz timer period for 8000KHz sampling rate for recording -20 ->2/3  +20 -> 1/2    0 -> 1/2 
#define TMR8KP 1500 //72 MHz timer period for 8000KHz sampling rate for playing +20 ->2/3  -20 -> 1/2
//#define TMR1MC 119   //72 MHz timer divider for 100 KHz clock
#define OUT_RATE_COEF 1

//ADC settings
#define ADC_MOD 0  //EADC module used for recording
#define ADC_CH 1   //PB1, pin45 EADC channel used for recording

//DMA settingth
#define ADC_PDMA 2 //PDMA channel used for recording
#define DAC_PDMA 0 //PDMA channel used for playing
//#define TXD_PDMA 1 //PDMA channel used for data UART0 TX 
//#define RXD_PDMA 4 //pdma channel used for data UART0 RX
//#define RX_TOUT 2000 //commbytetimeout in 10 uS: 20 mS


void aud_init(void);

void aud_set(short rate);
unsigned short* aud_grab(void);
unsigned short* aud_play(void);


//signed char com_poll(void);
//unsigned char* com_read(void);
//unsigned char com_write(unsigned char* data, unsigned char len);




