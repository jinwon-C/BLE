#include "erc_application.h"

static uint32 obj_temp_DigiPile;
static uint16 amp_temp_DigiPile;
static uint16 adcVal_PPG;

extern bool connected;
/*******************************************/
void ERC_Application(void)
{
  // TODO: use timer to measure sensor value
  measureSensor();
  
  if(connected){
    notiSensor();
  }
}

/* measure *******************************************/  
void measureSensor(void)
{
  //measure_PPG();
  
  measure_DigiPile(&obj_temp_DigiPile, &amp_temp_DigiPile);
}

void measure_PPG(void)
{
  adcVal_PPG = HalAdcRead(HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_12);
}

void measure_DigiPile(uint32 *pObj, uint16 *pAmp)
{  
  uint32 dv1=0;
  uint16 dv2=0;
  int iter;
  
  // P1.3 to high
  P1_3 = 1;
  // delay 90 us
  delay_us(150);
  
  for(iter=0; iter<17; iter++){
    // P1.3 to Output
    P1DIR = (P1DIR & 0xf7) | 0x08; // xxxx 1xxx
    // P1.3 low
    P1_3 = 0;
    // delay 200 ns
    delay_32ns(15);
    // P1.3 high
    P1_3 = 1;
    // delay 200 ns
    delay_32ns(15);
    
    // P1.3 to Input
    P1DIR = (P1DIR & 0xf7);
    // P1.3 to pulldown
    P1INP = (P1INP & 0xf7); // 1111 0111
    P2INP = (P2INP | 0x40); // 0100 0000
    
    // wait
    delay_us(5);
    
    dv1 = dv1 << 1;
    if(P1_3)
      dv1++;
  }
  
  for(iter=0; iter<14; iter++){
    // P1.3 to Output
    P1DIR = (P1DIR & 0xf7) | 0x08; // xxxx 1xxx
    // P1.3 low
    P1_3 = 0;
    // delay 200 ns
    delay_32ns(15);
    // P1.3 high
    P1_3 = 1;
    // delay 200 ns
    delay_32ns(15);
    
    // P1.3 to Input
    P1DIR = (P1DIR & 0xf7);
    // P1.3 to pulldown
    P1INP = (P1INP & 0xf7); // 1111 0111
    P2INP = (P2INP | 0x40); // 0100 0000
    
    // wait
    delay_us(5);
    
    dv2 = dv2 << 1;    
    if(P1_3)
      dv2++;
  }
  
  // P1.3 to Output
  P1DIR = (P1DIR & 0xf7) | 0x08; // xxxx 1xxx
  // P1.3 low
  P1_3 = 0;
  // delay 200 ns
  delay_32ns(15);
  
  // P1.3 to Input
  P1DIR = (P1DIR & 0xf7);
  // P1.3 to pulldown
  P1INP = (P1INP & 0xf7); // 1111 0111
  P2INP = (P2INP | 0x40); // 0100 0000
  delay_us(100);
  
  *pObj = dv1;
  *pAmp = dv2;
}

/* send *******************************************/
void notiSensor(void)
{    
  uint8 idx = 0;
  uint8 sendArrPacket[SIMPLEPROFILE_CHAR4_LEN] = {0,};
  
  UTCTimeStruct time;
  osal_ConvertUTCTime(&time, osal_getClock());
  sendArrPacket[idx++] = time.day;
  sendArrPacket[idx++] = time.hour;
  sendArrPacket[idx++] = time.minutes;
  sendArrPacket[idx++] = time.seconds;
  
  sendArrPacket[idx++] = BREAK_UINT32(obj_temp_DigiPile, 0);
  sendArrPacket[idx++] = BREAK_UINT32(obj_temp_DigiPile, 1);
  sendArrPacket[idx++] = BREAK_UINT32(obj_temp_DigiPile, 2);
  sendArrPacket[idx++] = BREAK_UINT32(obj_temp_DigiPile, 3);
  
  sendArrPacket[idx++] = LO_UINT16(amp_temp_DigiPile);
  sendArrPacket[idx++] = HI_UINT16(amp_temp_DigiPile);
  
  SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN, sendArrPacket);
  
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
  static uint8 notiCnt = 0;
  HalLcdWriteStringValue( "Noti:", (uint16)notiCnt++, 10,  HAL_LCD_LINE_1 );   
#endif

}


/* init *******************************************/
void Sensor_Init(void)
{
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
  HalLcdWriteString( "Sensor_Init", HAL_LCD_LINE_1 );
#endif  
  DigiPile_Init(); 
}

void DigiPile_Init(void)
{
  // P1.3 to GPIO
  P1SEL = (P1SEL & 0xf7); // xxxx 0xxx
  // P1.3 to output
  P1DIR = (P1DIR & 0xf7) | 0x08;
  // P1.3 to low level
  P1_3 = 0;
}