#include "ioCC2540.h"
//#inlcude "ioCC2541.h"
#include "hal_types.h"
#include "hal_adc.h"
#include "hal_lcd.h"
#include "simpleGATTprofile.h"
#include "bcomdef.h"
#include "att.h"
#include "OSAL.h"
#include "OSAL_Clock.h"
#include "OSAL_Clock.h"

#include "queue.h"
#include "delays.h"


// CC2540   => SmartRF
// USE P1_3 => P18_4

// P20_20, P18_20 == GND
// P20_3 == VCC


typedef struct DigiPilePacketS{
  uint8 time_day;
  uint8 time_hour;
  uint8 time_minutes;
  uint8 time_seconds;
  uint8 obj[4];
  uint8 amp[2];  
}DigiPilePacketTypedef;

void ERC_Application(void);
void DigiPile_Init(void);

void measureSensor(void);
void measure_PPG(void);
void measure_DigiPile(uint32 *pObj, uint16 *pAmp);

void notiSensor(void);
void Sensor_Init(void);

