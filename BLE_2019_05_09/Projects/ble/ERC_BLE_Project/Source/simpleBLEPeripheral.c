/******************************************************************************
@file  ERC BLE CC2540 Soruce File

Group: KJW
Target Device: CC2540, CC2541
******************************************************************************/

/*********************************************************************
* INCLUDES
*/

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OSAL_Timers.h"
#include "OSAL_Clock.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"
#include "simplekeys.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "simpleBLEPeripheral.h"

#include "erc_application.h"
#include "delays.h"

/*********************************************************************
* MACROS
*/

/*********************************************************************
* CONSTANTS
*/

#define SBP_PERIODIC_EVT_PERIOD               60000//300000
#define DEFAULT_ADVERTISING_INTERVAL          160                               // units of 625us, 160=100ms
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL          // General discoverable mode advertises indefinitely
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80                                // units of 1.25ms, 80=100ms
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800                               // units of 1.25ms, 800=1000ms
#define DEFAULT_DESIRED_SLAVE_LATENCY         0                                 // Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000                              // Supervision timeout value (units of 10ms, 1000=10s)
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE                              // Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6                                 // Connection Pause Peripheral time value (in seconds)
#define TI_COMPANY_ID                         0x000D                            // Company Identifier: Texas Instruments Inc. (13)
#define INVALID_CONNHANDLE                    0xFFFF                            
#define B_ADDR_STR_LEN                        15                                // Length of bd addr as a string

/*********************************************************************
* GLOBAL VARIABLES
*/

bool connected = FALSE;

uint8 adcBufferLength = 0;
uint8 sendSize = 0;

/*********************************************************************
* LOCAL VARIABLES
*/
static uint8 simpleBLEPeripheral_TaskID;                                        // Task ID for internal task/event processing
static gaprole_States_t gapProfileState = GAPROLE_INIT; 
// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =                                                    
{
  // complete name
  0x0b,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'E','R','C','_','D','e','v','i','c','e',
  
  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),
  
  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

static uint8 advertData[] =
{
  0x02, 
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  0x03,
  GAP_ADTYPE_16BIT_MORE,      
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),
  
};

static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Peripheral";

/*********************************************************************
* LOCAL FUNCTIONS
*/
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEPeripheral_ProcessGATTMsg( gattMsgEvent_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void simpleProfileChangeCB( uint8 paramID );

static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );

static char *bdAddr2Str ( uint8 *pAddr );

void read_DigiPile(uint32 *tovj, uint16 *tamp);

void delay_32ns(uint16 iteration);
void delay_us(uint16 microSecs);
void delay_ms(int milliSecs);

/*********************************************************************
* PROFILE CALLBACKS
*/

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};
/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
* @fn      SimpleBLEPeripheral_Init
* @param   task_id - the ID assigned by OSAL.  This ID should be used to send messages and set timers.
* @return  none
*/
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  simpleBLEPeripheral_TaskID = task_id;
  
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
 
  {
    uint8 initial_advertising_enable = TRUE;
    uint16 gapRole_AdvertOffTime = 0;   
    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );   
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
  
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;
    
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }
  
  {
    uint32 passkey = 0; 
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }
  
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile\
  
  {
    uint8 charValue1 = 1;
    uint8 charValue2 = 2;
    uint8 charValue3 = 3;
  
    uint8 charValue4[SIMPLEPROFILE_CHAR4_LEN] = { 0, };
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
  
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN, charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
  }
  
  RegisterForKeys( simpleBLEPeripheral_TaskID );
  
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
  HalLcdWriteString( "BLE Peripheral", HAL_LCD_LINE_1 );
#endif
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );
  
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );
  
  Sensor_Init();
  
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );
}

/*********************************************************************
* @fn      SimpleBLEPeripheral_ProcessEvent
* @param   task_id  - The OSAL assigned task ID.
* @param   events - events to process.  This is a bit map and can
*                   contain more than one event.
* @return  events not processed
*/
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id;
  
  if(events & SYS_EVENT_MSG){
    uint8 *pMsg;
    
    if((pMsg = osal_msg_receive(simpleBLEPeripheral_TaskID)) != NULL){
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
      VOID osal_msg_deallocate( pMsg );                                         // Release the OSAL message
    }
    return (events ^ SYS_EVENT_MSG);
  }
  
  if(events & SBP_START_DEVICE_EVT){
    VOID GAPRole_StartDevice(&simpleBLEPeripheral_PeripheralCBs);
    VOID GAPBondMgr_Register(&simpleBLEPeripheral_BondMgrCBs);
    osal_start_timerEx(simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD);
    
    return (events ^ SBP_START_DEVICE_EVT);
  }
  
  if(events & SBP_PERIODIC_EVT){
    if(SBP_PERIODIC_EVT_PERIOD)                                              // Restart timer
      osal_start_timerEx(simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD);
    performPeriodicTask();
    
    return (events ^ SBP_PERIODIC_EVT);
  }
  
  return 0;
}

/*********************************************************************
* @fn      simpleBLEPeripheral_ProcessOSALMsg
* @param   pMsg - message to process
* @return  none
*/
static void simpleBLEPeripheral_ProcessOSALMsg(osal_event_hdr_t *pMsg)
{
  switch(pMsg->event)
  {
  case KEY_CHANGE:
    simpleBLEPeripheral_HandleKeys(((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys);
    break;
    
  case GATT_MSG_EVENT:                                                          // Process GATT message
    simpleBLEPeripheral_ProcessGATTMsg((gattMsgEvent_t *)pMsg);
    break;
    
  default:
    break;
  }
}

/*********************************************************************
* @fn      simpleBLEPeripheral_HandleKeys
* @param   shift - true if in shift/alt.
* @param   keys - bit field for key events. Valid entries: HAL_KEY_SW_2, HAL_KEY_SW_1
* @return  none
*/
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;
  VOID shift;                                                                   // Intentionally unreferenced parameter
  
  if(keys & HAL_KEY_SW_1){
    SK_Keys |= SK_KEY_LEFT;
  }  
  if(keys & HAL_KEY_SW_2){
    SK_Keys |= SK_KEY_RIGHT;
    uint8 current_adv_enabled_status, new_adv_enabled_status;
    
    GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status);  //Find the current GAP advertisement status
    
    if(current_adv_enabled_status == FALSE){
      new_adv_enabled_status = TRUE;
    }else{
      new_adv_enabled_status = FALSE;
    }
    
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &new_adv_enabled_status);//change the GAP advertisement status to opposite of current status
  }
  SK_SetParameter(SK_KEY_ATTR, sizeof(uint8), &SK_Keys);
}


/*********************************************************************
* @fn      simpleBLEPeripheral_ProcessGATTMsg
* @return  none
*/
static void simpleBLEPeripheral_ProcessGATTMsg( gattMsgEvent_t *pMsg )
{  
  GATT_bm_free( &pMsg->msg, pMsg->method );
}

/*********************************************************************
* @fn      peripheralStateNotificationCB
* @param   newState - new state
* @return  none
*/
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  switch (newState)
  {
  case GAPROLE_STARTED:
    {
      uint8 ownAddress[B_ADDR_LEN];
      uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
      
      GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
      
      systemId[0] = ownAddress[0];
      systemId[1] = ownAddress[1];
      systemId[2] = ownAddress[2];
      systemId[4] = 0x00;
      systemId[3] = 0x00;
      systemId[7] = ownAddress[5];
      systemId[6] = ownAddress[4];
      systemId[5] = ownAddress[3];
      
      DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);  
      
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
      HalLcdWriteString(bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2);
      HalLcdWriteString("Initialized",  HAL_LCD_LINE_3);
#endif
    }
    break;
    
  case GAPROLE_ADVERTISING:
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
    HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
#endif
    break;
    
  case GAPROLE_CONNECTED:
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
    HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
    connected = TRUE;
#endif
    
    break;
    
  case GAPROLE_CONNECTED_ADV:
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
    HalLcdWriteString( "Connected Advertising",  HAL_LCD_LINE_3 );
#endif
    break;
    
  case GAPROLE_WAITING:
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
    HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
    connected = FALSE;
#endif
    break;
    
  case GAPROLE_WAITING_AFTER_TIMEOUT:
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
    HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
#endif
    break;
    
  case GAPROLE_ERROR:
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
    HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
#endif
    break;
    
  default:
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
    HalLcdWriteString( "",  HAL_LCD_LINE_3 );
#endif
    break; 
  }
  
  gapProfileState = newState;
  
#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with "CC2540 Slave" configurations
#endif
}

/*********************************************************************
* @fn      performPeriodicTask
* @param   none
* @return  none
*/
static void performPeriodicTask( void )
{
  ERC_Application();
}

/*********************************************************************
* @fn      simpleProfileChangeCB
*
* @brief   Callback from SimpleBLEProfile indicating a value change
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  none
*/
static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newValue;
  
  switch( paramID )
  {
  case SIMPLEPROFILE_CHAR1:
    SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue );
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
    HalLcdWriteStringValue( "Char 1:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );   
#endif
    break;
    
  case SIMPLEPROFILE_CHAR3:
    SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue );
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
    HalLcdWriteStringValue( "Char 3:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
#endif
    break;
    
  default:
    break;
  }
}

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
* @fn      bdAddr2Str
*
* @brief   Convert Bluetooth address to string. Only needed when
*          LCD display is used.
*
* 
@return  none
*/
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

/*********************************************************************
*********************************************************************/