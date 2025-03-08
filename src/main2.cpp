/* BCU
Minimum Voltage of 2 Module : 3.1*20 = 62V , Nominal Volage 3.6*20 = 72V
Maximum allowable Voltage for 2 module : 83V => 830 => 0x 03 3E
Maximum allowable current for 2 module : 5A => 50 => 0x 00 32
Priority Table, Check Charge -> NO
Read From SDC
Read from BMU
Read from BAMOCAR
RTOS and Push ROS topics
*/
/************************* Includes ***************************/
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <driver/gpio.h>
#include <driver/twai.h>
#include <freertos/timers.h>       
// #include "LittleFS.h"
// #include "SD.h"
// #include "SPI.h"
#include <Arduino.h>
#include <EEPROM.h>
// utility function
#include <util.h>
#include <AMS.h>
// #include <micro_ros_arduino.h>
/************************* Define macros *****************************/
#define CAN_RX  GPIO_NUM_36 // This will be 
#define CAN_TX  GPIO_NUM_35
// ADC pin
#define APS1 GPIO_NUM_4
#define APS2 GPIO_NUM_5
#define BPS1 GPIO_NUM_6
#define BPS2 GPIO_NUM_7
#define CURR GPIO_NUM_8
// Digital input
#define BSPDIN GPIO_NUM_15  // Check BSPD OK signal from BSPD directly 
#define IMDIN GPIO_NUM_16   // Check IMD OK signal from IMD directly
#define SDCIN GPIO_NUM_17 // AIR+   // Check OK signal from Shutdown Circuit => Its status must match all below Signal pin , otherwise faulty
#define EMR_O GPIO_NUM_18
#define OBCIN GPIO_NUM_9   // Check OK Signal from Charging Shutdown Circuit, indicates that it is charged
// Digital Output 
// #define AMS_OUT GPIO_NUM_47  // OUTPUT Fault Signal to BMS relay
#define AMS_OUT GPIO_NUM_21 // SL 1 => Check with multimeter
// #define AMS_OUT GPIO_NUM_47 // SL 1 => Check with multimeter
#define LVlight GPIO_NUM_48
#define TEMPlight GPIO_NUM_47
// Macros
#define OBC_SYNC_TIME 500
#define SYNC_TIME 200
#define DISCONNENCTION_TIMEOUT 650
#define BCU_ADD 0x7FF
#define OBC_ADD 0x1806E5F4
/**************** Setup Variables *******************/
twai_message_t sendMessage;
twai_message_t J1939msg;
twai_message_t receivedMessage;

// Software Timer
unsigned long reference_time = 0; 
unsigned long reference_time2 = 0;
unsigned long reference_time3 = 0;
unsigned long communication_timer1 = 0;
unsigned long shutdown_timer1 = 0;
// unsigned long lastModuleResponse[BMU_NUM] = {0};
unsigned long lastModuleResponse[BMU_NUM];
// Hardware Timer
hw_timer_t *My_timer1 = NULL;
hw_timer_t *My_timer2 = NULL;

// BMU data , Accumulator data structure, Sensing data.
BMUdata BMU_Package[BMU_NUM];
AMSdata AMS_Package;
LVsignal SDC_Signal_Board;
OBCdata OBC_Package;

// Alias names
bool &AMS_OK = AMS_Package.AMS_OK;
uint8_t &OBCFault = OBC_Package.OBCstatusbit;
bool &ACCUM_Full = AMS_Package.OVERVOLT_WARNING;
float &ACCUM_MAXVOLTAGE = AMS_Package.ACCUM_MAXVOLTAGE; 
float &ACCUM_MINVOLTAGE = AMS_Package.ACCUM_MINVOLTAGE;  

// Flags
volatile bool CAN_SEND_FLG1 = false;
volatile bool CAN_SEND_FLG2 = false;
bool CHARGER_PLUGGED = false;
bool CAN_TIMEOUT_FLG = false;
bool ACCUM_ReadytoCharge = 0;
bool ACCUM_OverDivCritical = 0;
bool LOW_VOLT_WARN = 0;
bool OVER_TEMP_WARN = 0;

// Default Parameter preparing for BMU to save in its non-volatile memory.
const byte Sync = SYNC_TIME; 
const byte VmaxCell = (byte) (VMAX_CELL / 0.1) ;
const byte VminCell = (byte) (VMIN_CELL / 0.1) ;
const byte TempMaxCell = (TEMP_MAX_CELL);
const byte dVmax = (DVMAX / 0.1); 
const bool BMUUpdateEEPROM = 0; // Flag for BMU to update its EEPROM

// Task handler
TaskHandle_t Task1, Task2, Task3, Task4;
// Task 0 Handle SDC relay [prio]
// Task 1 send (use two global flag)
// Task 2 received ()
// Task 3 get AMSout

/**************** Local Function Delcaration *******************/
void packBMUmsg ( twai_message_t *BCUsent, uint16_t Sync_time,  bool &is_charger_plugged);
void packOBCmsg ( twai_message_t *BCUsent, bool &BMS_OK , bool &ReadytoCharge, bool &OverDivCritical_Yes, bool &Voltage_is_Full);
void unpackOBCmsg ( twai_message_t *BCUreceived );
void unpackBMUmsgtoAMS ( twai_message_t *BCUreceived, BMUdata *BMS_ROSPackage);
void debugBMUmsg(int Module);
void debugBMUFault(int Module);
void debugOBCmsg();
void debugFrame();
bool checkModuleDisconnect();
void twaiTroubleshoot();
void resetAllStruct();
void resetModuleData(int moduleIndex);
void sensorReading();
void dynamicModulereset();

/*******************************************************************
  ==============================Setup==============================
********************************************************************/

void IRAM_ATTR onTimer1() {
  CAN_SEND_FLG1 = 1;
}
void IRAM_ATTR onTimer2() {
  CAN_SEND_FLG2 = 1;
}

void setup() {

  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  /* Shutdown System setup */
  pinMode(SDCIN,INPUT_PULLDOWN); 
  pinMode(OBCIN,INPUT_PULLDOWN);
  pinMode(IMDIN,INPUT_PULLDOWN);
  pinMode(BSPDIN,INPUT_PULLDOWN);
  pinMode(EMR_O,INPUT_PULLDOWN);
  pinMode(AMS_OUT,OUTPUT);
  pinMode(LVlight,OUTPUT);
  pinMode(TEMPlight,OUTPUT);

  // ADC
  // punMode(BPS1,INPUT);
      
  /* CAN Communication Setup */
  sendMessage.extd = false;
  receivedMessage.extd = false;
  J1939msg.extd = true;
  twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  general_config.tx_queue_len = 800; // worstcase is 152 bit/frame , this should hold about 5 frame
  general_config.rx_queue_len = 1300; // RX queue hold about 8 frame
  twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // May config this later
  // Install the TWAI driver
  if (twai_driver_install(&general_config, &timing_config, &filter_config) == !ESP_OK) {
    Serial.println("TWAI Driver install failed__");
    while(1);
  }
  // Start the TWAI driver
    if (twai_start() == ESP_OK) {
      Serial.println("TWAI Driver installed , started");
      // Reconfigure the alerts to detect the error of frames received, Bus-Off error and RX queue full error
      uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
      if (twai_reconfigure_alerts(alerts_to_enable, NULL) == !ESP_OK) {
        Serial.println("Failed to reconfigure alerts");
        while(1);
      }
    }

  // Hardware periodic timer 100ms & 500ms
  My_timer1 =  timerBegin(0, 80, true);  // Timer with prescaler
  timerAttachInterrupt(My_timer1, &onTimer1, true);
  timerAlarmWrite(My_timer1, 100 * 1000, true);  // 100ms interval
  timerAlarmEnable(My_timer1);

  // Setup timer for 500ms intervals 
  My_timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(My_timer2, &onTimer2, true);
  timerAlarmWrite(My_timer2, 500 * 1000, true);  // 500ms interval
  timerAlarmEnable(My_timer2);

  
  // FreeRTOS tasks core 0 USB or Uart serial (Disable BackGround task like WiFi BLe , to make core 0 faster)

  /* CORE0 for coordinating BMS and Electrical System*/

  /*-------------- BMSROS_Package debug with SD card logger -------------*/
  // Packing data to SD
  // packing data to Remote System using CORE0
  /*-------------- CORE 0 Tasks -------------*/
  // ROS , Ethernet publishy function

  /*-------------- CORE 1 Tasks -------------*/
    Serial.println("BCU__initialized__"); 

}

/*******************************************************************
  =======================Mainloop===========================
********************************************************************/
/* CORE0 for coordinating with Telemetry socket CAN*/
/* CORE1 for coordinating BMS and Electrical System*/
  
void loop(){
  // Check if Charger LV AUX plug is actually into ACCUM 2nd Floor connector (May change to external interrupt)
  // (digitalRead(OBCIN)) ? (CHARGER_PLUGGED = true) : (CHARGER_PLUGGED = false);
  /*___Task 1 : Communication ====================================================*/
  // ------------------------------------Message Transmission
  
  // BCU CMD & SYNC   (100ms cycle Broadcast to all BMU modules in Bus) 
  if (CAN_SEND_FLG1)
  {
    CAN_SEND_FLG1 = 0; // reset
    packBMUmsg(&sendMessage, SYNC_TIME , CHARGER_PLUGGED);
    twai_transmit(&sendMessage, 1);
  } 
  if(CAN_SEND_FLG2 && CHARGER_PLUGGED)
  {
    CAN_SEND_FLG2 = 0; // reset
    packOBCmsg(&sendMessage, AMS_OK, ACCUM_ReadytoCharge , ACCUM_OverDivCritical , ACCUM_Full);
    // **NOTE** 3rd to 5th Parameter will be updated via unpackBMUmsgtoAMS(); function
    twai_transmit(&sendMessage, 1);
  }
 
  //------------------------------------Message Reception

  // if RX buffer isn't cleared after its full within 1 tick Queue overflow alert will fired
  if (twai_receive(&receivedMessage, 1) == ESP_OK) 
  {
    // Serial.printf("BMU1 connect: %d, ",BMU_Package[0].BMUconnected);
    // Serial.printf("BMU2 connect: %d, ",BMU_Package[1].BMUconnected);
    // Serial.printf("BMU3 connect: %d \n",BMU_Package[2].BMUconnected);
    // debugFrame();
    // Reset BMU data for the one that has disconnected from CAN Bus

    // Reset BMU struct Value
    dynamicModulereset();
    // Unpack CAN frame and insert to BMU_Package[i] , AMS_Package:
    unpackBMUmsgtoAMS(&receivedMessage, BMU_Package); // 200ms cycle & 500ms cycle of faultcode

    if(CHARGER_PLUGGED)
      unpackOBCmsg(&receivedMessage); // Unpack CAN frame and insert to OBC_Package:  500ms cycle 
      
    // Update timeout flag and communication_timer
    CAN_TIMEOUT_FLG = false;
    communication_timer1 = millis();
  }
  // if No module is connected from CAN bus , run this code and return until the bus is active
  else if (millis()-communication_timer1 >= DISCONNENCTION_TIMEOUT){
    // Shutdown
    digitalWrite(AMS_OUT,HIGH);
    if( millis()-shutdown_timer1 >= 400 ){
        Serial.println("NO_BYTE_RECV");
        shutdown_timer1 = millis();
    }
    // resetAllStruct() only once
    if(CAN_TIMEOUT_FLG == false)
      resetAllStruct();
    // Mark flag true to bypass resetAllstruct() afterward.
    CAN_TIMEOUT_FLG = true;
    return;
  }

  // In case if any module disconnect
  if(checkModuleDisconnect() == 0){
    // Serial.println("DISCON");

    // Shutdown immediately
    digitalWrite(AMS_OUT,LOW);
    if( millis()-shutdown_timer1 >= 400 ){
        Serial.println("MODULE_DISCONNECT -- SHUTDOWN");
        Serial.printf("BMU1 connect: %d, ",BMU_Package[0].BMUconnected);
        Serial.printf("BMU2 connect: %d, ",BMU_Package[1].BMUconnected);
        Serial.printf("BMU3 connect: %d ",BMU_Package[2].BMUconnected);
        Serial.printf("BMU4 connect: %d \n",BMU_Package[3].BMUconnected);
        shutdown_timer1 = millis();
    }
    return;
  }
  
  
    
  /*___Task 2 : Determine AMS_OK relay state ==================================================== */

    // Fault Table , OR-ed each type of Fault code bit which has been filled up when unpackBMUmsgtoAMS();
    // Warning Condition for Dashboard light: LowVoltage , Module OverTemperature , Full Voltage
    (AMS_Package.ACCUM_VOLTAGE >= 0.9 * ACCUM_MAXVOLTAGE) ? (ACCUM_Full = 1) : (ACCUM_Full = 0) ;
    (AMS_Package.ACCUM_VOLTAGE <= 1.10 * ACCUM_MINVOLTAGE) ? (LOW_VOLT_WARN = 1) : (LOW_VOLT_WARN = 0);

    // (AMS_Package.ACCUM_VOLTAGE >= ACCUM_MAXVOLTAGE) ? (AMS_Package.OVERVOLT_CRITICAL = 1) : (AMS_Package.OVERVOLT_CRITICAL = 0) ;
    // (AMS_Package.ACCUM_VOLTAGE <= ACCUM_MINVOLTAGE) ? (AMS_Package.LOWVOLT_CRITICAL = 1) : (AMS_Package.LOWVOLT_CRITICAL = 0);

    (AMS_Package.ACCUM_VOLTAGE > ACCUM_MAXVOLTAGE) ? (AMS_Package.OVERVOLT_CRITICAL = 1) : (AMS_Package.OVERVOLT_CRITICAL = 0) ;
    (AMS_Package.ACCUM_VOLTAGE < ACCUM_MINVOLTAGE) ? (AMS_Package.LOWVOLT_CRITICAL = 1) : (AMS_Package.LOWVOLT_CRITICAL = 0);

    
    
    // may delete
    (AMS_Package.OVERTEMP_WARNING > 0) ? (OVER_TEMP_WARN = 1) : (OVER_TEMP_WARN = 0);
    
    // Fault Condition for AMS_OK Shutdown
    bool ACCUMULATOR_Fault;
    // Dummy Condition for headless testing
    // AMS_Package.LOWVOLT_CRITICAL = 0;
    // AMS_Package.OVERVOLT_CRITICAL = 0;
    // AMS_Package.OVERTEMP_CRITICAL = 0;
    ACCUMULATOR_Fault = AMS_Package.OVERVOLT_CRITICAL | AMS_Package.LOWVOLT_CRITICAL | AMS_Package.OVERTEMP_CRITICAL;
    (ACCUMULATOR_Fault > 0) ? (AMS_OK = 0) : (AMS_OK = 1);
    
    /*------- Coordiante BMU cell Balancing with OBC ------------*/
    if(CHARGER_PLUGGED)
    {
      // if differential voltage between any cells are critical, then at AMS level is fault
      (AMS_Package.OVERDIV_CRITICAL > 0) ? (ACCUM_OverDivCritical = 1) : (ACCUM_OverDivCritical = 0);
      // Set ReadytoCharge Flag, only if All BMU are ready to charge.
      (AMS_Package.ACCUM_CHG_READY > 0) ? (ACCUM_ReadytoCharge = 1) : (ACCUM_ReadytoCharge = 0);
      ((ACCUMULATOR_Fault | OBCFault) > 0) ? (AMS_OK = 0) : (AMS_OK = 1);
    }
     
  /* Task 3 : Shutdown , Dash Board interaction (Should be 1st priority ) ==================================================== */ 
    // LOW_VOLT_WARN = 0;
    
    // For Active High Switch (Butterfly board)
    (AMS_OK) ? digitalWrite(AMS_OUT,HIGH) : digitalWrite(AMS_OUT,LOW);
    
    // For Active Low switch (Yss Blackboard)
    // (AMS_OK) ? digitalWrite(AMS_OUT,LOW) : digitalWrite(AMS_OUT,HIGH);
    
    (LOW_VOLT_WARN) ? digitalWrite(LVlight,HIGH) : digitalWrite(LVlight,LOW);
    (OVER_TEMP_WARN) ? digitalWrite(TEMPlight,HIGH) : digitalWrite(TEMPlight,LOW);
    // sensorReading();

    // Debug AMS state
    if(millis()-reference_time >= 200) {
      // Serial.printf("BMU 0 ov cri: %d \n", BMU_Package[0].OVERVOLTAGE_CRITICAL);
      // Serial.printf("AMS ov cri: %d \n", AMS_Package.OVERVOLT_CRITICAL);
      Serial.printf("AMS_OK: %d \n", AMS_OK);
      Serial.printf("AMS_VOLT: %f \n", AMS_Package.ACCUM_VOLTAGE);
      Serial.printf("AMS_MAX: %f \n", AMS_Package.ACCUM_MAXVOLTAGE);
      Serial.printf("AMS_MIN: %f \n", AMS_Package.ACCUM_MINVOLTAGE);
      Serial.printf("OV_CRIT: %d \n", AMS_Package.OVERVOLT_CRITICAL);
      Serial.printf("LV_CRT: %d \n", AMS_Package.LOWVOLT_CRITICAL);
      Serial.printf("OVT_CRT: %d \n", AMS_Package.OVERTEMP_CRITICAL);

      // Serial.printf("OBC_VOLT: %d \n",OBC_Package.OBCVolt);
      // Serial.printf("OBC_AMP: %d \n",OBC_Package.OBCAmp);
      
      // Serial.printf("AIRplus: %c \n", SDC_Signal_Board.AIRplus);
      // Serial.printf("IMD_Relay: %c \n", SDC_Signal_Board.IMD_Relay);
      // Serial.printf("BSPD_Relay: %c \n", SDC_Signal_Board.BSPD_Relay);
      // Serial.printf("Emergency Button: %c \n", SDC_Signal_Board.BSPD_Relay);

      // Serial.printf("BrakePressure1: %2f \n", SDC_Signal_Board.BrakePressure1);
      // Serial.printf("BrakePressure2: %2f \n", SDC_Signal_Board.BrakePressure2);
      // Serial.printf("AccelPedal1: %2f \n", SDC_Signal_Board.AccelPedal1);
      // Serial.printf("AccelPedal2: %2f \n", SDC_Signal_Board.AccelPedal2);
      // Serial.printf("Current_Sense: %2f \n", SDC_Signal_Board.CurrentSense);

      // 2nd one is connected to dummy test kit
      // debugBMUmsg(0);
      // debugBMUFault(0);
      // twaiTroubleshoot();
      reference_time= millis();
    }

    
} 

/* ==================================Main Local Functions==============================*/
// Decalre Subfunction
  bool isModuleActive(int moduleIndex);
  void resetModuleData(int moduleIndex);
  void recalculateAMSPackage (int moduleIndex);

void packBMUmsg ( twai_message_t *BCUsent, uint16_t Sync_time,  bool &is_charger_plugged) {

  BCUsent->identifier  = BCU_ADD; // BCU ID
  BCUsent->data_length_code = 8;
  // BMU synchronize time
  BCUsent->data[0] = Sync_time;
  // Notify Charge
  (is_charger_plugged) ? (BCUsent->data[1] = 1) : (BCUsent->data[1] = 0);
  // Distribute Default parameter
  BCUsent->data[2] = VmaxCell; 
  BCUsent->data[3] = VminCell; 
  BCUsent->data[4] = TempMaxCell; 
  BCUsent->data[5] = dVmax;
  BCUsent->data[6] = BMUUpdateEEPROM; 
  BCUsent->data[7] = 0x00; // Reserved
}
void unpackBMUmsgtoAMS ( twai_message_t* BCUreceived , BMUdata *BMU_Package) {
  
  // decodeCANID according to BP16 agreement
  StandardCANIDDecoded decodedCANID;
  decodeStandardCANID(&decodedCANID, (BCUreceived->identifier) );
  
  // Distingush Module ID
    int i = decodedCANID.SRC - 1; // SRC stats at 0x01, subtract 1 for indexing.
    if(i >= BMU_NUM) return;      // Bounds check
  
    // Mark timestamp of successfully received Module, No update for disconnected BMU.
    lastModuleResponse[i] = millis();
  
  /* ---------------- unpack ReceiveFrame to BMUframe ------------------- */
  /*  Message Priority 11 :: BMUModule & Cells data  */
  if(decodedCANID.PRIORITY == 0x02)
  {
    switch (decodedCANID.MSG_NUM) { 
      // MSG1 == Operation status
      case 1:
        // Charging Ready
        BMU_Package[i].BMUreadytoCharge = BCUreceived->data[0];
        // Balancing Discharge cell number
        BMU_Package[i].BalancingDischarge_Cells = mergeHLbyte(BCUreceived->data[1],BCUreceived->data[2]);
        // Vbatt (Module) , dVmax(cell)
        BMU_Package[i].V_MODULE = BCUreceived->data[3]; 
        BMU_Package[i].DV = BCUreceived->data[4]; 
        // Temperature sensor
        BMU_Package[i].TEMP_SENSE[0] = BCUreceived->data[5];
        BMU_Package[i].TEMP_SENSE[1] = BCUreceived->data[6];
        break;

      case 2:
        // Low series Side Cell C1-C8
        for(short j=0; j< 8; j++)
          BMU_Package[i].V_CELL[j] = BCUreceived->data[j];
        break;

      case 3:
        // High series side Cell C8-CellNumber
        for( short j = 8; j < CELL_NUM; j++ )
          BMU_Package[i].V_CELL[j] = BCUreceived->data[(j-8)];
        break;
    }
  }
  /*  Message Priority 10 :: FaultCode  */
  else if(decodedCANID.PRIORITY == 0x01)
  {
    switch (decodedCANID.MSG_NUM) {
      case 1:
        // Merge H and L byte of each FaultCode back to 10 bit binary
        BMU_Package[i].OVERVOLTAGE_WARNING =  mergeHLbyte( BCUreceived->data[0], BCUreceived->data[1] );  
        BMU_Package[i].OVERVOLTAGE_CRITICAL = mergeHLbyte( BCUreceived->data[2], BCUreceived->data[3] );  
        BMU_Package[i].LOWVOLTAGE_WARNING = mergeHLbyte( BCUreceived->data[4], BCUreceived->data[5] );
        BMU_Package[i].LOWVOLTAGE_CRITICAL = mergeHLbyte( BCUreceived->data[6], BCUreceived->data[7] ); 
        break;
      case 2:
        // Merge H and L byte of each FaultCode back to 10 bit binary
        BMU_Package[i].OVERTEMP_WARNING = mergeHLbyte( BCUreceived->data[0], BCUreceived->data[1] );  
        BMU_Package[i].OVERTEMP_CRITICAL = mergeHLbyte( BCUreceived->data[2], BCUreceived->data[3] ); 
        BMU_Package[i].OVERDIV_VOLTAGE_WARNING = mergeHLbyte( BCUreceived->data[4], BCUreceived->data[5] ); 
        BMU_Package[i].OVERDIV_VOLTAGE_CRITICAL = mergeHLbyte( BCUreceived->data[6], BCUreceived->data[7] );
        break;
    }
    // Serial.println(BMU_Package[i].OVERVOLTAGE_CRITICAL); //
  }
  
  // Reset Accumulator Parameter before dynamically recalculate based on BMU current state
  AMS_Package = AMSdata();
  // Pack BMUframe to AMSframe according to the following condition
  for(int j = 0; j <BMU_NUM ; j++)
  {
    // if Module is connected to CANbus, set as connect, and recalculateAMS package a new
    if(isModuleActive(j)){ 
        BMU_Package[j].BMUconnected = 1;
        recalculateAMSPackage(j);
    } else {
        BMU_Package[j].BMUconnected = 0;
    }
  }
}
void packOBCmsg ( twai_message_t *BCUsent, bool &AMS_OK , bool &ReadytoCharge, bool &OverDivCritical_Yes, bool &Voltage_is_Full) {

  /* Set up BMS CAN frame*/
  BCUsent->identifier  = OBC_ADD; // refers to specification sheet
  BCUsent->data_length_code = 8;
  // Reserved
  BCUsent->data[5] = 0x00;
  BCUsent->data[6] = 0x00;
  BCUsent->data[7] = 0x00;
  // Condition 1 iF BMS_OK AND ACCUM is Ready Any Module not Critically OVERDIV, ACCUMULATOR VOLTAGE ISN"T FULL YET
  if((AMS_OK && ReadytoCharge) || !OverDivCritical_Yes || !Voltage_is_Full) {
    BCUsent->data[0] = 0x0C; // V highbyte 
    BCUsent->data[1] = 0x80; // V lowbyte 72.0 V fake data -> Range 69-72-74 V
    BCUsent->data[2] = 0x00; // A Highbyte
    BCUsent->data[3] = 0x32; // A Lowbyte 5.0 A fake data
    BCUsent->data[4] = 0x00; // Control Byte 0 charger operate
  } else {
    // Condition 0 Shutdown message
    BCUsent->data[0] = 0x00; 
    BCUsent->data[1] = 0x00; 
    BCUsent->data[2] = 0x00; 
    BCUsent->data[3] = 0x00;
    BCUsent->data[4] = 0x01; // Control Byte 1 charger shutdown
  } 
}
void unpackOBCmsg ( twai_message_t *BCUreceived ) {
  // if message ID isnt 0x18FF50E5 , return
  if(BCUreceived->identifier != 0x18FF50E5)
    return;
  
    // Monitor & Translate current Frame data
    uint8_t VoutH = BCUreceived->data[0];
    uint8_t VoutL = BCUreceived->data[1];
    uint8_t AoutH = BCUreceived->data[2];
    uint8_t AoutL = BCUreceived->data[3];
    OBC_Package.OBCstatusbit =  BCUreceived->data[4]; // Status Byte
    OBC_Package.OBCVolt = mergeHLbyte(VoutH,VoutL);
    OBC_Package.OBCAmp = mergeHLbyte(AoutH,AoutL);

}

/* ==================================Sub Functions==============================*/
void twaiTroubleshoot(){
  //Debug and troubleshoot TWAI bus
  /*
      TWAI_ALERT_RX_DATA        0x00000004    Alert(4)    : A frame has been received and added to the RX queue
      TWAI_ALERT_ERR_PASS       0x00001000    Alert(4096) : TWAI controller has become error passive
      TWAI_ALERT_BUS_ERROR      0x00000200    Alert(512)  : A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus
      TWAI_ALERT_RX_QUEUE_FULL  0x00000800    Alert(2048) : The RX queue is full causing a frame to be lost
      */
  //Error Alert message
  uint32_t alerts_triggered;
  twai_status_info_t status_info;
  // Check if alert triggered
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(1));
  twai_get_status_info(&status_info);
  Serial.println(alerts_triggered);
  
}

bool isModuleActive(int moduleIndex) {
  unsigned int MAX_SILENCE = DISCONNENCTION_TIMEOUT;
  return (millis() - lastModuleResponse[moduleIndex]) <= (MAX_SILENCE);
}
void resetModuleData(int moduleIndex){
  BMU_Package[moduleIndex].~BMUdata(); // Explicitly call destructor (optional)
  new (&BMU_Package[moduleIndex]) BMUdata();
}
void recalculateAMSPackage (int moduleIndex) {
  int &i = moduleIndex;
  // Recalculate AMS based on current BMU states
  AMS_Package.ACCUM_VOLTAGE += (static_cast<float> (BMU_Package[i].V_MODULE)) * 0.2;
  AMS_Package.OVERVOLT_WARNING |= BMU_Package[i].OVERVOLTAGE_WARNING;
  AMS_Package.LOWVOLT_WARNING |= BMU_Package[i].LOWVOLTAGE_WARNING;
  AMS_Package.OVERTEMP_WARNING |= BMU_Package[i].OVERTEMP_WARNING;
  AMS_Package.OVERDIV_CRITICAL |= BMU_Package[i].OVERDIV_VOLTAGE_WARNING;
  AMS_Package.OVERVOLT_CRITICAL |= BMU_Package[i].OVERVOLTAGE_CRITICAL;
  AMS_Package.LOWVOLT_CRITICAL |= BMU_Package[i].LOWVOLTAGE_CRITICAL;
  AMS_Package.OVERTEMP_CRITICAL |= BMU_Package[i].OVERTEMP_CRITICAL;

  AMS_Package.ACCUM_CHG_READY &= (BMU_Package[i].BMUreadytoCharge);
  AMS_Package.OVERDIV_CRITICAL |= BMU_Package[i].OVERDIV_VOLTAGE_CRITICAL;
  
  // Available Module , and 
}
void resetAllStruct(){
  for (int i = 0; i < BMU_NUM; i++){
    resetModuleData(i);
    lastModuleResponse[i] = 0;
  }
  AMS_Package = AMSdata();
  SDC_Signal_Board = LVsignal();
  OBC_Package = OBCdata();
}
bool checkModuleDisconnect(){

  bool disconnectedFromCAN = 1; 
  for(short i =0; i< BMU_NUM ; i++){
    if(BMU_Package[i].BMUconnected == 0)
      disconnectedFromCAN = 0; 
  }
    return disconnectedFromCAN;
}
void dynamicModulereset(){ 
  for(short i =0; i< BMU_NUM ; i++){
    // if any of the board aren't in connection => throw error
    if(BMU_Package[i].BMUconnected == 0){
      resetModuleData(i); // Reset that module data (revert voltage , temp., flags , etc. Back to zero)
    }   
  }
}

void sensorReading(){
  // Read SDC output signal to AIR+
  SDC_Signal_Board.AIRplus = digitalRead(SDCIN);
  SDC_Signal_Board.IMD_Relay = digitalRead(IMDIN);
  SDC_Signal_Board.BSPD_Relay = digitalRead(BSPDIN);
  SDC_Signal_Board.EMERGENCY_BUTTON = digitalRead(EMR_O);

  SDC_Signal_Board.BrakePressure1 = analogRead(BPS1) * 3.3/4095;
  SDC_Signal_Board.BrakePressure2 = analogRead(BPS2) * 3.3/4095;
  SDC_Signal_Board.AccelPedal1 = analogRead(APS1)* 3.3/4095;
  SDC_Signal_Board.AccelPedal2 = analogRead(APS2)* 3.3/4095;
  SDC_Signal_Board.CurrentSense = analogRead(CURR)* 3.3/4095;  

}
/* ==================================Serial Debugger==============================*/
// Use Non-Block string , there's literally no perfect way to Serial Debug asynchronusly
void debugFrame(){
  Serial.printf("%X\n", receivedMessage.identifier);
  for (int i = 0; i < receivedMessage.data_length_code; i++) 
      Serial.printf("%X",receivedMessage.data[i]);
    Serial.println();
}
void debugBMUmsg(int Module){

    // Serial.print("BMU_CHGready: "); Serial.println(BMU_Package[Module].BMUreadytoCharge);
    // Serial.print("V_Disch: "); Serial.println(BMU_Package[Module].BalancingDischarge_Cells,BIN);
    Serial.print("V_CELL[10]: ");
    // can change to vector , for easy looping funcion
    for(short i=0; i< CELL_NUM; i++){
      Serial.print(BMU_Package[Module].V_CELL[i] * 0.02); Serial.print("V.  ");
    } Serial.println();
    
    // Serial.print("V_MODULE: "); Serial.print(BMU_Package[Module].V_MODULE *0.2); Serial.println("V.  ");
    // Serial.print("DV: ") ; Serial.print(BMU_Package[Module].DV * 0.1); Serial.println("V.  ");

    Serial.print("TEMP[2]: ");
      Serial.print(BMU_Package[Module].TEMP_SENSE[0]*0.6 + 2); Serial.println("C.  ");
      Serial.print(BMU_Package[Module].TEMP_SENSE[1]*0.6 + 2); Serial.println("C.  ");

    Serial.println();

}
void debugBMUFault(int Module){
  
    Serial.print("OV_CRI: ");
    Serial.println(BMU_Package[Module].OVERVOLTAGE_CRITICAL,HEX);
    Serial.print("LV_CRI: ");
    Serial.println(BMU_Package[Module].LOWVOLTAGE_CRITICAL,HEX);
    Serial.print("OVT_CRI: ");
    Serial.println(BMU_Package[Module].OVERTEMP_CRITICAL, HEX);
    Serial.print("OVDIV_CRI: ");
    Serial.println(BMU_Package[Module].OVERDIV_VOLTAGE_CRITICAL,HEX);
    Serial.println();
  
    Serial.print("OV_W: ");
    Serial.println(BMU_Package[Module].OVERVOLTAGE_WARNING,HEX);
    Serial.print("LV_W: ");
    Serial.println(BMU_Package[Module].LOWVOLTAGE_WARNING,HEX);
    Serial.print("OVT_W: ");
    Serial.println(BMU_Package[Module].OVERTEMP_WARNING, HEX);
    Serial.print("OVDIV_CRI: ");
    Serial.println(BMU_Package[Module].OVERDIV_VOLTAGE_WARNING,HEX);
    Serial.println();  
    
}
void debugOBCmsg(){
    
    Serial.print("Voltage from OBC: "); Serial.print(OBC_Package.OBCVolt); Serial.println("V");
    Serial.print("Current from OBC: "); Serial.print(OBC_Package.OBCAmp); Serial.println("A");
    Serial.print("OBC status bit"); Serial.println(OBC_Package.OBCstatusbit);

    // Intepret Individual bit meaning
    bool *obcstatbitarray =  toBitarrayLSB(OBC_Package.OBCstatusbit); // Status Byte
    
    Serial.print("OBC status bit: ");
    switch (obcstatbitarray[0]) {
      case 1:
        Serial.println("ChargerHW = Faulty");
        break;
    }
    switch (obcstatbitarray[1]) {
      case 1:
        Serial.println("ChargerTemp = Overheat");
        break;
    }
    switch (obcstatbitarray[2]) {
      case 1:
        Serial.println("ChargerACplug = Reversed");
        break;
    }
    switch (obcstatbitarray[3]) {
      case 1:
        Serial.println("Charger detects: NO BATTERY VOLTAGES");
        break;
    }
    switch (obcstatbitarray[4]) {
      case 1:
        Serial.println("OBC Detect COMMUNICATION Time out: (6s)");
        break;
    } 
}