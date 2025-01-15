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
// #include <freertos/FreeRTOS.h>
// #include <micro_ros_arduino.h>
// #include "FS.h"
// #include "SD.h"
// #include "SPI.h"
#include <Arduino.h>
#include <EEPROM.h>
// utility function
#include <util.h>
#include <AMS.h>

/************************* Define macros *****************************/
#define CAN_RX  GPIO_NUM_13
#define CAN_TX  GPIO_NUM_14
#define BSPDIN GPIO_NUM_4  // Check BSPD OK signal from BSPD directly 
#define IMDIN GPIO_NUM_5   // Check IMD OK signal from IMD directly
#define SDCIN GPIO_NUM_9    // Check OK signal from Shutdown Circuit => Its status must match all below Signal pin , otherwise faulty
#define OBCIN GPIO_NUM_10   // Check OK Signal from Charging Shutdown Circuit, indicates that it is charged

#define BMSOUT GPIO_NUM_47  // OUTPUT Fault Signal to BMS relay
// #define BMSOUT LED_BUILTIN - SOC_GPIO_PIN_COUNT
// #define BMSOUT LED_BUILTIN  // OUTPUT Fault Signal to BMS relay
#define LVlight GPIO_NUM_48
#define TEMPlight GPIO_NUM_46

#define EEPROM_SIZE 5
#define OBC_SYNC_TIME 500
#define SYNC_TIME 200
#define TIMEOUT_TIME 4000
#define BMU_NUM 8
#define OBC_ADD 0x1806E5F4
#define BCU_ADD 0x01EE5000

/**************** Setup Variables *******************/
twai_message_t sendMessage;
twai_message_t receivedMessage;

// Software Timer reference
unsigned long reference_time = 0; 
unsigned long reference_time2 = 0;
unsigned long reference_time3 = 0;
unsigned long communication_timer1 = 0;
unsigned long shutdown_timer1 = 0;
unsigned long lastModuleResponse[BMU_NUM] = {0};
// Ticker , Timer interrupt
hw_timer_t *My_timer1 = NULL;
hw_timer_t *My_timer2 = NULL;

// BMU data , Accumulator data structure , process in CORE1
BMUdata BMU_Package[BMU_NUM];
AMSdata AMS_Package;
// Logic Shifting data , process in CORE0
SDCsignal SDC_Signal_Board;
OBCdata OBC_Package;

// Alias name
bool &AMS_OK = AMS_Package.AMS_OK;
uint8_t &OBCFault = OBC_Package.OBCstatusbit;
bool &ACCUM_Full = AMS_Package.OVERVOLT_WARNING;
float &ACCUM_MAXVOLTAGE = AMS_Package.ACCUM_MAXVOLTAGE; 
float &ACCUM_MINVOLTAGE = AMS_Package.ACCUM_MINVOLTAGE; 
bool &AIRplus = SDC_Signal_Board.AIRplus; 
bool &IMD_Relay = SDC_Signal_Board.IMD_Relay; 
bool &BSPD_Relay = SDC_Signal_Board.BSPD_Relay; 

// Flag
const bool EEPROM_WRITE_FLAG = false;
volatile bool CAN_SEND_FLG1 = false;
volatile bool CAN_SEND_FLG2 = false;
bool CHARGER_PLUGGED = false;
bool CAN_TIMEOUT_FLG = false;
bool ACCUM_ReadytoCharge = 0;
bool ACCUM_OverDivCritical = 0;
bool LOW_VOLT_WARN = 0;
bool OVER_TEMP_WARN = 0;



// Default Parameter
byte  VmaxCell ,VminCell ,TempMaxCell ,dVmax;

/**************** Local Function Delcaration *******************/
void packBMUmsg ( twai_message_t *BCUsent, uint16_t Sync_time,  bool &is_charger_plugged);
void packOBCmsg ( twai_message_t *BCUsent, bool &BMS_OK , bool &ReadytoCharge, bool &OverDivCritical_Yes, bool &Voltage_is_Full);
void unpackOBCmsg ( twai_message_t *BCUreceived );
void unpackBMUmsgtoAMS ( twai_message_t *BCUreceived, BMUdata *BMS_ROSPackage);
void debugBMUmsg(int Module);
void debugBMUFault(int Module);
void debugOBCmsg();
void debugSDC();
void debugFrame();
void checkModuleTimeout();
void twaiTroubleshoot();
void resetAllStruct();

/*******************************************************************
  ==============================Setup==============================
********************************************************************/
void resetModuleData(int moduleIndex);
void IRAM_ATTR onTimer1() {
  CAN_SEND_FLG1 = 1;
}
void IRAM_ATTR onTimer2() {
  CAN_SEND_FLG2 = 1;
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  /* Shutdown System setup */
  pinMode(SDCIN,INPUT_PULLDOWN); 
  pinMode(OBCIN,INPUT_PULLDOWN);
  pinMode(IMDIN,INPUT_PULLDOWN);
  pinMode(BSPDIN,INPUT_PULLDOWN);
  pinMode(BMSOUT,OUTPUT);
  pinMode(LVlight,OUTPUT);
  pinMode(TEMPlight,OUTPUT);
  
  /* Write Default Parameter to ESP32 Flashmemory only once*/
    EEPROM.begin(EEPROM_SIZE); // only for esp32 , that use virtual eeprom
    if (EEPROM_WRITE_FLAG) {
      Serial.println("write to EEPROM.");
      EEPROM.write(0, (byte) (VMAX_CELL / 0.1) ); // 42
      EEPROM.write(1, (byte) (VMIN_CELL / 0.1) ); // 32
      EEPROM.write(2, (byte) (TEMP_MAX_CELL) ); // 60
      EEPROM.write(3, (byte) (DVMAX / 0.1) ); // 2
      EEPROM.commit();
      Serial.println("Writing to EEPROM. complete");
    }
    // Display and Check EEPROM Data
    Serial.println("EEPROM Data:");
    EEPROM.get(0,VmaxCell); Serial.println(VmaxCell);
    EEPROM.get(1,VminCell); Serial.println(VminCell);
    EEPROM.get(2,TempMaxCell); Serial.println(TempMaxCell);
    EEPROM.get(3,dVmax); Serial.println(dVmax);
      
  /* CAN Communication Setup */
  sendMessage.extd = true;
  twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  general_config.tx_queue_len = 800; // worstcase is 152 bit/frame , this should hold about 5 frame
  general_config.rx_queue_len = 1300; // RX queue hold about 8 frame

  twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_250KBITS();
  // Set Filter (Default now us accept all , but I will reconfigure it later)
  twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

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

  // Setup timer for 100ms intervals
  unsigned int time1 = (SYNC_TIME/2) * 1000 ;
  My_timer1 =  timerBegin(0, 80, true);  // Timer with prescaler
  timerAttachInterrupt(My_timer1, &onTimer1, true);
  timerAlarmWrite(My_timer1, time1, true);  // 100ms interval
  timerAlarmEnable(My_timer1);

  // Setup timer for 500ms intervals 
  unsigned int time2 = (OBC_SYNC_TIME)*1000 ;
  My_timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(My_timer2, &onTimer2, true);
  timerAlarmWrite(My_timer2, time2, true);  // 500ms interval
  timerAlarmEnable(My_timer2);

  Serial.println("BCU__initialized__");
}

/*******************************************************************
  =======================Mainloop===========================
********************************************************************/
/* CORE0 for coordinating with Telemetry socket CAN*/
/* CORE1 for coordinating BMS and Electrical System*/
  
void loop(){
/* CORE0 for coordinating with Telemetry socket CAN*/
  // Check if Charger LV AUX plug is actually into ACCUM 2nd Floor connector (May change to external interrupt)
  (digitalRead(OBCIN)) ? (CHARGER_PLUGGED = true) : (CHARGER_PLUGGED = false);
  /*___Task 0 : Communication ====================================================*/
  // ------------------------------------Message Transmission
  
  // BCU CMD & SYNC   (100ms cycle Broadcast to all BMU modules in Bus) 
  if (CAN_SEND_FLG1)
  {
    CAN_SEND_FLG1=0; // reset
    packBMUmsg(&sendMessage, SYNC_TIME , CHARGER_PLUGGED);
    twai_transmit(&sendMessage, 1);
  } 
  if(CAN_SEND_FLG2 && CHARGER_PLUGGED)
  {
    CAN_SEND_FLG2=0; // reset
    packOBCmsg(&sendMessage, AMS_OK, ACCUM_ReadytoCharge , ACCUM_OverDivCritical , ACCUM_Full);
    // **NOTE** 3rd to 5th Parameter will be updated via unpackBMUmsgtoAMS(); function
    twai_transmit(&sendMessage, 1);
  }
 
  //------------------------------------Message Reception

  // if RX buffer isn't cleared after its full within 1 tick Queue overflow alert will fired
  if (twai_receive(&receivedMessage, 1) == ESP_OK) 
  {
    // debugFrame();
    // Reset every BMU struct that has been disconnected from CAN Bus
    checkModuleTimeout();

    // Unpack CAN frame and insert to BMU_Package[i] , AMS_Package: 
    // ----Driving
    unpackBMUmsgtoAMS(&receivedMessage, BMU_Package); // 200ms cycle & 1000ms cycle of faultcode
    // ----Charging
    if(CHARGER_PLUGGED)
    // Unpack CAN frame and insert to OBC_Package: 
      unpackOBCmsg(&receivedMessage); // 500ms cycle
    
    // Update timeout flag and communication_timer
    CAN_TIMEOUT_FLG = false;
    communication_timer1 = millis();
  } 

  // if no byte received from CAN bus , run this code and return until the bus is active
  else if (millis()-communication_timer1 >= TIMEOUT_TIME){
    digitalWrite(BMSOUT,LOW);
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
    
  /*___Task 1 : Determine AMS_OK relay state ==================================================== */

    // Fault Table , OR-ed each type of Fault code bit which has been filled up when unpackBMUmsgtoAMS();

    // Warning Condition for Dashboard light: LowVoltage , Module OverTemperature , Full Voltage
    (AMS_Package.ACCUM_VOLTAGE >= 0.9 * ACCUM_MAXVOLTAGE) ? (ACCUM_Full = 1) : (ACCUM_Full = 0) ;
    (AMS_Package.ACCUM_VOLTAGE <= 1.12 * ACCUM_MINVOLTAGE) ? (LOW_VOLT_WARN = 1) : (LOW_VOLT_WARN = 0);
    (AMS_Package.OVERTEMP_WARNING > 0) ? (OVER_TEMP_WARN = 1) : (OVER_TEMP_WARN = 0);
    
    // Fault Condition for AMS_OK Shutdown
    bool ACCUMULATOR_Fault;
    ACCUMULATOR_Fault = AMS_Package.OVERVOLT_CRITICAL | AMS_Package.LOWVOLT_CRITICAL | AMS_Package.OVERTEMP_CRITICAL;

    (ACCUMULATOR_Fault > 0) ? (AMS_OK = 0) : (AMS_OK = 1);
    
    /*------- Coordiante BMU cell Balancing with OBC ------------*/
    if(CHARGER_PLUGGED){
      // if differential voltage between any cells are critical, then at AMS level is fault
      (AMS_Package.OVERDIV_CRITICAL > 0) ? (ACCUM_OverDivCritical = 1) : (ACCUM_OverDivCritical = 0);
      // Set ReadytoCharge Flag, only if All BMU are ready to charge.
      (AMS_Package.ACCUM_CHG_READY > 0) ? (ACCUM_ReadytoCharge = 1) : (ACCUM_ReadytoCharge = 0);

      ((ACCUMULATOR_Fault | OBCFault) > 0) ? (AMS_OK = 0) : (AMS_OK = 1);
      // Debug BMU_Package[0] , Passive Balancing Cells
        // Serial.println(BMS_ROSPackage[0].BalancingDischarge_Cells,BIN); 
    }

    // Debug AMS state
    if(millis()-reference_time >= 200){
      Serial.printf("AMS_OK: %c \n",AMS_OK);
      Serial.printf("AMS_VOLT: %x \n",AMS_Package.ACCUM_VOLTAGE);
      // Serial.printf("OBC_VOLT: %d \n",OBC_Package.OBCVolt);
      // Serial.printf("OBC_AMP: %d \n",OBC_Package.OBCAmp);
      reference_time= millis();
    }
     
  /* Task 1 : Shutdown , Dash Board interaction ==================================================== */ 
   
    if(!AMS_OK){
      digitalWrite(BMSOUT,LOW);
    } else {
      digitalWrite(BMSOUT,HIGH);
    }

    if(LOW_VOLT_WARN){ 
      digitalWrite(LVlight,HIGH);
    } 
    else {
      digitalWrite(LVlight,LOW);
    }
      
    if(OVER_TEMP_WARN){ 
      digitalWrite(TEMPlight,HIGH);
    } 
    else {
      digitalWrite(TEMPlight,LOW);
    }
    
/* CORE1 for coordinating BMS and Electrical System*/

  /*-------------- BMSROS_Package debug with SD card logger -------------*/
  // Packing data to SD
  // packing data to Remote System using CORE0
  /*-------------- CORE 0 Tasks -------------*/
  // ROS , Ethernet publishy function

  /*-------------- CORE 1 Tasks -------------*/
  // After publishing to ROS , reset all BMU data with for loop

  // Read SDC output signal to AIR+
  (digitalRead(SDCIN)) ? (AIRplus = 1) : (AIRplus = 0);
  // Read IMD_Ok and BSPD_OK status from SDC
  (digitalRead(IMDIN)) ? (IMD_Relay = 1) : (IMD_Relay = 0);
  (digitalRead(BSPDIN)) ? (BSPD_Relay = 1) : (BSPD_Relay = 0);

} 

/* ==================================Main Local Functions==============================*/
// declare subfunction for main use
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
  BCUsent->data[6] = 0x00; // Reserved
  BCUsent->data[7] = 0x00;
    
}
void unpackBMUmsgtoAMS ( twai_message_t* BCUreceived , BMUdata *BMU_Package) {
  
  // Decode logic
  CANIDDecoded decodedCANID;
  decodeExtendedCANID(&decodedCANID, (BCUreceived->identifier));
  if(decodedCANID.BASE_ID != 0x0E || decodedCANID.DEST != 0xE5)
    return;
  
  // Distingush Module ID
    int i = decodedCANID.SRC - 1; // SRC stats at 0x01, subtract 1 for indexing.
    if(i >= BMU_NUM) return; // Bounds check

  // Mark timestamp of successfully received Module, 
  // No update for disconnected BMU.
    lastModuleResponse[i] = millis();
  
  // unpack ReceiveFrame to BMUframe
  /*  Message Priority 11 :: BMUModule & Cells data  */
  if(decodedCANID.PRIORITY == 0x11)
  {
    switch (decodedCANID.MSG_NUM) 
    { 
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
  else if(decodedCANID.PRIORITY == 0x10)
  {
    switch (decodedCANID.MSG_NUM)
    {
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
  }
  
  // Reset Accumulator Parameter before dynamically recalculate based on BMU current state
  AMS_Package = AMSdata();
  // Pack BMUframe to AMSframe according to the following condition
  for(int j = 0; j <BMU_NUM ; j++)
  {
    // if Module is connected to CANbus, set , and recalculateAMS
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
  unsigned int MAX_SILENCE = (SYNC_TIME) + 350;
  return (millis() - lastModuleResponse[moduleIndex]) <= (MAX_SILENCE);
}
void resetModuleData(int moduleIndex){
  BMU_Package[moduleIndex].~BMUdata(); // Explicitly call destructor (optional)
  new (&BMU_Package[moduleIndex]) BMUdata();
}
void recalculateAMSPackage (int moduleIndex) {
  int &i = moduleIndex;
  // Recalculate based on current BMU states
  AMS_Package.ACCUM_VOLTAGE += (BMU_Package[i].V_MODULE) * 0.2;
  AMS_Package.OVERVOLT_WARNING |= BMU_Package[i].OVERVOLTAGE_WARNING;
  AMS_Package.LOWVOLT_WARNING |= BMU_Package[i].LOWVOLTAGE_WARNING;
  AMS_Package.OVERTEMP_WARNING |= BMU_Package[i].OVERTEMP_WARNING;
  AMS_Package.OVERDIV_CRITICAL |= BMU_Package[i].OVERDIV_VOLTAGE_WARNING;
  AMS_Package.OVERVOLT_CRITICAL |= BMU_Package[i].OVERVOLTAGE_CRITICAL;
  AMS_Package.LOWVOLT_CRITICAL |= BMU_Package[i].LOWVOLTAGE_CRITICAL;
  AMS_Package.OVERTEMP_CRITICAL |= BMU_Package[i].OVERTEMP_CRITICAL;

  AMS_Package.ACCUM_CHG_READY &= (BMU_Package[i].BMUreadytoCharge);
  AMS_Package.OVERDIV_CRITICAL |= BMU_Package[i].OVERDIV_VOLTAGE_CRITICAL;
}
void resetAllStruct(){
  for (int i = 0; i < BMU_NUM; i++){
    resetModuleData(i);
    lastModuleResponse[i] = 0;
  }
  AMS_Package = AMSdata();
  SDC_Signal_Board = SDCsignal();
  OBC_Package = OBCdata();
}
void checkModuleTimeout(){
for(short i =0; i<BMU_NUM ; i++){
    if(BMU_Package[i].BMUconnected == 0)
      resetModuleData(i);
  }
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

    Serial.print("BMU Operation Status: "); Serial.println(BMU_Package[Module].BMUreadytoCharge);
    Serial.print("Cell balancing discharge: "); Serial.println(BMU_Package[Module].BalancingDischarge_Cells);
    Serial.print("V_CELL C1-10: ");
    // can change to vector , for easy looping funcion
    for(short i=0; i< CELL_NUM; i++){
      Serial.print(BMU_Package[Module].V_CELL[i]); Serial.print("V.  ");
    } Serial.println();
    Serial.print("V_MODULE: "); Serial.print(BMU_Package[Module].V_MODULE); Serial.println("V.  ");
    Serial.print("AVG_CELL_VOLTAGE_DIFF: ") ; Serial.print(BMU_Package[Module].DV); Serial.println("V.  ");

    Serial.print("TEMP_SENSOR T1-T2: ");
    for(short i=0; i< TEMP_SENSOR_NUM; i++){
      Serial.print(BMU_Package[Module].TEMP_SENSE[i]); Serial.print("C.  ");
    } Serial.println();

    Serial.println();

}
void debugBMUFault(int Module){
  
    Serial.print("OVERVOLTAGE_CRITICAL_CELLS (C1-C10): ");
    Serial.println(BMU_Package[Module].OVERVOLTAGE_CRITICAL,HEX);
    Serial.print("UNDERVOLTAGE_CRITICAL_CELLS (C1-C10): ");
    Serial.println(BMU_Package[Module].LOWVOLTAGE_CRITICAL,HEX);
    Serial.print("OVERTEMP_CRITICAL (C1-C10): ");
    Serial.println(BMU_Package[Module].OVERTEMP_CRITICAL, HEX);
    Serial.print("OVERDIV_CRITICAL (C1-C10): ");
    Serial.println(BMU_Package[Module].OVERDIV_VOLTAGE_CRITICAL,HEX);
    Serial.println();
  
    Serial.print("OVERVOLTAGE_WARNING_CELLS (C1-C10): ");
    Serial.println(BMU_Package[Module].OVERVOLTAGE_WARNING,HEX);
    Serial.print("UNDERVOLTAGE_WARNING_CELLS (C1-C10): ");
    Serial.println(BMU_Package[Module].LOWVOLTAGE_WARNING,HEX);
    Serial.print("OVERTEMP_WARNING (C1-C10): ");
    Serial.println(BMU_Package[Module].OVERTEMP_WARNING, HEX);
    Serial.print("OVERDIV_WARNING (C1-C10): ");
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
void debugSDC(){
  
  // Signal = 1 means at fault
  if(CAN_TIMEOUT_FLG)
    Serial.println();

  // Signal = 0 means at fault
  if(!SDC_Signal_Board.AIRplus)
    Serial.println("SDC_OK_SiGNAL: OFF");
  if(!SDC_Signal_Board.IMD_Relay)
    Serial.println("IMD_OK = 0");
  if(!SDC_Signal_Board.BSPD_Relay)
    Serial.println("BSPD_OK = 0");
  
}