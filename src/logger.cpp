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
#include <Arduino.h>
#include <EEPROM.h>

// File system and ESP32 SPI SD lib
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// utility function
#include <_util_AMS.h>
#include <_util_SD.h>
/************************* Define macros *****************************/
// CAN/TWAI bus pins - For LilyGo
#define TWAI_TX_PIN GPIO_NUM_5  // Connect to CAN transceiver TX
#define TWAI_RX_PIN GPIO_NUM_4  // Connect to CAN transceiver RX

/**************** Setup Variables *******************/
twai_message_t receivedMessage;
twai_message_t J1938msg;

// Software Timer
unsigned long lastlogtime1 = 0; 
unsigned long lastStatTime = 0; 
unsigned long communication_timer1 = 0;
unsigned long shutdown_timer1 = 0;
unsigned long lastModuleResponse[BMU_NUM];
// Hardware Timer
hw_timer_t *My_timer1 = NULL;

// BMU data , Accumulator data structure, Sensing data.
BMUdata BMU_Package[BMU_NUM];
AMSdata AMS_Package;
LVsignal Signal_Package;
OBCdata OBC_Package;

// Alias names
bool &AMS_OK = AMS_Package.AMS_OK;
float &ACCUM_MAXVOLTAGE = AMS_Package.ACCUM_MAXVOLTAGE; 
float &ACCUM_MINVOLTAGE = AMS_Package.ACCUM_MINVOLTAGE;  

// Flags
volatile bool ISR_FLG1 = false;
volatile bool CAN_SEND_FLG2 = false;
bool CHARGER_PLUGGED = false;
bool CAN_TIMEOUT_FLG = false;
unsigned long logCount = 0;

// ===========================================================================================

// Assign pin for SD Card pins for LilyGO board
#define SD_SCK  14
#define SD_MISO 2
#define SD_MOSI 15
#define SD_CS   13

// Data logging configuration
#define LOG_INTERVAL_MS 500  // How often to write summary data (every 5 seconds)
#define CSV_BMU_package "/firstFloor_log.csv"
#define CSV_AMS_package "/secondFloor_log.csv"
#define CSV_BMU_HEADER "Time,bmu_id,bmu_volt,bmu_temp1,bmu_temp2,bmu_dv,bmu_connect,bmu_ready_chg,bmu_cell_in_balance,bmu_ov_crti,bmu_ov_warn,bmu_lv_crit,bmu_lv_warn,bmu_ovt_crit,bmu_ovt_warn,bmu_ovd_crit,bmu_ovd_warn,Cell1,Cell2,Cell3,Cell4,Cell5,Cell6,Cell7,Cell8,Cell9,Cell10\n"
#define CSV_AMS_HEADER "Time,accel_ped1,accel_ped2,break_ped1,break_ped2,current_A,bspd_in,imd_in,air+,emr_o,obc_in,temp_light,lv_light,ams_ok,ams_volt,ams_ov_crit,ams_ov_wanr,ams_lv_crit,ams_lv_warn,ams_ovt_crit,ams_ovt_warn,ams_ovd_crit,ams_ovd_warn\n"

// #define mystring "LLL"
/**************** Local Function Delcaration *******************/
void processOBCmsg ( twai_message_t *receivedframe );
void processBMUmsg ( twai_message_t *receivedframe, BMUdata *BMS_ROSPackage);
void debugBMUmsg(int Module);
void debugBMUFault(int Module);
void debugOBCmsg();
void debugFrame();
bool checkModuleDisconnect(BMUdata *BMU_Package);
void twaiTroubleshoot();
void resetAllStruct();
void sensorReading();
void log1stFloordata(int moduleindex);
void log2ndFloordata();

/*******************************************************************
  ==============================Setup==============================
********************************************************************/
void IRAM_ATTR onTimer1() {
  // May or may not be critical section , --- later to be thought out
  ISR_FLG1 = 1;
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);

  /* CAN Communication Setup */
    receivedMessage.extd = false;
    J1938msg.extd = true;
    twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
    general_config.tx_queue_len = 800; // worstcase is 152 bit/frame , this should hold about 5 frame
    general_config.rx_queue_len = 1300; // RX queue hold about 8 frame
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_250KBITS();
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

  // Hardware periodic timer SYNC_TIME
  My_timer1 =  timerBegin(0, 80, true);  // Timer with prescaler
  timerAttachInterrupt(My_timer1, &onTimer1, true);
  timerAlarmWrite(My_timer1, LOG_INTERVAL_MS * 1000, true);  // SYNCTIME ms interval
  timerAlarmEnable(My_timer1);

  // Initialize SD card
  SD_SPI_init( SD_SCK, SD_MISO, SD_MOSI, SD_CS );

  // Create 1st Floor Log File
  if (!SD.exists(CSV_BMU_package)) {
    Serial.println("Creating 1stFloor log file with headers");
    writeFile(SD, CSV_BMU_package, CSV_BMU_HEADER);
  } else {
    Serial.println("Log file exists, appending data");
  }

  if (!SD.exists(CSV_AMS_package)) {
    Serial.println("Creating 1stFloor log file with headers");
    writeFile(SD, CSV_AMS_package, CSV_AMS_HEADER);
  } else {
    Serial.println("Log file exists, appending data");
  }
}

/*******************************************************************
  =======================Mainloop===========================
********************************************************************/
  
void loop() {
  unsigned long currentMillis = millis();

  if (twai_receive(&receivedMessage, 1) == ESP_OK) 
  {
    // debugFrame();
    // Reset BMU data for the one that has disconnected from CAN Bus
    // Unpack CAN frame and insert to BMU_Package[i] , AMS_Package:
    processBMUmsg(&receivedMessage, BMU_Package); // 200ms cycle & 500ms cycle of faultcode

    if(CHARGER_PLUGGED)
      processOBCmsg(&J1938msg); // Unpack CAN frame and insert to OBC_Package:  500ms cycle 
      
    // Update timeout flag and communication_timer
    CAN_TIMEOUT_FLG = false;
    communication_timer1 = millis();
  }
  // if No module is connected from CAN bus , run this code and return until the bus is active
  else if (currentMillis - communication_timer1 >= DISCONNENCTION_TIMEOUT){
    // Shutdown
    if( currentMillis - shutdown_timer1 >= 400 ){
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
  if(checkModuleDisconnect(BMU_Package) == 0){
    // Shutdown and return immediately
    if( currentMillis - shutdown_timer1 >= 400 ){
        Serial.println("MODULE_DISCONNECT -- SHUTDOWN");
        Serial.printf("BMU1 connect: %d, ",BMU_Package[0].BMUconnected);
        Serial.printf("BMU2 connect: %d, ",BMU_Package[1].BMUconnected);
        Serial.printf("BMU3 connect: %d ",BMU_Package[2].BMUconnected);
        Serial.printf("BMU4 connect: %d \n",BMU_Package[3].BMUconnected);
        shutdown_timer1 = millis();
    }
    return;
  }

/*================================= Logger Function*/
  if(ISR_FLG1){
    ISR_FLG1 = 0; // reset
    // Log all BMU_Package
    for(int i =0; i < BMU_NUM ; i++) log1stFloordata(i);
    // Log AMS_Package
    log2ndFloordata();
  }

  // // Log data Every 200 ms
  // if (currentMillis - lastlogtime1  >= LOG_INTERVAL_MS) {
  //   lastlogtime1 = currentMillis;
  //   // Log all BMU_Package
  //   for(int i =0; i < BMU_NUM ; i++) 
  //     log1stFloordata(i);
  //   // Log AMS_Package
  //   log2ndFloordata();
  //   // batteryData.dataUpdated = false;  // Reset flag until next update
  // }
    
  // Print stats periodically
  if (currentMillis - lastStatTime >= 10000) { // Every 10 seconds
    lastStatTime = currentMillis;
    getSDsize();
    // get some ...
  }
} 

/* ==================================Main Local Functions==============================*/

bool isModuleActive(int moduleIndex); // Check the latest response from each module
void resetModuleData(int moduleIndex); // Reset that Module struct if !ismoduleActive
void packing_AMSstruct (int moduleIndex); // recalculate data to AMS struct in sync with number of Active BMU
void dynamicModulereset(BMUdata *BMU_Package);

void processBMUmsg ( twai_message_t* receivedframe , BMUdata *BMU_Package) {
  // Reset BMU struct Value
  dynamicModulereset(BMU_Package);

  // decodeCANID according to BP16 agreement
  StandardCANIDDecoded decodedCANID;
  decodeStandardCANID(&decodedCANID, (receivedframe->identifier) );
  
  // Distingush Module ID
    int i = decodedCANID.SRC - 1;
    if(i >= BMU_NUM) return;      
  
  // Mark timestamp of successfully received Module, No update for disconnected BMU.
  lastModuleResponse[i] = millis();

  /* ---------------- unpack ReceiveFrame to BMUframe ------------------- */
  /*  Message Priority 02 :: BMUModule & Cells data  */
  if(decodedCANID.PRIORITY == 0x02)
  {
    switch (decodedCANID.MSG_NUM) { 
      // MSG1 == Operation status
      case 1:
        // Charging Ready
        BMU_Package[i].BMUreadytoCharge = receivedframe->data[0];
        // Balancing Discharge cell number
        BMU_Package[i].BalancingDischarge_Cells = mergeHLbyte(receivedframe->data[1],receivedframe->data[2]);
        // Vbatt (Module) , dVmax(cell)
        BMU_Package[i].V_MODULE = receivedframe->data[3]; 
        BMU_Package[i].DV = receivedframe->data[4]; 
        // Temperature sensor
        BMU_Package[i].TEMP_SENSE[0] = receivedframe->data[5];
        BMU_Package[i].TEMP_SENSE[1] = receivedframe->data[6];
        break;

      case 2:
        // Low series Side Cell C1-C8
        for(short j=0; j< 8; j++)
          BMU_Package[i].V_CELL[j] = receivedframe->data[j];
        break;

      case 3:
        // High series side Cell C8-CellNumber
        for( short j = 8; j < CELL_NUM; j++ )
          BMU_Package[i].V_CELL[j] = receivedframe->data[(j-8)];
        break;
    }
  }
  /*  Message Priority 01 :: FaultCode  */
  else if(decodedCANID.PRIORITY == 0x01)
  {
    switch (decodedCANID.MSG_NUM) {
      case 1:
        // Merge H and L byte of each FaultCode back to 10 bit binary
        BMU_Package[i].OVERVOLTAGE_WARNING =  mergeHLbyte( receivedframe->data[0], receivedframe->data[1] );  
        BMU_Package[i].OVERVOLTAGE_CRITICAL = mergeHLbyte( receivedframe->data[2], receivedframe->data[3] );  
        BMU_Package[i].LOWVOLTAGE_WARNING = mergeHLbyte( receivedframe->data[4], receivedframe->data[5] );
        BMU_Package[i].LOWVOLTAGE_CRITICAL = mergeHLbyte( receivedframe->data[6], receivedframe->data[7] ); 
        break;
      case 2:
        // Merge H and L byte of each FaultCode back to 10 bit binary
        BMU_Package[i].OVERTEMP_WARNING = mergeHLbyte( receivedframe->data[0], receivedframe->data[1] );  
        BMU_Package[i].OVERTEMP_CRITICAL = mergeHLbyte( receivedframe->data[2], receivedframe->data[3] ); 
        BMU_Package[i].OVERDIV_VOLTAGE_WARNING = mergeHLbyte( receivedframe->data[4], receivedframe->data[5] ); 
        BMU_Package[i].OVERDIV_VOLTAGE_CRITICAL = mergeHLbyte( receivedframe->data[6], receivedframe->data[7] );
        break;
    }
  }
  
  // Reset Accumulator Parameter before dynamically recalculate based on BMU current state
  AMS_Package = AMSdata();
  // Pack BMUframe to AMSframe according to the following condition
  for(int j = 0; j <BMU_NUM ; j++)
  {
    // if Module is connected to CANbus, set as connect, and recalculateAMS package a new
    if(isModuleActive(j)){ 
        BMU_Package[j].BMUconnected = 1;
        packing_AMSstruct(j);
    } else {
        BMU_Package[j].BMUconnected = 0;
    }
  }
}

void processOBCmsg ( twai_message_t *J1939frame ) {
  // if message ID isnt 0x18FF50E5 , return
  if(J1939frame->identifier != 0x18FF50E5)
    return;
  
    // Monitor & Translate current Frame data
    uint8_t VoutH = J1939frame->data[0];
    uint8_t VoutL = J1939frame->data[1];
    uint8_t AoutH = J1939frame->data[2];
    uint8_t AoutL = J1939frame->data[3];
    OBC_Package.OBCstatusbit =  J1939frame->data[4]; // Status Byte
    OBC_Package.OBCVolt = mergeHLbyte(VoutH,VoutL);
    OBC_Package.OBCAmp = mergeHLbyte(AoutH,AoutL);

}

/* ==================================Sub Functions==============================*/


bool isModuleActive(int moduleIndex) {
  unsigned int MAX_SILENCE = DISCONNENCTION_TIMEOUT;
  return (millis() - lastModuleResponse[moduleIndex]) <= (MAX_SILENCE);
}
void resetModuleData(int moduleIndex){
  BMU_Package[moduleIndex].~BMUdata(); // Explicitly call destructor (optional)
  new (&BMU_Package[moduleIndex]) BMUdata();
}
void packing_AMSstruct (int moduleIndex) {
  int &i = moduleIndex;
  // Recalculate AMS based on current BMU states
  AMS_Package.ACCUM_VOLTAGE += ( static_cast<float> (BMU_Package[i].V_MODULE)) * 0.2;
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
  Signal_Package = LVsignal();
  OBC_Package = OBCdata();
}
bool checkModuleDisconnect(BMUdata *BMU_Package){

  bool disconnectedFromCAN = 1; 
  for(short i =0; i< BMU_NUM ; i++){
    if(BMU_Package[i].BMUconnected == 0)
      disconnectedFromCAN = 0; 
  }
    return disconnectedFromCAN;
}
void dynamicModulereset(BMUdata *BMU_Package){ 
  for(short i =0; i< BMU_NUM ; i++){
    // if any of the board aren't in connection => throw error
    if(BMU_Package[i].BMUconnected == 0){
      resetModuleData(i); // Reset that module data (revert voltage , temp., flags , etc. Back to zero)
    }   
  }
}

// void sensorReading(){
//   // Read SDC output signal to AIR+
//   Signal_Package.AIRplus = digitalRead(SDCIN);
//   Signal_Package.IMD_Relay = digitalRead(IMDIN);
//   Signal_Package.BSPD_Relay = digitalRead(BSPDIN);
//   Signal_Package.EMERGENCY_BUTTON = digitalRead(EMR_O);

//   Signal_Package.BrakePressure1 = analogRead(BPS1) * 3.3/4095;
//   Signal_Package.BrakePressure2 = analogRead(BPS2) * 3.3/4095;
//   Signal_Package.AccelPedal1 = analogRead(APS1)* 3.3/4095;
//   Signal_Package.AccelPedal2 = analogRead(APS2)* 3.3/4095;
//   Signal_Package.CurrentSense = analogRead(CURR)* 3.3/4095;  

// }
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


void log1stFloordata(int moduleindex) {
  int &i = moduleindex;
  unsigned long timestamp = millis();

  char dataString1[100];
  // right now 17
  sprintf(dataString1, "%lu,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u," , 
          timestamp, 
          BMU_Package[i].bmu_id, 
          BMU_Package[i].V_MODULE,
          BMU_Package[i].TEMP_SENSE[0], 
          BMU_Package[i].TEMP_SENSE[1], 
          BMU_Package[i].DV, 
          BMU_Package[i].BMUconnected,
          BMU_Package[i].BMUreadytoCharge,
          BMU_Package[i].BalancingDischarge_Cells,
          BMU_Package[i].OVERVOLTAGE_CRITICAL,
          BMU_Package[i].OVERVOLTAGE_WARNING,
          BMU_Package[i].LOWVOLTAGE_CRITICAL,
          BMU_Package[i].LOWVOLTAGE_WARNING,
          BMU_Package[i].OVERTEMP_CRITICAL,
          BMU_Package[i].OVERTEMP_WARNING,
          BMU_Package[i].OVERDIV_VOLTAGE_CRITICAL,
          BMU_Package[i].OVERDIV_VOLTAGE_WARNING
        );
      // THe actual size is 50 , but * 2 for fail safe

  // Second set of dataString hold cells data
  char dataString2[CELL_NUM];  
  int offset = 0;  // Track write position

  for(int j = 0; j < 10; j++) {
      char voltage = BMU_Package[i].V_CELL[j];
      offset += snprintf(dataString2 + offset, sizeof(dataString2) - offset, "%u,", voltage);
  }

  // Replace last comma as \n line break
  if (offset > 0){
    dataString2[offset - 1] = '\n';
  } 
        
  // Log both data string to SD card
  appendFile(SD, CSV_BMU_package, dataString1);
  appendFile(SD, CSV_BMU_package, dataString2);
  logCount++;
}

void log2ndFloordata(){

unsigned long timestamp = millis();

char dataString[100];
sprintf(dataString, "%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", 
        timestamp, 
        Signal_Package.AccelPedal1 ,
        Signal_Package.AccelPedal2 ,
        Signal_Package.BrakePressure1,
        Signal_Package.BrakePressure2,
        Signal_Package.CurrentSense,
        Signal_Package.BSPD_Relay,
        Signal_Package.IMD_Relay,
        Signal_Package.AIRplus,
        Signal_Package.EMERGENCY_BUTTON,
        Signal_Package.OBC_AUX_INPUT,
        Signal_Package.Temperature_warning_led,
        Signal_Package.lowvoltage_warning_led,
        AMS_Package.AMS_OK,
        AMS_Package.ACCUM_VOLTAGE,
        AMS_Package.OVERVOLT_CRITICAL,
        AMS_Package.OVERVOLT_WARNING,
        AMS_Package.LOWVOLT_CRITICAL,
        AMS_Package.LOWVOLT_WARNING,
        AMS_Package.OVERTEMP_CRITICAL,
        AMS_Package.OVERTEMP_WARNING,
        AMS_Package.OVERDIV_CRITICAL,
        AMS_Package.OVERDIV_WARNING
      );
  
  // Log to SD card
  appendFile(SD, CSV_AMS_package, dataString);
  logCount++;
}

// Future update

// void logOBC(){

// }

