#include <iostream>
#include <stdio.h>
#include <cstring>

// =================================================== Data Conversion Methods

// Split and merge High byte low byte of 16 bit unsigned integer
unsigned char* splitHLbyte(unsigned int num);
unsigned int mergeHLbyte(unsigned char Hbyte, unsigned char Lbyte);

// --Encode and Decode byte array
// Encode float or uint16 into arrays of uint8
unsigned char *Encode_bytearray(float f);
// This is for voltage monitoring
float Decode_bytearray(unsigned char* c);

// Return array of 16 binary digit from 16 bit Binary input
bool *toBitarrayMSB(uint16_t num);
bool *toBitarrayLSB(uint16_t num);


uint16_t toUint16FromBitarrayMSB(const bool *bitarr);
uint16_t toUint16FromBitarrayLSB(const bool *bitarr);

// ============ AMS data
// Containing Different Object representing the whole Electrical System

// Default configuration of AMS
#define CELL_NUM 10
#define BMU_NUM 6 // Demo config
/*Amita Battery*/
#define VMAX_CELL 4.2
#define VMIN_CELL 3.2
/*Thermistor*/
#define TEMP_MAX_CELL 60
#define TEMP_SENSOR_NUM 2
#define DVMAX 0.2

// AMS Communication
// #define STANDARD_BIT_RATE 250E3
#define DISCONNENCTION_TIMEOUT 650
#define BCU_ADD 0x7FF
#define OBC_ADD 0x1806E5F4

struct BMUdata {
  // Basic BMU Data
  uint32_t bmu_id = 0x00;
  uint8_t V_CELL[CELL_NUM] = {0};
  uint8_t TEMP_SENSE[TEMP_SENSOR_NUM] = {0};
  uint8_t V_MODULE = 0;
  uint8_t DV = 0;
  // FaultCode 10 bit binary representation of C
  uint16_t OVERVOLTAGE_WARNING = 0;
  uint16_t OVERVOLTAGE_CRITICAL = 0;  
  uint16_t LOWVOLTAGE_WARNING = 0;
  uint16_t LOWVOLTAGE_CRITICAL = 0; 
  uint16_t OVERTEMP_WARNING = 0;
  uint16_t OVERTEMP_CRITICAL = 0;
  uint16_t OVERDIV_VOLTAGE_WARNING = 0 ; // Trigger cell balancing of the cell at fault
  uint16_t OVERDIV_VOLTAGE_CRITICAL = 0; // Trigger Charger disable in addition to Cell balancing
  // Status
  uint16_t BalancingDischarge_Cells = 0;
  bool BMUconnected = 0;   // Default as Active true , means each BMU is on the bus
  bool BMUreadytoCharge = 0;
}; 

// ACCUMULATOR Data , Local to BCU (Make this a struct later , or not? , I don't want over access)
struct AMSdata {

  float ACCUM_VOLTAGE = 0.0; 
  float ACCUM_MAXVOLTAGE = (VMAX_CELL * CELL_NUM * BMU_NUM); // Default value
  float ACCUM_MINVOLTAGE = (VMIN_CELL * CELL_NUM * BMU_NUM); // Defualt value assum 8 module
  // float ACCUM_MAXVOLTAGE = (0); // Default value
  // float ACCUM_MINVOLTAGE = (0); // Defualt value assum 8 module
  bool ACCUM_CHG_READY = 0;

  bool OVERVOLT_WARNING = 0;
  bool LOWVOLT_WARNING = 0;
  bool OVERTEMP_WARNING = 0;
  bool OVERDIV_WARNING = 0;

  bool OVERVOLT_CRITICAL = 0;
  bool LOWVOLT_CRITICAL =  0;
  bool OVERTEMP_CRITICAL = 0;
  bool OVERDIV_CRITICAL = 0;

  // bool AMS_OK = 0; // Use this for Active Low Output
  bool AMS_OK = 1; // Use this for Active High Output
};

// Physical condition of OBC On board charger
struct OBCdata {
  uint16_t OBCVolt = 0;
  uint16_t OBCAmp = 0;
  uint8_t OBCstatusbit = 0 ;   // Saftety information
  bool OBC_OK = 1;
};


// Physical condition of SDC and LV Circuit
struct LVsignal {
  bool AIRplus = 1; // AIR+
  bool IMD_Relay = 1; // IMD_OUT
  bool BSPD_Relay = 1; // BSPD_OUT
  bool EMERGENCY_BUTTON = 1;
  bool OBC_AUX_INPUT = 0;
  bool Temperature_warning_led = 0;
  bool lowvoltage_warning_led = 0;

  // BSPDADCreadingStatus
  uint16_t BrakePressure1;
  uint16_t BrakePressure2;
  uint16_t AccelPedal1;
  uint16_t AccelPedal2;
  uint16_t CurrentSense;
};

//==================================================== CAN bus Methods

// For ID custom protocol
struct extCANIDDecoded {
    uint8_t PRIORITY;
    uint8_t BASE_ID;
    uint8_t MSG_NUM;
    uint8_t SRC;
    uint8_t DEST;
};

//standard CAN edit by jackie
struct StandardCANIDDecoded {
    uint8_t PRIORITY;
    uint8_t MSG_NUM;
    uint8_t SRC;
};

uint16_t createCANID(uint8_t PRIORITY, uint8_t SRC_ADDRESS, uint8_t MSG_NUM);
// Structure of CAN ID :: Receiver side
void decodeExtendedCANID(struct extCANIDDecoded* myCAN ,uint32_t canID);
void decodeStandardCANID(struct StandardCANIDDecoded *myCAN, uint32_t canID);
// typedef uint16_t __canidExtr;

uint16_t toUint16FromBitarrayMSB(const bool *bitarr);
uint16_t toUint16FromBitarrayLSB(const bool *bitarr);


// //==================================================== UART, USB Methods
#ifdef ARDUINO_ARCH_AVR
#endif


#ifdef ESP32

#endif
