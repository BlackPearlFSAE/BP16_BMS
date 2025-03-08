// Data Aggregation , and Relay to Other Sub System , e.g. Telemetry , DataLogger, BMS GUI
// Report BMS data, and Faulty status code // Serialize this data in 8 bit buffer

/* Data send to Remote System */
// BMU data strcuture
#include <stdio.h>
#include <cstring>

// Default configuration of AMS
#define CELL_NUM 10
// #define BMU_NUM 2 // Demo config
#define BMU_NUM 6 // Demo config
// #define BMU_NUM 8 // Real config
// #define BMU_NUM 0

/*Amita Battery*/
#define VMAX_CELL 4.2
// #define VMAX_CELL 3.6
#define VMIN_CELL 3.2
// #define VMIN_CELL 3.9
/*Thermistor*/
#define TEMP_MAX_CELL 60
#define TEMP_SENSOR_NUM 2
#define DVMAX 0.2


struct BMUdata {
  // Basic BMU Data
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

// Physical condition of OBC On board charger , charging power , and safety information
struct OBCdata {
  uint16_t OBCVolt = 0;
  uint16_t OBCAmp = 0;
  uint8_t OBCstatusbit = 0 ;   // During Charging
  bool OBC_OK = 1;
};


// Physical condition of SDC and LV Circuit, safety information , and relay it to Telemetry system
struct LVsignal {
  // Logic Shifter sensor Reading
  bool AIRplus = 1; // AIR+
  bool IMD_Relay = 1; // IMD_OUT
  bool BSPD_Relay = 1; // BSPD_OUT
  bool EMERGENCY_BUTTON = 1;

  // BSPDADCreadingStatus
  // ADC
  uint16_t BrakePressure1;
  uint16_t BrakePressure2;
  uint16_t AccelPedal1;
  uint16_t AccelPedal2;
  uint16_t CurrentSense;
};