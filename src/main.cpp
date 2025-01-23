/*! Analog Devices DC2259A Demonstration Board.
 * LTC6811: Multicell Battery Monitors
 *
 *@verbatim
 *NOTES
 * Setup:
 *   Set the terminal baud rate to 115200 and select the newline terminator.
 *   Ensure all jumpers on the demo board are installed in their default positions from the factory.
 *   Refer to Demo Manual.
 *
 *USER INPUT DATA FORMAT:
 * decimal : 1024
 * hex     : 0x400
 * octal   : 02000  (leading 0)
 * binary  : B10000000000
 * float   : 1024.0
 *@endverbatim
 *
 * https://www.analog.com/en/products/ltc6811-1.html
 * https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/dc2259a.html
 *
 ********************************************************************************
 * Copyright 2019(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*! @file
    @ingroup LTC6811-1
*/

/************************************* Read me *******************************************
In this sketch book:
  -All Global Variables are in Upper casing
  -All Local Variables are in lower casing
  -The Function wakeup_sleep(TOTAL_IC) : is used to wake the LTC681x from sleep state.
   It is defined in LTC681x.cpp
  -The Function wakeup_idle(TOTAL_IC) : is used to wake the ICs connected in daisy chain
   via the LTC6820 by initiating a dummy SPI communication. It is defined in LTC681x.cpp
*******************************************************************************************/

/************************* Includes ***************************/
#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include <EEPROM.h>
// #include <CAN.h>
#include <mcp2515.h>
// #include <mcp_can.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "LT_I2C.h"
#include "QuikEval_EEPROM.h"
#include "UserInterface.h"
#include "LTC681x.h"
#include "LTC6811.h"
#include "util.h"

/************************* Defines *****************************/
#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0

/**************** Local Function Declaration *******************/
void measurement_loop(uint8_t datalog_en, int timeOut, int sync, int charging, int balanceActive, int voltFull);
void print_menu(void);
void print_wrconfig(void);
void print_rxconfig(void);
void print_cells(uint8_t datalog_en);
void print_aux(uint8_t datalog_en);
void print_stat(void);
void print_sumofcells(void);
void check_mux_fail(void);
void print_selftest_errors(uint8_t adc_reg, int8_t error);
void print_overlap_results(int8_t error);
void print_digital_redundancy_errors(uint8_t adc_reg, int8_t error);
void print_open_wires(void);
void print_pec_error_count(void);
int8_t select_s_pin(void);
void print_wrpwm(void);
void print_rxpwm(void);
void print_wrsctrl(void);
void print_rxsctrl(void);
void print_wrcomm(void);
void print_rxcomm(void);
void print_conv_time(uint32_t conv_time);
void check_error(int error);
void serial_print_text(char data[]);
void serial_print_hex(uint8_t data);
char read_hex(void);
char get_char(void);
void run_command(uint32_t cmd);


void packCANData(bool sync, bool timeOut, bool charging, bool balanceActive, bool voltFull, uint16_t balanceStatus, float vbatt, float deltaV, float temp1, float temp2);
void packCANData2();
void packCANFrame45(float cellVoltage, float temp,  int e, float maxVolt, float minVolt);
void send1CANFrame();
void sendCANFrame3();
void sendCANFrame45();
void debugFrame1();
void debugFault(int byte);
void readCAN();
void extractCAN();
// void select_MCP2515();
// void deselect_MCP2515();
void select_LTC6811();
void deselect_LTC6811();


float sumCell(int cell);
void discharge(int selected);
void clearDischarge();
void getCellVoltage();
float getTemp(int pin);
void saveEEPROM();
/*******************************************************************
  Setup Variables
  The following variables can be modified to configure the software.
********************************************************************/
const uint8_t TOTAL_IC = 1; //!< Number of ICs in the daisy chain
const uint8_t CELL_PER_IC = 12;

//variable for packCANData
bool timeOut = 0;
bool charging = 0;
bool balanceActive = 0;
bool voltFull = 0; 
float temp1,temp2;
float vbatt;
int sync = 0;//save to EEPROM
int SYNC = 0;//get from EEPROM
float tMax;//save to EEPROM
float TMAX;//get from EEPROM
float maxVolt,minVolt; // save from EEPROM
float MAXVOLT,MINVOLT; // get from EEPROM
float dVMax; // save from EEPROM
float DVMAX; // get from EEPROM


//variable for void loop 
int slot = 0;
int state = 1;
int stateC = 0;
float avrBatt;
float deltaV;
int checkDis=0;

//for writing EEPROM
bool writeFlag =0;

 
unsigned long current = 0;
unsigned long previousMill = 0;// for sending package 123
unsigned long previousMill2 = 0;// for sending package 45
unsigned long previousMill3 = 0;// for discharging
unsigned long previousMill4 = 0;
unsigned long previousMill5 = 0;// for sending 45 (charging)
unsigned long previousMill6 = 0;// for receiving BCU message
unsigned long actual;
const long interval = 200;//for message 123
const long intervalFault = 500;// for message 45
const long intervalDis = 3000;// for discharging
const long intervalClear = 3000;// for clearDischarging
const long intervalReceive = 100;// for receiving from BCU 
unsigned long dischargeStartTime = 0;
bool isDischarge = false;

unsigned long www = 0;

// int int= 0;

uint16_t balanceStatus;




uint8_t fault;
can_frame frame;
can_frame frame2;
can_frame frame3;
can_frame frame4;
can_frame frame5;
can_frame readFrame;

uint32_t received_id;


uint8_t data[8];

//temporary 
uint16_t get_cell_value(uint8_t *data, uint8_t cell_num) {
    // cell_num is 0-2 for the three cells in this register
    uint16_t cell_value;
    uint8_t cell_data_index = cell_num * 2;  // each cell takes 2 bytes
    
    // Combine the bytes (low byte first, then high byte)
    cell_value = data[cell_data_index] + (data[cell_data_index + 1] << 8);
    
    return cell_value;
}

// ADC Command Configurations. See LTC681x.h for options.
const uint8_t ADC_OPT = ADC_OPT_DISABLED;         //!< ADC Mode option bit
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ; //!< ADC Mode
const uint8_t ADC_DCP = DCP_ENABLED;              //!< Discharge Permitted
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL;   //!< Channel Selection for ADC conversion
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL;     //!< Channel Selection for ADC conversion
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL;   //!< Channel Selection for ADC conversion
const uint8_t SEL_ALL_REG = REG_ALL;              //!< Register Selection
const uint8_t SEL_REG_A = REG_1;                  //!< Register Selection
const uint8_t SEL_REG_B = REG_2;                  //!< Register Selection

const uint16_t MEASUREMENT_LOOP_TIME = 500; //!< Loop Time in milliseconds(ms)

// Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 41000; //!< Over voltage threshold ADC Code. LSB = 0.0001 ---(4.1V)
const uint16_t UV_THRESHOLD = 30000; //!< Under voltage threshold ADC Code. LSB = 0.0001 ---(3V)

// Loop Measurement Setup. These Variables are ENABLED or DISABLED. Remember ALL CAPS
const uint8_t WRITE_CONFIG = DISABLED; //!< This is to ENABLED or DISABLED writing into to configuration registers in a continuous loop
const uint8_t READ_CONFIG = DISABLED;  //!< This is to ENABLED or DISABLED reading the configuration registers in a continuous loop
const uint8_t MEASURE_CELL = ENABLED;  //!< This is to ENABLED or DISABLED measuring the cell voltages in a continuous loop
const uint8_t MEASURE_AUX = DISABLED;  //!< This is to ENABLED or DISABLED reading the auxiliary registers in a continuous loop
const uint8_t MEASURE_STAT = DISABLED; //!< This is to ENABLED or DISABLED reading the status registers in a continuous loop
const uint8_t PRINT_PEC = DISABLED;    //!< This is to ENABLED or DISABLED printing the PEC Error Count in a continuous loop
/************************************
  END SETUP
*************************************/

/******************************************************
 Global Battery Variables received from 681x commands.
 These variables store the results from the LTC6811
 register reads and the array lengths must be based
 on the number of ICs on the stack
 ******************************************************/
cell_asic BMS_IC[TOTAL_IC]; //!< Global Battery Variable

/*********************************************************
 Set the configuration bits.
 Refer to the Configuration Register Group from data sheet.
**********************************************************/
bool REFON = true;                                                                                         //!< Reference Powered Up Bit
bool ADCOPT = false;                                                                                       //!< ADC Mode option bit
bool GPIOBITS_A[5] = {false, false, true, true, true};                                                     //!< GPIO Pin Control // Gpio 1,2,3,4,5
uint16_t UV = UV_THRESHOLD;                                                                                //!< Under-voltage Comparison Voltage
uint16_t OV = OV_THRESHOLD;                                                                                //!< Over-voltage Comparison Voltage
bool DCCBITS_A[12] = {false, false, false, false, false, false, false, false, false, false, false, false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
bool DCTOBITS[4] = {true, false, true, false};                                                             //!< Discharge time value // Dcto 0,1,2,3 // Programed for 4 min
/*Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet */

/*!**********************************************************************

 ***********************************************************************/
// const int spiCSPin = 10;
// MCP_CAN CANN(spiCSPin);
float cellVoltage;
float cell_voltage[TOTAL_IC][CELL_PER_IC];

int totalCell = 10;

// float fakeCell[10] = {3.6,3.4,3.5,3.8,3.8,4.1,3.7,3.6,3.5,3.4};
float fakeCell[10] = {3.6,3.7,3.6,3.6,3.6,3.6,3.6,3.7,3.6,3.6};
uint8_t keepCell[10] = {};

// int idModule[8] = {{0x211,0x212,2}};
// int sync, 
// Initialize the MCP_CAN object
const int LTC6811_CS_PIN = 9;
MCP2515 mcp2515(10);


// #define tempA0 = A0;
// #define tempA1 = A1;


void setup()
{
    // Set CS pins as outputs
  // pinMode(LTC6811_CS_PIN, INPUT);
  // digitalWrite(LTC6811_CS_PIN, HIGH);
  // Set both CS pins HIGH initially

  pinMode(A0,INPUT);
  pinMode(A3,INPUT);

  wakeup_sleep(TOTAL_IC);
  LTC6811_wrcfg(TOTAL_IC, BMS_IC);


  frame.can_id  = 0x211; //FIRST frame
  frame2.can_id  = 0x212;//SECOND frame
  frame3.can_id  = 0x213;//THIRD frame
  frame4.can_id  = 0x111;//FOURTH frame
  frame5.can_id  = 0x112;//FIFTH frame

  while (!Serial);
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS,MCP_8MHZ);
  mcp2515.setNormalMode();

  
  // mcp2515.setFilter(MCP2515::RXF0,true,0x01EE5000);
  // In measurement loop0.
  wakeup_idle(TOTAL_IC);
  delay(100);
  // quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV16); 
  LTC6811_init_cfg(TOTAL_IC, BMS_IC);
  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    LTC6811_set_cfgr(current_ic, BMS_IC, REFON, ADCOPT, GPIOBITS_A, DCCBITS_A, DCTOBITS, UV, OV);
  }
  LTC6811_reset_crc_count(TOTAL_IC, BMS_IC);
  LTC6811_init_reg_limits(TOTAL_IC, BMS_IC);
  // discharge(10);
}  

/*!*********************************************************************
 \brief Main loop
 @return void
***********************************************************************/
void loop()
{
  current = millis();
  uint8_t error;
  // Serial.println("dww");
  // mcp2515.sendMessage(&frame);
  // mcp2515.sendMessage(&frame2);
    // readCAN();
    // extractCAN();
  // if(current - previousMill6 >= intervalReceive){
  //   previousMill6 = current; 

  // }

  // if(writeFlag){
    // saveEEPROM();
  // }

  while(!charging){
    // Serial.print("charging : ");Serial.println(charging);
    temp1 = getTemp(A0);
    temp2 = getTemp(A3);
    // delay(500);
    if(writeFlag){
      // saveEEPROM();

          // Serial.print("writeFlag before : ");Serial.println(writeFlag);
      
          // Serial.print("writeFlag after reset : ");Serial.println(writeFlag);
    }
    // Serial.println("outside already");
    writeFlag = 0;

    // Serial.print("writeFlag after IF : ");Serial.println(writeFlag);
    
    readCAN(); 
    if(readFrame.can_id == 0x7FF){
      // Serial.print("ID ");Serial.println(readFrame.can_id);
      extractCAN();
    }
    measurement_loop(DATALOG_DISABLED, timeOut, sync, charging, balanceActive, voltFull);
    
    // delay(100);

  }


  getCellVoltage();
  float cAv=0;
  float sumBat = sumCell(10);

  cAv = sumBat / totalCell; 

 
  int count = 0;
  int countDis = 0;

  // if(current - previousMill3 >= intervalDis){
  //   previousMill3 = current;
  //   // clearDischarge();
  //   Serial.println("RWADWA");
  //   // dischargeStartTime = current;
  //   // delay(4000);
  // }
  Serial.print("charging : ");Serial.println(charging);
    if(stateC == 0){
      if(current-previousMill3 >= intervalDis){
        Serial.println("inside dww");
        previousMill3 = current;
        previousMill4 = current;
        for(int i = 1;i<=totalCell;i++){
          cellVoltage = BMS_IC[0].cells.c_codes[i-1] * 0.0001;
          Serial.print("cell ");Serial.print(i);Serial.print(": ");Serial.print(cellVoltage);Serial.println();
          if(cellVoltage-cAv >= 0.2){

            Serial.print("i : ");Serial.println(i);
            Serial.print("Cell : ");Serial.println(cellVoltage);
            // Serial.print("cAv : ");Serial.println(cAv);
            // Serial.print("divVolt : ");Serial.println(cellVoltage - cAv);
            discharge(i);
            balanceStatus |= (1 << (i-1));
            // Serial.print("KeepCell : ");Serial.println(keepCell[count]);
            // Serial.print("Count : ");Serial.println(count);
            // Serial.print("Countdis : ");Serial.println(countDis);
            keepCell[count] = i;
            count++;
            countDis++;
            checkDis=0;


            // delay(2000);
        }   
      }
      stateC = 1;
      Serial.println("----------------");
    }
  }

  // for debugging purpose & clearDischarge
  if(stateC == 1){
    if(current - previousMill4 >= intervalClear){
      previousMill4 = current;
      Serial.print("checkDis");Serial.println(checkDis); 
      Serial.print("keepCellCount : ");Serial.println(keepCell[count]);    

      for(int i =0;i<totalCell;i++){
        Serial.print((balanceStatus >> i)&0x01);
      }
      // Serial.println(countDis);
      clearDischarge(); 
      checkDis = 1;
      // Serial.println("------------");
      // if(countDis >= 1){
      //   if(current - dischargeStartTime >= 4000){  
      //     Serial.println("after 4");
      //   }
      // }
      stateC = 0;
    }
    packCANData(sync,timeOut,charging,balanceActive,voltFull,balanceStatus,vbatt,deltaV,temp1,temp2);
    packCANData2();
    sendCANFrame3();
  }




    // if(current - previousMill2 >= intervalDis){
    //   previousMill2 = current;
    //   dischargeStartTime = current;
    //   isDischarge = true;
    //   // delay(4000);   
    // }

   
  // }
  // if(current - previousMill3 >= 1000){
  //   discharge(6);
  //   Serial.println('discharge');
  // }
    
  // if(current - previousMill2 >= 3000){
  //   clearDischarge();
  //   Serial.println('cleardischarge');
  // }
  



  // if (Serial.available()) // Check for user input
  // {
  //   uint32_t user_command;
  //   user_command = read_int(); // Read the user command
  //   if (user_command == 'm')
  //   {
  //     print_menu();
  //   }
  //   else
  //   {
  //     Serial.println(user_command);
  //     run_command(user_command);
  //   }
  // }
}

float sumCell(int cell){
  float sumCell = 0;
  for(int i = 0;i<cell;i++){
    cellVoltage = BMS_IC[0].cells.c_codes[i] * 0.0001;

    sumCell = sumCell + cellVoltage;
  }
  return sumCell;
}
/*!*****************************************
 \brief Executes the user command
 @return void
*******************************************/
void run_command(uint32_t cmd)
{
  uint8_t streg = 0;
  int8_t error = 0;
  uint32_t conv_time = 0;
  int8_t s_pin_read = 0;

  switch (cmd)
  {
  case 1: // Write and Read Configuration Register
    wakeup_sleep(TOTAL_IC);
    LTC6811_wrcfg(TOTAL_IC, BMS_IC); // Write into Configuration Register
    print_wrconfig();
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdcfg(TOTAL_IC, BMS_IC); // Read Configuration Register
    check_error(error);
    print_rxconfig();
    break;

  case 2: // Read Configuration Register
    wakeup_sleep(TOTAL_IC);
    error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
    check_error(error);
    print_rxconfig();
    break;

  case 3: // Start Cell ADC Measurement
    wakeup_sleep(TOTAL_IC);
    LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
    conv_time = LTC6811_pollAdc();
    print_conv_time(conv_time);
    break;

  case 4: // Read Cell Voltage Registers
    wakeup_sleep(TOTAL_IC);
    error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all cell voltage registers
    check_error(error);
    print_cells(DATALOG_DISABLED);
    break;

  case 5: // Start GPIO ADC Measurement
    wakeup_sleep(TOTAL_IC);
    LTC6811_adax(ADC_CONVERSION_MODE, AUX_CH_TO_CONVERT);
    conv_time = LTC6811_pollAdc();
    print_conv_time(conv_time);
    break;

  case 6: // Read AUX Voltage Registers
    wakeup_sleep(TOTAL_IC);
    error = LTC6811_rdaux(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all aux registers
    check_error(error);
    print_aux(DATALOG_DISABLED);
    break;

  case 7: // Start Status ADC Measurement 
    wakeup_sleep(TOTAL_IC);
    LTC6811_adstat(ADC_CONVERSION_MODE, STAT_CH_TO_CONVERT);
    conv_time = LTC6811_pollAdc();
    print_conv_time(conv_time);
    break;

  case 8: // Read Status registers
    wakeup_sleep(TOTAL_IC);
    error = LTC6811_rdstat(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all stat registers
    check_error(error);
    print_stat();
    break;

  case 9: // Start Combined Cell Voltage and GPIO1, GPIO2 Conversion and Poll Status
    wakeup_sleep(TOTAL_IC);
    LTC6811_adcvax(ADC_CONVERSION_MODE, ADC_DCP);
    conv_time = LTC6811_pollAdc();
    print_conv_time(conv_time);
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all cell voltage registers
    check_error(error);
    print_cells(DATALOG_DISABLED);
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdaux(SEL_REG_A, TOTAL_IC, BMS_IC); // Set to read back aux registers A
    check_error(error);
    print_aux(DATALOG_DISABLED);
    break;

  case 10: // Start Combined Cell Voltage and Sum of cells
    wakeup_sleep(TOTAL_IC);
    LTC6811_adcvsc(ADC_CONVERSION_MODE, ADC_DCP);
    conv_time = LTC6811_pollAdc();
    print_conv_time(conv_time);
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all cell voltage registers
    check_error(error);
    print_cells(DATALOG_DISABLED);
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdstat(SEL_REG_A, TOTAL_IC, BMS_IC); // Set to read stat registers A
    check_error(error);
    print_sumofcells();
    break;

  case 11: // Loop Measurements of configuration register or cell voltages or auxiliary register or status register without data-log output
    wakeup_sleep(TOTAL_IC);
    LTC6811_wrcfg(TOTAL_IC, BMS_IC);
    
    // measurement_loop(DATALOG_DISABLED, timeOut,sync,charging,balanceActive,voltFull);
    print_menu();
    break;

  case 12: // Data-log print option Loop Measurements of configuration register or cell voltages or auxiliary register or status register
    wakeup_sleep(TOTAL_IC);
    LTC6811_wrcfg(TOTAL_IC, BMS_IC);
    // measurement_loop(DATALOG_ENABLED);
    print_menu();
    break;

  case 13: // Clear all ADC measurement registers
    wakeup_sleep(TOTAL_IC);
    LTC6811_clrcell();
    LTC6811_clraux();
    LTC6811_clrstat();
    wakeup_idle(TOTAL_IC);
    LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Read back all cell voltage registers
    print_cells(DATALOG_DISABLED);

    LTC6811_rdaux(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Read back all aux registers
    print_aux(DATALOG_DISABLED);

    LTC6811_rdstat(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Read back all stat
    print_stat();
    break;

  case 14: // Read CV,AUX and ADSTAT Voltages
    wakeup_sleep(TOTAL_IC);
    LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
    conv_time = LTC6811_pollAdc();
    print_conv_time(conv_time);
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all cell voltage registers
    check_error(error);
    print_cells(DATALOG_DISABLED);

    wakeup_sleep(TOTAL_IC);
    LTC6811_adax(ADC_CONVERSION_MODE, AUX_CH_TO_CONVERT);
    conv_time = LTC6811_pollAdc();
    print_conv_time(conv_time);
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdaux(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all aux registers
    check_error(error);
    print_aux(DATALOG_DISABLED);

    wakeup_sleep(TOTAL_IC);
    LTC6811_adstat(ADC_CONVERSION_MODE, STAT_CH_TO_CONVERT);
    conv_time = LTC6811_pollAdc();
    print_conv_time(conv_time);
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdstat(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all status registers
    check_error(error);
    print_stat();
    break;

  case 15: // Run the Mux Decoder Self Test
    wakeup_sleep(TOTAL_IC);
    LTC6811_diagn();
    conv_time = LTC6811_pollAdc();
    print_conv_time(conv_time);
    error = LTC6811_rdstat(SEL_REG_B, TOTAL_IC, BMS_IC); // Set to read back status register B
    check_error(error);
    check_mux_fail();
    break;

  case 16: // Run the ADC/Memory Self Test
    error = 0;
    wakeup_sleep(TOTAL_IC);
    error = LTC6811_run_cell_adc_st(CELL, TOTAL_IC, BMS_IC, ADC_CONVERSION_MODE, ADCOPT);
    print_selftest_errors(CELL, error);

    error = 0;
    wakeup_sleep(TOTAL_IC);
    error = LTC6811_run_cell_adc_st(AUX, TOTAL_IC, BMS_IC, ADC_CONVERSION_MODE, ADCOPT);
    print_selftest_errors(AUX, error);

    error = 0;
    wakeup_sleep(TOTAL_IC);
    error = LTC6811_run_cell_adc_st(STAT, TOTAL_IC, BMS_IC, ADC_CONVERSION_MODE, ADCOPT);
    print_selftest_errors(STAT, error);
    print_menu();
    break;

  case 17: // Run ADC Overlap self test
    error = 0;
    wakeup_sleep(TOTAL_IC);
    error = (int8_t)LTC6811_run_adc_overlap(TOTAL_IC, BMS_IC);
    print_overlap_results(error);
    break;

  case 18: // Run ADC Digital Redundancy self test
    error = 0;
    wakeup_sleep(TOTAL_IC);
    error = LTC6811_run_adc_redundancy_st(ADC_CONVERSION_MODE, AUX, TOTAL_IC, BMS_IC);
    print_digital_redundancy_errors(AUX, error);

    error = 0;
    wakeup_sleep(TOTAL_IC);
    error = LTC6811_run_adc_redundancy_st(ADC_CONVERSION_MODE, STAT, TOTAL_IC, BMS_IC);
    print_digital_redundancy_errors(STAT, error);
    break;

  case 19: // Open Wire test for single cell detection
    wakeup_sleep(TOTAL_IC);
    LTC6811_run_openwire_single(TOTAL_IC, BMS_IC);
    print_open_wires();
    break;

  case 20: // Open Wire test for multiple cell and two consecutive cells detection
    wakeup_sleep(TOTAL_IC);
    LTC6811_run_openwire_multi(TOTAL_IC, BMS_IC);
    break;

  case 21: // PEC Errors Detected
    print_pec_error_count();
    break;

  case 22: // Reset PEC Counter
    LTC6811_reset_crc_count(TOTAL_IC, BMS_IC);
    print_pec_error_count();
    break;

  case 23: // Enable a discharge transistor
    s_pin_read = select_s_pin();
    wakeup_sleep(TOTAL_IC);
    LTC6811_set_discharge(s_pin_read, TOTAL_IC, BMS_IC);
    LTC6811_wrcfg(TOTAL_IC, BMS_IC);
    print_wrconfig();
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
    check_error(error);
    print_rxconfig();
    break;

  case 24: // Clear all discharge transistors
    wakeup_sleep(TOTAL_IC);
    LTC6811_clear_discharge(TOTAL_IC, BMS_IC);
    LTC6811_wrcfg(TOTAL_IC, BMS_IC);
    print_wrconfig();
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
    check_error(error);
    print_rxconfig();
    break;

  case 25: // Write read pwm configuration
    /*****************************************************
       PWM configuration data.
       1)Set the corresponding DCC bit to one for pwm operation.
       2)Set the DCTO bits to the required discharge time.
       3)Choose the value to be configured depending on the
        required duty cycle.
       Refer to the data sheet.
    *******************************************************/
    wakeup_sleep(TOTAL_IC);
    for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
      BMS_IC[current_ic].pwm.tx_data[0] = 0x88; // Duty cycle for S pin 2 and 1
      BMS_IC[current_ic].pwm.tx_data[1] = 0x88; // Duty cycle for S pin 4 and 3
      BMS_IC[current_ic].pwm.tx_data[2] = 0x88; // Duty cycle for S pin 6 and 5
      BMS_IC[current_ic].pwm.tx_data[3] = 0x88; // Duty cycle for S pin 8 and 7
      BMS_IC[current_ic].pwm.tx_data[4] = 0x88; // Duty cycle for S pin 10 and 9
      BMS_IC[current_ic].pwm.tx_data[5] = 0x88; // Duty cycle for S pin 12 and 11
    }
    LTC6811_wrpwm(TOTAL_IC, 0, BMS_IC);
    print_wrpwm();

    wakeup_idle(TOTAL_IC);
    LTC6811_rdpwm(TOTAL_IC, 0, BMS_IC);
    print_rxpwm();
    break;

  case 26: // Write and read S Control Register Group
    wakeup_sleep(TOTAL_IC);
    /**************************************************************************************
       S pin control.
       1)Ensure that the pwm is set according to the requirement using the previous case.
       2)Choose the value depending on the required number of pulses on S pin.
       Refer to the data sheet.
    ***************************************************************************************/
    for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
      BMS_IC[current_ic].sctrl.tx_data[0] = 0xFF; // No. of high pulses on S pin 2 and 1
      BMS_IC[current_ic].sctrl.tx_data[1] = 0xFF; // No. of high pulses on S pin 4 and 3
      BMS_IC[current_ic].sctrl.tx_data[2] = 0xFF; // No. of high pulses on S pin 6 and 5
      BMS_IC[current_ic].sctrl.tx_data[3] = 0xFF; // No. of high pulses on S pin 8 and 7
      BMS_IC[current_ic].sctrl.tx_data[4] = 0xFF; // No. of high pulses on S pin 10 and 9
      BMS_IC[current_ic].sctrl.tx_data[5] = 0xFF; // No. of high pulses on S pin 12 and 11
    }
    LTC6811_wrsctrl(TOTAL_IC, streg, BMS_IC);
    print_wrsctrl();

    // Start S Control pulsing
    wakeup_idle(TOTAL_IC);
    LTC6811_stsctrl();

    // Read S Control Register Group
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdsctrl(TOTAL_IC, streg, BMS_IC);
    check_error(error);
    print_rxsctrl();
    break;

  case 27: // Clear S Control Register Group
    wakeup_sleep(TOTAL_IC);
    LTC6811_clrsctrl();

    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdsctrl(TOTAL_IC, streg, BMS_IC); // Read S Control Register Group
    check_error(error);
    print_rxsctrl();
    break;

  case 28: // SPI Communication
    /*************************************************************
       Ensure to set the GPIO bits to 1 in the CFG register group.
    *************************************************************/
    for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
      // Communication control bits and communication data bytes. Refer to the data sheet.
      BMS_IC[current_ic].com.tx_data[0] = 0x81; // Icom CSBM Low(8) + data D0 (0x11)
      BMS_IC[current_ic].com.tx_data[1] = 0x10; // Fcom CSBM Low(0)
      BMS_IC[current_ic].com.tx_data[2] = 0xA2; // Icom CSBM Falling Edge (A) +  D1 (0x25)
      BMS_IC[current_ic].com.tx_data[3] = 0x50; // Fcom CSBM Low(0)
      BMS_IC[current_ic].com.tx_data[4] = 0xA1; // Icom CSBM Falling Edge (A) +  D2 (0x17)
      BMS_IC[current_ic].com.tx_data[5] = 0x79; // Fcom CSBM High(9)
    }
    wakeup_sleep(TOTAL_IC);
    LTC6811_wrcomm(TOTAL_IC, BMS_IC); // write to comm register
    print_wrcomm();                   // print data in the comm register

    wakeup_idle(TOTAL_IC);
    LTC6811_stcomm(3); // data length=3 // initiates communication between master and the I2C slave

    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdcomm(TOTAL_IC, BMS_IC); // read from comm register
    check_error(error);
    print_rxcomm(); // print received data into the comm register
    break;

  case 29: // write byte I2C Communication on the GPIO Ports(using I2C eeprom 24LC025)
           /************************************************************
             Ensure to set the GPIO bits to 1 in the CFG register group.
          *************************************************************/
    for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
      // Communication control bits and communication data bytes. Refer to the data sheet.
      BMS_IC[current_ic].com.tx_data[0] = 0x6A; // Icom Start(6) + I2C_address D0 (0xA0)
      BMS_IC[current_ic].com.tx_data[1] = 0x08; // Fcom master NACK(8)
      BMS_IC[current_ic].com.tx_data[2] = 0x00; // Icom Blank (0) + eeprom address D1 (0x00)
      BMS_IC[current_ic].com.tx_data[3] = 0x08; // Fcom master NACK(8)
      BMS_IC[current_ic].com.tx_data[4] = 0x01; // Icom Blank (0) + data D2 (0x11)
      BMS_IC[current_ic].com.tx_data[5] = 0x19; // Fcom master NACK + Stop(9)
    }
    wakeup_sleep(TOTAL_IC);
    LTC6811_wrcomm(TOTAL_IC, BMS_IC); // write to comm register
    print_wrcomm();                   // print transmitted data from the comm register

    wakeup_idle(TOTAL_IC);
    LTC6811_stcomm(3); // data length=3 // initiates communication between master and the I2C slave

    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdcomm(TOTAL_IC, BMS_IC); // read from comm register
    check_error(error);
    print_rxcomm(); // print received data into the comm register
    break;

  case 30: // Read byte data I2C Communication on the GPIO Ports(using I2C eeprom 24LC025)
    /************************************************************
       Ensure to set the GPIO bits to 1 in the CFG register group.
    *************************************************************/
    for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
      // Communication control bits and communication data bytes. Refer to the data sheet.
      BMS_IC[current_ic].com.tx_data[0] = 0x6A; // Icom Start (6) + I2C_address D0 (A0) (Write operation to set the word address)
      BMS_IC[current_ic].com.tx_data[1] = 0x08; // Fcom master NACK(8)
      BMS_IC[current_ic].com.tx_data[2] = 0x00; // Icom Blank (0) + eeprom address(word address) D1 (0x00)
      BMS_IC[current_ic].com.tx_data[3] = 0x08; // Fcom master NACK(8)
      BMS_IC[current_ic].com.tx_data[4] = 0x6A; // Icom Start (6) + I2C_address D2 (0xA1)(Read operation)
      BMS_IC[current_ic].com.tx_data[5] = 0x18; // Fcom master NACK(8)
    }
    wakeup_sleep(TOTAL_IC);
    LTC6811_wrcomm(TOTAL_IC, BMS_IC); // write to comm register

    wakeup_idle(TOTAL_IC);
    LTC6811_stcomm(3); // data length=3 // initiates communication between master and the I2C slave

    for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
      // Communication control bits and communication data bytes. Refer to the data sheet.
      BMS_IC[current_ic].com.tx_data[0] = 0x0F; // Icom Blank (0) + data D0 (FF)
      BMS_IC[current_ic].com.tx_data[1] = 0xF9; // Fcom master NACK + Stop(9)
      BMS_IC[current_ic].com.tx_data[2] = 0x7F; // Icom No Transmit (7) + data D1 (FF)
      BMS_IC[current_ic].com.tx_data[3] = 0xF9; // Fcom master NACK + Stop(9)
      BMS_IC[current_ic].com.tx_data[4] = 0x7F; // Icom No Transmit (7) + data D2 (FF)
      BMS_IC[current_ic].com.tx_data[5] = 0xF9; // Fcom master NACK + Stop(9)
    }

    wakeup_idle(TOTAL_IC);
    LTC6811_wrcomm(TOTAL_IC, BMS_IC); // write to comm register

    wakeup_idle(TOTAL_IC);
    LTC6811_stcomm(1); // data length=1 // initiates communication between master and the I2C slave

    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdcomm(TOTAL_IC, BMS_IC); // read from comm register
    check_error(error);
    print_rxcomm(); // print received data from the comm register
    break;
  
  case 31: // Set or reset the gpio pins(to drive output on gpio pins)
    /***********************************************************************
     Please ensure you have set the GPIO bits according to your requirement
     in the configuration register.( check the global variable GPIOBITS_A )
    ************************************************************************/
    wakeup_sleep(TOTAL_IC);
    for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
      LTC6811_set_cfgr(current_ic, BMS_IC, REFON, ADCOPT, GPIOBITS_A, DCCBITS_A, DCTOBITS, UV, OV);
    }
    wakeup_idle(TOTAL_IC);
    LTC6811_wrcfg(TOTAL_IC, BMS_IC);
    print_wrconfig();
    break;
  case 32:
    for (int i = 1; i<=6;i++){
      LTC681x_rdcv_reg(i,1,data);
      Serial.print("Cell value : ");Serial.println(get_cell_value(data,0));
    }
    // Serial.print("data 0 : "); Serial.println(data[0]);
    // Serial.print("data 1 : "); Serial.println(data[1]);
    // Serial.print("data 2 : "); Serial.println(data[2]);
    // Serial.print("data 3 : "); Serial.println(data[3]);
    // Serial.print("data 4 : "); Serial.println(data[4]);
    // Serial.print("data 5 : "); Serial.println(data[5]);
    
    break;

  case 'm': // prints menu
    print_menu();
    break;

  default:
    char str_error[] = "Incorrect Option \n";
    serial_print_text(str_error);
    break;
  }
}

/*!**********************************************************************************************************************************************
 \brief For writing/reading configuration data or measuring cell voltages or reading aux register or reading status register in a continuous loop
 @return void
*************************************************************************************************************************************************/

void getCellVoltage(){
  uint8_t error;  
  select_LTC6811();
  wakeup_idle(TOTAL_IC);
  LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  LTC6811_pollAdc();
  wakeup_idle(TOTAL_IC);
  error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC);
  check_error(error);
  print_cells(DATALOG_DISABLED);
  deselect_LTC6811();

  
  // for(int i=0;i<10;i++){
  //   Serial.println(BMS_IC[0].cells.c_codes[i]* 0.0001);

  // }
}
// int ic;
// float difV ;
void measurement_loop(uint8_t datalog_en, int timeOut, int sync, int charging, int balanceActive, int voltFull)
{
  current = millis();
  float temp = 60.0;
  int8_t error = 0;
  // char input = 0;

  // Serial.println(F("Transmit 'm' to quit"));

    // if (WRITE_CONFIG == ENABLED)
    // {
    //   wakeup_sleep(TOTAL_IC);
    //   LTC6811_wrcfg(TOTAL_IC, BMS_IC);
    //   print_wrconfig();
    // }

    // if (READ_CONFIG == ENABLED)
    // {
    //   wakeup_sleep(TOTAL_IC);
    //   error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
    //   check_error(error);
    //   print_rxconfig();
    // }

    if (MEASURE_CELL == ENABLED)
    {
      select_LTC6811();
      wakeup_idle(TOTAL_IC);
      LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
      LTC6811_pollAdc();
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC);
      check_error(error);
      // print_cells(datalog_en);
      deselect_LTC6811();
      
      // clearing byte
      for (int a = 0;a<6;a++){
        frame4.data[a] = 0;
        frame5.data[a] = 0;
      }

      //NOT USE

      // if(state == 1){
      //   if(current-previousMill >= interval){
      //     packCANData(sync,timeOut,charging,balanceActive,voltFull,balanceStatus,vbatt,deltaV,temp1,temp2);
      //     // debugFrame1();
      //     packCANData2(temp);
      //     previousMill = current;
      //     sendCANFrame3();
      //     state = 2;
      //     // Serial.print("Time : ");Serial.println(actual);
      //   }

      // }

      avrBatt = 0;
      vbatt = 0;
      // int count =0;

      for (int w =0; w<10;w++){
        cellVoltage = BMS_IC[0].cells.c_codes[w] * 0.0001;
        vbatt = vbatt + cellVoltage;
      }

      
      avrBatt = vbatt / 10;
      deltaV = maxVolt - avrBatt;   




      //pack & send CAN 1st message
      if(current-previousMill >= interval){
        previousMill = current;
        // Serial.print("maxVolt : ");Serial.println(maxVolt);
        packCANData(sync,timeOut,charging,balanceActive,voltFull,balanceStatus,vbatt,deltaV,temp1,temp2);
        // debugFrame1();
        packCANData2();
        sendCANFrame3();
        
        // Serial.println("dwadwa");
        // Serial.print("Time : ");Serial.println(actual);
      }

      
      // Serial.println("Frame 2 data:");
      // for (int w = 0; w < 8; w++) {
      //     Serial.print("Byte ");
      //     Serial.print(w);
      //     Serial.print(": ");
      //     Serial.println(frame2.data[w], HEX);  // Print in hexadecimal for better readability
      // }

      // Serial.println("Frame 3 data:");
      // for (int w = 0; w < 8; w++) {
      //     Serial.print("Byte ");
      //     Serial.print(w);
      //     Serial.print(": ");
      //     Serial.println(frame3.data[w], HEX);
      // }
      
      
      for (int w =0; w<10;w++){
        cellVoltage = BMS_IC[0].cells.c_codes[w] * 0.0001;
        packCANFrame45(cellVoltage,temp,w, maxVolt, minVolt);
      }
      
      if(current - previousMill2 >= intervalFault){
        previousMill2 = current;
        sendCANFrame45();
              
      }
      
      // if(state == 2){
      //   if(current - previousMill >= intervalFault){
      //     previousMill = current;
      //     sendCANFrame45();
      //     state = 1;
      //   }
      // }
   
  
    }

    // DONT HAVE TO USE THESE BELOW

    // if (MEASURE_AUX == ENABLED)
    // {
    //   wakeup_idle(TOTAL_IC);
    //   LTC6811_adax(ADC_CONVERSION_MODE, AUX_CH_ALL);
    //   LTC6811_pollAdc();
    //   wakeup_idle(TOTAL_IC);
    //   error = LTC6811_rdaux(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all aux registers
    //   check_error(error);
    //   print_aux(datalog_en);
    // }

    // if (MEASURE_STAT == ENABLED)
    // {
    //   wakeup_idle(TOTAL_IC);
    //   LTC6811_adstat(ADC_CONVERSION_MODE, STAT_CH_ALL);
    //   LTC6811_pollAdc();
    //   wakeup_idle(TOTAL_IC);
    //   error = LTC6811_rdstat(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all aux registers
    //   check_error(error);
    //   print_stat();
    // }

    // if (PRINT_PEC == ENABLED)
    // {
    //   print_pec_error_count();
    // }

    // delay(MEASUREMENT_LOOP_TIME);
  
}

void packCANData(bool sync, bool timeOut, bool charging, bool balanceActive, bool voltFull, 
                  uint16_t balanceStatus, float vbatt, float deltaV, float temp1, float temp2) {
    // Byte 0 - Status flags
    frame.data[0] = 0; // Clear all bits first
    if(sync) frame.data[0] |= (1<<0);
    if(timeOut) frame.data[0] |= (1<<1);
    if(charging) frame.data[0] |= (1<<2);  
    if(balanceActive) frame.data[0] |= (1<<3);
    if(voltFull) frame.data[0] |= (1<<4);
    
    // Byte 1-2 - Balance cell status (10-bit representation)
    frame.data[1] = (balanceStatus >> 8) & 0xFF;  
    frame.data[2] = balanceStatus & 0xFF;        

    // Byte 3 - Vbatt (0-42.0V, scale 0.2V per unit)
    frame.data[3] = (byte)(vbatt / 0.2);
    
    // Byte 4 - Delta V (0-0.2V, scale 0.1V per unit)
    frame.data[4] = (byte)(deltaV / 0.1);
    
    // Byte 5 - Temperature 1
    // Vsense = 2 + (Raw × 0.0125)
    // Therefore: Raw = (Vsense - 2) / 0.0125
    // Serial.print("temp1 : ");Serial.println(temp1);
    byte temp1Raw = (byte)((temp1 - 2.0) / 0.6);
    frame.data[5] = temp1Raw;
    
    // Byte 6 - Temperature 2
    //  Serial.print("temp2 : ");Serial.println(temp2);
    byte temp2Raw = (byte)((temp2 - 2.0) / 0.6);
    frame.data[6] = temp2Raw;
    
    // Byte 7 - Reserved (for SOC/SOH)
    frame.data[7] = 0;  // Currently unused

    // state = 2;
    // Serial.println("CAN frame sent");
}

void packCANData2() {
  // Scale voltage (assuming conversion needed)
  // Serial.print("this is message 2");
  for(int cell = 0;cell<10;cell++){
    cellVoltage = BMS_IC[0].cells.c_codes[cell] * 0.0001;
    // Serial.print("cell volt : ");
    // Serial.println(cellVoltage);
    byte voltageScaled = (byte)(cellVoltage / 0.02);
    // Frame 1 (first 8 cells)
    if (cell + 1 <= 8) {
      frame2.data[cell] = voltageScaled;
    }
    if (cell+1 >= 9 && cell + 1 <= 10) {
      frame3.data[cell-8] = voltageScaled;
    }

    for (int i = 2; i < 8; i++) {
      frame3.data[i] = 0;
    }
    // state = 1;

  }

}

// void send1CANFrame(){
//   Serial.println("sent");
//   mcp2515.sendMessage(&frame);
  
// }
// void send2CANFrame(){
//   Serial.println("sent");
//   mcp2515.sendMessage(&frame);
//   mcp2515.sendMessage(&frame2);
 
// }

void packCANFrame45(float cellVoltage, float temp,  int e, float maxVolt, float minVolt){

  // for(int c = 0;c<7;c++){
  //   frame4.data[c] = 0;
  // }
    // Serial.println(deltaV);

  // first byte
    if(cellVoltage >= 0.95 * maxVolt){ // for OverVoltage Warning
      if(e<8){
        frame4.data[0] |= (1<<(7-e));
      }
      else{
        frame4.data[1] |= (1<<(15-e));
      }
    }

    //second byte
    if(cellVoltage > maxVolt){ // for OverVoltage Critical
      if(e<8){
        frame4.data[2] |= (1<<(7-e));
      }
      else{
        frame4.data[3] |= (1<<(15-e));
      }
    }

    //thirdw byte
    if(cellVoltage <= 1.12 * minVolt){ //for Low Voltage warning 
      if(e<8){
        frame4.data[4] |= (1<<(7-e));
      }
      else{
        frame4.data[5] |= (1<<(15-e));
      }
    }


    if(cellVoltage <= 1.1 * minVolt){ //for Low Voltage critical
      if(e<8){
        frame4.data[6] |= (1<<(7-e));
      }
      else{
        frame4.data[7] |= (1<<(15-e));
      }
    }

    if(temp >= 0.8*60){//over temp warning
      if(e<8){
        frame5.data[0] |= (1<<(7-e));
      }
      else{
        frame5.data[1] |= (1<<(15-e));
      }
    }

    if(temp >= 0.9*60){//overtemp critical
      if(e<8){
        frame5.data[2] |= (1<<(7-e));
      }
      else{
        frame5.data[3] |= (1<<(15-e));
      }
    }

    if(deltaV >= 0.18){ //Over Div Voltage Warning
      if(e<8){
        frame5.data[4] |= (1<<(7-e));
      }
      else{
        frame5.data[5] |= (1<<(15-e));
      }
    }

    if(deltaV >= 0.2){//Over Div Voltage Critical
      if(e<8){
        frame5.data[6] |= (1<<(7-e));
      }
      else{
        frame5.data[7] |= (1<<(15-e));
      }
    }
  
  // frame.data[0] = 0; // Clear all bits first
  // if() frame.data[0] |= (1<<0);
  // if(timeOut) frame.data[0] |= (1<<1);
  // if(charging) frame.data[0] |= (1<<2);
  // if(balanceActive) frame.data[0] |= (1<<3);
  // if(voltFull) frame.data[1] |= (1<<4);
  // if(voltFull) frame.data[1] |= (1<<5);
  // if(voltFull) frame.data[1] |= (1<<6);
  // if(voltFull) frame.data[1] |= (1<<7);
  // if(voltFull) frame.data[1] |= (1<<8);
  // if(voltFull) frame.data[1] |= (1<<9);
    
}
void debugFault(int byte){
  Serial.print("byte :");Serial.println(byte);
  for(int i = 7; i >= 0; i--) {
// Print each bit (0 or 1)
    Serial.print((frame5.data[byte] >> i) & 1);
    Serial.print(" ");
  }
  Serial.println();
  
//   for(int i = 7; i >= 0; i--) {
// // Print each bit (0 or 1)
//     Serial.print((frame4.data[byte+1] >> i) & 1);
//     Serial.print(" ");
//   }
  Serial.println();
}
void sendCANFrame3(){
  // mcp2515.sendMessage(MCP2515::TXB1, &frame);
  // mcp2515.sendMessage(MCP2515::TXB1, &frame2);
  // mcp2515.sendMessage(MCP2515::TXB1, &frame3);
  mcp2515.sendMessage(&frame);
  mcp2515.sendMessage(&frame2);
  mcp2515.sendMessage(&frame3);
  // Serial.println("sent");
}

void sendCANFrame45(){
  // mcp2515.sendMessage(MCP2515::TXB1, &frame4);
  // mcp2515.sendMessage(MCP2515::TXB1, &frame5);
  mcp2515.sendMessage(&frame4);
  mcp2515.sendMessage(&frame5);

}

void debugFrame1() {
    Serial.println("Frame 1 Data:");
    // Status bits
    Serial.print("Status : (0b");
    for (int i = 7; i >= 0; i--) {
        Serial.print((frame.data[0] >> i) & 1);
    }
    Serial.println(")");
    
    // Balance status
    uint16_t balanceStatus = (frame.data[1] << 8) | frame.data[2];
    Serial.print("Balance Status: 0b");
    for (int i = 9; i >= 0; i--) {
        Serial.print((balanceStatus >> i) & 0x01);
    }
    Serial.println();
    
    // Voltages and temperatures
    float vbatt = frame.data[3] * 0.2;
    float deltaV = frame.data[4] * 0.1;
    float temp1 = 2.0 + (frame.data[5] * 0.0125);
    float temp2 = 2.0 + (frame.data[6] * 0.0125);
    
    Serial.print("Vbatt: "); Serial.print(vbatt); Serial.println("V");
    Serial.print("Volt average: "); Serial.print(avrBatt); Serial.println("V");
    Serial.print("Delta V: "); Serial.print(deltaV); Serial.println("V");
    Serial.print("Temp1 Vsense: "); Serial.print(temp1); Serial.println("V");
    Serial.print("Temp2 Vsense: "); Serial.print(temp2); Serial.println("V");
}

void readCAN(){
    if (mcp2515.readMessage(&readFrame) == MCP2515::ERROR_OK) {
    received_id = frame.can_id & ~CAN_EFF_FLAG;
    // received_id = frame.can_id;
    // Serial.print(readFrame.can_id, HEX); // print ID
    // Serial.print(" "); 
    // Serial.print(readFrame.can_dlc, HEX); // print DLC
    // Serial.print(" ");
    // Serial.print("received iD : ");Serial.println(received_id,HEX);
    // for (int i = 0; i<readFrame.can_dlc; i++)  {
    //   Serial.print(readFrame.data[i],HEX);
    // }
    // Serial.println("-------------------------");     

  }else{
    readFrame = can_frame();
    // Serial.println("reset frame");
    // Serial.print("flag : ");Serial.println(readFrame.data[6]);
  }
}

void extractCAN(){
    sync = readFrame.data[0]; 
  
    charging = readFrame.data[1];

    maxVolt = readFrame.data[2];
  
    minVolt = readFrame.data[3];
  
    tMax = readFrame.data[4];

    dVMax = readFrame.data[5];

    writeFlag = readFrame.data[6];
    
    // Serial.print("maxVolt : ");Serial.println(maxVolt);
    // Serial.print("writeFlag : ");Serial.println(writeFlag);
    // // Serial.print("sync : ");Serial.println(sync);
    // Serial.print("charging : ");Serial.println(charging);
    // Serial.print("maxVOlt : ");Serial.println(maxVolt);
    // Serial.print("minVolt : ");Serial.println(minVolt);
    // Serial.print("tMax : ");Serial.println(tMax);
    // Serial.print("dVMax : ");Serial.println(dVMax);
}

void select_LTC6811() {
    digitalWrite(LTC6811_CS_PIN, LOW);
}

void deselect_LTC6811() {
    digitalWrite(LTC6811_CS_PIN, HIGH);
}

void discharge(int selected){
  // Serial.print("disdisdisdis"); 
  int8_t error = 0;
  wakeup_sleep(TOTAL_IC);
  LTC6811_set_discharge(selected, TOTAL_IC, BMS_IC);
  LTC6811_wrcfg(TOTAL_IC, BMS_IC);

}
 
void clearDischarge(){
  wakeup_sleep(TOTAL_IC);
  LTC6811_clear_discharge(TOTAL_IC, BMS_IC);
  LTC6811_wrcfg(TOTAL_IC, BMS_IC);
  Serial.println("clearDischarge");
}

void saveEEPROM(){
  
    Serial.println("write to EEPROM.");
    EEPROM.write(0, (byte) (sync) );            // 200
    EEPROM.write(1, (byte) (maxVolt / 0.1) );  // 42
    EEPROM.write(2, (byte) (minVolt / 0.1) );   // 32
    EEPROM.write(3, (byte) (tMax) );            // 60
    EEPROM.write(4, (byte) (dVMax / 0.1) );     // 2
    Serial.println("Writing to EEPROM. complete");

    Serial.print("sync : ");Serial.println(sync);
    Serial.print("voltMax : ");Serial.println(maxVolt);
    Serial.print("voltMin : ");Serial.println(minVolt);
    Serial.print("tMax : ");Serial.println(tMax);
    Serial.print("dVMax : ");Serial.println(dVMax);

    SYNC = EEPROM.get(0, sync); // 200
    MAXVOLT = EEPROM.get(1, maxVolt);  // 42
    MINVOLT = EEPROM.get(2, minVolt ); // 32
    TMAX = EEPROM.get(3, tMax); // 60
    DVMAX = EEPROM.get(4, dVMax); 

    MAXVOLT = MAXVOLT*0.1;
    MINVOLT = MINVOLT*0.1;
    DVMAX = DVMAX*0.1;

      // ไปทำส่วนที่ retrieve ข้อมูลเฉพาะจากตรงนี้ไปใช้
    
}

float getTemp(int pin){

  float voltage = analogRead(pin) * (5.0 / 1023.0);
  float a = 18.26*voltage;
  float b = 0.143*voltage;

  float aa = a-67.8;
  float bb = b-0.713;

  float tempC = aa/bb;

  // Serial.print("voltage :");Serial.println(voltage);
  Serial.print("tempC ");Serial.print(pin);Serial.print(": ");Serial.println(tempC);
  return tempC;
}
// Another set of variable

/*!*********************************
  \brief Prints the main menu
 @return void
***********************************/
void print_menu(void)
{
  Serial.println(F("List of 6811 Commands: "));
  Serial.println(F("Write and Read Configuration: 1                            |Loop measurements with data-log output: 12     |Set Discharge: 23"));
  Serial.println(F("Read Configuration: 2                                      |Clear Registers: 13                            |Clear Discharge: 24"));
  Serial.println(F("Start Cell Voltage Conversion: 3                           |Read CV,AUX and ADSTAT Voltages: 14            |Write and Read of PWM: 25"));
  Serial.println(F("Read Cell Voltages: 4                                      |Run Mux Self Test: 15                          |Write and Read of S control: 26"));
  Serial.println(F("Start Aux Voltage Conversion: 5                            |Run ADC Self Test: 16                          |Clear S control register: 27"));
  Serial.println(F("Read Aux Voltages: 6                                       |ADC overlap Test : 17                          |SPI Communication: 28"));
  Serial.println(F("Start Stat Voltage Conversion: 7                           |Run Digital Redundancy Test: 18                |I2C Communication Write to Slave: 29"));
  Serial.println(F("Read Stat Voltages: 8                                      |Open Wire Test for single cell detection: 19   |I2C Communication Read from Slave:30"));
  Serial.println(F("Start Combined Cell Voltage and GPIO1, GPIO2 Conversion: 9 |Open Wire Test for multiple cell detection: 20 |Set or Reset the GPIO pins: 31 "));
  Serial.println(F("Start  Cell Voltage and Sum of cells : 10                  |Print PEC Counter: 21                          |"));
  Serial.println(F("Loop Measurements: 11                                      |Reset PEC Counter: 22                          | \n "));

  Serial.println(F("Print 'm' for menu"));
  Serial.println(F("Please enter command: \n"));
}

/*!******************************************************************************
 \brief Prints the configuration data that is going to be written to the LTC6811
 to the serial port.
 @return void
 ********************************************************************************/
void print_wrconfig(void)
{
  int cfg_pec;

  Serial.println(F("Written Configuration: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F("CFGA IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].config.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    cfg_pec = pec15_calc(6, &BMS_IC[current_ic].config.tx_data[0]);
    serial_print_hex((uint8_t)(cfg_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(cfg_pec));
    Serial.println("\n");
  }
}

/*!*****************************************************************
 \brief Prints the configuration data that was read back from the
 LTC6811 to the serial port.
  @return void
 *******************************************************************/
void print_rxconfig(void)
{
  Serial.println(F("Received Configuration "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F("CFGA IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].config.rx_data[i]);
    }
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].config.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].config.rx_data[7]);
    Serial.println("\n");
  }
}

/*!************************************************************
  \brief Prints cell voltage to the serial port
   @return void
 *************************************************************/
void print_cells(uint8_t datalog_en)
{
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic + 1, DEC);
      Serial.print(": ");
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++)
      {
        Serial.print(" C");
        Serial.print(i + 1, DEC);
        Serial.print(":");
        Serial.print(BMS_IC[current_ic].cells.c_codes[i]* 0.0001, 4);
        Serial.print(",");
      }
      Serial.println();
    }
    else
    {
      Serial.print(" Cells :");
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++)
      {
        Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
        Serial.print(",");
      }
    }
  }
  Serial.println("\n");
}

/*!****************************************************************************
  \brief Prints GPIO voltage codes and Vref2 voltage code onto the serial port
 @return void
 *****************************************************************************/
void print_aux(uint8_t datalog_en)
{

  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic + 1, DEC);
      Serial.print(":");

      for (int i = 0; i < 5; i++)
      {
        Serial.print(F(" GPIO-"));
        Serial.print(i + 1, DEC);
        Serial.print(":");
        Serial.print(BMS_IC[current_ic].aux.a_codes[i] * 0.0001, 4);
        Serial.print(",");
      }
      Serial.print(F(" Vref2"));
      Serial.print(":");
      Serial.print(BMS_IC[current_ic].aux.a_codes[5] * 0.0001, 4);
      Serial.println();
    }
    else
    {
      Serial.print("AUX ");
      Serial.print(" IC ");
      Serial.print(current_ic + 1, DEC);
      Serial.print(": ");

      for (int i = 0; i < 6; i++)
      {
        Serial.print(BMS_IC[current_ic].aux.a_codes[i] * 0.0001, 4);
        Serial.print(",");
      }
    }
  }
  Serial.println("\n");
}

/*!****************************************************************************
  \brief Prints Status voltage codes and Vref2 voltage code onto the serial port
 @return void
 *****************************************************************************/
void print_stat(void)
{
  double itmp;
  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(": "));
    Serial.print(F(" SOC:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[0] * 0.0001 * 20, 4);
    Serial.print(F(","));
    Serial.print(F(" Itemp:"));
    itmp = (double)((BMS_IC[current_ic].stat.stat_codes[1] * (0.0001 / 0.0075)) - 273); // Internal Die Temperature(°C) = itmp • (100 µV / 7.5mV)°C - 273°C
    Serial.print(itmp, 4);
    Serial.print(F(","));
    Serial.print(F(" VregA:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[2] * 0.0001, 4);
    Serial.print(F(","));
    Serial.print(F(" VregD:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[3] * 0.0001, 4);
    Serial.println();
    Serial.print(F(" Flags:"));
    Serial.print(F(" 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.flags[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.flags[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.flags[2]);
    Serial.print(F("   Mux fail flag:"));
    Serial.print(F(" 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.mux_fail[0]);
    Serial.print(F("   THSD:"));
    Serial.print(F(" 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.thsd[0]);
    Serial.println("\n");
  }
}

/*!****************************************************************************
  \brief Prints Status voltage codes for SOC onto the serial port
 @return void
 *****************************************************************************/
void print_sumofcells(void)
{
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(": "));
    Serial.print(F(" SOC:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[0] * 0.0001 * 20, 4);
    Serial.print(F(","));
  }
  Serial.println("\n");
}

/*!****************************************************************
  \brief Function to check the MUX fail bit in the Status Register
   @return void
*******************************************************************/
void check_mux_fail(void)
{
  int8_t error = 0;
  for (int ic = 0; ic < TOTAL_IC; ic++)
  {
    Serial.print(" IC ");
    Serial.println(ic + 1, DEC);
    if (BMS_IC[ic].stat.mux_fail[0] != 0)
      error++;

    if (error == 0)
      Serial.println(F("Mux Test: PASS \n"));
    else
      Serial.println(F("Mux Test: FAIL \n"));
  }
}

/*!************************************************************
  \brief Prints Errors Detected during self test
   @return void
*************************************************************/
void print_selftest_errors(uint8_t adc_reg, int8_t error)
{
  if (adc_reg == 1)
  {
    Serial.println("Cell ");
  }
  else if (adc_reg == 2)
  {
    Serial.println("Aux ");
  }
  else if (adc_reg == 3)
  {
    Serial.println("Stat ");
  }
  Serial.print(error, DEC);
  Serial.println(F(" : errors detected in Digital Filter and Memory \n"));
}

/*!************************************************************
  \brief Prints the output of  the ADC overlap test
   @return void
*************************************************************/
void print_overlap_results(int8_t error)
{
  if (error == 0)
    Serial.println(F("Overlap Test: PASS \n"));
  else
    Serial.println(F("Overlap Test: FAIL \n"));
}

/*!************************************************************
  \brief Prints Errors Detected during Digital Redundancy test
   @return void
*************************************************************/
void print_digital_redundancy_errors(uint8_t adc_reg, int8_t error)
{
  if (adc_reg == 2)
  {
    Serial.println("Aux ");
  }
  else if (adc_reg == 3)
  {
    Serial.println("Stat ");
  }

  Serial.print(error, DEC);
  Serial.println(F(" : errors detected in Measurement \n"));
}

/*****************************************************************************
  \brief Prints Open wire test results to the serial port
  @return void
 *****************************************************************************/
void print_open_wires(void)
{
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    if (BMS_IC[current_ic].system_open_wire == 65535)
    {
      Serial.print("No Opens Detected on IC ");
      Serial.println(current_ic + 1, DEC);
    }
    else
    {
      Serial.print(F("There is an open wire on IC "));
      Serial.print(current_ic + 1, DEC);
      Serial.print(F(" Channel: "));
      Serial.println(BMS_IC[current_ic].system_open_wire);
    }
    Serial.println("\n");
  }
}

/*!************************************************************
  \brief Prints the PEC errors detected to the serial port
  @return void
 *************************************************************/
void print_pec_error_count(void)
{
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.println("");
    Serial.print(BMS_IC[current_ic].crc_count.pec_count, DEC);
    Serial.print(F(" : PEC Errors Detected on IC"));
    Serial.println(current_ic + 1, DEC);
  }
  Serial.println("\n");
}

/*!****************************************************
  \brief Function to select the S pin for discharge
  @return void
 ******************************************************/
int8_t select_s_pin(void)
{
  int8_t read_s_pin = 0;

  Serial.print(F("Please enter the Spin number: "));
  read_s_pin = (int8_t)read_int();
  Serial.println(read_s_pin);
  return (read_s_pin);
}

/*!******************************************************************************
 \brief Prints  PWM the configuration data that is going to be written to the LTC6811
 to the serial port.
  @return void
 ********************************************************************************/
void print_wrpwm(void)
{
  int pwm_pec;

  Serial.println(F("Written PWM Configuration: "));
  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F("IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].pwm.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    pwm_pec = pec15_calc(6, &BMS_IC[current_ic].pwm.tx_data[0]);
    serial_print_hex((uint8_t)(pwm_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(pwm_pec));
    Serial.println("\n");
  }
}

/*!*****************************************************************
 \brief Prints the PWM configuration data that was read back from the
 LTC6811 to the serial port.
 @return void
 *******************************************************************/
void print_rxpwm(void)
{
  Serial.println(F("Received pwm Configuration:"));
  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F("IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].pwm.rx_data[i]);
    }
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].pwm.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].pwm.rx_data[7]);
    Serial.println("\n");
  }
}

/*!************************************************************
  \brief Prints S control register data to the serial port
  @return void
 *************************************************************/
void print_wrsctrl(void)
{
  int sctrl_pec;

  Serial.println(F("Written Data in Sctrl register: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC: "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(" Sctrl register group:"));
    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrl.tx_data[i]);
    }

    Serial.print(F(", Calculated PEC: 0x"));
    sctrl_pec = pec15_calc(6, &BMS_IC[current_ic].sctrl.tx_data[0]);
    serial_print_hex((uint8_t)(sctrl_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(sctrl_pec));
    Serial.println("\n");
  }
}

/*!************************************************************
  \brief Prints s control register data that was read back from the
 LTC6811 to the serial port.
@return void
 *************************************************************/
void print_rxsctrl(void)
{
  Serial.println(F("Received Data:"));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);

    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrl.rx_data[i]);
    }

    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].sctrl.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].sctrl.rx_data[7]);
    Serial.println("\n");
  }
}

/*!************************************************************
  \brief Prints comm register data to the serial port
  @return void
 *************************************************************/
void print_wrcomm(void)
{
  int comm_pec;

  Serial.println(F("Written Data in COMM Register: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC- "));
    Serial.print(current_ic + 1, DEC);

    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].com.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    comm_pec = pec15_calc(6, &BMS_IC[current_ic].com.tx_data[0]);
    serial_print_hex((uint8_t)(comm_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(comm_pec));
    Serial.println("\n");
  }
}

/*!************************************************************
  \brief Prints comm register data that was read back from the
 LTC6811 to the serial port.
 @return void
 *************************************************************/
void print_rxcomm(void)
{
  Serial.println(F("Received Data in COMM register:"));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC- "));
    Serial.print(current_ic + 1, DEC);

    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].com.rx_data[i]);
    }
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].com.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].com.rx_data[7]);
    Serial.println("\n");
  }
}

/*!****************************************************************************
  \brief Function to print the Conversion Time
  @return void
 *****************************************************************************/
void print_conv_time(uint32_t conv_time)
{
  uint16_t m_factor = 1000; // to print in ms

  Serial.print(F("Conversion completed in:"));
  Serial.print(((float)conv_time / m_factor), 1);
  Serial.println(F("ms \n"));
}

/*!************************************************************
  \brief Function to check error flag and print PEC error message
  @return void
 *************************************************************/
void check_error(int error)
{
  if (error == -1)
  {
    Serial.println(F("A PEC error was detected in the received data"));
  }
}

/*!************************************************************
  \brief Function to print text on serial monitor
  @return void
*************************************************************/
void serial_print_text(char data[])
{
  Serial.println(data);
}

/*!************************************************************
 \brief Function to print in HEX form
 @return void
 *************************************************************/
void serial_print_hex(uint8_t data)
{
  if (data < 16)
  {
    Serial.print("0");
    Serial.print((byte)data, HEX);
  }
  else
    Serial.print((byte)data, HEX);
}

/*!************************************************************
 \brief Hex conversion constants
 *************************************************************/
char hex_digits[16] =
    {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

/*!************************************************************
 \brief Global Variables
 *************************************************************/
char hex_to_byte_buffer[5] =
    {
        '0', 'x', '0', '0', '\0'};

/*!************************************************************
 \brief Buffer for ASCII hex to byte conversion
 *************************************************************/
char byte_to_hex_buffer[3] =
    {
        '\0', '\0', '\0'};

/*!************************************************************
 \brief Read 2 hex characters from the serial buffer and convert them to a byte
 @return char data Read Data
 *************************************************************/
char read_hex(void)
{
  byte data;
  hex_to_byte_buffer[2] = get_char();
  hex_to_byte_buffer[3] = get_char();
  get_char();
  get_char();
  data = strtol(hex_to_byte_buffer, NULL, 0);
  return (data);
}

/*!************************************************************
 \brief Read a command from the serial port
 @return char
 *************************************************************/
char get_char(void)
{
  while (Serial.available() <= 0)
    ;
  return (Serial.read());
}