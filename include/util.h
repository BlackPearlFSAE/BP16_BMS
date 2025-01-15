#include <iostream>
#include <stdio.h>
#include <cstring>

// --General Util

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


// CANBUS function

//Socket can style frame format
struct _can_frame {
  uint32_t can_id;
  uint16_t can_dlc = 8;
  // uint8_t data[8] __attribute__((aligned(8)));
  uint8_t data[8];

  // Constructor function , to reset and initialize ID to nothing , and data frame to all 0
  _can_frame() 
        : can_id(0), can_dlc(8) {
        memset(data, 0, sizeof(data));
        }
};

// For ID custom protocol
struct CANIDDecoded {
    uint8_t PRIORITY;
    uint8_t BASE_ID;
    uint8_t MSG_NUM;
    uint8_t SRC;
    uint8_t DEST;
};


// Creating CAN ID :: Transmitter side
uint32_t createExtendedCANID(uint8_t PRIORITY, uint8_t BASE_ID, uint8_t MSG_NUM ,uint8_t SRC_ADDRESS, uint8_t DEST_ADDRESS);
// Structure of CAN ID :: Receiver side
void decodeExtendedCANID(struct CANIDDecoded* CANIDDecoded ,uint32_t canID);




uint16_t toUint16FromBitarrayMSB(const bool *bitarr);
uint16_t toUint16FromBitarrayLSB(const bool *bitarr);


// --Shutdown mechanism & BMS specific Util

