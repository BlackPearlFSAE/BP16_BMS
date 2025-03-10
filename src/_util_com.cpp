#include <_util_com.h>
#include <AMS.h>
#include <Arduino.h>

/* ==================================================== General Functions 
*/
// Split uint16_t to High byte and Low byte
unsigned char* splitHLbyte(unsigned int num){
  static uint8_t temp[2]; // initialize
  temp[0] = (num >> 8) & 255;  // Extract the high byte
  temp[1] = num & 255;         // Extract the low byte
  return temp;
}

// Merge 2 bytes into uint16_t
unsigned int mergeHLbyte(unsigned char Hbyte, unsigned char Lbyte){
  uint16_t temp = (Hbyte << 8) | Lbyte; // bitshiftLeft by 8 OR with low byte
  return temp;
}

// Convert float or uint16 into byte arrays
unsigned char* Encode_bytearray(float f) { 
    static uint8_t c[sizeof(f)]; 
    memcpy(c, &f, sizeof(f));
    return c; 
}

// Convert byte arrays into float or uint16
float Decode_bytearray(unsigned char* c) {
    float f;
    memcpy(&f, c, sizeof(f));
    return f;
}

// Convert Binary to Binary digit array
        // Split and Check bit from MSB -> LSB
        // 1st shift will shift to right by 7 position
                // 1 => 0b00000001 , then AND with 1 so anything that isn't one at 1st pos will be cut off
                // Ex. 42 = 0b00101010
                // 00101010 >> 0 = 00101010 & 00000001 = 0 (point at 1st bit 1 encounter from left . shift to right by 0 pos)
                // 00101010 >> 1 = 00010101 & 00000001 = 1
                // 00101010 >> 2 = 00001010 & 00000001 = 0
            // Check for bit 1 for immediate shutdown

// Convert N bit binary to 16 bit array from MSB-first (big endian) 
bool *toBitarrayMSB(uint16_t num){
  static bool bitarr[16]; // array to hold 8 binary number
  for (int i = 15; i >= 0; i--){
    bool bit = num & 1;
    bitarr[i] = bit;
    num >>= 1; // Right Shift num by 1 pos. before next loop , we AND with 1 again
  } 
  return bitarr; 
}

// Convert N bit binary to 16 bit array from LSB-first (little endian)
bool *toBitarrayLSB(uint16_t num){
  static bool bitarr[16]; // array to hold 8 binary number
  for (int i = 0; i < 16; i++){
    bool bit = num & 1;
    bitarr[i] = bit;
    num >>= 1; // Right Shift num by 1 pos. before next loop , we AND with 1 again
  } 
  return bitarr; 
}

// Convert MSB-first bit array back to uint16_t
uint16_t toUint16FromBitarrayMSB(const bool *bitarr) {
    uint16_t num = 0;
    for (int i = 0; i < 16; i++) {
        if (bitarr[i]) {
            num |= (1 << (15 - i));
        }
    }
    return num;
}

// Convert LSB-first bit array back to uint16_t
uint16_t toUint16FromBitarrayLSB(const bool *bitarr) {
    uint16_t num = 0;
    for (int i = 0; i < 16; i++) {
        if (bitarr[i]) {
            num |= (1 << i);
        }
    }
    return num;
}

/*=============================================================== CAN bus
-----( Check the spreadsheet in README.md for Communication Agreement )*/
// 1 CE 1 0A 00
// 0001 1100 1110 0001 0000 1010 0000 0000

uint16_t createCANID(uint8_t PRIORITY, uint8_t SRC_ADDRESS, uint8_t MSG_NUM) {
    uint16_t canID = 0;
    canID |= ((PRIORITY & 0x0F) << 8);        // (PP) Priority (bits 30–31)
    canID |= ((SRC_ADDRESS & 0x0F) << 4);      // (SS) Source address (bits 8–15)
    canID |= (MSG_NUM & 0x0F);            // (DD) Destination address (bits 0–7)
    return canID;
}

// Decode extended CAN ID
void decodeExtendedCANID(struct extCANIDDecoded *myCAN ,uint32_t canID) {

    myCAN->PRIORITY = (canID >> 24) & 0xFF;        // Extract priority (bits 24-29)
    myCAN->BASE_ID = (canID >> 20) & 0x0F;         // Extract Base ID (bits 20-23)
    myCAN->SRC = (canID >> 12) & 0xFF;             // Extract Source Address (bits 12-19)
    myCAN->DEST = (canID >> 4) & 0xFF;             // Extract destination address (bits 4-11)
    myCAN->MSG_NUM = canID & 0x0F;                 // Extract Message Number (bits 0-3)
    
}

void decodeStandardCANID(struct StandardCANIDDecoded *myCAN, uint32_t canID) {
    myCAN->PRIORITY = (canID >> 8) & 0x0F;        // Extract priority (bits 24-29)
    myCAN->SRC = (canID >> 4) & 0x0F;             // Extract destination address (bits 4-11)
    myCAN->MSG_NUM = canID & 0x0F;                 // Extract Message Number (bits 0-3)
}

// Preprocessor

/*========================================================== UART and USB
-----( Check the spreadsheet in README.md for Communication Agreement )*/

/*

// Use CAN Structure from AMS.h instead
// CAN Message Structure
struct CANMessage {
    uint32_t id;
    uint8_t data1;
    uint8_t data2;
    uint8_t data3; 
};

// Serialize message to byte array
void serialize(uint8_t* buffer, CANMessage *my_msg) {
  uint32_t id = my_msg->id;
  byte data1= my_msg->data1;
  byte data2= my_msg->data2;
  byte data3= my_msg->data3;

  // Copy memory block from id to buffer 1st block of memory by specified size
    memcpy(buffer, &id, sizeof(id)); // ID should have the size of 4 byte , so 1st four array block will be 
    memcpy(buffer + sizeof(id), &data1, sizeof(data1));
    memcpy(buffer + sizeof(id) + sizeof(data1), &data2, sizeof(data2));
    memcpy(buffer + sizeof(id) + sizeof(data1) + sizeof(data2), &data3, sizeof(data3));
}

// If BCU want to serialize its data to array
// (May be this should be declared in Struct method)
// But define elsewhere where

void serialize(uint8_t* buffer, BMUdata *my_msg) {
    uint32_t id = my_msg->bmu_id;
    byte data1 = my_msg->V_CELL;
    byte data2= my_msg->TEMP_SENSE;
    byte data3= my_msg->V_MODULE;
    byte data = my_msg->DV
  
    // Copy memory block from id to buffer 1st block of memory by specified size
      memcpy(buffer, &(my_msg->bmu_id), sizeof(id));
      for(){
        memcpy(buffer + sizeof(id))
      } 
      for(){
        memcpy(buffer + )
      }
      memcpy(buffer + sizeof(id), &(), sizeof(data1));
      memcpy(buffer + sizeof(id) + sizeof(data1), &data2, sizeof(data2));
      memcpy(buffer + sizeof(id) + sizeof(data1) + sizeof(data2), &data3, sizeof(data3));
  }

void serialize(uint8_t* buffer, AMSdata *my_msg) {
    byte data1= my_msg->data1;
    byte data2= my_msg->data2;
    byte data3= my_msg->data3;

    // Copy memory block from id to buffer 1st block of memory by specified size
        memcpy(buffer, &id, sizeof(id)); // ID should have the size of 4 byte , so 1st four array block will be 
        memcpy(buffer + sizeof(id), &data1, sizeof(data1));
        memcpy(buffer + sizeof(id) + sizeof(data1), &data2, sizeof(data2));
        memcpy(buffer + sizeof(id) + sizeof(data1) + sizeof(data2), &data3, sizeof(data3));
}


// // // Deserialize byte array to message (retrun static struct)
// // static CANMessage deserialize(const uint8_t* buffer) {
// //     CANMessage msg;
// //     memcpy(&msg.id, buffer, sizeof(msg.id));
// //     memcpy(&msg.data1, buffer + sizeof(msg.id), sizeof(msg.data1));
// //     memcpy(&msg.data2, buffer + sizeof(msg.id) + sizeof(msg.data1), sizeof(msg.data2));
// //     memcpy(&msg.data3, buffer + sizeof(msg.id) + sizeof(msg.data1) + sizeof(msg.data2), sizeof(msg.data3));
// //     return msg;
// // }

*/






