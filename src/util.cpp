#include <util.h>

/*------------------------------ General Functions ----*/
// Split uint16_t to High byte and Low byte
unsigned char* splitHLbyte(unsigned int num){
  static uint8_t temp[2]; // initialize
  temp[0] = (num >> 8) & 255;  // Extract the high byte
  temp[1] = num & 255;         // Extract the low byte
  return temp;
}

// Merge 2 bytes into uint16_t
unsigned int mergeHLbyte(unsigned char Hbyte, unsigned char Lbyte){
  // ** May pass reference , think later
  uint16_t temp = (Hbyte << 8) | Lbyte; // bitshiftLeft by 8 OR with the lower byte, then put to 2 different variable
  return temp;
}

// Convert float or uint16 into byte arrays
unsigned char* Encode_bytearray(float f) { 
    static uint8_t c[sizeof(f)]; 
    memcpy(c, &f, sizeof(f));
    // Copy to address of array , Copy from address of float , size of float: Now, c[0] to c[3] contain the bytes of the float
    return c; 
}

// Convert byte arrays into float or uint16
float Decode_bytearray(unsigned char* c) {
    float f;
    // Use memcpy to copy the bytes from the array back into the float
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


/*------------------------------ CAN comminication Functions ----
-----(Check the spreadsheet in README.md for CAN ID custom rules)*/
// 1 CE 1 0A 00
// 0001 1100 1110 0001 0000 1010 0000 0000

// Create Extended CAN ID of Priority , BaseID , msg Number , src , dest 
uint32_t createExtendedCANID(uint8_t PRIORITY, uint8_t BASE_ID,uint8_t SRC_ADDRESS, uint8_t DEST_ADDRESS, uint8_t MSG_NUM ) {
    uint32_t canID = 0;

    canID |= (PRIORITY & 0xFF) << 24;        // (PP)Priority (bits 24-29)
    canID |= (BASE_ID & 0x0F) << 20;         // (B) Base id denotes CAN channel (bit 20-24)
    canID |= (SRC_ADDRESS & 0xFF) << 12;     // (SS)Source BMU address (bits 9-16)
    canID |= (DEST_ADDRESS & 0xFF) << 4;     // (DD)Destination BCU address (bits 8-15)
    canID |= MSG_NUM;                        // (X) Message number (bits 0-7)
    return canID;
}

// Decode extended CAN ID
void decodeExtendedCANID(struct CANIDDecoded *myCAN ,uint32_t canID) {

    myCAN->PRIORITY = (canID >> 24) & 0xFF;        // Extract priority (bits 24-29)
    myCAN->BASE_ID = (canID >> 20) & 0x0F;         // Extract Base ID (bits 20-23)
    myCAN->SRC = (canID >> 12) & 0xFF;             // Extract Source Address (bits 12-19)
    myCAN->DEST = (canID >> 4) & 0xFF;             // Extract destination address (bits 4-11)
    myCAN->MSG_NUM = canID & 0x0F;                 // Extract Message Number (bits 0-3)
    
}


// C++ program to illustrate how to implement a circular
// buffer using std::vector
// #include <stdexcept>
// #include <vector>

// class CircularBuffer {
// private:
//     vector<int> buffer;
//     int head;
//     int tail;
//     int capacity;

// public:
//     // Constructor to intialize circular buffer's data
//     // members
//     CircularBuffer(int capacity)
//     {
//         this->capacity = capacity;
//         this->head = 0;
//         this->tail = 0;
//         buffer.resize(capacity);
//     }

//     // Function to add an element to the buffer
//     void push_back(int element)
//     {
//         buffer[head] = element;
//         head = (head + 1) % capacity;
//         if (head == tail) {
//             tail = (tail + 1) % capacity;
//         }
//     }

//     // Function to remove an element from the buffer
//     void pop_front()
//     {
//         if (empty()) {
//             throw out_of_range("Buffer is empty");
//         }
//         tail = (tail + 1) % capacity;
//     }

//     // Function to check if the buffer is empty
//     bool empty() const { return head == tail; }

//     // Function to check if the buffer is full
//     bool full() const
//     {
//         return (head + 1) % capacity == tail;
//     }

//     // Function to get the size of the buffer
//     int size() const
//     {
//         if (head >= tail) {
//             return head - tail;
//         }
//         return capacity - (tail - head);
//     }

//     // Function to print the elements of the buffer
//     void printBuffer() const
//     {
//         int idx = tail;
//         while (idx != head) {
//             printf("%d",buffer[idx]); printf("%s"," ");
//             idx = (idx + 1) % capacity;
//         }
//         printf("\n");
//     }
// };

