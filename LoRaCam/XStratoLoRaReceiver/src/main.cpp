#include <Arduino.h>
#include <LoRa.h>
#include <TeleFile.h>
#include <Capsule.h>
#include <LoopbackStream.h>
#include "../../../test.h"

#define DEBUG true

#define SERIAL_TO_PC      USBSerial
#define SERIAL_TO_PC_BAUD 115200

#define LORA_FREQ   868.0e6
#define LORA_POWER  2
#define LORA_BW     500.0e3
#define LORA_SF     7
#define LORA_CR     5
#define LORA_PREAMBLE_LEN 8
#define LORA_SYNC_WORD    0x12
#define LORA_CRC          true
#define LORA_CURRENT_LIMIT 120

#define LORA_SCK                9
#define LORA_MOSI               10
#define LORA_MISO               11
#define LORA_CS                 46
#define LORA_INT0               12
#define LORA_RST                -1

#define BOARD_LED 14

#define FRAGMENT_SIZE_PC 200
#define IMAGE_START     0x42
#define IMAGE_MIDDLE    0x53
#define IMAGE_END       0x64
#define TRANSMISSION_DATA 0x12
#define LED_ID          0x99

// The fragment size is the size in bytes of each fragment of the file that will be sent using teleFile. 
// The coding rate is the degree of redundancy used to encode the file.
#define FRAGMENT_SIZE   100
#define CODING_RATE     1.2

void handlePacketLoRa(int packetSize);
void handlePacketDevice1(byte, byte [], unsigned);
void handlePacketDevice2(byte, byte [], unsigned);
void handleFileTransfer1(byte dataIn[], unsigned dataSize);

//SPIClass SDSPI(HSPI);
CapsuleStatic device1(handlePacketDevice1);
CapsuleStatic device2(handlePacketDevice2);
TeleFile fileTransfer1(FRAGMENT_SIZE,CODING_RATE,handleFileTransfer1);
LoopbackStream LoRaRxBuffer(1024);

void setup() {
    SERIAL_TO_PC.begin(SERIAL_TO_PC_BAUD);
    SERIAL_TO_PC.setTxTimeoutMs(0);
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);
    pinMode(BOARD_LED, OUTPUT);

    if (DEBUG) {
      SERIAL_TO_PC.println("LoRa Receiver.. initialising");
    }

    LoRa.setPins(LORA_CS, LORA_RST, LORA_INT0);
    LoRa.begin(LORA_FREQ);
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setCodingRate4(LORA_CR);
    //LoRa.setPreambleLength(LORA_PREAMBLE_LEN);
    //LoRa.setSyncWord(LORA_SYNC_WORD);
    //LoRa.enableCrc();
    LoRa.setTxPower(LORA_POWER);
    //LoRa.setOCP(LORA_CURRENT_LIMIT);
    LoRa.onReceive(handlePacketLoRa);
    LoRa.receive();
}

void loop() {
  while (LoRaRxBuffer.available()) {
    device1.decode(LoRaRxBuffer.read());
  }

  while (SERIAL_TO_PC.available()) {
    device2.decode(SERIAL_TO_PC.read());
  } 
}

void handlePacketLoRa(int packetSize) {
  for (int i = 0; i < packetSize; i++) {
    LoRaRxBuffer.write(LoRa.read());
  }
}
// This function is called everytime a packet has been received by the Capsule class.
// Every packet with prefix 0x00 is a fragment of the file, we pass this fragment to teleFile class for decoding. 
void handlePacketDevice1(byte packetId, byte dataIn[], unsigned len) {
  switch (packetId) {
    case 0x00:
      {
        if (DEBUG) {
          SERIAL_TO_PC.print("Received fragment number: "); SERIAL_TO_PC.println(dataIn[0] << 8 | dataIn[1]); 
        }
        uint16_t fragmentNumberTemp = dataIn[0] << 8 | dataIn[1];
        static uint16_t fragmentCounter = 0;

        if (fragmentNumberTemp<fragmentCounter) {
          fragmentCounter = 1;
        }

        fragmentCounter++;

        fileTransfer1.decode(dataIn, len);
        uint8_t packetId = TRANSMISSION_DATA;
        uint8_t *packetData = new uint8_t[4];
        uint8_t len = 4;
        
        packetData[0] = fragmentCounter & 0xFF;
        packetData[1] = fragmentCounter >> 8;
        packetData[2] = dataIn[3];
        packetData[3] = dataIn[2];

        uint8_t* packetToSend = device1.encode(packetId,packetData,len);
        SERIAL_TO_PC.write(packetToSend,device1.getCodedLen(len));
        delete[] packetToSend;
        delete[] packetData;
      }
    break;
    case 0x01:
      if (DEBUG) {
        SERIAL_TO_PC.println("Received Telemetry Packet ");
      }
    break;
    default:
    break;
  }
}


// This function is called everytime a full file has been received by teleFile class. 
void handleFileTransfer1(byte data[], unsigned dataSize) {
  // Do something with the file
  if (DEBUG) {
    SERIAL_TO_PC.print("Received file of size "); SERIAL_TO_PC.println(dataSize);
  }

  size_t imgL = dataSize;
  double numberOfFragmentsRaw = double(imgL) / double(FRAGMENT_SIZE_PC);
  int numberOfFragmentsReal = imgL / FRAGMENT_SIZE_PC;

  if (numberOfFragmentsRaw > numberOfFragmentsReal) {
    numberOfFragmentsReal++;
  }

  for (int i = 0; i < numberOfFragmentsReal; i++) {
    uint8_t packetId;
    // First fragment
    if (i == 0) {
      packetId = IMAGE_START;
    }
    // Data fragment
    else if (i == numberOfFragmentsReal - 1) {
      packetId = IMAGE_END;
    }
    // End fragment
    else {
      packetId = IMAGE_MIDDLE;
    }
    uint8_t* packetData = new uint8_t[FRAGMENT_SIZE_PC];
    uint8_t len = FRAGMENT_SIZE_PC;
    for (int j = 0; j < FRAGMENT_SIZE_PC; j++) {
      if (i*FRAGMENT_SIZE_PC + j >= imgL) {
        len = j;
        break;
      }
      packetData[j] = data[i*FRAGMENT_SIZE_PC + j];
    }
    uint8_t* packetToSend = device1.encode(packetId,packetData,len);
    SERIAL_TO_PC.write(packetToSend,device1.getCodedLen(len));
    delay(10);
    delete[] packetToSend;
    delete[] packetData;
  }
}


void handlePacketDevice2(byte packetId, byte dataIn[], unsigned len) {
  switch (packetId) {
    case LED_ID:
      if (DEBUG) {
        Serial.println("Received Command Packet ");
      }
      {
        size_t codedSize = device2.getCodedLen(len);
        byte* codedBuffer = new byte[codedSize];
        codedBuffer = device2.encode(packetId, dataIn, len);

        LoRa.beginPacket();
        LoRa.write(codedBuffer, codedSize);
        LoRa.endPacket();
        LoRa.receive();

        delete[] codedBuffer;
      }
    break;
    default:
    break;
  }
}


