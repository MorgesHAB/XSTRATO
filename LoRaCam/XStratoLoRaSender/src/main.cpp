// Librairies
#include <Arduino.h>
// LoRa
#include <SPI.h>
#include <LoRa.h>

// WS2812B LED
#include <Adafruit_NeoPixel.h>

#include <TeleFile.h>
#include <Capsule.h>

#include "Camera.h"
#include "Config.h"

#include <FS.h>
#include <SD.h>
#include <SD_MMC.h> // with CLK, CMD; D0-3
#include <LoopbackStream.h>

#define DEBUG true

#define SD_CMD                  6
#define SD_CLK                  7
#define SD_D0                   15
#define SD_D1                   16
#define SD_D2                   4
#define SD_D3                   5

#define LORA_FREQ   868.0E6
#define LORA_POWER  2
#define LORA_BW     500.0e3
#define LORA_SF     7
#define LORA_CR     5
#define LORA_PREAMBLE_LEN 8
#define LORA_SYNC_WORD    0x12
#define LORA_CRC          true
#define LORA_CURRENT_LIMIT 120

// PIN/GPIO Definition
#define LORA_SCK                9
#define LORA_MOSI               10
#define LORA_MISO               11
#define LORA_CS                 46
#define LORA_INT0               12
#define LORA_RST                -1

#define NEOPIXEL_PIN            21
#define GREEN_LED_PIN           14

#define LED_ID                  0x99

// XSTRATO supports only USBSerial
#define SERIAL_TO_PC                 USBSerial

char filename[50];

uint32_t colors[] = {
    0x000000,
    0x32A8A0,
    0x0000FF,
    0xFFEA00,
    0x00FF00,
    0xFF0000,
    0xCF067C,
    0xFF0800
};


void startTransmission(framesize_t framesize = FRAMESIZE_96X96, int jpeg_quality = 12);
void updateTransmission();

// The fragment size is the size in bytes of each fragment of the file that will be sent using teleFile. 
// The coding rate is the degree of redundancy used to encode the file.
#define FRAGMENT_SIZE   100
#define CODING_RATE     1.5

void handlePacketLoRa(int packetSize);
void handlePacketDevice1(byte, byte [], unsigned);
void handleFileTransfer1(byte dataIn[], unsigned dataSize);

CapsuleStatic device1(handlePacketDevice1);
TeleFile fileTransfer1(FRAGMENT_SIZE,CODING_RATE,handleFileTransfer1);
Adafruit_NeoPixel led(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800); // 1 led
LoopbackStream LoRaRxBuffer(1024);

uint8_t *output;

void setup() {
    SERIAL_TO_PC.begin(115200);
    SERIAL_TO_PC.setTxTimeoutMs(0);

    pinMode(GREEN_LED_PIN, OUTPUT);
    led.begin();

    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); 
    LoRa.setPins(LORA_CS, LORA_RST, LORA_INT0);
    LoRa.setSPI(SPI);
    
    if (!LoRa.begin(LORA_FREQ)) {
      if (DEBUG) {
        SERIAL_TO_PC.println("Starting LoRa failed!");
      }
    }
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

    SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0, SD_D1, SD_D2, SD_D3);

    // 20000 is the max operational speed ?
    if (!SD_MMC.begin("/sdcard", false, true, 20000)) {  
      if (DEBUG) {
        SERIAL_TO_PC.println("Card Mount Failed..");
      }
    }
    if (DEBUG) {
      SERIAL_TO_PC.println("SD Card Mounted..");
    }
}

void loop() {
  // Sending a new picture if more than 15 seconds have passed since the last transmission.
  if ((millis() - fileTransfer1.getLastEndTime()) > 5000 and fileTransfer1.isTransmissionOver()) {
      // Will encode the file.
      // startTransmission(FRAMESIZE_HD, 12);
      startTransmission(FRAMESIZE_SVGA, 20);
  } else if (!fileTransfer1.isTransmissionOver()) {
      // Will send fragments one by one. There can be delay between two fragments if wanted.
      // This function is blocking for the time of one LoRa packet transmission.
      // However the code can be adapted to not be blocking for the time of one LoRa transmission
      // What you have to do is wait enough before the next time you call updateTransmission()
      // and to use LoRa.endPacket(true).
      updateTransmission();
  }

  while (LoRaRxBuffer.available()) {
    device1.decode(LoRaRxBuffer.read());
  }
}

void updateTransmission() {
  const unsigned fragmentCutSize = fileTransfer1.getFragmentSize();
  const unsigned numberOfCodedFragments = fileTransfer1.getNumberOfCodedFragments();
  const unsigned numberOfUncodedFragments = fileTransfer1.getNumberOfUncodedFragments();

  // We need to send each fragment one by one, because this will take some time, this is not integrated
  // in the teleFile class, this is something the user has to do himself. But it's quite easy. Cut the fragment 
  // to lenght and send it. 
  unsigned currentPacketNumber = fileTransfer1.getLastPacketSent()+1;

  if (currentPacketNumber<=numberOfCodedFragments) {
    if (DEBUG) {
      SERIAL_TO_PC.println("Packet n° " + String(currentPacketNumber)+"/"+String(numberOfCodedFragments));
    }
    unsigned fragmentLen = fragmentCutSize+4;
    byte packetData[fragmentLen];
    byte packetId = 0x00;

    packetData[0] = currentPacketNumber >> 8;
    packetData[1] = currentPacketNumber & 0xFF;
    // The two next data bytes are the total number of packets (numberOfCodedFragments) as a 16 bit integer
    packetData[2] = numberOfUncodedFragments >> 8;
    packetData[3] = numberOfUncodedFragments & 0xFF;

    for (unsigned j = 0; j < fragmentCutSize; j++) {
      packetData[j+4] = output[(currentPacketNumber-1)*fragmentCutSize+j];
    }

    digitalWrite(GREEN_LED_PIN, HIGH);
    {
      byte* packetToSend = device1.encode(packetId,packetData,fragmentLen);
      LoRa.beginPacket();
      LoRa.write(packetToSend, device1.getCodedLen(fragmentLen));
      LoRa.endPacket();
      LoRa.receive();

      delete[] packetToSend;
      fileTransfer1.setLastPacketSent(currentPacketNumber);
    }
    digitalWrite(GREEN_LED_PIN, LOW);
  }
  else {
    if (DEBUG) {
      SERIAL_TO_PC.println("Transmission over..");
    }
    fileTransfer1.endTransmission();
    delete[] output;
  }
}

void startTransmission(framesize_t frameSize, int jpegQuality) {

  fileTransfer1.setTransmissionOver(false);

  if (!setupCam(frameSize, jpegQuality)) {
    if (DEBUG) {
      SERIAL_TO_PC.println("Camera init failed");
    }
    return;
  }

  sensor_t* s = esp_camera_sensor_get();

  s->set_framesize(s, FRAMESIZE_SVGA);  
  s->set_quality(s, jpegQuality);
  //s->set_brightness(s, 0);     // -2 to 2
  //s->set_contrast(s, 0);       // -2 to 2
  //s->set_saturation(s, 0);     // -2 to 2
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 1);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  
  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  s->set_aec_value(s, 300);    // 0 to 1200
  s->set_aec2(s, 1);           // 0 = disable , 1 = enable
  //s->set_lenc(s, 0);           // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  
  // capture a frame
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    if (DEBUG) {
      SERIAL_TO_PC.println("Error with frame buffer pointer");
    }
    return;
  }

  if (DEBUG) {
    SERIAL_TO_PC.println("Size: " + String(fb->len) + " Bytes");
  }
  // display_image(fb->width, fb->height, fb->pixformat, fb->buf, fb->len); 

  const unsigned imageTrueSize = fb->len;
  const unsigned imageBufferSize = fileTransfer1.computeUncodedSize(imageTrueSize);

  uint8_t *dataArray;
  dataArray = new uint8_t[imageBufferSize];

  memcpy(dataArray, fb->buf, imageTrueSize);

  esp_camera_fb_return(fb);
  esp_camera_deinit();
  turnOffCam();

  const unsigned outputLen = fileTransfer1.computeCodedSize(imageTrueSize);
  output = new byte[outputLen];

  fileTransfer1.encode(dataArray, imageTrueSize, output);
  delete[] dataArray;
}

void handlePacketLoRa(int packetSize) {
  for (int i = 0; i < packetSize; i++) {
    LoRaRxBuffer.write(LoRa.read());
  }
}

void handlePacketDevice1(byte packetId, byte *packetData, unsigned len) {
  // Nothing required for the sender
  switch (packetId) {
    case LED_ID:
      // Parse packetData in a uint32_t
      uint32_t ledColor = 0;
      for (unsigned i = 0; i < len; i++) {
        ledColor = ledColor << 8;
        ledColor = ledColor | packetData[i];
      }
      SERIAL_TO_PC.println("LED color: " + String(ledColor, HEX));
      led.fill(ledColor);
      led.show();
    break;
  }
}

void handleFileTransfer1(byte dataIn[], unsigned dataSize) {
    // Nothing required for the sender
}
