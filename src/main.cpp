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
#include "GPS.h"
#include "../XRF_interface/PacketDefinition.h"

#include <FS.h>
#include <SD.h>
#include <SD_MMC.h> // with CLK, CMD; D0-3
#include <LoopbackStream.h>

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

void sendImagePacket();
void sendPositionPacket();
void startTransmission();
void LoRaSendPacketLR(uint8_t *packetData, unsigned len);
void updateTransmission();

// These are the parameters that will be used to send stuff from ground to the balloon
RFsettingsPacket LoRaSettings;
CameraSettingsPacket CameraSettings;
CameraSettingsPacket CameraSettingsDefault;

void handlePacketLoRa(int packetSize);
void handlePacketDevice1(byte, byte [], unsigned);
void handlePacketDevice2(byte, byte [], unsigned);
void handleFileTransfer1(byte dataIn[], unsigned dataSize);

//SPIClass SDSPI(HSPI);
CapsuleStatic device1(handlePacketDevice1);
CapsuleStatic device2(handlePacketDevice2);
TeleFile fileTransfer1(FRAGMENT_SIZE,CODING_RATE,handleFileTransfer1);
Adafruit_NeoPixel led(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800); // 1 led
LoopbackStream LoRaRxBuffer(1024);
TinyGPSPlus gps;

TinyGPSCustom fixType(gps, "GNGSA", 2);
unsigned char serial2bufferRead[1000];

uint8_t *output;

void setup() {
    LoRaSettings.BW = LORA_LR_BW_DEFAULT;
    LoRaSettings.CR = LORA_LR_CR_DEFAULT;
    LoRaSettings.SF = LORA_LR_SF_DEFAULT;

    CameraSettingsDefault.framesize = CAM_FRAMESIZE_DEFAULT;
    CameraSettingsDefault.quality = CAM_QUALITY_DEFAULT;
    CameraSettingsDefault.whiteBalanceEnable = CAM_WB_DEFAULT;
    CameraSettingsDefault.awbGainEnable = CAM_AWB_GAIN_DEFAULT;
    CameraSettingsDefault.wbMode = CAM_WB_MODE_DEFAULT;
    CameraSettingsDefault.exposureEnable = CAM_EXPOSURE_DEFAULT;
    CameraSettingsDefault.exposureValue = CAM_EXPOSURE_VALUE_DEFAULT;
    CameraSettingsDefault.aec2Enable = CAM_AEC2_DEFAULT;
    CameraSettingsDefault.rawGmaEnable = CAM_RAW_GMA_DEFAULT;

    CameraSettings = CameraSettingsDefault;

    SERIAL_TO_PC.begin(SERIAL_TO_PC_BAUD);
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


    if (SENDER) {
      LoRa.setSpreadingFactor(LORA_SF);
      LoRa.setSignalBandwidth(LORA_BW);
      LoRa.setCodingRate4(LORA_CR);
    }
    else {
      LoRa.setCodingRate4(LoRaSettings.CR);
      LoRa.setSpreadingFactor(LoRaSettings.SF);
      LoRa.setSignalBandwidth(LoRaSettings.BW);
    }

    //LoRa.setPreambleLength(LORA_PREAMBLE_LEN);
    //LoRa.setSyncWord(LORA_SYNC_WORD);
    //LoRa.enableCrc();

    LoRa.setTxPower(LORA_POWER);
    //LoRa.setOCP(LORA_CURRENT_LIMIT);
    LoRa.onReceive(handlePacketLoRa);
    LoRa.receive();

    //SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0, SD_D1, SD_D2, SD_D3);

    // 20000 is the max operational speed ?
    //if (!SD_MMC.begin("/sdcard", false, true, 20000)) {  
    //  if (DEBUG) {
    //    SERIAL_TO_PC.println("Card Mount Failed..");
    //  }
    //}
    //if (DEBUG) {
    //  SERIAL_TO_PC.println("SD Card Mounted..");
    //}

  if (SEND_POSITION_PACKET) {
    GPS_PORT.begin(9600, 134217756U, GPS_RX, GPS_TX); // This for cmdIn
    gpsSetup(57600, GPS_RATE, 2, 1, 0); // baud, Hz, mode, nmea, cog filter (0 = Off, 1 = On)
  }
}

void loop() {
  while (LoRaRxBuffer.available()) {
    device1.decode(LoRaRxBuffer.read());
  }

  while (SERIAL_TO_PC.available()) {
    device2.decode(SERIAL_TO_PC.read());
  } 

  if (SEND_POSITION_PACKET) {
    while (GPS_PORT.available()) { 
      gps.encode(GPS_PORT.read()); 
    }
  }

  if (SEND_IMAGE_PACKET) {
    sendImagePacket();
  }
   
  if (SEND_POSITION_PACKET) {
    sendPositionPacket();
  }
}

void handlePacketLoRa(int packetSize) {
  for (int i = 0; i < packetSize; i++) {
    LoRaRxBuffer.write(LoRa.read());
  }
}


void handlePacketDevice1(byte packetId, byte *packetData, unsigned len) {
  // Nothing required for the sender
  switch (packetId) {
    case CAPSULE_ID::IMAGE_LORA:
      {
        TeleFileImgInfo imgInfo;
        memcpy(&imgInfo, packetData, TeleFileImgInfoSize);
        uint16_t fragmentNumberTemp = imgInfo.currentPacketNumber;
        static uint16_t fragmentCounter = 0;

        if (fragmentNumberTemp<fragmentCounter) {
          fragmentCounter = 1;
        }
        fragmentCounter++;
        if (DEBUG) {
          SERIAL_TO_PC.print("Received fragment number: "); SERIAL_TO_PC.println(fragmentNumberTemp); 
        }
        fileTransfer1.decode(packetData, len);
  
        Xstrato_img_info imgToPcInfo;
        imgToPcInfo.nbr_rx_packet = fragmentCounter;
        imgToPcInfo.nbr_tot_packet = imgInfo.numberOfUncodedFragments;

        uint8_t *packetData = new uint8_t[Xstrato_img_info_size];
        memcpy(packetData, &imgToPcInfo, Xstrato_img_info_size);
        
        uint8_t* packetToSend = device1.encode(CAPSULE_ID::IMAGE_DATA,packetData,Xstrato_img_info_size);
        SERIAL_TO_PC.write(packetToSend,device1.getCodedLen(Xstrato_img_info_size));
        delete[] packetToSend;
        delete[] packetData;
      }
    break;
    case CAPSULE_ID::TELEMETRY:
      if (DEBUG) {
        SERIAL_TO_PC.println("Received Telemetry Packet ");
      }
      {
      size_t codedSize = device1.getCodedLen(len);
      byte* codedBuffer = new byte[codedSize];
      codedBuffer = device1.encode(packetId, packetData, len);

      SERIAL_TO_PC.write(codedBuffer, codedSize);
      delete[] codedBuffer;
      }
    break;
    case CAPSULE_ID::LED:
    {
      // Parse packetData in a uint32_t
      uint32_t ledColor = 0;
      for (unsigned i = 0; i < len; i++) {
        ledColor = ledColor << 8;
        ledColor = ledColor | packetData[i];
      }
      if (DEBUG) {
        SERIAL_TO_PC.println("LED color: " + String(ledColor, HEX));
      }
      led.fill(ledColor);
      led.show();
    }
    break;
    case CAPSULE_ID::RF_PARAM:
    {
      memcpy(&LoRaSettings, packetData, RFsettingsPacket_size);
      if (DEBUG) {
        SERIAL_TO_PC.println("LoRa Settings: SF: " + String(LoRaSettings.SF) + " BW: " + String(LoRaSettings.BW) + " CR: " + String(LoRaSettings.CR));
      }
    }
    break;
    case CAPSULE_ID::CAM_PARAM:
    {
      memcpy(&CameraSettings, packetData, CameraSettingsPacketSize);
      if (DEBUG) {
        String toPrint = "";
        toPrint += "Camera Settings: ";
        toPrint += "Frame Size: " + String(CameraSettings.framesize) + " ";
        toPrint += "Quality: " + String(CameraSettings.quality) + " ";
        toPrint += "White Balance Enable: " + String(CameraSettings.whiteBalanceEnable) + " ";
        toPrint += "AWB Gain Enable: " + String(CameraSettings.awbGainEnable) + " ";
        toPrint += "WB Mode: " + String(CameraSettings.wbMode) + " ";
        toPrint += "Exposure Enable: " + String(CameraSettings.exposureEnable) + " ";
        toPrint += "Exposure Value: " + String(CameraSettings.exposureValue) + " ";
        toPrint += "AEC2 Enable: " + String(CameraSettings.aec2Enable) + " ";
        toPrint += "Raw Gma Enable: " + String(CameraSettings.rawGmaEnable) + " ";
        SERIAL_TO_PC.println(toPrint);
      }
    }
    break;
  }
}

void handlePacketDevice2(byte packetId, byte dataIn[], unsigned len) {
  switch (packetId) {
    case CAPSULE_ID::LED:
      if (DEBUG) {
        SERIAL_TO_PC.println("Received Command Packet ");
      }
      {
        size_t codedSize = device2.getCodedLen(len);
        byte* codedBuffer = new byte[codedSize];
        codedBuffer = device2.encode(packetId, dataIn, len);

        LoRaSendPacketLR(codedBuffer, codedSize);

        //LoRa.beginPacket();
        //LoRa.write(codedBuffer, codedSize);
        //LoRa.endPacket();

        LoRa.receive();

        delete[] codedBuffer;
      }
    break;
    case CAPSULE_ID::RF_PARAM: 
    {
      // get data from struct
      memcpy(&LoRaSettings, dataIn, RFsettingsPacket_size);

      size_t codedSize = device2.getCodedLen(len);
      byte* codedBuffer = new byte[codedSize];
      codedBuffer = device2.encode(packetId, dataIn, len);

      LoRaSendPacketLR(codedBuffer, codedSize);

      LoRa.receive();
      delete[] codedBuffer;
    }
    break;
    case CAPSULE_ID::CAM_PARAM: 
    {
      // get data from struct
      memcpy(&CameraSettings, dataIn, CameraSettingsPacketSize);

      size_t codedSize = device2.getCodedLen(len);
      byte* codedBuffer = new byte[codedSize];
      codedBuffer = device2.encode(packetId, dataIn, len);

      LoRaSendPacketLR(codedBuffer, codedSize);

      LoRa.receive();
      delete[] codedBuffer;
    }
    break;
    default:
    break;
  }
}

void handleFileTransfer1(byte dataIn[], unsigned dataSize) {
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
      packetData[j] = dataIn[i*FRAGMENT_SIZE_PC + j];
    }
    uint8_t* packetToSend = device1.encode(packetId,packetData,len);
    SERIAL_TO_PC.write(packetToSend,device1.getCodedLen(len));
    delay(10);
    delete[] packetToSend;
    delete[] packetData;
  }
}

void sendImagePacket() {
  // Sending a new picture if more than 15 seconds have passed since the last transmission.
    if ((millis() - fileTransfer1.getLastEndTime()) > 5000 and fileTransfer1.isTransmissionOver()) {
        // Will encode the file.
        // startTransmission(FRAMESIZE_HD, 12);
        startTransmission();
    } else if (!fileTransfer1.isTransmissionOver()) {
        // Will send fragments one by one. There can be delay between two fragments if wanted.
        // This function is blocking for the time of one LoRa packet transmission.
        // However the code can be adapted to not be blocking for the time of one LoRa transmission
        // What you have to do is wait enough before the next time you call updateTransmission()
        // and to use LoRa.endPacket(true).
        updateTransmission();
    }
}

void startTransmission() {

  fileTransfer1.setTransmissionOver(false);

  if (!setupCam(framesize_t(CameraSettings.framesize), CameraSettings.quality)) {
    if (DEBUG) {
      SERIAL_TO_PC.println("Camera init failed");
    }
    return;
  }

  sensor_t* s = esp_camera_sensor_get();

  s->set_framesize(s, framesize_t(CameraSettings.framesize));  
  s->set_quality(s, CameraSettings.quality);
  //s->set_brightness(s, 0);     // -2 to 2
  //s->set_contrast(s, 0);       // -2 to 2
  //s->set_saturation(s, 0);     // -2 to 2
  s->set_whitebal(s, CameraSettings.whiteBalanceEnable);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, CameraSettings.awbGainEnable);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, CameraSettings.wbMode);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  
  s->set_exposure_ctrl(s, CameraSettings.exposureEnable);  // 0 = disable , 1 = enable
  s->set_aec_value(s, CameraSettings.exposureValue);    // 0 to 1200
  s->set_aec2(s, CameraSettings.aec2Enable);           // 0 = disable , 1 = enable
  //s->set_lenc(s, 0);           // 0 = disable , 1 = enable
  s->set_raw_gma(s, CameraSettings.rawGmaEnable);        // 0 = disable , 1 = enable
  
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

  if (imageTrueSize < MAX_IMAGE_SIZE) {
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
  else {
    if (DEBUG) {
      SERIAL_TO_PC.println("Image too big");
    }
    esp_camera_fb_return(fb);
    esp_camera_deinit();
    turnOffCam();
    CameraSettings = CameraSettingsDefault;
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

    TeleFileImgInfo imgInfo;
    imgInfo.currentPacketNumber = currentPacketNumber;
    imgInfo.numberOfUncodedFragments = numberOfUncodedFragments;

    memcpy(packetData, &imgInfo, TeleFileImgInfoSize);

    for (unsigned j = 0; j < fragmentCutSize; j++) {
      packetData[j+4] = output[(currentPacketNumber-1)*fragmentCutSize+j];
    }

    digitalWrite(GREEN_LED_PIN, HIGH);
    {
      byte* packetToSend = device1.encode(CAPSULE_ID::IMAGE_LORA,packetData,fragmentLen);

      LoRaSendPacketLR(packetToSend, device1.getCodedLen(fragmentLen));
      
      //LoRa.beginPacket();
      //LoRa.write(packetToSend, device1.getCodedLen(fragmentLen));
      //LoRa.endPacket();
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

void sendPositionPacket() {
  static unsigned long lastPositionSent = 0;
  if (millis()-lastPositionSent > (1000.0/POSITION_PACKET_RATE) and gps.location.isUpdated()) {
    lastPositionSent = millis();
    // Send LoRa Packet using capsule 
    uint8_t *packetData; 
    uint8_t *packetToSend;
    packetData = new uint8_t[12];
    packetToSend = new uint8_t[device1.getCodedLen(12)];
    PositionPacket position;
    position.lat = gps.location.lat();
    position.lon = gps.location.lng();
    position.alt = gps.altitude.meters();
    memcpy(packetData, &position, sizeof(position));
    packetToSend = device1.encode(CAPSULE_ID::TELEMETRY, packetData, sizeof(position));
    
    LoRaSendPacketLR(packetToSend, device1.getCodedLen(12));
    
    //LoRa.beginPacket();
    //LoRa.write(packetToSend, device1.getCodedLen(12));
    //LoRa.endPacket();

    LoRa.receive();
    delete[] packetData;
    delete[] packetToSend;
  }
}

void LoRaSendPacketLR(uint8_t *packetData, unsigned len) {
  if (SENDER) {
    LoRa.setCodingRate4(LoRaSettings.CR);
    LoRa.setSpreadingFactor(LoRaSettings.SF);
    LoRa.setSignalBandwidth(LoRaSettings.BW);

    LoRa.beginPacket();
    LoRa.write(packetData, len);
    LoRa.endPacket();

    LoRa.setCodingRate4(LORA_CR);
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
  }
  else {
    LoRa.setCodingRate4(LORA_CR);
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    
    LoRa.beginPacket();
    LoRa.write(packetData, len);
    LoRa.endPacket();

    LoRa.setCodingRate4(LoRaSettings.CR);
    LoRa.setSpreadingFactor(LoRaSettings.SF);
    LoRa.setSignalBandwidth(LoRaSettings.BW);
  }
}

