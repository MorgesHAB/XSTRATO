//////////////////////////////////////////////////////////////////////////
//  XSTRATO Stratospheric Balloon Mission 1
//
//  EPFL Rocket Team - Nordend Project 2023
//
//  Lionel Isoz
//  26.02.2023 
//////////////////////////////////////////////////////////////////////////

// Librairies
#include <Arduino.h>
// WiFi
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
// LoRa
#include <SPI.h>
#include <LoRa.h>
// BME680 Sensor
#include <Wire.h>
// #include <Adafruit_Sensor.h> // need to change sensor_t to sensor_tt (conflict with esp_cam sensor_t)
#include <Adafruit_BME680.h>
// GPS u-blox M8Q
#include <TinyGPSPlus.h>
// WS2812B LED
#include <Adafruit_NeoPixel.h>

#include "RadioPacket.h"
#include "LogSD.h"
#include "Camera.h"

///////////////////////////////////////////////////////////////////////////////////////
// PIN/GPIO Definition
#define LORA_SCK                9
#define LORA_MOSI               10
#define LORA_MISO               11
#define LORA_CS                 46
#define LORA_INT0               12
#define LORA_RST                -1

#define BME_I2C_SDA             17
#define BME_I2C_SCL             18

#define GPS_UART1_TX_PIN        3
#define GPS_UART1_RX_PIN        8

#define NEOPIXEL_PIN            21
#define GREEN_LED_PIN           14
#define BATTERY_MEASURE_PIN     13


// XSTRATO supports only USBSerial
#define SERIAL_                 USBSerial

#define LORA_BUFFER_SIZE        256

#define SEALEVELPRESSURE_HPA    (1013.25)

///////////////////////////////////////////////////////////////////////////////////////
// WiFi AP parameters
const char *ssid = "XSTRATO Wi-Fi";
const char *password = "bonjour123$";

// WiFi IP Address:   http://192.168.4.1/H
WiFiServer server(80); 

Adafruit_NeoPixel led(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800); // 1 led

Adafruit_BME680 bme; // I2C
TwoWire I2CBME = TwoWire(0);

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

uint8_t LoRa_tx_buf[LORA_BUFFER_SIZE];
uint8_t LoRa_rx_buf[LORA_BUFFER_SIZE];

void server_loop();
void BME_loop();
void LoRa_send_file(const char *path, uint8_t length = 250U);
void XSTRATO_GS_loop();
void XSTRATO_R2H_loop();
bool capture_img_and_save_in_SD(framesize_t framesize = FRAMESIZE_96X96, int jpeg_quality = 12);
void LoRa_handle_cmd();
void fill_BME_data(radio_packet_t* packet);
void print_packet(radio_packet_t packet);


void setup() {
    // GPS
    Serial1.begin(4800, 134217756U, GPS_UART1_RX_PIN,GPS_UART1_TX_PIN); // GPS
    // Serial1.begin(4800, 134217756U, 8, 3); // 3 8 also
    // Serial2.begin(9600, 134217756U, 46, 9);
    SERIAL_.begin(9600);
    pinMode(14, OUTPUT);
    pinMode(43, OUTPUT);

    SERIAL_.println("XSTRATO is alive !");

    led.begin();             // init NeoPixel
    led.setBrightness(10);  // not so bright, range available: [0, 255]
    led.fill(0x0000FF);
    led.show();

    pinMode(BATTERY_MEASURE_PIN, INPUT);

    SERIAL_.println("Led inited");

    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);  // SPI LoRa
    LoRa.setPins(LORA_CS, LORA_RST, LORA_INT0);
    LoRa.setSPI(SPI);
    if (!LoRa.begin(868E6)) {
        SERIAL_.println("Starting LoRa failed!");
    }
    LoRa.setSignalBandwidth(250E3);

    SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0, SD_D1, SD_D2, SD_D3);

    // BME Init
    I2CBME.begin(BME_I2C_SDA, BME_I2C_SCL, (uint32_t)100000);
    if (!bme.begin(0x76, &I2CBME)) {
        USBSerial.println("Could not find a valid BME680 sensor, check wiring!");
    } else {
        // Set up oversampling and filter initialization
        bme.setTemperatureOversampling(BME680_OS_8X);
        bme.setHumidityOversampling(BME680_OS_2X);
        bme.setPressureOversampling(BME680_OS_4X);
        bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme.setGasHeater(320, 150);  // 320*C for 150 ms
    }

    // 2nd XSTARTO work only until 20 MHz ? to checkif Xstrato1 can higher, -->No  20MHz max
    if (!SD_MMC.begin("/sdcard", false, true, 20000)) {  // work at 100 kHz & 10 MHz mais pas bien a 20 et pas du tout à 40 MHz (default)
        USBSerial.println("Card Mount Failed");
        // return;
    }
    USBSerial.println("SD Card Mounted :-)");

    // WiFi stuff
    USBSerial.println("Configuring access point...");
    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();
    USBSerial.print("AP IP address: ");
    USBSerial.println(myIP);
    server.begin();
    USBSerial.println("Server started");
}

void loop() {
#ifdef XSTRATO_R2H
    XSTRATO_R2H_loop();
#endif
#ifdef XSTRATO_GS
    XSTRATO_GS_loop();
#endif
}

void XSTRATO_R2H_loop() {
    // create empty packet
    static uint32_t packetNbr = 0;
    radio_packet_t packet;

    packet.prefix = XSTRATO_prefix;
    packet.packet_nbr = packetNbr++;

    fill_BME_data(&packet);
    packet.bat_level = analogRead(BATTERY_MEASURE_PIN);  // Coop Varta, 18.02.2023  v = 3100
    packet.SD_Bytes_used = SD_MMC.usedBytes();
    packet.rssi = LoRa.rssi();
    packet.snr = LoRa.packetSnr();

    bool r = capture_img_and_save_in_SD(FRAMESIZE_HD);
    USBSerial.println("Cam to SD result: " + String((r) ? "Success" : "Fail"));
    // LoRa_send_file("/texte.txt"); // send image
    
    // TODO: fun Check if LoRa receive a cmd
    LoRa_handle_cmd();

    // LoRa send packet
    LoRa.beginPacket();
    LoRa.write((uint8_t *)&packet, LoRa_telemetry_packet_size); // TODO not always full packet !
    LoRa.endPacket(true);  // true = asynch

    delay(8000);
}

bool capture_img_and_save_in_SD(framesize_t framesize, int jpeg_quality) {
    static uint32_t pic_Nbr = 0;

    if (!setup_cam(framesize, jpeg_quality)) {
        USBSerial.println("Camera init failed");
        return false;
    }
    // capture a frame
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        USBSerial.println("Error with frame buffer pointer");
        return false;
    }
    // replace this with your own function
    USBSerial.println("Size: " + String(fb->len) + " Bytes");
    // display_image(fb->width, fb->height, fb->pixformat, fb->buf, fb->len);

    // Save picture in memory
    char filename[50];
    sprintf(filename, "/rx_img_%u.jpeg", pic_Nbr++);
    save_image(SD_MMC, filename, fb->buf, fb->len);

    // return the frame buffer back to be reused
    esp_camera_fb_return(fb);
    
    esp_camera_deinit();
    turn_off_cam();

    return true;
}

void LoRa_handle_cmd() {

}

void XSTRATO_GS_loop() {
    // try to parse packet
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        // received a packet
        SERIAL_.println("Received packet with RSSI: " + String(LoRa.packetRssi()) + " dBm");

        // read packet
        size_t size = 0;
        while (LoRa.available() && size < LORA_BUFFER_SIZE) {
            LoRa_rx_buf[size++] = LoRa.read();
        }
        radio_packet_t rxpacket;
        memcpy(&rxpacket, LoRa_rx_buf, size);

        // save buf in SD memory
        logAppendFile(SD_MMC, "/rx_log_file.txt", LoRa_rx_buf, size);

        print_packet(rxpacket);
    }
}

void loop_debug() {
    // static uint32_t t = 0;
    // if (t - millis() > 500) {

    // }
    readFile(SD_MMC, "/texte.txt");
    // test_SD_card();
    static uint32_t v = 0;
    for (int i = 0; i < 8; i++) {
        led.fill(colors[i]);
        led.show();

        // digitalWrite(GREEN_LED_PIN, HIGH);
        delay(500);
        // digitalWrite(GREEN_LED_PIN, LOW);
        delay(500);

        uint16_t v = analogRead(BATTERY_MEASURE_PIN);  // Coop Varta, 18.02.2023  v = 3100
        SERIAL_.println(v);
    }
    // server_loop();
}

void LoRa_send_file(const char *path, uint8_t length) {
    // send packet
    LoRa.beginPacket();
    LoRa.print("hello: ");
    getFile(SD_MMC, path, LoRa_tx_buf, length);
    LoRa.write(LoRa_tx_buf, length);
    LoRa.endPacket(true);
    SERIAL_.print("Sending packet: ");
}

void fill_BME_data(radio_packet_t *packet) {
    if (!bme.performReading()) {
        USBSerial.println("Failed to perform reading :(");
        return;
    }
    packet->bme_temp = bme.temperature;
    packet->bme_press = bme.pressure / 100.0;
    packet->bme_hum = bme.humidity;
}

void print_packet(radio_packet_t packet) {
    // USBSerial.println("Printing received packet content: ");
    USBSerial.printf("PacketNbr: %u ", packet.packet_nbr);
    USBSerial.printf("\n\rBME: %u hPa | %.1f °C | %.1f %%", packet.bme_press, packet.bme_temp, packet.bme_hum);
    USBSerial.printf("\n\rBattery: %u ", packet.bat_level);
    USBSerial.printf("\n\rLoRa strato: SNR: %.1f  RSSI: %d ", packet.snr, packet.rssi);

    USBSerial.println("\n\r#############################################");
}

//////////////////////////////////////////////////////////////////////////////////
// DEBUG 

void BME_loop() {
    if (!bme.performReading()) {
        USBSerial.println("Failed to perform reading :(");
        return;
    }
    USBSerial.print("Temperature = ");
    USBSerial.print(bme.temperature);
    USBSerial.println(" *C");

    USBSerial.print("Pressure = ");
    USBSerial.print(bme.pressure / 100.0);
    USBSerial.println(" hPa");

    USBSerial.print("Humidity = ");
    USBSerial.print(bme.humidity);
    USBSerial.println(" %");

    USBSerial.print("Gas = ");
    USBSerial.print(bme.gas_resistance / 1000.0);
    USBSerial.println(" KOhms");

    USBSerial.print("Approx. Altitude = ");
    USBSerial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    USBSerial.println(" m");

    USBSerial.println();
}

void server_loop() {
    WiFiClient client = server.available();  // listen for incoming clients

    if (client) {                          // if you get a client,
        USBSerial.println("New Client.");  // print a message out the serial port
        String currentLine = "";           // make a String to hold incoming data from the client
        while (client.connected()) {       // loop while the client's connected
            if (client.available()) {      // if there's bytes to read from the client,
                char c = client.read();    // read a byte, then
                USBSerial.write(c);        // print it out the serial monitor
                if (c == '\n') {           // if the byte is a newline character

                    // if the current line is blank, you got two newline characters in a row.
                    // that's the end of the client HTTP request, so send a response:
                    if (currentLine.length() == 0) {
                        // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
                        // and a content-type so the client knows what's coming, then a blank line:
                        client.println("HTTP/1.1 200 OK");
                        client.println("Content-type:text/html");
                        client.println();

                        // the content of the HTTP response follows the header:
                        client.print("Click <a href=\"/H\">here</a> to turn ON the LED.<br>");
                        client.print("Click <a href=\"/L\">here</a> to turn OFF the LED.<br>");

                        // The HTTP response ends with another blank line:
                        client.println();
                        // break out of the while loop:
                        break;
                    } else {  // if you got a newline, then clear currentLine:
                        currentLine = "";
                    }
                } else if (c != '\r') {  // if you got anything else but a carriage return character,
                    currentLine += c;    // add it to the end of the currentLine
                }

                // Check to see if the client request was "GET /H" or "GET /L":
                if (currentLine.endsWith("GET /H")) {
                    digitalWrite(GREEN_LED_PIN, HIGH);  // GET /H turns the LED on
                }
                if (currentLine.endsWith("GET /L")) {
                    digitalWrite(GREEN_LED_PIN, LOW);  // GET /L turns the LED off
                }
            }
        }
        // close the connection:
        client.stop();
        USBSerial.println("Client Disconnected.");
    }
}