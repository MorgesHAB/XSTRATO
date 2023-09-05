//#define SENDER false  // In platformio.ini directly

#define DEBUG                   false
#if !SENDER
#define USE_UDP_BROADCAST       true      // transmit Xstrato Telemetry packet to Android App via WiFi broadcast
#endif

#define SERIAL_TO_PC            USBSerial
#define SERIAL_TO_PC_BAUD       115200

#define SD_CMD                  6
#define SD_CLK                  7
#define SD_D0                   15
#define SD_D1                   16
#define SD_D2                   4
#define SD_D3                   5

#define LORA_FREQ               868.42E6
#define LORA_POWER              20
#define LORA_BW                 125.0e3
#define LORA_SF                 11
#define LORA_CR                 7
#define LORA_PREAMBLE_LEN       8
#define LORA_SYNC_WORD          0x12
#define LORA_CRC                true
#define LORA_CURRENT_LIMIT      120

#define LORA_LR_BW_DEFAULT      125e3
#define LORA_LR_SF_DEFAULT      7
#define LORA_LR_CR_DEFAULT      7

// PIN/GPIO Definition
#ifdef RADIO_MODULE
// PIN/GPIO Definition on Radio Module ERT
#define LORA_SCK                42
#define LORA_MOSI               44
#define LORA_MISO               43
#define LORA_CS                 41
#define LORA_INT0               21
#define LORA_INT5               39
#define LORA_RST                -1

#define NEOPIXEL_PIN            18

#else // XSTRATO Board
// PIN/GPIO Definition on XSTRATO PCB
#define LORA_SCK                9
#define LORA_MOSI               10
#define LORA_MISO               11
#define LORA_CS                 46
#define LORA_INT0               12
#define LORA_RST                -1

#define NEOPIXEL_PIN            21

#endif

#define GREEN_LED_PIN           14

#define BATTERY_MEASURE_PIN     13

// GPS
#if SENDER == true
#define SEND_POSITION_PACKET    true
#define SEND_IMAGE_PACKET       true 
#else 
#define SEND_POSITION_PACKET    false
#define SEND_IMAGE_PACKET       false
#endif

// Rate of position packet transmission (in Hz), 1/30 = once every 30 seconds
#define POSITION_PACKET_RATE   1.0/10.0

// The fragment size is the size in bytes of each fragment of the file that will be sent using teleFile. 
// The coding rate is the degree of redundancy used to encode the file.
#define FRAGMENT_SIZE           100

#define TRANSMISSION_MARGIN_RATE_DEFAULT    1.6
#define TRANSMISSION_MARGIN_RATE_MIN        1.5
#define TRANSMISSION_MARGIN_RATE_MAX        3

#define TRANSMISSION_SILENCE_TIME_DEFAULT   20
#define TRANSMISSION_SILENCE_TIME_MIN       10
#define TRANSMISSION_SILENCE_TIME_MAX       100

#define TRANSMISSION_ENABLE_DEFAULT         true

// The size of the packet that we send to the PC to display the pictur 
#define FRAGMENT_SIZE_PC        200

#define MAX_IMAGE_SIZE          50e3

