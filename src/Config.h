#define DEBUG false

#define SERIAL_TO_PC    USBSerial
#define SERIAL_TO_PC_BAUD 115200

//#define SD_CMD                  6
//#define SD_CLK                  7
//#define SD_D0                   15
//#define SD_D1                   16
//#define SD_D2                   4
//#define SD_D3                   5

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

// GPS 
#define SEND_POSITION_PACKET  false 
#define SEND_IMAGE_PACKET     false  

// Rate of position packet transmission (in Hz), 1/30 = once every 30 seconds
#define POSITION_PACKET_RATE   1.0/10.0