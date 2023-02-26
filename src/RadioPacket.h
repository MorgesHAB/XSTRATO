//////////////////////////////////////////////////////////////////////////
// XSTRATO packet structure
#include <Arduino.h>

#define IMAGE_LENGTH_PER_PACKET		210		// Byte/packet

typedef struct __attribute__((__packed__)) radio_packet {
    // Packet content
	uint16_t prefix;
	// uint32_t timestamp;
    uint32_t packet_nbr;

    uint32_t bme_press;
    float bme_temp;
    float bme_hum;

	uint16_t bat_level;

	uint32_t SD_Bytes_used;

    int rssi;
    float snr;

    uint8_t cmd_type;
    uint8_t cmd_data;

    // float gnss_lon;
    // float gnss_lat;
    // int32_t gnss_alt;

    // uint8_t av_state;

	uint8_t img_packet_nbr;
	uint8_t img_packet_nbr_tot;

	uint8_t image_data[IMAGE_LENGTH_PER_PACKET];

} radio_packet_t;

const uint32_t LoRa_full_packet_size = sizeof(radio_packet_t);
const uint32_t LoRa_telemetry_packet_size = LoRa_full_packet_size - IMAGE_LENGTH_PER_PACKET;

uint16_t XSTRATO_prefix = 'X' << 8 | 'S';


//////////////////////////////////////////////////////////////////////////