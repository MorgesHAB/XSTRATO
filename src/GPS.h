#include <TinyGPS++.h>

#define GPS_PORT          Serial1
#define GPS_BAUDRATE      57600
#define GPS_RATE          1

#define GPS_TX            8
#define GPS_RX            3

void sendPacket(byte *packet, byte len);
void gpsSetup(int a, int b, int c, int d, int e);