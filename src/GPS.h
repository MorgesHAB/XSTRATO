#include <TinyGPS++.h>

#define GPS_PORT          Serial1
#define GPS_BAUDRATE      9600
#define GPS_RATE          1

#define GPS_TX            3
#define GPS_RX            8

void makeConfig();
void sendPacket(byte *packet, byte len);
void gpsSetup(int a, int b, int c, int d, int e);