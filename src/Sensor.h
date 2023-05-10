
#include "../XRF_interface/PacketDefinition.h"

#define BME_I2C_SDA             17
#define BME_I2C_SCL             18

#define SEALEVELPRESSURE_HPA    (1013.25)

bool BME_setup();

bool fill_BME_data(BarometerPacket *packet);

void BME_loop();
