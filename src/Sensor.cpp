#include "Sensor.h"
#include "Config.h"

#include <Wire.h>
// #include <Adafruit_Sensor.h> // need to change sensor_t to sensor_tt (conflict with esp_cam sensor_t)
#include <Adafruit_BME680.h>

Adafruit_BME680 bme;  // I2C
TwoWire I2CBME = TwoWire(0);

bool BME_setup() {
    // BME Init
    I2CBME.begin(BME_I2C_SDA, BME_I2C_SCL, (uint32_t)100000);
    if (!bme.begin(0x76, &I2CBME)) {
        if (DEBUG) {
            SERIAL_TO_PC.println("Could not find a valid BME680 sensor, check wiring!");
        }
        return false;
    } else {
        // Set up oversampling and filter initialization
        bme.setTemperatureOversampling(BME680_OS_8X);
        bme.setHumidityOversampling(BME680_OS_2X);
        bme.setPressureOversampling(BME680_OS_4X);
        bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme.setGasHeater(320, 150);  // 320*C for 150 ms
        if (DEBUG) {
            SERIAL_TO_PC.println("BME inited :-)");
        }
        return true;
    }
}

bool fill_BME_data(BarometerPacket *packet) {
    if (!bme.performReading()) {
        if (DEBUG) {
            SERIAL_TO_PC.println("Failed to perform reading :(");
        }
        return false;
    }
    packet->temperature = bme.temperature;
    packet->pressure = bme.pressure / 100.0;
    packet->humidity = bme.humidity;
    return true;
}

void print_debug() {
    if (!bme.performReading()) {
        SERIAL_TO_PC.println("Failed to perform reading :(");
        return;
    }
    SERIAL_TO_PC.print("Temperature = ");
    SERIAL_TO_PC.print(bme.temperature);
    SERIAL_TO_PC.println(" *C");

    SERIAL_TO_PC.print("Pressure = ");
    SERIAL_TO_PC.print(bme.pressure / 100.0);
    SERIAL_TO_PC.println(" hPa");

    SERIAL_TO_PC.print("Humidity = ");
    SERIAL_TO_PC.print(bme.humidity);
    SERIAL_TO_PC.println(" %");

    SERIAL_TO_PC.print("Gas = ");
    SERIAL_TO_PC.print(bme.gas_resistance / 1000.0);
    SERIAL_TO_PC.println(" KOhms");

    SERIAL_TO_PC.print("Approx. Altitude = ");
    SERIAL_TO_PC.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    SERIAL_TO_PC.println(" m");

    SERIAL_TO_PC.println();
}