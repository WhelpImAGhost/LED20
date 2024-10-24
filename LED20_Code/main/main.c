#include <stdio.h>
#include "driver/i2c_master.h"
#include "hal/spi_types.h"

// SPI connection and classifications

#define PIN_MISO 20 // MISO GPIO of host
#define PIN_MOSI 18 // MOSI GPIO of host
#define PIN_SCK 19  // SCK GPIO of host
#define PIN_CS 17   // CS GPIO of host
#define PIN_INT1 1  // GPIO Input for interrupt 1
#define PIN_INT2 2  // GPIO Input for interrupt 2

// LED Connection and classifications

#define LED_NORTH 21 // LED Data Out pin for North Hemisphere
#define LED_NORTH_CHAIN 10 // Number of LEDs in North Hemisphere
#define LED_SOUTH 22 // LED Data Out pin for South Hemisphere
#define LED_SOUTH_CHAIN 10 // Number of LEDs in South Hemisphere


// Sensor classifications and constants
#define TILT_TOLERANCE 0.1 // Percent of tilt to regeister a zone
#define ACCEL_THRESHOLD 0  // Threshold value for assuming the die is stationary
#define STATIONARY_TIME 1  // Time delay (in seconds) after stationary to determine orientation




void app_main(void)
{


}
