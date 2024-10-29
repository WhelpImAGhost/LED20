#include <stdio.h>
#include <unistd.h>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "hal/spi_types.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_log.h"

// SPI connection and classifications

#define SPI_HOST SPI2_HOST      // SPI1 not available

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



// Testing and debugging
#define BLINK_GPIO 1
#define BLINK_INTERVAL_USEC 500000

// static uint8_t s_led_state = 0;


void app_main(void)
{
    esp_err_t spi_error;

    spi_bus_config_t spi_bus_cfg = {
        .miso_io_num = PIN_MISO,    // Set up IO pins 
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_device_handle_t accel;
    spi_device_interface_config_t accel_config = {

        .clock_speed_hz = 5 * 1000 * 1000,  // Set clock speed (Hz)
        .spics_io_num = PIN_CS,             // Set CS pin
        .mode = 3,                          // According to datasheet page 30
        .queue_size = 4                     // Queue, for, something?


    };
    spi_error = spi_bus_initialize(SPI_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK_WITHOUT_ABORT(spi_error);
    spi_error = spi_bus_add_device(SPI_HOST, &accel_config, &accel);
    ESP_ERROR_CHECK_WITHOUT_ABORT(spi_error);


    /*
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);  // Set pin to OUTPUT mode
    while(1){

        gpio_set_level(BLINK_GPIO, s_led_state); // Set LED state based on current LED state
        s_led_state = !s_led_state; // Invert LED state
        usleep(BLINK_INTERVAL_USEC);


    }
    */
}
