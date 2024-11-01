/* LED20
 * @author: ECE 411 Team 01
 * 
 * 
 * This program connects a Seeed Studio ESP32-C6 to
 * a LSM6DSO32 Accelerometer/Gyroscope IC. The end goal
 * is to create a D20 or 20-sided die that illuminates 
 * the top-most side after being rolled. If a 1 or a 20
 * are rolled, the die then plays a specialized animation 
 * on the LEDs and a sound or melody on the piezo speaker.
 * 
 * 
 * 
*/


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


// SPI bus device setup
spi_device_handle_t accel;              // Set up accel/gyro device 
spi_device_interface_config_t accel_config = {

    .clock_speed_hz = 8 * 1000 * 1000,  // Set clock speed (Hz)
    .spics_io_num = PIN_CS,             // Set CS pin
    .mode = 3,                          // According to datasheet page 30
    .queue_size = 12,                    // Max number of queued transactions
    .command_bits = 8,                  // Specify 8 bits for command (register address)
    .address_bits = 0,                  // Set address_bits to 0 since only cmd is used for address

};


// Testing and debugging
#define BLINK_GPIO 1
#define BLINK_INTERVAL_USEC 5000000

static const char* TAG = "LED20";
static uint8_t s_led_state = 0;


void app_main(void)
{
    // Local variable declarations
    esp_err_t spi_error;

    // SPI bus and device setup
    spi_bus_config_t spi_bus_cfg = {        // configure SPI bus properties and pins

        .miso_io_num = PIN_MISO,            // Set up IO pins 
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };



    // One-off SPI transfer setup
    spi_transaction_t test = {
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,           // Enable send and recieve data
        .cmd = 0x8F,                                                    // Set Read mode for WHO_AM_I register
        .length = 8,                                                    // Length of data in / out
        .rxlength = 8,                                                  // Length of data out
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };

    spi_error = spi_bus_initialize(SPI_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(spi_error);

    spi_error = spi_bus_add_device(SPI2_HOST, &accel_config ,&accel );
    ESP_ERROR_CHECK(spi_error);

    ESP_LOGI(TAG,"SPI was set up with no errors");
    ESP_LOGI(TAG,"Attempting transfer");

    spi_error = spi_device_transmit(accel, &test);
    ESP_ERROR_CHECK(spi_error);

    uint8_t whoami = test.rx_data[0];
    ESP_LOGI(TAG, "Gathered data was: 0x%02X", whoami );
    



    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);  // Set pin to OUTPUT mode
    while(1){

        gpio_set_level(BLINK_GPIO, s_led_state); // Set LED state based on current LED state
        s_led_state = !s_led_state; // Invert LED state
        usleep(BLINK_INTERVAL_USEC);


    }
    
}
