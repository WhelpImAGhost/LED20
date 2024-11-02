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
#include <string.h>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "hal/spi_types.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "defines.c"

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

#define SPI_CLOCK_SPEED 8           // SPI transfer speed in MHz



#define TILT_TOLERANCE 0.1          // Percent of tilt to regeister a zone
#define ACCEL_THRESHOLD 0           // Threshold value for assuming the die is stationary
#define STATIONARY_TIME 1           // Time delay (in seconds) after stationary to determine orientation


// Testing and debugging
#define BLINK_GPIO 1
#define BLINK_INTERVAL_USEC 5000


// SPI bus device setup
spi_device_handle_t accel;              // Set up accel/gyro device 
spi_device_interface_config_t accel_config = {

    .clock_speed_hz = SPI_CLOCK_SPEED * 1000 * 1000,    // Set clock speed (Hz)
    .spics_io_num = PIN_CS,                             // Set CS pin
    .mode = 3,                                          // According to datasheet page 30
    .queue_size = 12,                                   // Max number of queued transactions
    .command_bits = 8,                                  // Specify 8 bits for command (register address)
    .address_bits = 0,                                  // Set address_bits to 0 since only cmd is used for address

};




static const char* TAG = "LED20";
static uint8_t s_led_state = 0;


esp_err_t w_trans (uint16_t address, uint16_t data ){
    
    spi_transaction_t transfer = { 0 };
    //ESP_LOGD(TAG, "Attempting to clear prior transfer");
    //memset(&transfer, 0, sizeof(transfer));
    //ESP_LOGD(TAG,"Transfer cleared, setting up next one");
    
    transfer.length = 8;
    ESP_LOGD(TAG, "Setting flags");
    transfer.flags = SPI_TRANS_USE_TXDATA;
    ESP_LOGD(TAG, "Setting address");   
    transfer.cmd = address;
    ESP_LOGD(TAG, "Setting data");
    transfer.tx_data[0] = data;
    ESP_LOGD(TAG, "Attempting write at address 0x%02X", address);
    return spi_device_transmit(accel, &transfer);


}

void r_trans(uint16_t address, uint8_t data[2]){

    spi_transaction_t transfer = {0};
    esp_err_t err;
    transfer.length = 16;
    transfer.rxlength = 16;
    ESP_LOGD(TAG, "Setting flags");
    transfer.flags = SPI_TRANS_USE_RXDATA;
    ESP_LOGD(TAG, "Setting address");   
    transfer.cmd = (address | 0x80) ;
    ESP_LOGD(TAG, "Attempting read at address 0x%02X and 0x%02x", address, address + 0x1);

    err = spi_device_transmit(accel, &transfer);
    ESP_ERROR_CHECK(err);

    data[0] = transfer.rx_data[0];
    data[1] = transfer.rx_data[1];
    return;


    
}



void starburst ( uint8_t start_address, uint16_t data[12]){     // A function to burst read 12 consecutive data segments

    spi_transaction_t trans[3];                                 // Set up 3 transactions of 4 bytes
    esp_err_t error;


    for (int i = 0; i < 3; i++){


        trans[i].length = 32;                                   // Each transaction has a length of 4 bytes
        trans[i].rxlength = 32;                                 
        trans[i].cmd = ( (start_address + (i*4)) | 0x80 );      // Increment starting addres by 4 each iteration
        trans[i].flags = SPI_TRANS_USE_RXDATA;                  // Use rx_data until DMA access becomes necessary
        trans[i].rx_buffer = NULL;
        trans[i].tx_buffer = NULL;

        ESP_LOGD(TAG, "Queued transaction at address 0x%02X",   // Queue 4 byte burst
        start_address + i );

        error = spi_device_queue_trans(accel, &trans[i], portMAX_DELAY);
        ESP_ERROR_CHECK(error);



    }

    for (int i = 0; i < 3; i++){

        spi_transaction_t *receive;

        error = spi_device_get_trans_result(accel, &receive,    // Run all queued transactions
         portMAX_DELAY);
        ESP_ERROR_CHECK(error);

        data[4*i] = receive->rx_data[0];                        // Write gathered data to array
        data[4*i+1] = receive->rx_data[1];
        data[4*i+2] = receive->rx_data[2];
        data[4*i+3] = receive->rx_data[3];
        ESP_LOGD(TAG, "Gathered data from transaction at address 0x%02X", start_address + i);




    }
    return;


}


int app_main(void)
{
    // Local variable declarations
    esp_err_t spi_error;
    uint8_t read_data[2];
    uint16_t read_burst[12];


    // SPI bus and device setup
    spi_bus_config_t spi_bus_cfg = {        // configure SPI bus properties and pins

        .miso_io_num = PIN_MISO,            // Set up IO pins 
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,

    };



    // One-off SPI transfer setup
    spi_transaction_t transfer = {
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

    ESP_LOGD(TAG,"SPI was set up with no errors");
    ESP_LOGD(TAG,"Attempting transfer");

    spi_error = spi_device_transmit(accel, &transfer);
    ESP_ERROR_CHECK(spi_error);


    uint8_t whoami = transfer.rx_data[0];
    if (whoami != 0x6C){
        ESP_LOGE(TAG, "DATA READ FAILURE");
        return -1;
    }
    ESP_LOGD(TAG, "Gathered data was: 0x%02X", whoami );
    
    

    spi_error = w_trans(CTRL3_C, 0b101);
    ESP_ERROR_CHECK(spi_error);
    ESP_LOGD(TAG, "Transfer completed");

    spi_error = w_trans(CTRL1_XL, 0xA4);
    ESP_ERROR_CHECK(spi_error);
    ESP_LOGD(TAG, "Transfer completed");

    spi_error = w_trans(CTRL2_G, 0xAC);
    ESP_ERROR_CHECK(spi_error);
    ESP_LOGD(TAG, "Transfer completed");

    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);  // Set pin to OUTPUT mode


    while(1){                                          // Temporary loop to test reading registers and LED states

        gpio_set_level(BLINK_GPIO, s_led_state); // Set LED state based on current LED state
        s_led_state = !s_led_state; // Invert LED state
        starburst( GYRO_X_LOW, read_burst );            // Get 12 bytes of data from accelerometer and gyroscope
        ESP_LOGD(TAG, "Gyroscope values: X 0x%04X, Y 0x%04X, Z 0x%04X", (read_burst[1]  << 8) | read_burst[0] , (read_burst[3]  << 8) | read_burst[2], (read_burst[5]  << 8) | read_burst[4] );
        ESP_LOGD(TAG, "Accelerometer values: X 0x%04X, Y 0x%04X, Z 0x%04X", (read_burst[7]  << 8) | read_burst[6] , (read_burst[9]  << 8) | read_burst[8], (read_burst[11]  << 8) | read_burst[10] );
        usleep(BLINK_INTERVAL_USEC);


    }
    return 0;
    
}
