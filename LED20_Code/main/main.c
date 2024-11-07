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
#include <math.h>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "hal/spi_types.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/rmt_common.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_types.h"
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

#define LED_NORTH 23 // LED Data Out pin for North Hemisphere
#define LED_NORTH_CHAIN 5 // Number of LEDs in North Hemisphere


// Sensor classifications and constants

#define SPI_CLOCK_SPEED 8           // SPI transfer speed in MHz



// Testing and debugging
#define BLINK_USEC  500000
static uint8_t s_led_state = 0;


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


// RMT transfer setup
rmt_channel_handle_t rmt_LED_north = NULL;
rmt_tx_channel_config_t rmt_north_config = {
        
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .gpio_num = LED_NORTH,
    .mem_block_symbols = (128),
    .resolution_hz = 10 * 1000 * 1000,
    .trans_queue_depth = 1,
    .flags.invert_out = false,
    .flags.with_dma = false,
};

static rmt_encoder_handle_t bytes_encoder = NULL;
rmt_bytes_encoder_config_t bytes_encoder_config = {
    .bit0 = { .level0 = 1, .duration0 = T0H, .level1 = 0, .duration1 = T0L },   // 0 bit: 0.3 µs high, 0.9 µs low
    .bit1 = { .level0 = 1, .duration0 = T1H, .level1 = 0, .duration1 = T1L },   // 1 bit: 0.6 µs high, 0.6 µs low
    .flags.msb_first = true, // SK6812MINI typically sends MSB first as well
};







static const char* TAG = "LED20";


void w_trans (uint16_t address, uint16_t data ){
    
    spi_transaction_t transfer = { 0 };
    
    transfer.length = 8;
    ESP_LOGD(TAG, "Setting flags");
    transfer.flags = SPI_TRANS_USE_TXDATA;
    ESP_LOGD(TAG, "Setting address");   
    transfer.cmd = address;
    ESP_LOGD(TAG, "Setting data");
    transfer.tx_data[0] = data;
    ESP_LOGD(TAG, "Attempting write at address 0x%02X", address);
    ESP_ERROR_CHECK( spi_device_transmit(accel, &transfer) );
    ESP_LOGD(TAG, "Write at address 0x%02X completed", address);

    return;

}

uint8_t r_int(){


    spi_transaction_t transfer = {0};
    esp_err_t err;
    transfer.length = 8;
    transfer.rxlength = 8;
    transfer.flags = SPI_TRANS_USE_RXDATA;  
    transfer.cmd = (0x35 | 0x80) ;
    ESP_LOGD(TAG, "Attempting to read Interrupt Status Register ");

    err = spi_device_transmit(accel, &transfer);
    ESP_ERROR_CHECK(err);


    return transfer.rx_data[0];


    
}

uint8_t r_trans(uint8_t address){

    spi_transaction_t transfer = {0};
    esp_err_t err;
    transfer.length = 8;
    transfer.rxlength = 8;
    ESP_LOGD(TAG, "Setting flags");
    transfer.flags = SPI_TRANS_USE_RXDATA;
    ESP_LOGD(TAG, "Setting address");   
    transfer.cmd = (address | 0x80) ;
    ESP_LOGD(TAG, "Attempting read at address 0x%02x", address);

    err = spi_device_transmit(accel, &transfer);
    ESP_ERROR_CHECK(err);
    return transfer.rx_data[0];

}

void r2_trans(uint8_t address, int8_t data[2]){

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



void starburst ( uint8_t start_address, int8_t data[12]){     // A function to burst read 12 consecutive data segments

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
        start_address + 4*i );

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
        ESP_LOGD(TAG, "Gathered data from transaction at address 0x%02X", start_address + 4*i);




    }
    return;


}

void led_w(uint8_t *led_data){
    
     ESP_ERROR_CHECK (rmt_enable( rmt_LED_north));

    ESP_LOGD(TAG, "Setting up LED write");
    rmt_transmit_config_t led_tx = {
        .loop_count = 0  // No looping
    };
    ESP_LOGD(TAG, "Attempting LED write");
    ESP_ERROR_CHECK(rmt_transmit(rmt_LED_north, bytes_encoder, led_data, LED_NORTH_CHAIN * 3, &led_tx));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(rmt_LED_north, portMAX_DELAY));

    ESP_ERROR_CHECK (rmt_disable( rmt_LED_north));

    return;
}


int app_main(void)
{
    // Local variable declarations
    esp_err_t spi_error;
    //int8_t read_temp[2];
    int8_t read_burst[12];


    uint8_t led_one[5 * 3] = {
    0x00, 0x00, 0x00,  // Green (128), Red (0), Blue (0) - Red
    0x00, 0x00, 0x00,  // Green (0), Red (128), Blue (0) - Green
    0x00, 0x00, 0x80,  // Green (0), Red (0), Blue (128) - Blue
    0x00, 0x00, 0x00,  // Green (128), Red (128), Blue (0) - Yellow
    0x00, 0x00, 0x00   // Green (128), Red (0), Blue (128) - Cyan
};

    uint8_t led_rgby[15] = {
    0x00, 0x80, 0x00,  // Green (128), Red (0), Blue (0) - Red
    0x80, 0x00, 0x00,  // Green (0), Red (128), Blue (0) - Green
    0x00, 0x00, 0x80,  // Green (0), Red (0), Blue (128) - Blue
    0x80, 0x80, 0x00,  // Green (128), Red (128), Blue (0) - Yellow
    0x00, 0x80, 0x80   // Green (128), Red (0), Blue (128) - Cyan

    };

    uint8_t led_off[LED_NORTH_CHAIN * 3];
    for (int i = 0; i < LED_NORTH_CHAIN * 3; i++){

        led_off[i]=0;
    }

    // LED RMT setup

   

    if (rmt_LED_north) {
        rmt_del_channel(rmt_LED_north);
        rmt_LED_north = NULL;   
    }   


    ESP_ERROR_CHECK( rmt_new_tx_channel(&rmt_north_config, &rmt_LED_north));
   

    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bytes_encoder_config, &bytes_encoder));


    
    

    

    // SPI bus and device setup
    spi_bus_config_t spi_bus_cfg = {        // configure SPI bus properties and pins

        .miso_io_num = PIN_MISO,            // Set up IO pins 
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,

    };



    spi_error = spi_bus_initialize(SPI_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(spi_error);

    spi_error = spi_bus_add_device(SPI2_HOST, &accel_config ,&accel );
    ESP_ERROR_CHECK(spi_error);

    ESP_LOGD(TAG,"SPI was set up with no errors");
    
    uint8_t whoami = r_trans(0x0F);
    ESP_LOGD(TAG, "Value: 0x%02X", whoami);
    if (whoami != 0x6C){
        ESP_LOGE(TAG, "DATA READ FAILURE");
        return -1;
    }
    
    
    // Reset and intialize sensors
    w_trans(CTRL3_C, 0x1);    // Reset the sensor
    w_trans(CTRL1_XL, 0xA4);    // Set the speed and rate of the accel
    w_trans(CTRL2_G, 0xAC);     // Set the speed and rate of the gyro
    
    // Set up Significant Motion Detection interrupt on INT1
    w_trans(FUNC_CFG_ACCESS, 0x80);  // Access embedded functions
    w_trans(EMB_FUNC_EN_A, 0x20);    // Enable SMD interrupt
    w_trans(EMB_FUNC_INT1, 0x20);    // Route SMD to INT1
    w_trans(PAGE_RW, 0x80);          // Latching interrupts enable
    w_trans(FUNC_CFG_ACCESS, 0x00);  // Return to control registers
    w_trans(MD1_CFG, 0x02);             // Set INT1_EMB_FUNC in MD1_CFG

    // Set up Inactivity Detection intterupt on INT2
    w_trans(WAKEUP_DUR, 0x02);       // Set inactivity time 
    w_trans(WAKEUP_THS, 0x01);       // Set inactivity threshold
    w_trans(TAP_CFG0, 0x00);         // Set sleep-change notification
    w_trans(TAP_CFG2, 0xE0);         // Enable interrupt
    w_trans(MD2_CFG, 0x80);          // Route interrupt to INT2


    led_w(led_one);
    sleep(1);
    led_w(led_off);
    sleep(1);
    led_w(led_rgby);
    sleep(3);
    led_w(led_off);

    while(1){                                          // Temporary loop to test reading registers and LED states

        
        starburst( GYRO_X_LOW, read_burst );            // Get 12 bytes of data from accelerometer and gyroscope
        ESP_LOGI(TAG, "Gyroscope values: X %d, Y %d, Z %d", (read_burst[1]  << 8) | read_burst[0] , (read_burst[3]  << 8) | read_burst[2], (read_burst[5]  << 8) | read_burst[4] );
        ESP_LOGI(TAG, "Accelerometer values: X %d, Y %d, Z %d", (read_burst[7]  << 8) | read_burst[6] , (read_burst[9]  << 8) | read_burst[8], (read_burst[11]  << 8) | read_burst[10] );

        if (s_led_state) {
            
            led_w(led_off);
            s_led_state = !s_led_state;

        }
        else{
            led_w(led_rgby);
            s_led_state = !s_led_state;
        } 

        usleep(BLINK_USEC);
        

        


    }
    return 0;
    
}
