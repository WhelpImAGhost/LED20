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
#include "esp_sleep.h"
#include "driver/ledc.h"
#include "esp_timer.h"


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

#define LED_NORTH 22 // LED Data Out pin for North Hemisphere
#define LED_NORTH_CHAIN 20 // Number of LEDs in North Hemisphere


// Sensor classifications and constants

#define SPI_CLOCK_SPEED 6           // SPI transfer speed in MHz
#define ACCEL_SENSITIVITY_4G 0.000122
#define GYRO_SENSITIVITY_500DPS 0.0175
#define Z_OFFSET_ACC -0.007
#define Z_OFFSET_GYRO -178
#define TOLERANCE 0.2

#define SENSOR_DATA_GRAB_REPEATS 10

// Piezo speaker setup
#define PIEZO_PIN 0

// Testing and debugging
#define BLINK_USEC  250000
#define LED_HEARTBEAT 15
#define TIMER_DUR 40

bool heartbeat_state = false;
bool first_inact = false;
static bool interrupt_triggered = false;
static bool active_detect = false;
static bool inactive_detect = false;
volatile bool is_drdy = true;
int8_t read_burst[12];
int8_t accel_temp_burst[8];
float accel_results[3];
TaskHandle_t drdy_task_handle = NULL;
static esp_timer_handle_t timer_handle = NULL;

const float faces[NUM_FACES][3] = {
    FACE_1, FACE_2, FACE_3, FACE_4, FACE_5,
    FACE_6, FACE_7, FACE_8, FACE_9, FACE_10,
    FACE_11, FACE_12, FACE_13, FACE_14, FACE_15,
    FACE_16, FACE_17, FACE_18, FACE_19, FACE_20
};
                        // 20  8 10  2 12 15  7 17  3 16  6 14  4 18  5 13 11 19  9  1
// Put LED order here,      1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20
const int led_order[20] = {19, 3, 8,12,14,10, 6, 1,18, 2,16, 4,15,11, 5, 9, 7,13,17,0};


// Global LED value 
uint8_t led_red[LED_NORTH_CHAIN * 3];
uint8_t led_green[LED_NORTH_CHAIN * 3];
uint8_t led_blue[LED_NORTH_CHAIN * 3];
uint8_t led_off[LED_NORTH_CHAIN * 3];
uint8_t led_current[LED_NORTH_CHAIN * 3];

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

ledc_timer_config_t piezo_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,   // High-speed mode
        .timer_num = LEDC_TIMER_0,           // Timer 0
        .duty_resolution = LEDC_TIMER_10_BIT,// Resolution of PWM duty (0-1023)
        .freq_hz = 2000,                     // Frequency in Hertz (e.g., 2 kHz for a tone)
        .clk_cfg = LEDC_AUTO_CLK             // Use the default clock source
};

ledc_channel_config_t piezo_channel = {
        .gpio_num = PIEZO_PIN,             // GPIO number for the buzzer
        .speed_mode = LEDC_LOW_SPEED_MODE,  // High-speed mode
        .channel = LEDC_CHANNEL_0,           // Use channel 0
        .timer_sel = LEDC_TIMER_0,           // Select timer 0
        .duty = 512,                         // Initial duty cycle (50%)
        .hpoint = 0                          // Hardware timer high point (0 by default)
};

// Function prototypes

void w_trans (uint16_t address, uint16_t data );
uint8_t r_trans(uint8_t address);
void led_w(uint8_t *led_data);
void IRAM_ATTR gpio_isr_handler(void* arg);
void IRAM_ATTR drdy_isr_handler(void* arg);
void handle_activity_task(void* arg);
void handle_drdy_task(void* arg);
void activity_sequence();
void inactivity_sequence();
void accel_get_modes();
int get_face();
int compare_vectors(const float a[3], const float b[3], float tolerance);
void light_face(int face);
void led_test();
void play_tune(int (*tune)[2], int num_notes);
void initialize_timer();
void timer_sleep(void *arg);
void stop_timer();
void restart_timer(uint64_t interval_us);
void roll_1();
void roll_20();

int app_main(void)
{
    // Local variable declarations
    int8_t z_offset_acc = (int8_t)(Z_OFFSET_ACC * USR_OFFSET_FACTOR+0.5);

    // ISR setup
    gpio_config_t activity_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_INT2),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    gpio_config_t drdy_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_INT1),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,

    };
    
    gpio_config_t heartbeat_conf = {
        .pin_bit_mask = (1ULL << LED_HEARTBEAT), // Pin mask for GPIO15
        .mode = GPIO_MODE_OUTPUT,                 // Set as output mode
        .pull_up_en = GPIO_PULLUP_DISABLE,        // Disable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,    // Disable pull-down
        .intr_type = GPIO_INTR_DISABLE            // Disable interrupts
    };
    gpio_config(&heartbeat_conf);

    


    // Off LED setup
    for (int i = 0; i < LED_NORTH_CHAIN * 3; i++){
        led_off[i]=0;
    }

    // Red LED setup
    for (int i = 0; i < LED_NORTH_CHAIN; i++){

        led_red[3*i]   = 0x00;
        led_red[3*i+1] = 0xF;
        led_red[3*i+2] = 0x00;
    }

    // Green LED setup
    for (int i = 0; i < LED_NORTH_CHAIN; i++){

        led_green[3*i]   = 0xF;
        led_green[3*i+1] = 0x00;
        led_green[3*i+2] = 0x00;
    }

    // Blue LED setup
    for (int i = 0; i < LED_NORTH_CHAIN; i++){

        led_blue[3*i]   = 0x00;
        led_blue[3*i+1] = 0x00;
        led_blue[3*i+2] = 0xF;
    }

    // LED RMT setup
    if (rmt_LED_north) {
        rmt_del_channel(rmt_LED_north);
        rmt_LED_north = NULL;   
    }   
    ESP_ERROR_CHECK(rmt_new_tx_channel(&rmt_north_config, &rmt_LED_north));
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bytes_encoder_config, &bytes_encoder));


    // SPI bus and device setup
    spi_bus_config_t spi_bus_cfg = {        // configure SPI bus properties and pins
        .miso_io_num = PIN_MISO,            // Set up IO pins 
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };



    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));
    

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &accel_config ,&accel ));
    ESP_LOGD("SETUP","SPI was set up with no errors");
    
    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t whoami = r_trans(0x0F);
    ESP_LOGD("WHOAMI", "Value: 0x%02X", whoami);
    if (whoami != 0x6C){
        ESP_LOGE("WHOAMI", "DATA READ FAILURE");
        return -1;
    }
    

    // Reset and intialize sensors
    ESP_LOGD("SETUP","Initializing Accel and Gyro");
    w_trans(CTRL3_C, 0x1);    // Reset the sensor
    w_trans(CTRL1_XL, 0x92);    // Set the speed and rate of the accel
    w_trans(CTRL2_G, 0x94);     // Set the speed and rate of the gyro
    w_trans(CTRL6_C, 0x00);     // Set high performance for accel and xyz offset resolution (2^-10)
    w_trans(CTRL7_G, 0x02);     // Enable User offsets for low pass filter
    w_trans(CTRL8_XL, 0x00);    // Set up Low Pass Filter for accel

    // Set up DRDY interrupt detection on INT1
    w_trans(INT1_CTRL, 0x01); // Accelerometer data-ready interrupt on INT1

    ESP_LOGD("SETUP","Initializing interrupt detection for Activity/Inactivity");
    // Set up Inactivity Detection intterupt on INT2
    w_trans(WAKEUP_DUR, 0x08);       // Set inactivity time 
    w_trans(WAKEUP_THS, 0x10);       // Set inactivity threshold
    w_trans(TAP_CFG0, 0x20);         // Set sleep-change notification
    w_trans(TAP_CFG2, 0xE0);         // Enable interrupt
    w_trans(MD2_CFG, 0x80);          // Route interrupt to INT2
    w_trans(Z_OFS_ACC, z_offset_acc); // Set the calculated z offset



    led_w(led_green);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 2000 ms
    led_w(led_off);
    initialize_timer();

    ESP_LOGD("SETUP","Activating interrupt handling for Activity/Inactivity interrupts");
    gpio_config(&activity_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_INT2,gpio_isr_handler, NULL);
    xTaskCreate(handle_activity_task, "handle_activity_task", 4096, NULL, 10, NULL);
    
    gpio_config(&drdy_conf);
    gpio_isr_handler_add(PIN_INT1, drdy_isr_handler, NULL);
    xTaskCreate(handle_drdy_task, "handle_drdy_task", 4096, NULL, 10, &drdy_task_handle);
    
    
    ESP_LOGD("SETUP","Setup complete!");
    ledc_timer_config(&piezo_timer);
    ledc_channel_config(&piezo_channel);
    vTaskDelay(pdMS_TO_TICKS(50)); // Delay for 100 ms
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0); // Turn off PWM 
    //led_test();
    
    //play_tune(tune, 9);

    
    while(1){

        if(heartbeat_state) {
            gpio_set_level(LED_HEARTBEAT, 1);
            heartbeat_state = false;
        }
        else {
            gpio_set_level(LED_HEARTBEAT, 0);
            heartbeat_state = true;
        }
        usleep(BLINK_USEC);
       
    }

    return 0;
    
}

void w_trans (uint16_t address, uint16_t data ){
    
    spi_transaction_t transfer = { 0 };
    
    transfer.length = 8;
    ESP_LOGD("WRITE_TRANS", "Setting flags");
    transfer.flags = SPI_TRANS_USE_TXDATA;
    ESP_LOGD("WRITE_TRANS", "Setting address");   
    transfer.cmd = address;
    ESP_LOGD("WRITE_TRANS", "Setting data");
    transfer.tx_data[0] = data;
    ESP_LOGD("WRITE_TRANS", "Attempting write at address 0x%02X", address);
    ESP_ERROR_CHECK(spi_device_transmit(accel, &transfer) );
    ESP_LOGD("WRITE_TRANS", "Write at address 0x%02X completed", address);

    return;

}
 
uint8_t r_trans(uint8_t address){

    spi_transaction_t transfer = {0};
    transfer.length = 8;
    transfer.rxlength = 8;
    ESP_LOGD("READ_TRANS", "Setting flags");
    transfer.flags = SPI_TRANS_USE_RXDATA;
    ESP_LOGD("READ_TRANS", "Setting address");   
    transfer.cmd = (address | 0x80) ;
    ESP_LOGD("READ_TRANS", "Attempting read at address 0x%02x", address);
    ESP_ERROR_CHECK(spi_device_transmit(accel, &transfer));

    ESP_LOGD("READ_TRANS", "Read at address 0x%02X completed", address);

    return transfer.rx_data[0];

}


void r_accel(){

    if (!is_drdy) {
        while(!is_drdy);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    is_drdy = false;

    spi_transaction_t trans[3] = {};
    spi_transaction_t *receive[3] = {};

    ESP_LOGD("ACCEL_READ", "Setting up read transaction 0");
    trans[0].length = 32;                                   // Each transaction has a length of 4 bytes (32 bits)
    trans[0].rxlength = 32;                                 
    trans[0].cmd = ( ACCEL_X_LOW  | 0x80 );                 // Increment starting addres by 4 each iteration
    trans[0].flags = SPI_TRANS_USE_RXDATA;                  // Use rx_data until DMA access becomes necessary
    trans[0].rx_buffer = NULL;
    trans[0].tx_buffer = NULL;
    ESP_LOGD("ACCEL_READ", "Queueing read transaction 0");
    ESP_ERROR_CHECK(spi_device_queue_trans(accel, &trans[0], portMAX_DELAY));


    ESP_LOGD("ACCEL_READ", "Setting up read transaction 1");
    trans[1].length = 16;
    trans[1].rxlength = 16;
    trans[1].cmd = (ACCEL_Z_LOW | 0x80);
    trans[1].flags = SPI_TRANS_USE_RXDATA;
    trans[1].rx_buffer = NULL;
    trans[1].tx_buffer = NULL;
    ESP_LOGD("ACCEL_READ", "Queueing read transaction 1");
    ESP_ERROR_CHECK(spi_device_queue_trans(accel, &trans[1], portMAX_DELAY));


    ESP_LOGD("ACCEL_READ", "Setting up read transaction 2");
    trans[2].length = 16;
    trans[2].rxlength = 16;
    trans[2].cmd = (TEMP_LOW | 0x80);
    trans[2].flags = SPI_TRANS_USE_RXDATA;
    trans[2].rx_buffer = NULL;
    trans[2].tx_buffer = NULL;
    ESP_LOGD("ACCEL_READ", "Queueing read transaction 2");
    ESP_ERROR_CHECK(spi_device_queue_trans(accel, &trans[2], portMAX_DELAY));

    ESP_LOGD("ACCEL_READ", "Receiving data from read transaction 0");
    ESP_ERROR_CHECK(spi_device_get_trans_result(accel, &receive[0],portMAX_DELAY));
    ESP_LOGD("ACCEL_READ", "Receiving data from read transaction 1");
    ESP_ERROR_CHECK(spi_device_get_trans_result(accel, &receive[1],portMAX_DELAY));
    ESP_LOGD("ACCEL_READ", "Receiving data from read transaction 2");
    ESP_ERROR_CHECK(spi_device_get_trans_result(accel, &receive[2],portMAX_DELAY));

    ESP_LOGD("ACCEL_READ", "Transferring data from read transactions");
    accel_temp_burst[0] = receive[0]->rx_data[0];
    accel_temp_burst[1] = receive[0]->rx_data[1];
    accel_temp_burst[2] = receive[0]->rx_data[2];
    accel_temp_burst[3] = receive[0]->rx_data[3];
    accel_temp_burst[4] = receive[1]->rx_data[0];
    accel_temp_burst[5] = receive[1]->rx_data[1];
    accel_temp_burst[6] = receive[2]->rx_data[0];
    accel_temp_burst[7] = receive[2]->rx_data[1];

    ESP_LOGD("ACCEL_READ", "Burst Data: %02X %02X %02X %02X %02X %02X %02X %02X",
    accel_temp_burst[0], accel_temp_burst[1], accel_temp_burst[2], accel_temp_burst[3],
    accel_temp_burst[4], accel_temp_burst[5], accel_temp_burst[6], accel_temp_burst[7]);

    ESP_LOGD("ACCEL_READ", "Transaction complete");
    return;
}

void led_w(uint8_t *led_data){
    
    ESP_ERROR_CHECK (rmt_enable( rmt_LED_north));

    ESP_LOGD("LEDW", "Setting up LED write");
    rmt_transmit_config_t led_tx = {
        .loop_count = 0  // No looping
    };

    ESP_LOGD("LEDW", "Attempting LED write");
    ESP_ERROR_CHECK(rmt_transmit(rmt_LED_north, bytes_encoder, led_data, LED_NORTH_CHAIN * 3, &led_tx));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(rmt_LED_north, portMAX_DELAY));
    ESP_LOGD("LEDW", "LED write completed");
    ESP_ERROR_CHECK (rmt_disable( rmt_LED_north));

    return;
}

void IRAM_ATTR drdy_isr_handler(void* arg) {
    is_drdy = true;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(drdy_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}


void IRAM_ATTR gpio_isr_handler(void* arg) {
    // Check the level of the GPIO to determine edge
    interrupt_triggered = true;
    int level = gpio_get_level(PIN_INT2);

    if (level == 1) {
        // Rising edge detected (inactivity detected)
        // Handle inactivity start here
        inactive_detect = true;
        esp_sleep_enable_ext1_wakeup(1ULL << PIN_INT2, 0);
    } 
    
    else {
        // Falling edge detected (activity resumed)
        // Handle activity resumption here
        active_detect = true;
    }

    return;
}

void handle_activity_task(void* arg) {
    while (1) {
        if (interrupt_triggered) {
            interrupt_triggered = false;  // Clear the flag

            if(inactive_detect){


                inactive_detect = false;
                
                ESP_LOGI("HANDLER", "Inactive detected");
                if (!first_inact){
                    first_inact = true;
                    ESP_LOGI("HANDLER", "Ignoring first inactivity");
                } else {
                ESP_LOGD("HANDLER", "Running inactivity sequence");
                inactivity_sequence();
                }
            }
            else if (active_detect) {
                ESP_LOGD("HANDLER", "Running activity sequence");
                activity_sequence();

            }
            
        }
        
        // Short delay to yield CPU time
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void handle_drdy_task(void* pvParameters) {
    while (1) {
        // Wait for the ISR notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Block until notified

        ESP_LOGD("DRDY_ISR", "DRDY interrupt triggered");
    }
}

void activity_sequence() {
    active_detect = false;
    ESP_LOGI("ACTIVE", "Active detected");
    while(!inactive_detect){
        ESP_LOGD("ACTIVE", "In while loop");
        led_w(led_blue);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return;
}

void inactivity_sequence(){
    int detected_face = -1;
    stop_timer();
    restart_timer(TIMER_DUR * 1000000);
    inactive_detect = false;
    ESP_LOGD("INACTIVE", "Inactive detected");
    led_w(led_off);
    accel_get_modes();     
    ESP_LOGE("INACTIVE", "Accelerometer values: X %.3f g, Y %.3f g, Z %.3f g",
    accel_results[0],
    accel_results[1],
    accel_results[2]);
    
    detected_face = get_face();
    while (detected_face == -1) {
        accel_get_modes();
        detected_face = get_face();
        vTaskDelay(pdMS_TO_TICKS(10));

    } 
    
    ESP_LOGE("MODES", "Face gathered: %d", detected_face + 1);

    if (detected_face == 0) roll_1();
    else if (detected_face == 19) roll_20();

    light_face(detected_face);
    vTaskDelay(pdMS_TO_TICKS(100));
    


    return;
}


void accel_get_modes() {
    float x_arr[SENSOR_DATA_GRAB_REPEATS] = {};
    float y_arr[SENSOR_DATA_GRAB_REPEATS] = {};
    float z_arr[SENSOR_DATA_GRAB_REPEATS] = {};

    int x_counter = 0, y_counter = 0, z_counter = 0;
    float x_sum = 0, y_sum = 0, z_sum = 0;

    for (int i = 0; i < SENSOR_DATA_GRAB_REPEATS; i++) {
        ESP_LOGD("MODES", "Sampling sensor");
        
        r_accel();

        // Convert raw data
        int16_t x_raw = (int16_t)((accel_temp_burst[0]) | (accel_temp_burst[1] << 8));
        int16_t y_raw = (int16_t)((accel_temp_burst[2]) | (accel_temp_burst[3] << 8));
        int16_t z_raw = (int16_t)((accel_temp_burst[4]) | (accel_temp_burst[5] << 8));

        x_arr[i] = x_raw * ACCEL_SENSITIVITY_4G;
        y_arr[i] = y_raw * ACCEL_SENSITIVITY_4G;
        z_arr[i] = z_raw * ACCEL_SENSITIVITY_4G;

        // Log raw values
        ESP_LOGD("MODES", "X_RAW: %d, Y_RAW: %d, Z_RAW: %d", x_raw, y_raw, z_raw);

        // Accumulate non-zero values
        if (fabs(x_arr[i]) > 0.01) { x_sum += x_arr[i]; x_counter++; }
        if (fabs(y_arr[i]) > 0.01) { y_sum += y_arr[i]; y_counter++; }
        if (fabs(z_arr[i]) > 0.01) { z_sum += z_arr[i]; z_counter++; }

    }

    accel_results[0] = (x_counter > 0) ? x_sum / x_counter : 0;
    accel_results[1] = (y_counter > 0) ? y_sum / y_counter : 0;
    accel_results[2] = (z_counter > 0) ? z_sum / z_counter : 0;

    ESP_LOGD("MODES", "Averages: X=%.3f g, Y=%.3f g, Z=%.3f g", accel_results[0], accel_results[1], accel_results[2]);

    
    

}

int get_face(){

    for (int i = 0; i < NUM_FACES; i++) {
        if (compare_vectors(accel_results, faces[i], TOLERANCE)) {
            ESP_LOGD("FACE MATCH", "Match found with FACE_%d\n", i + 1);
            return i; // Exit after finding the first match
        }
    }

    return -1;

}

int compare_vectors(const float a[3], const float b[3], float tolerance) {
    for (int i = 0; i < 3; i++) {
        if (fabs(a[i] - b[i]) > tolerance) {
            return 0; // Not equal
        }
    }
    return 1; // Equal
}

void light_face(int face){

    memcpy( led_current, led_off, sizeof(led_off));


    led_current[3*(led_order[face]) + 1] = 0xFF;
    led_current[3*(led_order[face]) + 2] = 0xFF;

    led_w(led_current);

    return;
}

void led_test(){

    uint8_t led_test[LED_NORTH_CHAIN * 3] = {0};

    for (int i = 0; i < LED_NORTH_CHAIN; i++){
        
        led_w(led_off);
        vTaskDelay(pdMS_TO_TICKS(2)); // Delay for 200 ms
        memcpy( led_test, led_off, sizeof(led_off));
        

        led_test[3*i] = 0x00;
        led_test[3*i+1] = 0x00;
        led_test[3*i+2] = 0x0F;

        

        led_w(led_test);
        vTaskDelay(pdMS_TO_TICKS(100)); 


    }

    led_w(led_blue);
    vTaskDelay(pdMS_TO_TICKS(1000));
    led_w(led_green);
    vTaskDelay(pdMS_TO_TICKS(1000));
    led_w(led_red);
    vTaskDelay(pdMS_TO_TICKS(1000));

    return;
}

void play_tune(int (*tune)[2], int num_notes) {
    
    // Play each note in the tune
    for (int i = 0; i < num_notes; i++) {
        int frequency = tune[i][0]; // Frequency in Hz
        int duration = tune[i][1];  // Duration in milliseconds

        // Set the frequency for the current note
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, frequency);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512); // Set duty cycle to 50%
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);   // Apply duty cycle

        // Play the note for the specified duration
        vTaskDelay(pdMS_TO_TICKS(duration));

        // Pause between notes
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0); // Turn off the buzzer
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(100)); // Short pause between notes
    }

    // Stop the buzzer after the tune
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0); // Turn off PWM and set output low
}


void initialize_timer()
{
    // Define the timer configuration
    const esp_timer_create_args_t timer_config = {
        .callback = &timer_sleep,  // Callback function
        .arg = NULL,               // Argument passed to the callback (optional)
        .name = "one_shot_timer"   // Timer name (useful for debugging)
    };


    // Create the timer
    esp_err_t ret = esp_timer_create(&timer_config, &timer_handle);
    if (ret != ESP_OK) {
        ESP_LOGE("TIMER", "Failed to create timer: %s", esp_err_to_name(ret));
        return;
    }

    // Start the timer (2 minutes = 120,000,000 microseconds)
    ret = esp_timer_start_once(timer_handle, 120 * 1000000); // Timer interval in microseconds (20s)
    if (ret != ESP_OK) {
        ESP_LOGE("TIMER", "Failed to start timer: %s", esp_err_to_name(ret));
        esp_timer_delete(timer_handle); // Clean up in case of failure
        return;
    }

    ESP_LOGI("TIMER", "Timer initialized to go off in 2 minutes.");
}

void timer_sleep(void *arg)
{
    ESP_LOGE("TIMER", "Timer expired! Calling timer_sleep() function.");
    led_w(led_off);
    esp_deep_sleep_start();
}

void stop_timer()
{
    if (timer_handle) {
        esp_err_t ret = esp_timer_stop(timer_handle);
        if (ret == ESP_OK) {
            ESP_LOGE("STOP TIMER", "Timer stopped successfully.");
        } else if (ret == ESP_ERR_INVALID_STATE) {
            ESP_LOGE("STOP TIMER", "Timer was not running.");
        } else {
            ESP_LOGE("STOP TIMER", "Failed to stop timer: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE("STOP TIMER", "Timer handle is NULL. Timer might not be initialized.");
    }
}

void restart_timer(uint64_t interval_us)
{
    if (timer_handle) {
        esp_err_t ret = esp_timer_start_once(timer_handle, interval_us);
        if (ret == ESP_OK) {
            ESP_LOGE("RESET TIMER", "Timer restarted successfully with interval %llu microseconds.", interval_us);
        } else if (ret == ESP_ERR_INVALID_STATE) {
            ESP_LOGE("RESET TIMER", "Timer is already running. Stop it before restarting.");
        } else {
            ESP_LOGE("RESET TIMER", "Failed to restart timer: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE("RESET TIMER", "Timer handle is NULL. Timer might not be initialized.");
    }
}

void roll_1(){

    int tune[11][2] = {
    {NOTE_D4, 500},
    {NOTE_D4, 375},
    {NOTE_D4, 125}, 
    {NOTE_D4, 500},  
    {NOTE_F4, 250},  
    {NOTE_E4, 250},
    {NOTE_E4, 375},
    {NOTE_D4, 125}, 
    {NOTE_D4, 375}, 
    {NOTE_CS4,125}, 
    {NOTE_D4, 1000}, 
    };
    ESP_LOGE("NAT1", "You rolled a 1");
    led_w(led_red);
    vTaskDelay(pdMS_TO_TICKS(500)); 
    led_w(led_off);
    vTaskDelay(pdMS_TO_TICKS(100));
    led_w(led_red);
    vTaskDelay(pdMS_TO_TICKS(500)); 
    led_w(led_off);
    vTaskDelay(pdMS_TO_TICKS(100));
    led_w(led_red);
    vTaskDelay(pdMS_TO_TICKS(500)); 
    led_w(led_off);
    vTaskDelay(pdMS_TO_TICKS(100));
    play_tune(tune,11);
    return; 

}

void roll_20(){

    int tune[9][2] = {
    {NOTE_B4, 83},
    {NOTE_B4, 83},
    {NOTE_B4, 83}, 
    {NOTE_B4, 250},  
    {NOTE_G4, 250},  
    {NOTE_A4, 250},
    {NOTE_B4, 166},
    {NOTE_A4, 83}, 
    {NOTE_B4, 500 }, 
    };
    ESP_LOGE("NAT20", "You rolled a 20");
    led_w(led_green);
    vTaskDelay(pdMS_TO_TICKS(500)); 
    led_w(led_off);
    vTaskDelay(pdMS_TO_TICKS(100));
    led_w(led_green);
    vTaskDelay(pdMS_TO_TICKS(500)); 
    led_w(led_off);
    vTaskDelay(pdMS_TO_TICKS(100));
    led_w(led_green);
    vTaskDelay(pdMS_TO_TICKS(500)); 
    led_w(led_off);
    vTaskDelay(pdMS_TO_TICKS(100));

    play_tune(tune,9);
    return; 



}