/**
 * A comprehensive list of registers used in the LED20
 * 
 * 
 */


// Control Registes


#define FUNC_CFG_ACCESS 0x01
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL3_C 0x12
#define WAKEUP_DUR 0x5C
#define WAKEUP_THS 0x5B
#define TAP_CFG0 0x56
#define TAP_CFG2 0x58
#define MD1_CFG 0x5E
#define MD2_CFG 0x5F


// Embedded Function Control Registers

#define EMB_FUNC_EN_A 0x04
#define EMB_FUNC_INT1 0x0A
#define EMB_FUNC_INIT_A 0x66
#define PAGE_RW 0x17


// Data storage registers
#define GYRO_X_LOW 0x22

/*  The rest of these do not need to be defined, 
    as incremental register reading takes care
    of reading all 12 addresses.


#define GYRO_X_HGIH 0x23
#define GYRO_Y_LOW 0x24
#define GYRO_Y_HIGH 0x25
#define GYRO_Z_LOW 0x26
#define GYRO_Z_HIGH 0x27

#define ACCEL_X_LOW 0x28
#define ACCEL_X_HIGH 0x29
#define ACCEL_Y_LOW 0x2A
#define ACCEL_Y_HIGH 0x2B
#define ACCEL_Z_LOW 0x2C
#define ACCEL_Z_HIGH 0x2D
*/

#define TEMP_LOW 0x20
#define TEMP_HIGH 0x21



// WS2812 Timing values
#define T0H 3   // 0.3 µs high for "0" bit
#define T0L 9   // 0.85 µs low for "0" bit
#define T1H 6   // 0.6 µs high for "1" bit
#define T1L 6   // 0.6 µs low for "1" bit