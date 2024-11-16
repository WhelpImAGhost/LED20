/**
 * A comprehensive list of registers used in the LED20
 * 
 * 
 */


// Control Registes
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#define FUNC_CFG_ACCESS 0x01
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL3_C 0x12
#define CTRL6_C 0x15
#define CTRL7_G 0x16
#define CTRL8_XL 0x17
#define WAKEUP_DUR 0x5C
#define WAKEUP_THS 0x5B
#define TAP_CFG0 0x56
#define TAP_CFG2 0x58
#define MD1_CFG 0x5E
#define MD2_CFG 0x5F
#define X_OFS_ACC 0x73
#define Y_OFS_ACC 0x74
#define Z_OFS_ACC 0x75
#define USR_OFFSET_FACTOR (1 << 10)


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


// Die face accel values
// Order: X, Y, Z
// Value range: 0 to 1 g
#define FACE_1  [int16_t]{1, 2, 3}
#define FACE_2  [int16_t]{4, 5, 6}
#define FACE_3  [int16_t]{7, 8, 9}
#define FACE_4  [int16_t]{10, 11, 12}
#define FACE_5  [int16_t]{13, 14, 15}
#define FACE_6  [int16_t]{16, 17, 18}
#define FACE_7  [int16_t]{19, 20, 21}
#define FACE_8  [int16_t]{22, 23, 24}
#define FACE_9  [int16_t]{25, 26, 27}
#define FACE_10 [int16_t]{28, 29, 30}
#define FACE_11 [int16_t]{31, 32, 33}
#define FACE_12 [int16_t]{34, 35, 36}
#define FACE_13 [int16_t]{37, 38, 39}
#define FACE_14 [int16_t]{40, 41, 42}
#define FACE_15 [int16_t]{43, 44, 45}
#define FACE_16 [int16_t]{46, 47, 48}
#define FACE_17 [int16_t]{49, 50, 51}
#define FACE_18 [int16_t]{52, 53, 54}
#define FACE_19 [int16_t]{55, 56, 57}
#define FACE_20 [int16_t]{58, 59, 60}