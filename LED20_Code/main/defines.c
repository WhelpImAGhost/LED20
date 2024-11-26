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
#define INT1_CTRL 0x0D
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL3_C 0x12
#define CTRL4_C 0x13
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


#define TEMP_LOW 0x20
#define TEMP_HIGH 0x21



// WS2812 Timing values
#define T0H 3   // 0.3 µs high for "0" bit
#define T0L 9   // 0.85 µs low for "0" bit
#define T1H 6   // 0.6 µs high for "1" bit
#define T1L 6   // 0.6 µs low for "1" bit

#define NUM_FACES 20

// Die face accel values
// Order: X, Y, Z
// Value range: -1 to 1 g
#define FACE_1  {0, 0, 1}
#define FACE_2  {0, 1, 0}
#define FACE_3  {1, 0, 0}
#define FACE_4  {0, 0, -1}
#define FACE_5  {0, -1, 0}
#define FACE_6  {-1, 0, 0}
#define FACE_7  {19, 20, 21}
#define FACE_8  {22, 23, 24}
#define FACE_9  {25, 26, 27}
#define FACE_10 {28, 29, 30}
#define FACE_11 {31, 32, 33}
#define FACE_12 {34, 35, 36}
#define FACE_13 {37, 38, 39}
#define FACE_14 {40, 41, 42}
#define FACE_15 {43, 44, 45}
#define FACE_16 {46, 47, 48}
#define FACE_17 {49, 50, 51}
#define FACE_18 {52, 53, 54}
#define FACE_19 {55, 56, 57}
#define FACE_20 {58, 59, 60}