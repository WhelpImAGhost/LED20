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
#define FACE_1  { 0.000, -1.000,  0.000}
#define FACE_2  {-0.240,  0.340, -0.720}
#define FACE_3  { 0.200,  0.260,  0.430}
#define FACE_4  { 0.000, -0.700, -0.600}
#define FACE_5  {-0.930,  0.000,  0.000}
#define FACE_6  { 0.800,  0.000, -0.100}
#define FACE_7  {-0.300,  0.234,  0.800}
#define FACE_8  { 0.200,  0.780, -0.330}
#define FACE_9  { 0.600, -0.660, -0.280}
#define FACE_10 { 0.220,  0.300, -0.200}
#define FACE_11 {-0.250,  0.000,  0.900}
#define FACE_12 {-0.590,  0.590, -0.245}
#define FACE_13 {-0.150, -0.600,  0.400}
#define FACE_14 { 0.450, -0.300, -0.400}
#define FACE_15 {-0.520,  0.300,  0.200}
#define FACE_16 { 0.800,  0.300,  0.000}
#define FACE_17 { 0.000,  0.620,  0.560}
#define FACE_18 {-0.350, -0.300, -0.350}
#define FACE_19 { 0.440, -0.010,  0.840}
#define FACE_20 { 0.000,  1.000,  0.000}

// Noise notes
#define NOTE_A4  440
#define NOTE_B4  493
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_REST 0