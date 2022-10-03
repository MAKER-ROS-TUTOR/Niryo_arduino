#ifndef NIRYO_CONFIG_H
#define NIRYO_CONFIG_H


/*
 * Firmware version : 2.0.0
 */

#define NIRYO_STEPPER_VERSION_MAJOR 2
#define NIRYO_STEPPER_VERSION_MINOR 0
#define NIRYO_STEPPER_VERSION_PATCH 0

/*
 * All constants for Niryo One are defined below
 */


/*
 *    ----------- AS5600 Position sensor -------------
 */

#define AS5600_CPR 4096
#define AS5600_CPR_HALF 2048

#define AS5600_ADDRESS 0x36

#define AS5600_REG_CONF    0x07
#define AS5600_REG_ANGLE_H 0x0E
#define AS5600_REG_ANGLE_L 0x0F

/*
 *    ----------- Stepper controller -------------
 */

#define STEPPER_CPR 200

#define STEPPER_CONTROL_MODE_RELAX    0
#define STEPPER_CONTROL_MODE_STANDARD 1

#define STEPPER_DELAY_MIN 200

#define STEPPER_DEFAULT_MICRO_STEPS 8   // default 8
#define ONE_FULL_STEP 1800   // default 1800

#define STEPPER_CALIBRATION_OK        1
#define STEPPER_CALIBRATION_TIMEOUT   2
#define STEPPER_CALIBRATION_BAD_PARAM 3

#define ACCELSTEPPER_STEPS 16000

/*
 *    ----------- CAN bus -------------
 */

#define CAN_BROADCAST_ID  5

#define CAN_CMD_POSITION     0x03
#define CAN_CMD_TORQUE       0x04
#define CAN_CMD_MODE         0x07
#define CAN_CMD_MICRO_STEPS  0x13 
#define CAN_CMD_OFFSET       0x14
#define CAN_CMD_CALIBRATE    0x15
#define CAN_CMD_SYNCHRONIZE  0x16
#define CAN_CMD_MAX_EFFORT   0x17
#define CAN_CMD_MOVE_REL     0x18
#define CAN_CMD_RESET        0x19 // not yet implemented

#define CAN_DATA_POSITION    0x03
#define CAN_DATA_DIAGNOSTICS 0x08
#define CAN_DATA_CALIBRATION_RESULT 0x09
#define CAN_DATA_FIRMWARE_VERSION 0x10

// see https://github.com/arduino/ArduinoCore-samd/blob/master/variants/arduino_zero/variant.cpp
#define CAN_PIN_CS  10   
#define CAN_PIN_INT 9  


#endif
