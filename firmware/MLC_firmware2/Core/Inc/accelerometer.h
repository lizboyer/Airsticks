/*
 * accelerometer.h
 */

#ifndef INC_ACCELEROMETER_H_
#define INC_ACCELEROMETER_H_

/************************** DEVICE ADDRESSES **************************/
#define ACC0_W_ADDR			0xd4
#define ACC0_R_ADDR			0xd5
#define ACC1_W_ADDR			0xd6
#define ACC1_R_ADDR			0xd7


/************************** DEVICE REGISTERS **************************/
#define REG_CTRL1_XL		0x10
#define REG_CTRL2_G			0x11
#define REG_INT1_CTRL 		0x0d
#define REG_INT2_CTRL 		0x0e


/************************** REGISTER CONFIGS **************************/
#define ACC_104HZ_2G		0x40
#define ACC_104HZ_8G 		0x4C

#define GYRO_OFF			0x00
#define DATA_RDY			0x01

#define OUTX_L_A			0x28
#define OUTX_H_A			0x29
#define OUTY_L_A			0x2a
#define OUTY_H_A			0x2b
#define OUTZ_L_A			0x2c
#define OUTZ_H_A			0x2d

/************************** MACHINE LEARNING REGISTERS **************************/
#define MLC0_SRC 			0x70 // decision tree 1 result (0-15?)
#define MLC_STATUS_MAINPAGE 0x38 // outputs from all decision trees
#define MLC_INT2			0x11 // used to drive change in decision tree to INT1
#define MLC_EN				0x05 // enable machine learning
#define MLC_INT1 			0x0D // route MLC to interrupt pin 1
#define FUNC_CFG_ACCESS 	0x01 // allows writing to initialize MLC

#include "stm32l0xx.h"
#include <stdint.h>

/**************************** TYPEDEFS ********************************/
typedef enum axis {
	X_AXIS,
	Y_AXIS,
	Z_AXIS,
	ALL_AXIS
} axis_t;

typedef struct accelerometer_t {
	uint8_t slave_r_addr;
	uint8_t slave_w_addr;
	int16_t x_xlr;
	int16_t y_xlr;
	int16_t z_xlr;
	int64_t x_pos;
	int64_t y_pos;
	int64_t z_pos;
	uint16_t irq_pin;
} accelerometer_t;


/*********************** FUNCTION PROTOTYPES ***************************/

/*
 * acc_init
 * inputs:
 * 		- accelerometer_t acc: pointer to the accelerometer to initialize
 * returns:
 * 		- none
 * side effects:
 * 		- configure the accelerometer to 100Hz polling, and turn off gyro
 */
extern HAL_StatusTypeDef acc_init(volatile accelerometer_t* acc);

/*
 * read_axis
 * inputs:
 * 		- accelerometer_t acc: pointer the the accelerometer struct from
 * 				which to read the acceleration values
 * 		- axis_t axis: enumerator representing the axis to read
 * returns:
 * 		- none
 * 	side effects:
 * 		- updates the acceleration values inside the accelerometer struct
 **/
extern HAL_StatusTypeDef read_axis(volatile accelerometer_t* acc, axis_t axis);


/*
 * read_axis
 * inputs:
 * 	    - accelerometer_t acc: pointer the the accelerometer struct to
 * 	    		which to write data to
 * 		- int8_t reg: the register within the accelerometer to write to
 * 		- int8_t data: the data to write
 * returns:
 * 		- none
 * 	side effects:
 * 		- performs and I2C write to the passed in accelerometer
 **/
extern HAL_StatusTypeDef accelerometer_write(volatile accelerometer_t* acc, uint8_t reg, uint8_t data);

extern uint32_t lsm6dsox_mlc_out_get(volatile accelerometer_t* acc, uint8_t *buff);

extern HAL_StatusTypeDef accelerometer_read(volatile accelerometer_t* acc, uint8_t reg, uint8_t data);


#endif /* INC_ACCELEROMETER_H_ */ß
