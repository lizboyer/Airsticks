/*
 * accelerometer.c
 */

#include "accelerometer.h"
#include "stm32l0xx_hal.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c1;

/*
 * acc_init
 * inputs:
 * 		- accelerometer_t acc: pointer to the accelerometer to initialize
 * returns:
 * 		- none
 * side effects:
 * 		- configure the accelerometer to 100Hz polling, and turn off gyro
 **/
void acc_init (accelerometer_t* acc)
{

	// configure the accelerometer to 104Hz
	accelerometer_write(acc, REG_CTRL1_XL, ACC_104HZ_2G);

	// turn the gyroscope off
	accelerometer_write(acc, REG_CTRL2_G, GYRO_OFF);

	// enable interrupts on new data on accelerometer INT2
    accelerometer_write(acc, REG_INT2_CTRL, DATA_RDY);

}


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
void read_axis(accelerometer_t* acc, axis_t axis)
{
    static uint8_t read_buffer[] = { 0 };

    switch(axis){
	  case ALL_AXIS:
	  case X_AXIS:
		  HAL_I2C_Mem_Read(&hi2c1, acc->slave_r_addr, OUTX_H_A, I2C_MEMADD_SIZE_8BIT, read_buffer, sizeof(read_buffer), HAL_MAX_DELAY);
		  acc->x_measurement = *read_buffer << 8;
		  HAL_I2C_Mem_Read(&hi2c1, acc->slave_r_addr, OUTX_L_A, I2C_MEMADD_SIZE_8BIT, read_buffer, sizeof(read_buffer), HAL_MAX_DELAY);
		  acc->x_measurement = acc->x_measurement + *read_buffer;
		  if(axis != ALL_AXIS) break;
	  case Y_AXIS:
		  HAL_I2C_Mem_Read(&hi2c1, acc->slave_r_addr, OUTY_H_A, I2C_MEMADD_SIZE_8BIT, read_buffer, sizeof(read_buffer), HAL_MAX_DELAY);
		  acc->y_measurement = *read_buffer << 8;
		  HAL_I2C_Mem_Read(&hi2c1, acc->slave_r_addr, OUTY_L_A, I2C_MEMADD_SIZE_8BIT, read_buffer, sizeof(read_buffer), HAL_MAX_DELAY);
		  acc->y_measurement = acc->y_measurement + *read_buffer;
		  if(axis != ALL_AXIS) break;
	  case Z_AXIS:
		  HAL_I2C_Mem_Read(&hi2c1, acc->slave_r_addr, OUTZ_H_A, I2C_MEMADD_SIZE_8BIT, read_buffer, sizeof(read_buffer), HAL_MAX_DELAY);
		  acc->z_measurement = *read_buffer << 8;
		  HAL_I2C_Mem_Read(&hi2c1, acc->slave_r_addr, OUTZ_L_A, I2C_MEMADD_SIZE_8BIT, read_buffer, sizeof(read_buffer), HAL_MAX_DELAY);
		  acc->z_measurement = acc->z_measurement + *read_buffer;
		  break;
	}

    return;
}


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
void accelerometer_write(accelerometer_t* acc, int8_t reg, int8_t data)
{
    static uint8_t write_buffer[] = { 0 };

	*write_buffer = data;
	HAL_I2C_Mem_Write(&hi2c1, acc->slave_w_addr, reg, I2C_MEMADD_SIZE_8BIT, write_buffer, sizeof(write_buffer), HAL_MAX_DELAY);
}

