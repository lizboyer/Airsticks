/*
 * accelerometer.c
 */

#include "accelerometer.h"
#include "stm32g4xx.h"
#include "MLC_configuration.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c1;

/*
 * acc_init
 * inputs:
 * 		- accelerometer_t acc: pointer to the accelerometer to initialize
 * returns:
 * 		- HAL Status
 * side effects:
 * 		- configure the accelerometer to 100Hz polling, and turn off gyro
 **/
HAL_StatusTypeDef acc_init (volatile accelerometer_t* acc)
{

	HAL_StatusTypeDef status = HAL_OK;

	// configure the accelerometer to 104Hz
	status = accelerometer_write(acc, REG_CTRL1_XL, ACC_104HZ_8G);
	if(status != HAL_OK) return status;

	// turn the gyroscope off
	status = accelerometer_write(acc, REG_CTRL2_G, GYRO_OFF);
	if (status != HAL_OK) return status;

	// enable interrupts on new data on accelerometer INT2
    status = accelerometer_write(acc, REG_INT2_CTRL, DATA_RDY);
    if (status != HAL_OK) return status;

    // read the axes to get interrupts to kick off
    status = read_axis(acc, ALL_AXIS);

    // enable cfg reg
//    status = accelerometer_write(acc, MLC_INT1, 0x80);
//    if (status != HAL_OK) return status;

    // enable MLC
//    status = accelerometer_write(acc, MLC_EN, 0x10);
//    if (status != HAL_OK) return status;

    // MLC route to INT1
//    status = accelerometer_write(acc, MLC_INT1, 0x01);
//    if (status != HAL_OK) return status;

//    for (int i = 0; i < (sizeof(mlc_configuration) /
//                      sizeof(ucf_line_t) ); i++ ) {
//      accelerometer_write(acc, mlc_configuration[i].address, (uint8_t)mlc_configuration[i].data);
//    }

    // disable cfg reg
//    status = accelerometer_write(acc, MLC_INT1, 0x00);
//    if (status != HAL_OK) return status;

    return status;

}


/*
 * read_axis
 * inputs:
 * 		- accelerometer_t acc: pointer the the accelerometer struct from
 * 				which to read the acceleration values
 * 		- axis_t axis: enumerator representing the axis to read
 * returns:
 * 		- HAL Status
 * 	side effects:
 * 		- updates the acceleration values inside the accelerometer struct
 **/
HAL_StatusTypeDef read_axis(volatile accelerometer_t* acc, axis_t axis)
{

//	__disable_irq();

    static uint8_t read_buffer[] = { 0 };

    HAL_StatusTypeDef status = HAL_OK;

    switch(axis){
	  case ALL_AXIS:
	  case X_AXIS:
		  status = HAL_I2C_Mem_Read(&hi2c1, acc->slave_r_addr, OUTX_H_A, I2C_MEMADD_SIZE_8BIT, read_buffer, sizeof(read_buffer), HAL_MAX_DELAY);
		  if(status != HAL_OK) break;
		  acc->x_xlr = *read_buffer << 8;
		  status = HAL_I2C_Mem_Read(&hi2c1, acc->slave_r_addr, OUTX_L_A, I2C_MEMADD_SIZE_8BIT, read_buffer, sizeof(read_buffer), HAL_MAX_DELAY);
		  if(status != HAL_OK) break;
		  acc->x_xlr = acc->x_xlr + *read_buffer;
		  if(axis != ALL_AXIS) break;
	  case Y_AXIS:
		  status = HAL_I2C_Mem_Read(&hi2c1, acc->slave_r_addr, OUTY_H_A, I2C_MEMADD_SIZE_8BIT, read_buffer, sizeof(read_buffer), HAL_MAX_DELAY);
		  if(status != HAL_OK) break;
		  acc->y_xlr = *read_buffer << 8;
		  status = HAL_I2C_Mem_Read(&hi2c1, acc->slave_r_addr, OUTY_L_A, I2C_MEMADD_SIZE_8BIT, read_buffer, sizeof(read_buffer), HAL_MAX_DELAY);
		  if(status != HAL_OK) break;
		  acc->y_xlr = acc->y_xlr + *read_buffer;
		  if(axis != ALL_AXIS) break;
	  case Z_AXIS:
		  status = HAL_I2C_Mem_Read(&hi2c1, acc->slave_r_addr, OUTZ_H_A, I2C_MEMADD_SIZE_8BIT, read_buffer, sizeof(read_buffer), HAL_MAX_DELAY);
		  if(status != HAL_OK) break;
		  acc->z_xlr = *read_buffer << 8;
		  status = HAL_I2C_Mem_Read(&hi2c1, acc->slave_r_addr, OUTZ_L_A, I2C_MEMADD_SIZE_8BIT, read_buffer, sizeof(read_buffer), HAL_MAX_DELAY);
		  if(status != HAL_OK) break;
		  acc->z_xlr = acc->z_xlr + *read_buffer;
		  break;
	}

//    __enable_irq();

    return status;
}


/*
 * read_axis
 * inputs:
 * 	    - accelerometer_t acc: pointer the the accelerometer struct to
 * 	    		which to write data to
 * 		- int8_t reg: the register within the accelerometer to write to
 * 		- int8_t data: the data to write
 * returns:
 * 		- HAL Status
 * 	side effects:
 * 		- performs and I2C write to the passed in accelerometer
 **/
HAL_StatusTypeDef accelerometer_write(volatile accelerometer_t* acc, uint8_t reg, uint8_t data)
{
	__disable_irq();
    uint8_t write_buffer[] = { 0 };
	*write_buffer = data;
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, acc->slave_w_addr, reg, I2C_MEMADD_SIZE_8BIT, write_buffer, sizeof(write_buffer), HAL_MAX_DELAY);
	__enable_irq();
	return status;
}
HAL_StatusTypeDef accelerometer_read(volatile accelerometer_t* acc, uint8_t reg, uint8_t* data)
{
	__disable_irq();
    uint8_t read_buffer[] = { 1 };
//	*read_buffer = data;
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, acc->slave_r_addr, reg, I2C_MEMADD_SIZE_8BIT, read_buffer, sizeof(read_buffer), HAL_MAX_DELAY);
	*data = *read_buffer;
	__enable_irq();
	return status;
}
